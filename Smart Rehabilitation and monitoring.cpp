/*
 * ============================================================
 * Q-REHAB OVERALL CONTROL
 * QNX 8.0 | Raspberry Pi 4 | AArch64
 *
 * This file keeps the original skeleton architecture and adds:
 *   1. Arduino JSON feedback through UART
 *   2. QNX native message passing between threads
 *   3. Real-time joint-limit validation
 *   4. Last-safe-position storage
 *   5. STOP and recovery commands
 *   6. Periodic watchdog using a QNX timer pulse
 *
 * Communication:
 *   Arduino -> QNX : newline-terminated JSON over UART
 *   QNX internal : MsgSend()/MsgReceive()
 *   QNX -> Arduino : JSON command over UART
 *
 * This is a prototype. Confirm the UART device, joint limits,
 * command format, and physical emergency-stop circuit before
 * operating the robotic arm.
 * ============================================================
 */

/*
 *  PROCESS OVERVIEW
 * 1. UART reader: receives newline-terminated JSON feedback from Arduino.
 * 2. QNX IPC: transfers complete feedback packets to the safety coordinator.
 * 3. Safety coordinator: validates joint angles and monitors timeout conditions.
 * 4. UART writer: sends STOP, recovery, and Peltier commands to Arduino.
 * 5. Timer pulse: triggers periodic checks without busy-waiting.
 *
 * Each process is intentionally separated so serial I/O does not directly
 * perform safety decisions, and safety decisions remain centralized.
 */

#include <sys/neutrino.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <stdint.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>

#define ARDUINO_SERIAL_DEVICE "/dev/ser1"
#define SERIAL_BAUD_RATE           B115200

#define SAFETY_CHECK_INTERVAL_MS    100
#define ARDUINO_FEEDBACK_TIMEOUT_MS 500
#define MAX_PELTIER_RUNTIME_MS   5000

#define ARDUINO_THREAD_PRIORITY    25
#define SAFETY_THREAD_PRIORITY     30

#define SAFETY_TIMER_PULSE_CODE         (_PULSE_CODE_MINAVAIL + 1)

enum MessageType : uint16_t {
    MESSAGE_ARDUINO_FEEDBACK = 1,
    MESSAGE_SHUTDOWN     = 2
};

struct RobotJointState {
    double baseAngle;
    double shoulderAngle;
    double elbowAngle;
    double wristAngle;
    double gripperAngle;
    int controlMode;
    uint64_t feedbackTimestampMs;
};

struct CoordinatorMessage {
    uint16_t type;
    uint16_t reserved;
    RobotJointState joints;
    char line[256];
};

struct ArduinoCommand {
    char data[256];
};

struct SafetyControllerState {
    bool armOperationAllowed;
    bool safetyLockLatched;
    bool peltierEnabled;
    bool hasLastSafePosition;
    RobotJointState lastSafeJointState;
    uint64_t lastFeedbackTimestampMs;
    uint64_t peltierStartTimestampMs;
};

struct JointSafetyLimit {
    const char* name;
    double minimumAngle;
    double maximumAngle;
};

static const JointSafetyLimit ROBOT_JOINT_SAFETY_LIMITS[] = {
    {"base",     10.0, 170.0},
    {"shoulder", 20.0, 160.0},
    {"elbow",    10.0, 170.0},
    {"wrist",    10.0, 170.0},
    {"gripper",  20.0, 100.0}
};

static volatile sig_atomic_t applicationRunning = 1;

static int coordinatorChannelId = -1;
static int coordinatorConnectionId = -1;
static int arduinoWriterChannelId = -1;
static int arduinoWriterConnectionId = -1;
static int arduinoSerialFileDescriptor = -1;

/* Process: Read a monotonic clock so timeout calculations are not affected by system-time changes.\n * This timestamp is used by the feedback watchdog and Peltier safety timer. */
static uint64_t getMonotonicTimeMilliseconds()
{
    struct timespec ts{};

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;
    }

    return static_cast<uint64_t>(ts.tv_sec) * 1000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000000ULL;
}

/* Process: Apply the requested real-time scheduling priority to the current thread.\n * If the operating system rejects it, log the reason and continue safely. */
static void configureCurrentThreadPriority(int priority)
{
    struct sched_param parameter{};
    parameter.sched_priority = priority;

    const int result = pthread_setschedparam(
        pthread_self(), SCHED_FIFO, &parameter
    );

    if (result != 0) {
        std::cerr << "[PRIORITY] " << std::strerror(result)
                  << " (continuing with current priority)"
                  << std::endl;
    }
}

/* Process: Configure the Arduino UART for raw 115200-baud communication.\n * The timeout settings allow the reader thread to check shutdown conditions regularly. */
static bool configureSerialPort(int fd)
{
    struct termios tty{};

    if (tcgetattr(fd, &tty) != 0) {
        perror("[UART] tcgetattr");
        return false;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, SERIAL_BAUD_RATE);
    cfsetospeed(&tty, SERIAL_BAUD_RATE);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

    /*
     * VTIME = 1 means read() waits for approximately 100 ms
     * before returning when no character is available.
     */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("[UART] tcsetattr");
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

/* Process: Send an Arduino feedback event to the safety coordinator through QNX IPC.\n * MsgSend blocks until the coordinator receives and replies to the message. */
static bool sendCoordinatorMessage(const CoordinatorMessage& event)
{
    if (coordinatorConnectionId == -1) {
        return false;
    }

    const int result = MsgSend(
        coordinatorConnectionId,
        &event,
        sizeof(event),
        nullptr,
        0
    );

    if (result != 0) {
        std::cerr << "[IPC] Event MsgSend failed: "
                  << std::strerror(result) << std::endl;
        return false;
    }

    return true;
}

/* Process: Place an outbound Arduino command into the writer thread's QNX channel.\n * The writer thread performs the actual UART write, keeping output serialized. */
static bool queueArduinoCommand(const char* command)
{
    if (arduinoWriterConnectionId == -1 || command == nullptr) {
        return false;
    }

    ArduinoCommand packet{};
    std::strncpy(packet.data, command, sizeof(packet.data) - 2);

    const size_t length = std::strlen(packet.data);
    if (length == 0 || length >= sizeof(packet.data) - 1) {
        return false;
    }

    if (packet.data[length - 1] != '\n') {
        packet.data[length] = '\n';
        packet.data[length + 1] = '\0';
    }

    const int result = MsgSend(
        arduinoWriterConnectionId,
        &packet,
        sizeof(packet),
        nullptr,
        0
    );

    if (result != 0) {
        std::cerr << "[IPC] Arduino command MsgSend failed: "
                  << std::strerror(result) << std::endl;
        return false;
    }

    return true;
}

/*
 * Finds a numeric JSON field.
 * This is intentionally small because the Arduino packet format
 * is known and controlled by this project.
 */
static bool readJsonNumber(
    const char* json,
    const char* key,
    double& value
)
{
    if (json == nullptr || key == nullptr) {
        return false;
    }

    char searchKey[64]{};
    std::snprintf(searchKey, sizeof(searchKey), "\"%s\"", key);

    const char* keyPosition = std::strstr(json, searchKey);
    if (keyPosition == nullptr) {
        return false;
    }

    const char* colon = std::strchr(keyPosition, ':');
    if (colon == nullptr) {
        return false;
    }

    char* endPosition = nullptr;
    const double parsed = std::strtod(colon + 1, &endPosition);

    if (endPosition == colon + 1 || !std::isfinite(parsed)) {
        return false;
    }

    value = parsed;
    return true;
}

static bool readJsonInteger(
    const char* json,
    const char* key,
    int& value
)
{
    double number = 0.0;

    if (!readJsonNumber(json, key, number)) {
        return false;
    }

    value = static_cast<int>(number);
    return true;
}

/* Process: Extract the expected servo-state values from one Arduino JSON packet.\n * A packet is accepted only when all required joint angles are present and finite. */
static bool parseServoStateFeedback(
    const char* json,
    RobotJointState& joints
)
{
    if (json == nullptr ||
        std::strstr(json, "\"type\":\"servo_state\"") == nullptr) {
        return false;
    }

    double baseAngle = 0.0;
    double shoulderAngle = 0.0;
    double elbowAngle = 0.0;
    double wristAngle = 0.0;
    double gripperAngle = 0.0;
    int controlMode = 0;

    const bool complete =
        readJsonNumber(json, "base", baseAngle) &&
        readJsonNumber(json, "shoulder", shoulderAngle) &&
        readJsonNumber(json, "elbow", elbowAngle) &&
        readJsonNumber(json, "wrist", wristAngle) &&
        readJsonNumber(json, "gripper", gripperAngle);

    if (!complete) {
        return false;
    }

    (void)readJsonInteger(json, "mode", controlMode);

    joints.baseAngle = baseAngle;
    joints.shoulderAngle = shoulderAngle;
    joints.elbowAngle = elbowAngle;
    joints.wristAngle = wristAngle;
    joints.gripperAngle = gripperAngle;
    joints.controlMode = controlMode;
    joints.feedbackTimestampMs = getMonotonicTimeMilliseconds();

    return true;
}

/* Process: Compare every reported joint angle with its configured safe range.\n * The first invalid joint is returned so the safety layer can explain the fault. */
static bool validateJointSafetyLimits(
    const RobotJointState& joints,
    const char*& failedJoint,
    double& failedValue
)
{
    const double values[] = {
        joints.baseAngle,
        joints.shoulderAngle,
        joints.elbowAngle,
        joints.wristAngle,
        joints.gripperAngle
    };

    for (size_t i = 0; i < 5; ++i) {
        if (!std::isfinite(values[i]) ||
            values[i] < ROBOT_JOINT_SAFETY_LIMITS[i].minimumAngle ||
            values[i] > ROBOT_JOINT_SAFETY_LIMITS[i].maximumAngle) {
            failedJoint = ROBOT_JOINT_SAFETY_LIMITS[i].name;
            failedValue = values[i];
            return false;
        }
    }

    failedJoint = nullptr;
    failedValue = 0.0;
    return true;
}

static void printRobotJointState(const RobotJointState& joints)
{
    std::cout
        << "[ARDUINO] Parsed servo_state"
        << " controlMode=" << joints.controlMode
        << " baseAngle=" << joints.baseAngle
        << " shoulderAngle=" << joints.shoulderAngle
        << " elbowAngle=" << joints.elbowAngle
        << " wristAngle=" << joints.wristAngle
        << " gripperAngle=" << joints.gripperAngle
        << std::endl;
}

/* Process: Latch the safety state and request that motion and Peltier output stop.\n * The latch prevents repeated stop commands until a separate reset design is added. */
static void activateEmergencyStop(SafetyControllerState& state, const char* reason)
{
    if (!state.safetyLockLatched) {
        std::cerr << "[SAFETY] EMERGENCY STOP";

        if (reason != nullptr) {
            std::cerr << " | reason: " << reason;
        }

        std::cerr << std::endl;

        (void)queueArduinoCommand("{\"cmd\":\"STOP\"}");
        (void)queueArduinoCommand(
            "{\"cmd\":\"PELTIER\",\"state\":0}"
        );
    }

    state.armOperationAllowed = false;
    state.peltierEnabled = false;
    state.safetyLockLatched = true;
}

/* Process: Ask the Arduino to return to the most recently validated joint state.\n * Recovery is skipped when no valid safe position has been recorded yet. */
static void requestSafePositionRecovery(
    const SafetyControllerState& state,
    const char* failedJoint,
    double failedValue
)
{
    if (!state.hasLastSafePosition) {
        std::cerr << "[RECOVERY] No last-safe position available"
                  << std::endl;
        return;
    }

    std::cerr
        << "[RECOVERY] Joint " << failedJoint
        << " exceeded its limit with value " << failedValue
        << std::endl;

    char command[256]{};

    std::snprintf(
        command,
        sizeof(command),
        "{\"cmd\":\"REVERT_TO_LAST_SAFE\","
        "\"baseAngle\":%.2f,\"shoulderAngle\":%.2f,\"elbowAngle\":%.2f,"
        "\"wristAngle\":%.2f,\"gripperAngle\":%.2f}",
        state.lastSafeJointState.baseAngle,
        state.lastSafeJointState.shoulderAngle,
        state.lastSafeJointState.elbowAngle,
        state.lastSafeJointState.wristAngle,
        state.lastSafeJointState.gripperAngle
    );

    (void)queueArduinoCommand(command);
    (void)queueArduinoCommand(
        "{\"cmd\":\"JOYSTICK_RECOVERY_ENABLE\",\"state\":1}"
    );
}

/* Process: Enable or disable the Peltier output while enforcing arm-safety rules.\n * Every enable operation records its start time for the watchdog timeout. */
static void setPeltierOutput(SafetyControllerState& state, bool enable)
{
    if (enable) {
        if (!state.armOperationAllowed || state.safetyLockLatched) {
            std::cerr << "[SAFETY] Peltier request rejected"
                      << std::endl;
            return;
        }

        (void)queueArduinoCommand(
            "{\"cmd\":\"PELTIER\",\"state\":1}"
        );

        state.peltierEnabled = true;
        state.peltierStartTimestampMs = getMonotonicTimeMilliseconds();
    } else {
        (void)queueArduinoCommand(
            "{\"cmd\":\"PELTIER\",\"state\":0}"
        );

        state.peltierEnabled = false;
    }
}

/* Process: Create a QNX timer that sends a pulse at a fixed safety interval.\n * The coordinator handles each pulse to check feedback freshness and Peltier duration. */
static timer_t createPeriodicSafetyTimer(int timerCoid)
{
    struct sigevent event{};

    SIGEV_PULSE_INIT(
        &event,
        timerCoid,
        SAFETY_THREAD_PRIORITY,
        SAFETY_TIMER_PULSE_CODE,
        0
    );

    timer_t timerId{};

    if (timer_create(CLOCK_MONOTONIC, &event, &timerId) == -1) {
        perror("[TIMER] timer_create");
        return static_cast<timer_t>(-1);
    }

    struct itimerspec specification{};
    specification.it_value.tv_nsec = SAFETY_CHECK_INTERVAL_MS * 1000000L;
    specification.it_interval.tv_nsec = SAFETY_CHECK_INTERVAL_MS * 1000000L;

    if (timer_settime(timerId, 0, &specification, nullptr) == -1) {
        perror("[TIMER] timer_settime");
        timer_delete(timerId);
        return static_cast<timer_t>(-1);
    }

    return timerId;
}

/* Process: Receive queued commands and write them completely to the Arduino UART.\n * This dedicated writer prevents multiple threads from writing to the serial port together. */
static void* arduinoCommandWriterThread(void*)
{
    configureCurrentThreadPriority(ARDUINO_THREAD_PRIORITY);

    std::cout << "[ARDUINO-WRITER] Thread started" << std::endl;

    ArduinoCommand command{};

    while (applicationRunning) {
        const int rcvid = MsgReceive(
            arduinoWriterChannelId,
            &command,
            sizeof(command),
            nullptr
        );

        if (rcvid == -1) {
            if (errno == EINTR) {
                continue;
            }

            break;
        }

        if (rcvid == 0) {
            continue;
        }

        const size_t length = strnlen(
            command.data, sizeof(command.data)
        );

        size_t sent = 0;

        while (sent < length) {
            const ssize_t written = write(
                arduinoSerialFileDescriptor,
                command.data + sent,
                length - sent
            );

            if (written < 0) {
                if (errno == EINTR) {
                    continue;
                }

                perror("[UART] Arduino write");
                break;
            }

            if (written == 0) {
                break;
            }

            sent += static_cast<size_t>(written);
        }

        (void)MsgReply(rcvid, EOK, nullptr, 0);
    }

    return nullptr;
}

/* Process: Read UART bytes and assemble newline-terminated Arduino JSON packets.\n * Complete packets are forwarded to the safety coordinator through QNX message passing. */
static void* arduinoFeedbackReaderThread(void*)
{
    configureCurrentThreadPriority(ARDUINO_THREAD_PRIORITY);

    std::cout << "[ARDUINO-READER] Thread started" << std::endl;

    char line[256]{};
    size_t index = 0;

    while (applicationRunning) {
        char character = '\0';

        const ssize_t received = read(
            arduinoSerialFileDescriptor, &character, 1
        );

        if (received < 0) {
            if (errno == EINTR) {
                continue;
            }

            perror("[UART] Arduino read");
            break;
        }

        if (received == 0) {
            continue;
        }

        if (character == '\n' || character == '\r') {
            if (index == 0) {
                continue;
            }

            line[index] = '\0';

            /*
             * Print the actual packet. No generic
             * "Response received" message is used.
             */
            std::cout << "[ARDUINO] JSON RECEIVED: "
                      << line << std::endl;

            CoordinatorMessage event{};
            event.type = MESSAGE_ARDUINO_FEEDBACK;

            std::strncpy(
                event.line,
                line,
                sizeof(event.line) - 1
            );

            (void)sendCoordinatorMessage(event);
            index = 0;
        } else if (index < sizeof(line) - 1) {
            line[index++] = character;
        } else {
            std::cerr << "[UART] Oversized JSON packet discarded"
                      << std::endl;
            index = 0;
        }
    }

    return nullptr;
}

/* Process: Own the safety state, validate feedback, and react to timer pulses.\n * This coordinator is the central decision point for stop, timeout, and recovery actions. */
static void runSafetyCoordinator()
{
    configureCurrentThreadPriority(SAFETY_THREAD_PRIORITY);

    SafetyControllerState state{};
    state.armOperationAllowed = false;
    state.safetyLockLatched = true;
    state.peltierEnabled = false;
    state.hasLastSafePosition = false;
    state.lastFeedbackTimestampMs = getMonotonicTimeMilliseconds();
    state.peltierStartTimestampMs = 0;

    std::cout << "[SAFETY] Supervisor initialized" << std::endl;
    std::cout << "[SAFETY] Initial state: ARM DISABLED"
              << std::endl;
    std::cout << "[SAFETY] Initial state: SAFETY LATCHED"
              << std::endl;

    (void)queueArduinoCommand("{\"cmd\":\"STOP\"}");
    (void)queueArduinoCommand(
        "{\"cmd\":\"PELTIER\",\"state\":0}"
    );

    const int timerCoid = ConnectAttach(
        0, 0, coordinatorChannelId, _NTO_SIDE_CHANNEL, 0
    );

    if (timerCoid == -1) {
        perror("[IPC] ConnectAttach timer");
        return;
    }

    const timer_t timerId = createPeriodicSafetyTimer(timerCoid);

    if (timerId == static_cast<timer_t>(-1)) {
        ConnectDetach(timerCoid);
        return;
    }

    std::cout << "[TIMER] Safety watchdog started" << std::endl;
    std::cout << "[IPC] QNX message-passing coordinator ready"
              << std::endl;

    char receiveBuffer[sizeof(CoordinatorMessage)]{};

    while (applicationRunning) {
        const int rcvid = MsgReceive(
            coordinatorChannelId,
            receiveBuffer,
            sizeof(receiveBuffer),
            nullptr
        );

        if (rcvid == -1) {
            if (errno == EINTR) {
                continue;
            }

            break;
        }

        if (rcvid == 0) {
            struct _pulse pulse{};
            std::memcpy(&pulse, receiveBuffer, sizeof(pulse));

            if (pulse.code == SAFETY_TIMER_PULSE_CODE) {
                const uint64_t currentTime = getMonotonicTimeMilliseconds();

                if (currentTime - state.lastFeedbackTimestampMs >
                    ARDUINO_FEEDBACK_TIMEOUT_MS) {
                    activateEmergencyStop(
                        state,
                        "Arduino feedback timeout"
                    );
                }

                if (state.peltierEnabled &&
                    currentTime - state.peltierStartTimestampMs >
                    MAX_PELTIER_RUNTIME_MS) {
                    setPeltierOutput(state, false);
                }
            }

            continue;
        }

        CoordinatorMessage event{};
        std::memcpy(&event, receiveBuffer, sizeof(event));

        if (event.type == MESSAGE_ARDUINO_FEEDBACK) {
            RobotJointState currentJoints{};

            if (!parseServoStateFeedback(event.line, currentJoints)) {
                std::cerr << "[SAFETY] Invalid servo_state JSON"
                          << std::endl;
                (void)MsgReply(rcvid, EOK, nullptr, 0);
                continue;
            }

            state.lastFeedbackTimestampMs = getMonotonicTimeMilliseconds();
            printRobotJointState(currentJoints);

            const char* failedJoint = nullptr;
            double failedValue = 0.0;

            if (!validateJointSafetyLimits(
                    currentJoints,
                    failedJoint,
                    failedValue)) {

                std::cerr << "[SAFETY] Joint-limit validation: FAIL"
                          << std::endl;

                activateEmergencyStop(state, "Joint angle outside limit");
                requestSafePositionRecovery(
                    state, failedJoint, failedValue
                );
            } else {
                std::cout
                    << "[SAFETY] Joint-limit validation: PASS"
                    << std::endl;

                /*
                 * Only a valid packet becomes the new recovery
                 * reference. Invalid positions are never stored.
                 */
                state.lastSafeJointState = currentJoints;
                state.hasLastSafePosition = true;

                std::cout
                    << "[SAFETY] Last safe position updated"
                    << std::endl;
            }
        }

        (void)MsgReply(rcvid, EOK, nullptr, 0);
    }

    timer_delete(timerId);
    ConnectDetach(timerCoid);
}

/* Process: Convert SIGINT/SIGTERM into a controlled application shutdown request.\n * The main routine then closes the UART and destroys QNX IPC resources. */
static void handleTerminationSignal(int signalNumber)
{
    if (signalNumber == SIGINT || signalNumber == SIGTERM) {
        applicationRunning = 0;
    }
}

int main()
{
    signal(SIGINT, handleTerminationSignal);
    signal(SIGTERM, handleTerminationSignal);

    std::cout << "============================================"
              << std::endl;
    std::cout << " Q-REHAB QNX MASTER - UART ARCHITECTURE"
              << std::endl;
    std::cout << " TCP SOCKET: DISABLED" << std::endl;
    std::cout << " INPUT: LIVE ARDUINO JSON OVER UART"
              << std::endl;
    std::cout << " IPC: QNX NATIVE MESSAGE PASSING"
              << std::endl;
    std::cout << " MASTER: RASPBERRY PI RUNNING QNX"
              << std::endl;
    std::cout << " SLAVE: ARDUINO UNO ACTUATOR CONTROLLER"
              << std::endl;
    std::cout << "============================================"
              << std::endl;

    arduinoSerialFileDescriptor = open(
        ARDUINO_SERIAL_DEVICE,
        O_RDWR | O_NOCTTY
    );

    if (arduinoSerialFileDescriptor == -1) {
        perror("[UART] open Arduino UART");
        return EXIT_FAILURE;
    }

    if (!configureSerialPort(arduinoSerialFileDescriptor)) {
        close(arduinoSerialFileDescriptor);
        return EXIT_FAILURE;
    }

    std::cout << "[MASTER] Arduino UART initialized"
              << std::endl;

    coordinatorChannelId = ChannelCreate(0);
    if (coordinatorChannelId == -1) {
        perror("[IPC] ChannelCreate coordinator");
        close(arduinoSerialFileDescriptor);
        return EXIT_FAILURE;
    }

    coordinatorConnectionId = ConnectAttach(
        0, 0, coordinatorChannelId, _NTO_SIDE_CHANNEL, 0
    );

    if (coordinatorConnectionId == -1) {
        perror("[IPC] ConnectAttach coordinator");
        ChannelDestroy(coordinatorChannelId);
        close(arduinoSerialFileDescriptor);
        return EXIT_FAILURE;
    }

    arduinoWriterChannelId = ChannelCreate(0);
    if (arduinoWriterChannelId == -1) {
        perror("[IPC] ChannelCreate Arduino");
        ConnectDetach(coordinatorConnectionId);
        ChannelDestroy(coordinatorChannelId);
        close(arduinoSerialFileDescriptor);
        return EXIT_FAILURE;
    }

    arduinoWriterConnectionId = ConnectAttach(
        0, 0, arduinoWriterChannelId, _NTO_SIDE_CHANNEL, 0
    );

    if (arduinoWriterConnectionId == -1) {
        perror("[IPC] ConnectAttach Arduino");
        ChannelDestroy(arduinoWriterChannelId);
        ConnectDetach(coordinatorConnectionId);
        ChannelDestroy(coordinatorChannelId);
        close(arduinoSerialFileDescriptor);
        return EXIT_FAILURE;
    }

    pthread_t writerThread{};
    pthread_t readerThread{};

    if (pthread_create(
            &writerThread,
            nullptr,
            arduinoCommandWriterThread,
            nullptr) != 0) {
        perror("[THREAD] Arduino writer creation failed");
        applicationRunning = 0;
    }

    if (applicationRunning &&
        pthread_create(
            &readerThread,
            nullptr,
            arduinoFeedbackReaderThread,
            nullptr) != 0) {
        perror("[THREAD] Arduino reader creation failed");
        applicationRunning = 0;
    }

    if (applicationRunning) {
        runSafetyCoordinator();
    }

    applicationRunning = 0;

    /*
     * Ask the Arduino to stop before closing the UART.
     * This is a software command and is not a substitute for
     * a physical emergency-stop power circuit.
     */
    if (arduinoWriterConnectionId != -1) {
        (void)queueArduinoCommand("{\"cmd\":\"STOP\"}");
    }

    if (arduinoSerialFileDescriptor != -1) {
        close(arduinoSerialFileDescriptor);
        arduinoSerialFileDescriptor = -1;
    }

    if (arduinoWriterConnectionId != -1) {
        ConnectDetach(arduinoWriterConnectionId);
    }

    if (coordinatorConnectionId != -1) {
        ConnectDetach(coordinatorConnectionId);
    }

    if (arduinoWriterChannelId != -1) {
        ChannelDestroy(arduinoWriterChannelId);
    }

    if (coordinatorChannelId != -1) {
        ChannelDestroy(coordinatorChannelId);
    }

    std::cout << "[MASTER] Q-REHAB MASTER STOPPED"
              << std::endl;

    return EXIT_SUCCESS;
}
