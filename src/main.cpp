/****************************************************************/
/******************** Include Files ********************/
/****************************************************************/
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <csignal>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <condition_variable>
#include <iomanip>  // std::fixed and std::setprecision
#include "MotorController.h"
#include "DigitalPin.h"
#include "IMUSensor.h"
#include "FlightKinematics.h"
#include "TempHumiditySensor.h"
#include "PowerDistributionBoard.h"
#include "Util.h"
#include "LEDBoard.h"

/****************************************************************/
/***************** Local Function Declarations ******************/
/****************************************************************/

static void initPDB();
static void initThreads();
static void initSenAct();
static void initMotors();
static void signalHandler(int signum);
static void restoreTerminalSettings();
static void setNonBlockingInput();
static void keyInputListener();
static void processKinematics();
static void processSafety();
static void processTempHumi();
static void processPDB();
static void updateSensors();
static void updateIMU();
static void updateTempHumi();
static void motorControl();

static void emergencyLand();

/****************************************************************/
/*********************** Global Variables ***********************/
/****************************************************************/

// Global shared variables
std::atomic<bool> running(true);  // Control flag for threads
std::atomic<bool> emergency(false);  // Control flag for threads
std::atomic<int32_t> keyInput(0); // Store last key press
std::condition_variable keyCondition;
std::thread inputThread;  // Global thread variable
std::thread sensorThread; // Global thread variable
std::thread motorThread;  // Global thread variable
std::mutex sensorMutex;
std::mutex motorMutex;
std::mutex keyMutex;

constexpr int32_t threadTimestep = 100; // 100ms
constexpr float timeStep = (float) threadTimestep / 1000.f; // 0.1s = 100 ms

struct termios originalTermios;  // Store original terminal settings

constexpr int32_t pinTopLeft  = 9;
constexpr int32_t pinTopRight = 10;
constexpr int32_t pinBotLeft  = 11;
constexpr int32_t pinBotRight = 12;

MotorController motorController;

I2C i2cBus;

constexpr uint8_t imuSensorAddress = 0x68;
IMUSensor imu(i2cBus, imuSensorAddress); // Typical IMU I2C address

constexpr uint8_t tempHumiAddress = 0x40;
TempHumiditySensor tempHumi(i2cBus, tempHumiAddress);

constexpr uint8_t ledAddress = 0x50;
LEDBoard led(i2cBus, 0x50); // Example I2C address

FlightKinematics flightKinematics(timeStep, MAX_VELOCITY);

// Create Power Distribution Board with 2200mAh, 11.1V battery + 3.3V power rail for sensors
PowerDistributionBoard pdb(0.1, 11.1, 11.1, 3.3);

std::unordered_map<char, bool> keyPressed = {
    {'w', false}, {'s', false}, {'a', false}, {'d', false},
    {'f', false}, {'r', false}, {'h', false}
};

/****************************************************************/
/*********************** Main Application ***********************/
/****************************************************************/

int main(void) {

    signal(SIGINT, signalHandler);  // Register Ctrl+C handler

    initMotors();
    initPDB();
    initSenAct();
    initThreads();

    motorController.hover();

    while (running) {
        led.setColor(0, 255, 0);
        updateTempHumi();
        processPDB();
        processKinematics();
        processSafety();
        std::this_thread::sleep_for(std::chrono::milliseconds(threadTimestep)); // 100ms delay
        led.setColor(0, 255, 255);
    }

    if (emergency) {
        emergencyLand();
    }

    keyCondition.notify_all();

    if (motorThread.joinable()) {
        motorThread.join();  // Ensure thread joins before exiting
    }
    if (sensorThread.joinable()) {
        sensorThread.join();  // Ensure thread joins before exiting
    }
    if (inputThread.joinable()) {
        inputThread.join();  // Ensure thread joins before exiting
    }

    restoreTerminalSettings();  // Ensure settings are restored before exit

    std::cout << "Program exited cleanly." << std::endl;
    return 0;
}


/****************************************************************/
/****************** Local Function Definitions ******************/
/****************************************************************/

static void initPDB() {
    std::vector<ESC*> escs = motorController.getESCs();

    pdb.addSensor(&imu);
    pdb.addSensor(&tempHumi);
    pdb.addSensor(&led);
    for (auto& esc : escs) pdb.addMotor(esc);

    pdb.distributePower();

    for (auto& esc : escs) esc->enable();
    imu.enable();
    tempHumi.enable();
    led.enable();
}

/****************************************************************/

static void initThreads() {
    inputThread  = std::thread(keyInputListener);
    sensorThread = std::thread(updateSensors);
    motorThread  = std::thread(motorControl);
}

/****************************************************************/

// Sets the command_regs to start measurements
static void initSenAct() {
    i2cBus.write(
        imuSensorAddress,
        IMUSensor::COMMAND_REG,
        {0x01});
    imu.reset();

    i2cBus.write(
        tempHumiAddress,
        TempHumiditySensor::COMMAND_REG,
        {0x01});

    led.turnOn();
    led.setBrightness(128);
    led.setColor(255, 255, 255);
}

/****************************************************************/

static void initMotors() {
    /** Init the motors with the created digital pins. */
    motorController.init();
}

/****************************************************************/

static void signalHandler(int signum) {
    std::cout << "\nCaught signal "
              << signum
              << ", exiting gracefully..."
              << std::endl;

    running = false;  // Stop the infinite loops

    keyCondition.notify_all();  // Wake up all waiting threads

    if (motorThread.joinable()) {
        motorThread.join();  // Ensure thread joins before exiting
    }
    if (sensorThread.joinable()) {
        sensorThread.join();  // Ensure thread joins before exiting
    }
    if (inputThread.joinable()) {
        inputThread.join();  // Ensure thread joins before exiting
    }

    std::exit(signum);  // Exit program safely
}

/****************************************************************/

static void restoreTerminalSettings() {
    tcsetattr(STDIN_FILENO, TCSANOW, &originalTermios);
}

/****************************************************************/

static void setNonBlockingInput() {

    // Save current terminal settings
    tcgetattr(STDIN_FILENO, &originalTermios);
    struct termios newTermios = originalTermios;

    newTermios.c_lflag &= ~ICANON;  // Disable line buffering
    newTermios.c_lflag &= ~ECHO;    // Disable echoing of characters

    tcsetattr(STDIN_FILENO, TCSANOW, &newTermios);  // Apply new settings

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    // Ensure settings are restored when the program exits
    std::atexit(restoreTerminalSettings);
}

/****************************************************************/

static void processKinematics() {
    flightKinematics.updateFlightKinematics(
        imu.getAccelX(),
        imu.getAccelY(),
        imu.getAccelZ(),
        imu.getGyroX(),
        imu.getGyroY(),
        imu.getGyroZ()
    );

    // if (ENABLE_PRINTS) {
        std::vector<float> pos = flightKinematics.getPositions();
        std::cout << std::fixed << std::setprecision(2)
                << "Position X: " << pos.at(0) << ", "
                << "Position Y: " << pos.at(1) << ", "
                << "Position Z: " << pos.at(2)
                << std::endl;

        std::vector<float> vel = flightKinematics.getVelocities();
        std::cout << std::fixed << std::setprecision(2)
                << "Velocity X: " << vel.at(0) << ", "
                << "Velocity Y: " << vel.at(1) << ", "
                << "Velocity Z: " << vel.at(2)
                << std::endl;
    // }
}

/****************************************************************/

static void processSafety() {
    processTempHumi();
}

/****************************************************************/

static void processTempHumi() {
    if (tempHumi.getTemperature() > 70.0) {
        emergency = true;
        running = false;
        std::cout << "Temperature is too high!\n";
        return;
    }

    if (tempHumi.getHumidity() > 95.0) {
        emergency = true;
        running = false;
        std::cout << "It is too humid to safely fly!\n";
        return;
    }
}

/****************************************************************/

static void processPDB() {
    pdb.updateBattery(timeStep);
    pdb.displayStatus();
    if (pdb.isBatteryLow()) {
        std::cout << "Battery low! Emergency landing activated.\n";
        running = false;
        emergency = true;
        return;
    }
}

/****************************************************************/
// Function to update sensor values
static void updateSensors() {
    while (running) {
        updateIMU();
    }
}

/****************************************************************/

static void updateTempHumi() {
    // tempHumi.writeTemperature(60.0);
    // tempHumi.writeHumidity(60.0);
}

/****************************************************************/

static void updateIMU() {
    while (running) {
        {
            std::unique_lock<std::mutex> lock(keyMutex);
            keyCondition.wait(lock, [] { return keyInput != 0 || !running; });
        }

        if (!running) return;

        std::lock_guard<std::mutex> sensorLock(sensorMutex);
        char key = static_cast<char>(keyInput);

        imu.reset(); // Reset IMU before applying new values

        switch (key) {
            case 'w': imu.writeAccelY(+2.0f); imu.writeGyroX(+1.0f); break;
            case 's': imu.writeAccelY(-2.0f); imu.writeGyroX(-1.0f); break;
            case 'a': imu.writeAccelX(-2.0f); imu.writeGyroY(-1.0f); break;
            case 'd': imu.writeAccelX(+2.0f); imu.writeGyroY(+1.0f); break;
            case 'f': imu.writeAccelZ(+11.01f); imu.writeGyroZ(+0.5f); break;
            case 'r': imu.writeAccelZ(+8.01f); imu.writeGyroZ(-0.5f); break;
        }

        keyInput = 0; // Reset key input after processing
    }
}


/****************************************************************/
// Function to detect key inputs
static void keyInputListener() {
    setNonBlockingInput();

    while (running) {
        char key;
        int bytesRead = read(STDIN_FILENO, &key, 1);

        if (bytesRead > 0) {
            std::cout << "Pressed key: " << key << std::endl;

            {
                std::lock_guard<std::mutex> lock(keyMutex);
                if (keyPressed.find(key) != keyPressed.end()) {
                    keyInput = key;
                }
                if (key == 'q') {
                    running = false;
                    keyCondition.notify_all();
                    return;
                }

                // Reset all key states if a different key is pressed
                for (auto& [k, pressed] : keyPressed) {
                    if (k != key) {
                        pressed = false;
                    }
                }
            }

            keyCondition.notify_all();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(threadTimestep / 10));
    }
}

/****************************************************************/
// Function to control motors
static void motorControl() {
    while (running) {
        {
            std::unique_lock<std::mutex> lock(keyMutex);
            keyCondition.wait(lock, [] { return keyInput != 0 || !running; });
        }

        if (!running) return;

        std::lock_guard<std::mutex> motorLock(motorMutex);
        char key = static_cast<char>(keyInput);

        switch (key) {
            case 'w': motorController.moveForward(); break;
            case 's': motorController.moveBackward(); break;
            case 'a': motorController.moveLeft(); break;
            case 'd': motorController.moveRight(); break;
            case 'f': motorController.moveUp(); break;
            case 'r': motorController.moveDown(); break;
            case 'h': motorController.hover(); break;
        }

        motorController.displayState();

        keyInput = 0; // Reset key input after processing
    }
}

static void emergencyLand() {
    std::cout << "[EMERGENCY] Activating Emergency Landing..." << std::endl;

    // Override key input to prevent manual control
    {
        std::lock_guard<std::mutex> lock(keyMutex);
        keyInput = 0;
        keyCondition.notify_all();  // Wake all waiting threads
    }

    // Give time for other threads to complete their tasks
    std::this_thread::sleep_for(std::chrono::milliseconds(2 * threadTimestep));

    // Gradually descend until near ground level
    while (flightKinematics.getPositions()[2] > 0.1) {
        led.setColor(255, 0, 0);
        {
            std::lock_guard<std::mutex> motorLock(motorMutex);
            motorController.moveDown();  // Reduce throttle gradually
        }

        {
            std::lock_guard<std::mutex> sensorLock(sensorMutex);
            imu.reset();  // Reset IMU for consistent readings
            imu.writeAccelZ(8.1);  // Simulate controlled descent
            imu.writeGyroZ(0.0f);  // No rotation needed
        }

        // Let kinematics update the new position
        std::this_thread::sleep_for(std::chrono::milliseconds(threadTimestep));
        processPDB();
        processKinematics(); // Force kinematics update

        // Print debug info
        std::vector<float> pos = flightKinematics.getPositions();
        std::cout << "[EMERGENCY] Descending... Altitude: " << pos[2] << "m" << std::endl;
        pdb.displayStatus();
        if (pdb.isDepleted()) {
            std::terminate(); // good for multithreaded programs, instead of exit()
        }
        
        led.setColor(0, 0, 0);
    }

    // Ensure motors are completely stopped after landing
    {
        std::lock_guard<std::mutex> motorLock(motorMutex);
        motorController.turnOff();
    }

    std::cout << "[EMERGENCY] Landing Complete. Motors shut down." << std::endl;
}
