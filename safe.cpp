static void updateIMU() {
    {
        std::unique_lock<std::mutex> lock(keyMutex);
        keyCondition.wait(lock, [] { return keyInput != 0 || !running; });
    }

    if (!running) return;

    std::lock_guard<std::mutex> sensorLock(sensorMutex);
    char key = static_cast<char>(keyInput);

    if (keyPressed.find(key) != keyPressed.end() && !keyPressed[key]) {

        // Reset sensor values before applying new ones
        imu.reset();

        switch (motorController.getState()) {
            case FlightState::IDLE:
                break;
            case FlightState::HOVERING:
                break;
            case FlightState::MOVING_FWD:
                imu.writeAccelY(+2.0f);  // Move forward
                imu.writeGyroX(+1.0f); // Simulate forward pitch rotation
                break;
            case FlightState::MOVING_BWD:
                imu.writeAccelY(-2.0f); // Move backward
                imu.writeGyroX(-1.0f); // Simulate backward pitch rotation
                break;
            case FlightState::MOVING_LEFT:
                imu.writeAccelX(-2.0f); // Move left
                imu.writeGyroY(-1.0f); // Simulate left yaw rotation
                break;
            case FlightState::MOVING_RIGHT:
                imu.writeAccelX(+2.0f); // Move right
                imu.writeGyroY(+1.0f); // Simulate right yaw rotation
                break;
            case FlightState::ASCENDING:
                imu.writeAccelZ(+11.01f); // Move up
                imu.writeGyroZ(+0.5f); // Simulate upward roll
                break;
            case FlightState::DESCENDING:
                imu.writeAccelZ(+8.01f); // Move down
                imu.writeGyroZ(-0.5f); // Simulate downward roll
                break;
        }

        keyPressed[key] = true; // Mark key as pressed
    }

    keyInput = 0; // Reset key input
}