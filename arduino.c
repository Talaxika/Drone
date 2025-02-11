/*******************************************************/ 

  /******************** Include Files ********************/ 

  /*******************************************************/ 

  #include "LIS3DHTR.h"     // 3axis 

  #include <Wire.h>         // for I2C 

  #include <U8x8lib.h>      // monitor 

  #include "Seeed_BMP280.h" // barometer 

 

  /*******************************************************/ 

  /******************* Global Variables ******************/ 

  /*******************************************************/ 

  // accelerometer 

  LIS3DHTR<TwoWire> LIS; // I2C 

  #define WIRE Wire 

 

  // Signalization actuators 

  int ledPin = 4; // for LED 

  int buzzerPin = 5; // for buzzer 

 

  // air pressure 

  BMP280 bmp280; 

  float pressure = 0.0; 

  float altitude = 0.0; 

 

  // Record the total distance travelled 

  float ax = 0.0, ay = 0.0, az = 0.0; // for accelerometer 

  float totalDistanceX = 0.0; 

  float totalDistanceY = 0.0; 

  const float timeStep = 0.1; // Time step in seconds (100 ms) 

 

  // Monitor if the device tilts too much 

  bool isTooTilted = false; // If device tilts too much, signal the user 

  float deltaX = 0.0, deltaY = 0.0; // Counter tilt angles in degrees 

  float maxTiltAngle = 20.0; // In degrees 

 

  // For monitor 

  U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 

 

  // For sensor filtering 

  #define HISTORY_SIZE         7     // Number of samples stored for filtering. 

                                     // The bigger -> the more acurate, but less reaction time 

  static float ax_history[HISTORY_SIZE] = {0.0}; 

  static float ay_history[HISTORY_SIZE] = {0.0}; 

  static float az_history[HISTORY_SIZE] = {0.0}; 

  static int history_idx = 0; 

 

  /*******************************************************/ 

  /******************** Arduino Setup ********************/ 

  /*******************************************************/ 

 void setup() { 

    Serial.begin(9600); 

    while (!Serial) {}; 

 

    // init barometer 

    if (!bmp280.init()) { 

      Serial.println("Device not connected or broken!"); 

    } 

 

    // for signalization 

    pinMode(ledPin, OUTPUT);  

    pinMode(buzzerPin, OUTPUT); 

 

    // for monitor 

    axisSetup(); 

    monitorSetup(); 

  } 

 

  /*******************************************************/ 

  /******************* Setup functions *******************/ 

  /*******************************************************/ 

  void monitorSetup() 

  { 

    u8x8.begin(); 

    u8x8.setPowerSave(0);   

    u8x8.setFlipMode(1); 

    u8x8.setFont(u8x8_font_chroma48medium8_r); 

    u8x8.clear(); 

  } 

 

  /*******************************************************/ 

  void axisSetup() 

  { 

    // Initialize the LIS accelerometer with the 0x19 I2C address 

    LIS.begin(WIRE, 0x19); //IIC init 

    delay(100); 

    LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ); 

  } 

 

  /*******************************************************/ 

  /******************** Arduino Loop *********************/ 

  /*******************************************************/ 

  void loop() { 

    if (!LIS) { 

      Serial.println("LIS3DHTR didn't connect."); 

      while (1); 

      return; 

    } 

 

    // barometer 

    pressure = bmp280.getPressure(); 

    altitude = bmp280.calcAltitude(pressure); 

 

    //3 axis 

    ax = LIS.getAccelerationX(); 

    ay = LIS.getAccelerationY(); 

    az = LIS.getAccelerationZ(); 

 

    medianFilterWithHistory(&ax, &ay, &az); 

    calculateDistance(); 

    verifyAngle(); 

 

    activateMeasures(); 

    writeToMonitor(); 

    printToSerial(); 

 

    delay(100); // Wait for the time step to have correct distance measuered 

  } 

 

  /*******************************************************/ 

  /******************* Loop functions ********************/ 

  /*******************************************************/ 

  // Applies the median filter to the collected accelerometer data. 

  // The goal is to not have random invalid values 

  void medianFilterWithHistory(float *ax, float *ay, float *az) { 

    // Store new values in history 

    ax_history[history_idx] = *ax; 

    ay_history[history_idx] = *ay; 

    az_history[history_idx] = *az; 

 

    // Move index forward (circular buffer) 

    history_idx = (history_idx + 1) % HISTORY_SIZE; 

 

    // Create temporary arrays so the original isn't modified 

    float ax_temp[HISTORY_SIZE], ay_temp[HISTORY_SIZE], az_temp[HISTORY_SIZE]; 

    memcpy(ax_temp, ax_history, sizeof(ax_temp)); 

    memcpy(ay_temp, ay_history, sizeof(ay_temp)); 

    memcpy(az_temp, az_history, sizeof(az_temp)); 

 

    // Sort each temp array 

    qsort(ax_temp, HISTORY_SIZE, sizeof(float), compare_floats); 

    qsort(ay_temp, HISTORY_SIZE, sizeof(float), compare_floats); 

    qsort(az_temp, HISTORY_SIZE, sizeof(float), compare_floats); 

 

    // Median is the middle element in the sorted array 

    int mid = round(HISTORY_SIZE / 2.0); 

    *ax = ax_temp[mid]; 

    *ay = ay_temp[mid]; 

    *az = az_temp[mid]; 

  } 

 

  /******************************************************/ 

  // Comparator for qsort (used in median filtering) 

  int compare_floats(const void *a, const void *b) { 

    float fa = *(const float *)a; 

    float fb = *(const float *)b; 

    if (fa < fb) return -1; 

    if (fa > fb) return 1; 

    return 0; 

  } 

 

  /******************************************************/ 

  // Check if the device is too tilted 

  void verifyAngle() 

  { 

    if (calculateCorrectionValues(ax, ay, az, &deltaX, &deltaY)) { 

      isTooTilted = true; 

    } else { 

      isTooTilted = false; 

    } 

  } 

 

  /******************************************************/ 

 

  void calculateDistance() 

  { 

    // Calculate acceleration along x and y axes 

    float accelerationX = ax; 

    float accelerationY = ay; 

 

    // Update velocities (v = u + at) 

    float velocityX = 0.0; 

    float velocityY = 0.0; 

    velocityX += accelerationX * timeStep; 

    velocityY += accelerationY * timeStep; 

 

    // Calculate distances (d = ut + 0.5 * a * t^2) 

    totalDistanceX += velocityX * timeStep + 0.5 * accelerationX * timeStep * timeStep; 

    totalDistanceY += velocityY * timeStep + 0.5 * accelerationY * timeStep * timeStep; 

  }    

 

  /******************************************************/ 

  // Function to calculate the correction tilt values 

  bool calculateCorrectionValues(float x, float y, float z, float *deltaX, float *deltaY) { 

      // Normalize the input vector (x, y, z) 

      float magnitude = sqrt(x * x + y * y + z * z); 

      x /= magnitude; 

      y /= magnitude; 

      z /= magnitude; 

 

      // Calculate the current tilt angle in degrees (angle from the vertical axis) 

      float currentTiltAngle = acos(z) * (180.0 / PI); // Angle from vertical in degrees 

      // Threshold tilt angle in degrees 

      float maxTiltAngle = 20.0; 

 

      if (currentTiltAngle > maxTiltAngle) { 

          // Calculate the angle we need to correct to bring the tilt to the threshold 

          float correctionAngle = currentTiltAngle - maxTiltAngle; 

 

          // Calculate the proportion of tilt in the x and y axes relative to the current tilt 

          float tiltFactor = correctionAngle / currentTiltAngle; 

 

          // Counter-tilt needed in degrees for each axis 

          *deltaX = atan2(x, z) * (180.0 / PI) * tiltFactor; // Counter-tilt for x-axis in degrees 

          *deltaY = atan2(y, z) * (180.0 / PI) * tiltFactor; // Counter-tilt for y-axis in degrees 

 

          return true; 

      } else { 

          // No correction needed 

          *deltaX = 0.0; 

          *deltaY = 0.0; 

          Serial.println("Tilt is within acceptable range."); 

          return false; 

      } 

  } 

 

  /******************************************************/ 

  // Signals if the device tilts too much 

  void activateMeasures() 

  { 

    if (isTooTilted) { 

      analogWrite(buzzerPin, 5); 

      digitalWrite(ledPin, HIGH); 

      delay(10); 

      digitalWrite(ledPin, LOW); 

    } else { 

      analogWrite(buzzerPin, 0); 

    } 

  } 

 

  /******************************************************/ 

 

  void printDistance() 

  { 

    Serial.print("Total Distance X: "); 

    Serial.print(totalDistanceX); 

    Serial.print(" meters, Total Distance Y: "); 

    Serial.print(totalDistanceY); 

    Serial.println(" meters"); 

  } 

 

  /******************************************************/ 

 

  void printAltitude() 

  { 

    Serial.print("Altitude: "); 

    Serial.print(altitude); 

    Serial.println("m"); 

  } 

 

  /******************************************************/ 

 

  void printTiltCorrection() 

  { 

    Serial.print("Tilt exceeds "); 

    Serial.print(maxTiltAngle); 

    Serial.println(" degrees. Counter-tilt needed:"); 

    Serial.print("DeltaX (degrees): "); 

    Serial.println(deltaX, 2); 

    Serial.print("DeltaY (degrees): "); 

    Serial.println(deltaY, 2); 

  } 

 

  /******************************************************/ 

  void printToSerial() 

  { 

    if (isTooTilted) { 

      printTiltCorrection(); 

    } else { 

      printDistance(); 

      // printAltitude(); 

    } 

  } 

 

  /******************************************************/ 

 

  void writeToMonitor() 

  { 

    if (isTooTilted) { 

      u8x8.clear(); 

      u8x8.setCursor(0,0); 

      u8x8.print("Excessive tilt!"); 

    } else { 

      u8x8.setCursor(0,0); 

      u8x8.print("Distance X:"); 

      u8x8.print(totalDistanceX); 

      u8x8.setCursor(0,50); 

      u8x8.print("Distance Y:"); 

      u8x8.print(totalDistanceY); 

    } 

    u8x8.refreshDisplay(); 

  } 