/*
A high speed balancing robot, running on an ESP32.

Wouter Klop
wouter@elexperiment.nl
For updates, see elexperiment.nl

Use at your own risk. This code is far from stable.

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0
International License. To view a copy of this license, visit
http://creativecommons.org/licenses/by-sa/4.0/ This basically means: if you use
my code, acknowledge it. Also, you have to publish all modifications.

*/

#include <Arduino.h>
#include <MPU6050.h>
#include <PID.h>
#include <Streaming.h>
#include <WiFi.h>
#include <esp_now.h>
#include <fastStepper.h>
// #include <par.h>
#include <Preferences.h>  // for storing settings
#include "driver/adc.h"
#include "esp_adc_cal.h"

// ----- Input method

// Driving behaviour
float speedFactor = 0.7;  // how strong it reacts to inputs, lower = softer
                          // (limits max speed) (between 0 and 1)
float steerFactor = 1.0;  // how strong it reacts to inputs, lower = softer
                          // (limits max speed) (between 0 and 1)
float speedFilterConstant = 0.9;  // how fast it reacts to inputs, higher =
                                  // softer (between 0 and 1, but not 0 or 1)
float steerFilterConstant = 0.9;  // how fast it reacts to inputs, higher =
                                  // softer (between 0 and 1, but not 0 or 1)

// PPM (called CPPM, PPM-SUM) signal containing 8 RC-Channels in 1 PIN ("RX" on
// board) Channel 1 = steer, Channel 2 = speed #define INPUT_PPM #define PPM_PIN
// 16  // GPIO-Number #define minPPM 990  // minimum PPM-Value (Stick down)
// #define maxPPM 2015  // maximum PPM-Value (Stick up)

// FlySkyIBus signal containing 8 RC-Channels in 1 PIN ("RX" on board)
// #define INPUT_IBUS

// #define INPUT_PS3  // PS3 controller via bluetooth. Dependencies take up
// quite some program space!

// #define STEPPER_DRIVER_A4988 // Use A4988 stepper driver, which uses
// different microstepping settings

// Joystick struct with packed alignment
struct joystick {
  uint16_t x;  // Normalized to [0.0, 1.0]
  uint16_t y;  // Normalized to [0.0, 1.0]
  uint8_t button;
} __attribute__((packed)) __attribute__((aligned(4)));

struct message {
  joystick right;
  joystick left;
} __attribute__((packed)) __attribute__((aligned(4)));

message receivedData;

// ----- Type definitions
typedef union {
  struct {
    float val;  // Float (4 bytes) comes first, as otherwise padding will be
                // applied
    uint8_t cmd;
    uint8_t checksum;
  };
  uint8_t array[6];
} command;

typedef union {
  uint8_t arr[6];
  struct {
    uint8_t grp;
    uint8_t cmd;
    union {
      float val;
      uint8_t valU8[4];
    };
  } __attribute__((packed));
} cmd;

// Plot settings
struct {
  boolean enable = 0;  // Enable sending data
  uint8_t prescaler = 4;
} plot;

/* Remote control structure
Every remote should give a speed and steer command from -100 ... 100
To adjust "driving experience", e.g. a slow beginners mode, or a fast expert
mode, a gain can be adjusted for the speed and steer inputs. Additionaly, a
selfRight input can be used. When setting this bit to 1, the robot will enable
control in an attempt to self right. The override input can be used to control
the robot when it is lying flat. The robot will switch automatically from
override to balancing mode, if it happens to right itself. The disable control
input can be used to 1) disable the balancing mode 2) disable the self-right
attempt 3) disable the override mode Depending on which state the robot is in.
*/
struct {
  float speed = 0;
  float steer = 0;
  float speedGain = 0.15;
  float steerGain = 0.25;
  float speedOffset = 0.0;
  bool selfRight = 0;
  bool disableControl = 0;
  bool override = 0;
} remoteControl;

#define FORMAT_SPIFFS_IF_FAILED true

// ----- Function prototypes
void sendWifiList(void);
void parseSerial();
void parseCommand(char* data, uint8_t length);
void calculateGyroOffset(uint8_t nSample);
void readSensor();
void initSensor(uint8_t n);
void setMicroStep(uint8_t uStep);
void sendConfigurationData(uint8_t num);
#ifdef INPUT_PS3
void onPs3Notify();
void onPs3Connect();
void onPs3Disconnect();
#endif

void IRAM_ATTR motLeftTimerFunction();
void IRAM_ATTR motRightTimerFunction();

// ----- Definitions and variables

// -- EEPROM
Preferences preferences;

// -- Stepper motors
#define motEnablePin 13
#define motUStepPin1 16
#define motUStepPin2 17
#define motUStepPin3 18

fastStepper motLeft(26, 25, 0, motLeftTimerFunction, true);
fastStepper motRight(14, 27, 1, motRightTimerFunction);

uint8_t microStep = 16;
uint8_t motorCurrent = 150;
float maxStepSpeed = 1500;

// -- PID control
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS / 1000000.0

#define PID_ANGLE 0
#define PID_POS 1
#define PID_SPEED 2

#define PID_ANGLE_MAX 12
PID pidAngle(cPID, dT, PID_ANGLE_MAX, -PID_ANGLE_MAX);
#define PID_POS_MAX 35
PID pidPos(cPD, dT, PID_POS_MAX, -PID_POS_MAX);
PID pidSpeed(cP, dT, PID_POS_MAX, -PID_POS_MAX);

uint8_t controlMode = 1;  // 0 = only angle, 1 = angle+position, 2 = angle+speed

// Threshold for fall detection. If integral of error of angle controller is
// larger than this value, controller is disabled
#define angleErrorIntegralThreshold 30.0
#define angleErrorIntegralThresholdDuringSelfright \
  angleErrorIntegralThreshold * 3
#define angleEnableThreshold \
  5.0  // If (absolute) robot angle is below this threshold, enable control
#define angleDisableThreshold \
  70.0  // If (absolute) robot angle is above this threshold, disable control
        // (robot has fallen down)

// -- IMU
MPU6050 imu;

#define GYRO_SENSITIVITY 65.5

int16_t gyroOffset[3];
float accAngle = 0;
float filterAngle = 0;
float angleOffset = 2.0;
float gyroFilterConstant = 0.996;
float gyroGain = 1.0;

// Temporary values for debugging sensor algorithm
float rxg, ayg, azg;

// -- Others
#define PIN_LED 32
#define PIN_MOTOR_CURRENT 23
#define PIN_LED_LEFT 12
#define PIN_LED_RIGHT 12

// ADC definitions (for reading battery voltage)
#define ADC_CHANNEL_BATTERY_VOLTAGE ADC1_CHANNEL_6  // GPIO number 34
// Battery voltage is measured via a 100 and 3.3 kOhm resistor divider.
// Reference voltage is 1.1 V (if attenuation is set to 0dB)
#define BATTERY_VOLTAGE_SCALING_FACTOR (100 + 3.3) / 3.3
#define BATTERY_VOLTAGE_FILTER_COEFFICIENT 0.99
esp_adc_cal_characteristics_t adc_chars;

// -- WiFi
// const char host[] = "balancingrobot";
#define ROBOT_NAME_DEFAULT "balancingrobot"
char robotName[63] = ROBOT_NAME_DEFAULT;

// BT MAC
char BTaddress[20] = "00:00:00:00:00:00";

// Noise source (for system identification)
boolean noiseSourceEnable = 0;
float noiseSourceAmplitude = 1;

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  static boolean speedGain = 0;
  static boolean steerGain = 0;

  memcpy(&receivedData, data, sizeof(receivedData));

  speedGain = receivedData.left.button ? !speedGain : speedGain;
  steerGain = receivedData.right.button ? !steerGain : steerGain;

  if (speedGain) {
    remoteControl.speedGain = 0.4;
  } else {
    remoteControl.speedGain = 0.2;
  }

  if (steerGain) {
    remoteControl.steerGain = 1.2;
  } else {
    remoteControl.steerGain = 0.5;
  }

  remoteControl.speed =
      map(receivedData.left.x, 0, 4095, -100, 100) * remoteControl.speedGain;
  remoteControl.steer =
      -map(receivedData.right.y, 0, 4095, -100, 100) * remoteControl.steerGain;
}

// ----- Interrupt functions -----
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR motLeftTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motLeft.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}
void IRAM_ATTR motRightTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motRight.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ----- Main code
void setup() {
  Serial.begin(115200);
#ifdef INPUT_IBUS
  IBus.begin(Serial2);
#endif
  preferences.begin("settings", false);  // false = RW-mode
  // preferences.clear();  // Remove all preferences under the opened namespace

  pinMode(motEnablePin, OUTPUT);
  pinMode(motUStepPin1, OUTPUT);
  pinMode(motUStepPin2, OUTPUT);
  pinMode(motUStepPin3, OUTPUT);
  digitalWrite(motEnablePin, 1);  // Disable steppers during startup
  setMicroStep(microStep);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LED_LEFT, OUTPUT);
  pinMode(PIN_LED_RIGHT, OUTPUT);
  digitalWrite(PIN_LED, 0);
  digitalWrite(PIN_LED_LEFT, 1);  // Turn on one LED to indicate we are live
  digitalWrite(PIN_LED_RIGHT, 0);

  motLeft.init();
  motRight.init();
  motLeft.microStep = microStep;
  motRight.microStep = microStep;

  // Gyro setup
  delay(200);
  Wire.begin(21, 22, 400000UL);
  delay(100);
  Serial.println(imu.testConnection());
  imu.initialize();
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  // Calculate and store gyro offsets
  delay(50);

// Init EEPROM, if not done before
#define PREF_VERSION \
  1  // if setting structure has been changed, count this number up to delete
     // all settings
  if (preferences.getUInt("pref_version", 0) != PREF_VERSION) {
    preferences.clear();  // Remove all preferences under the opened namespace
    preferences.putUInt("pref_version", PREF_VERSION);
    Serial
        << "EEPROM init complete, all preferences deleted, new pref_version: "
        << PREF_VERSION << "\n";
  }

  // Read gyro offsets
  Serial << "Gyro calibration values: ";
  for (uint8_t i = 0; i < 3; i++) {
    char buf[16];
    sprintf(buf, "gyro_offset_%u", i);
    gyroOffset[i] = preferences.getShort(buf, 0);
    Serial << gyroOffset[i] << "\t";
  }
  Serial << endl;

  // Read angle offset
  angleOffset = preferences.getFloat("angle_offset", 0.0);

  // Perform initial gyro measurements
  initSensor(50);

  // Read robot name
  uint32_t len = preferences.getBytes("robot_name", robotName, 63);
  // strcpy(robotName, ROBOT_NAME_DEFAULT);
  // preferences.putBytes("robot_name", robotName, 63);

  // if (len==0) preferences.putBytes("robot_name", host, 63);
  Serial.println(robotName);

  pidAngle.setParameters(0.65, 1.0, 0.075, 15);
  pidPos.setParameters(1, 0, 1.2, 50);
  pidSpeed.setParameters(6, 5, 0, 20);

  // Setup WiFi
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));

// Setup PS3 controller
#ifdef INPUT_PS3
  // Ps3.begin("24:0a:c4:31:3d:86");
  Ps3.attach(onPs3Notify);
  Ps3.attachOnConnect(onPs3Connect);
  Ps3.attachOnDisconnect(onPs3Disconnect);
  Ps3.begin();
  String address = Ps3.getAddress();
  int bt_len = address.length() + 1;
  address.toCharArray(BTaddress, bt_len);
  Serial.print("Bluetooth MAC address: ");
  Serial.println(address);
#endif

  Serial.println("Ready");

  Serial.println("Booted, ready for driving!");

  digitalWrite(PIN_LED_RIGHT, 1);
}

float mapfloat(float x,
               float in_min,
               float in_max,
               float out_min,
               float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  static unsigned long tLast = 0;
  float pidAngleOutput = 0;
  float avgMotSpeed;
  float steer = 0;
  static float avgSteer;
  static float avgSpeed;
  static boolean enableControl = 0;
  static float avgMotSpeedSum = 0;
  int32_t avgMotStep;
  float pidPosOutput = 0, pidSpeedOutput = 0;
  static uint32_t lastInputTime = 0;
  uint32_t tNowMs;
  float absSpeed = 0;
  float noiseValue = 0;
  static boolean overrideMode = 0;
  static boolean selfRight = 0;
  static boolean disableControl = 0;
  static float angleErrorIntegral = 0;

  unsigned long tNow = micros();
  tNowMs = millis();

  if (tNow - tLast > dT_MICROSECONDS) {
    readSensor();  // Read receiver inputs

    if (remoteControl.selfRight &&
        !enableControl) {  // Start self-right action (stops when robot is
                           // upright)
      selfRight = 1;
      disableControl = 0;
      remoteControl.selfRight = 0;  // Reset single action bool
    } else if (remoteControl.disableControl &&
               enableControl) {  // Sort of kill-switch
      disableControl = 1;
      selfRight = 0;
      remoteControl.disableControl = 0;
    }

    // Filter speed and steer input
    avgSpeed = speedFilterConstant * avgSpeed +
               (1 - speedFilterConstant) * remoteControl.speed / 5.0;
    avgSteer = steerFilterConstant * avgSteer +
               (1 - steerFilterConstant) * remoteControl.steer;

    if (enableControl) {
      // Read receiver inputs

      // uint8_t lastControlMode = controlMode;
      // controlMode = (2000-IBus.readChannel(5))/450;

      if (abs(avgSpeed) < 0.2) {
        // remoteControl.speed = 0;
      } else {
        lastInputTime = tNowMs;
        if (controlMode == 1) {
          controlMode = 2;
          motLeft.setStep(0);
          motRight.setStep(0);
          pidSpeed.reset();
        }
      }

      steer = avgSteer;

      // Switch to position control if no input is received for a certain amount
      // of time
      if (tNowMs - lastInputTime > 2000 && controlMode == 2) {
        controlMode = 1;
        motLeft.setStep(0);
        motRight.setStep(0);
        pidPos.reset();
      }

      // Actual controller computations
      if (controlMode == 0) {
        pidAngle.setpoint = avgSpeed * 2;
      } else if (controlMode == 1) {
        avgMotStep = (motLeft.getStep() + motRight.getStep()) / 2;
        pidPos.setpoint = avgSpeed;
        pidPos.input = -((float)avgMotStep) / 1000.0;
        pidPosOutput = pidPos.calculate();
        pidAngle.setpoint = pidPosOutput;
      } else if (controlMode == 2) {
        pidSpeed.setpoint = avgSpeed;
        pidSpeed.input = -avgMotSpeedSum / 100.0;
        pidSpeedOutput = pidSpeed.calculate();
        pidAngle.setpoint = pidSpeedOutput;
      }

      pidAngle.input = filterAngle;

      pidAngleOutput = pidAngle.calculate();

      // Optionally, add some noise to angle for system identification purposes
      if (noiseSourceEnable) {
        noiseValue = noiseSourceAmplitude * ((random(1000) / 1000.0) - 0.5);
        pidAngleOutput += noiseValue;
      }

      avgMotSpeedSum += pidAngleOutput / 2;
      if (avgMotSpeedSum > maxStepSpeed) {
        avgMotSpeedSum = maxStepSpeed;
      } else if (avgMotSpeedSum < -maxStepSpeed) {
        avgMotSpeedSum = -maxStepSpeed;
      }
      avgMotSpeed = avgMotSpeedSum;
      motLeft.speed = avgMotSpeed + steer;
      motRight.speed = avgMotSpeed - steer;

      // Detect if robot has fallen. Concept: integrate angle controller error
      // over time. If absolute integrated error surpasses threshold, disable
      // controller
      angleErrorIntegral += (pidAngle.setpoint - pidAngle.input) * dT;
      if (selfRight) {
        if (abs(angleErrorIntegral) >
            angleErrorIntegralThresholdDuringSelfright) {
          selfRight = 0;
          disableControl = 1;
        }
      } else {
        if (abs(angleErrorIntegral) > angleErrorIntegralThreshold) {
          disableControl = 1;
        }
      }

      // Switch microstepping
      absSpeed = abs(avgMotSpeed);
      uint8_t lastMicroStep = microStep;

      if (absSpeed > (150 * 32 / microStep) && microStep > 1)
        microStep /= 2;
      if (absSpeed < (130 * 32 / microStep) && microStep < 32)
        microStep *= 2;

      if (microStep != lastMicroStep) {
        motLeft.microStep = microStep;
        motRight.microStep = microStep;
        setMicroStep(microStep);
      }

      // Disable control if robot is almost horizontal. Re-enable if upright.
      if ((abs(filterAngle) > angleDisableThreshold && !selfRight) ||
          disableControl) {
        enableControl = 0;
        overrideMode = 0;
        // disableControl = 0; // Reset disableControl flag
        motLeft.speed = 0;
        motRight.speed = 0;
        digitalWrite(motEnablePin, 1);  // Inverted action on enable pin
        digitalWrite(PIN_LED_LEFT, 0);
        digitalWrite(PIN_LED_RIGHT, 0);
      }
      if (abs(filterAngle) < angleEnableThreshold && selfRight) {
        selfRight = 0;
        angleErrorIntegral = 0;  // Reset, otherwise the fall detection will be
                                 // triggered immediately
      }
    } else {  // Control not active

      // Override control
      if (remoteControl.override) {  // Transition from disable to enable
        remoteControl.override = 0;
        // Enable override mode
        motLeft.speed = 0;
        motRight.speed = 0;
        digitalWrite(motEnablePin, 0);  // Enable motors
        overrideMode = 1;
      }

      if (remoteControl.disableControl) {
        remoteControl.disableControl = 0;
        digitalWrite(motEnablePin, 1);  // Disable motors
        overrideMode = 0;
      }

      if (abs(filterAngle) >
          angleEnableThreshold +
              5) {  // Only reset disableControl flag if angle is out of
                    // "enable" zone, otherwise robot will keep cycling between
                    // enable and disable states
        disableControl = 0;
      }

      if ((abs(filterAngle) < angleEnableThreshold || selfRight) &&
          !disableControl) {  // (re-)enable and reset stuff
        enableControl = 1;
        digitalWrite(PIN_LED_LEFT, 1);
        digitalWrite(PIN_LED_RIGHT, 1);

        controlMode = 1;
        // avgMotSpeedSum = 0;

        if (!overrideMode) {
          avgMotSpeedSum = 0;
          digitalWrite(motEnablePin, 0);  // Inverted action on enable pin
          pidAngle.reset();
        } else {
          avgMotSpeedSum = (motLeft.speed + motRight.speed) / 2;
          overrideMode = 0;
        }

        motLeft.setStep(0);
        motRight.setStep(0);
        pidPos.reset();
        pidSpeed.reset();

        angleErrorIntegral = 0;
        // delay(1);
      }

      if (overrideMode) {
        float spd = avgSpeed;
        float str = avgSteer;
        // if (spd<3) spd = 0;
        // if (str<3) str = 0;
        motLeft.speed = -30 * spd + 2 * str;
        motRight.speed = -30 * spd - 2 * str;

        // Run angle PID controller in background, such that it matches when
        // controller takes over, if needed
        pidAngle.input = filterAngle;
        pidAngleOutput = pidAngle.calculate();
        // pidSpeed.setpoint = avgSpeed;
        // pidSpeed.input = -(motLeft.speed+motRight.speed)/2/100.0;
        // pidSpeedOutput = pidSpeed.calculate();
      }
    }

    motLeft.update();
    motRight.update();

    tLast = tNow;

// Handle PS3 controller
#ifdef INPUT_PS3
    if (Ps3.isConnected()) {
      // PS3 input range is -127 ... 127
      remoteControl.speed =
          -1 * Ps3.data.analog.stick.ly / 1.27 * remoteControl.speedGain +
          remoteControl.speedOffset;
      remoteControl.steer =
          Ps3.data.analog.stick.rx / 1.27 * remoteControl.steerGain;
      // Other PS3 inputs are read in a separate interrupt function
    }
#endif

    // Serial << micros()-tNow << endl;
  }

  // delay(1);
}

void parseSerial() {
  static char serialBuf[63];
  static uint8_t pos = 0;
  char currentChar;

  while (Serial.available()) {
    currentChar = Serial.read();
    serialBuf[pos++] = currentChar;
    if (currentChar == 'x') {
      parseCommand(serialBuf, pos);
      pos = 0;
      while (Serial.available())
        Serial.read();
      memset(serialBuf, 0, sizeof(serialBuf));
    }
  }
}

void parseCommand(char* data, uint8_t length) {
  float val2;
  if ((data[length - 1] == 'x') && length >= 3) {
    switch (data[0]) {
      case 'c': {  // Change controller parameter
        uint8_t controllerNumber = data[1] - '0';
        char cmd2 = data[2];
        float val = atof(data + 3);

        // Make pointer to PID controller
        PID* pidTemp;
        switch (controllerNumber) {
          case 1:
            pidTemp = &pidAngle;
            break;
          case 2:
            pidTemp = &pidPos;
            break;
          case 3:
            pidTemp = &pidSpeed;
            break;
        }

        switch (cmd2) {
          case 'p':
            pidTemp->K = val;
            break;
          case 'i':
            pidTemp->Ti = val;
            break;
          case 'd':
            pidTemp->Td = val;
            break;
          case 'n':
            pidTemp->N = val;
            break;
          case 't':
            pidTemp->controllerType = (uint8_t)val;
            break;
          case 'm':
            pidTemp->maxOutput = val;
            break;
          case 'o':
            pidTemp->minOutput = -val;
            break;
        }
        pidTemp->updateParameters();

        Serial << controllerNumber << "\t" << pidTemp->K << "\t" << pidTemp->Ti
               << "\t" << pidTemp->Td << "\t" << pidTemp->N << "\t"
               << pidTemp->controllerType << endl;
        break;
      }
      case 'a':  // Change angle offset
        angleOffset = atof(data + 1);
        Serial << angleOffset << endl;
        break;
      case 'f':
        gyroFilterConstant = atof(data + 1);
        Serial << gyroFilterConstant << endl;
        break;
      case 'v':
        motorCurrent = atof(data + 1);
        Serial << motorCurrent << endl;
        dacWrite(PIN_MOTOR_CURRENT, motorCurrent);
        break;
      case 'm':
        val2 = atof(data + 1);
        Serial << val2 << endl;
        controlMode = val2;
        break;
      case 'u':
        microStep = atoi(data + 1);
        setMicroStep(microStep);
        break;
      case 'g':
        gyroGain = atof(data + 1);
        break;
      case 'p': {
        switch (data[1]) {
          case 'e':
            plot.enable = atoi(data + 2);
            break;
          case 'p':
            plot.prescaler = atoi(data + 2);
            break;
          case 'n':  // Noise source enable
            noiseSourceEnable = atoi(data + 2);
            break;
          case 'a':  // Noise source amplitude
            noiseSourceAmplitude = atof(data + 2);
            break;
        }
        break;
      }
      // case 'h':
      //   plot.enable = atoi(data+1);
      //   break;
      // case 'i':
      //   plot.prescaler = atoi(data+1);
      //   break;
      case 'j':
        gyroGain = atof(data + 1);
        break;
      case 'k': {
        uint8_t cmd2 = atoi(data + 1);
        if (cmd2 == 1) {  // calibrate gyro
          calculateGyroOffset(100);
        } else if (cmd2 == 2) {  // calibrate acc
          Serial << "Updating angle offset from " << angleOffset;
          angleOffset = filterAngle;
          Serial << " to " << angleOffset << endl;
          preferences.putFloat("angle_offset", angleOffset);
        }
        break;
      }
      case 'l':
        maxStepSpeed = atof(&data[1]);
        break;
      case 'n':
        gyroFilterConstant = atof(&data[1]);
        break;
      case 'w': {
        char cmd2 = data[1];
        char buf[63];
        uint8_t len;

        switch (cmd2) {
          case 'r':
            Serial.println("Rebooting...");
            ESP.restart();
            // pidParList.sendList(&wsServer);
            break;
          case 'l':  // Send wifi networks to WS client
            sendWifiList();
            break;
          case 's':  // Update WiFi SSID
            len = length - 3;
            memcpy(buf, &data[2], len);
            buf[len] = 0;
            preferences.putBytes("wifi_ssid", buf, 63);
            Serial << "Updated WiFi SSID to: " << buf << endl;
            break;
          case 'k':  // Update WiFi key
            len = length - 3;
            memcpy(buf, &data[2], len);
            buf[len] = 0;
            preferences.putBytes("wifi_key", buf, 63);
            Serial << "Updated WiFi key to: " << buf << endl;
            break;
          case 'm':  // WiFi mode (0=AP, 1=use SSID)
            preferences.putUInt("wifi_mode", atoi(&data[2]));
            Serial
                << "Updated WiFi mode to (0=access point, 1=connect to SSID): "
                << atoi(&data[2]) << endl;
            break;
          case 'n':  // Robot name
            len = length - 3;
            memcpy(buf, &data[2], len);
            buf[len] = 0;
            if (len >= 8) {
              preferences.putBytes("robot_name", buf, 63);
            }
            Serial << "Updated robot name to: " << buf << endl;
            break;
        }
        break;
      }
    }
  }
}

void calculateGyroOffset(uint8_t nSample) {
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  int16_t x, y, z;

  for (uint8_t i = 0; i < nSample; i++) {
    imu.getRotation(&x, &y, &z);
    sumX += x;
    sumY += y;
    sumZ += z;
    delay(5);
  }

  gyroOffset[0] = sumX / nSample;
  gyroOffset[1] = sumY / nSample;
  gyroOffset[2] = sumZ / nSample;

  for (uint8_t i = 0; i < 3; i++) {
    char buf[16];
    sprintf(buf, "gyro_offset_%u", i);
    preferences.putShort(buf, gyroOffset[i]);
  }

  Serial << "New gyro calibration values: " << gyroOffset[0] << "\t"
         << gyroOffset[1] << "\t" << gyroOffset[2] << endl;
}

void readSensor() {
  int16_t ax, ay, az, gx, gy, gz;
  float deltaGyroAngle;

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // accAngle = atan2f((float) ax, (float) az) * 180.0/M_PI;
  // deltaGyroAngle = -((float)((gy - gyroOffset[1])) / GYRO_SENSITIVITY) * dT *
  // gyroGain;
  accAngle = atan2f((float)ay, (float)az) * 180.0 / M_PI - angleOffset;
  deltaGyroAngle =
      ((float)((gx - gyroOffset[0])) / GYRO_SENSITIVITY) * dT * gyroGain;

  filterAngle = gyroFilterConstant * (filterAngle + deltaGyroAngle) +
                (1 - gyroFilterConstant) * (accAngle);

  // Serial << ay/1000.0 << "\t" << az/1000.0 << "\t" << accAngle << "\t" <<
  // filterAngle << endl;
  ayg = (ay * 9.81) / 16384.0;
  azg = (az * 9.81) / 16384.0;
  rxg = ((float)((gx - gyroOffset[0])) / GYRO_SENSITIVITY);
  // Serial << ayf << "\t"<< azf << "\t" << accAngle << endl;
}

void initSensor(uint8_t n) {
  float gyroFilterConstantBackup = gyroFilterConstant;
  gyroFilterConstant = 0.8;
  for (uint8_t i = 0; i < n; i++) {
    readSensor();
  }
  gyroFilterConstant = gyroFilterConstantBackup;
}

void setMicroStep(uint8_t uStep) {
  // input:                     1 2 4 8 16 32
  // uStep table corresponds to 0 1 2 3 4  5  in binary on uStep pins
  // So, we need to take the log2 of input
  uint8_t uStepPow = 0;
  uint8_t uStepCopy = uStep;
  while (uStepCopy >>= 1)
    uStepPow++;

  digitalWrite(motUStepPin1, uStepPow & 0x01);
  digitalWrite(motUStepPin2, uStepPow & 0x02);
  digitalWrite(motUStepPin3, uStepPow & 0x04);

#ifdef STEPPER_DRIVER_A4988  // The lookup table for uStepping of the 4988
                             // writes for some reason all three pins high for
                             // 1/16th step
  if (uStep == 16) {
    digitalWrite(motUStepPin1, 1);
    digitalWrite(motUStepPin2, 1);
    digitalWrite(motUStepPin3, 1);
  }
#endif
}

#ifdef INPUT_PS3
void onPs3Notify() {
  if (Ps3.event.button_down.down) {
    remoteControl.speedGain = 0.05;
    remoteControl.steerGain = 0.2;
  }
  if (Ps3.event.button_down.left) {
    remoteControl.speedGain = 0.2;
    remoteControl.steerGain = 0.4;
  }
  if (Ps3.event.button_down.up) {
    remoteControl.speedGain = 0.3;
    remoteControl.steerGain = 0.6;
  }
  if (Ps3.event.button_down.right) {
    remoteControl.speedGain = 0.7;
    remoteControl.steerGain = 0.8;
  }
  if (Ps3.event.button_down.circle)
    remoteControl.selfRight = 1;
  if (Ps3.event.button_down.cross)
    remoteControl.disableControl = 1;
  if (Ps3.event.button_down.square)
    remoteControl.override = 1;
  if (Ps3.event.button_down.r1) {
    if (remoteControl.speedOffset < 20.0) {
      remoteControl.speedOffset += 0.5;
    }
  }
  if (Ps3.event.button_down.r2) {
    if (remoteControl.speedOffset > -20.0) {
      remoteControl.speedOffset -= 0.5;
    }
  }
}

void onPs3Connect() {
  digitalWrite(PIN_LED, 1);
  Serial.println("Bluetooth controller connected");
}

void onPs3Disconnect() {
  digitalWrite(PIN_LED, 0);
  Serial.println("Bluetooth controller disconnected");
  remoteControl.speed = 0;
  remoteControl.steer = 0;
  remoteControl.speedGain = 1;
  remoteControl.steerGain = 1;
}
#endif
