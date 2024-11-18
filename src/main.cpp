#include <Arduino.h>
#include <Preferences.h>
#include <ReactESP.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <Ultrasonic.h>

using namespace reactesp;
Preferences NVS;

// Function prototypes
void parseCommand(String command);
void setLedMode(uint8_t mode);
void sendTelemetry();
void motorDrive(int left, int right);

// Pin definitions
const uint8_t QTR_LEFT_PINS[] = {32, 33};
const uint8_t QTR_RIGHT_PINS[] = {25, 26};
const uint8_t QTR_LEFT_EMITTER_PIN = 27;
const uint8_t QTR_RIGHT_EMITTER_PIN = 14;
const uint8_t DRIVER_AIN1 = 17;
const uint8_t DRIVER_BIN1 = 18;
const uint8_t DRIVER_AIN2 = 16;
const uint8_t DRIVER_BIN2 = 19;
const uint8_t DRIVER_PWMA = 4;
const uint8_t DRIVER_PWMB = 21;
const uint8_t DRIVER_STBY = 5;
const uint8_t ULTSONIC_SIG_PIN = 22;
const uint8_t LED = 2;

/*
  Available pins in ESP32:
  R: 15, !2, !0, /4, /16, /17, /5, /18, /19, /21, !3, !1, /22, 23
  L: 13, <12, /14, /27, /26, /25, /33, /32, !35, !34, !39, !36

  Legend:
    !: Can't be used
    /: Occupied
    <: Output only
    >: Input only
*/

// BLE setup
const char* SERVICE_UUID = "a16587d4-584a-4668-b279-6ccb940cdfd0";
const char* CHARACTERISTIC_UUID_TX = "a16587d4-584a-4668-b279-6ccb940cdfd1";
const char* CHARACTERISTIC_UUID_RX = "a16587d4-584a-4668-b279-6ccb940cdfd2";
BLEServer* Server = NULL;
BLECharacteristic *CharTX, *CharRX;
bool dev_connected = false;
bool last_dev_connected = false;
String tx_idle = "";
String tx_qtrcal = "";
String tx_combat = "";

// QTR sensor setup
const uint8_t SENSOR_COUNT = 2;
QTRSensors QTRLeft;
QTRSensors QTRRight;
uint16_t sensor_values_left[SENSOR_COUNT];
uint16_t sensor_values_right[SENSOR_COUNT];
uint16_t qtrcal_min_values_left[SENSOR_COUNT];
uint16_t qtrcal_max_values_left[SENSOR_COUNT];
uint16_t qtrcal_min_values_right[SENSOR_COUNT];
uint16_t qtrcal_max_values_right[SENSOR_COUNT];
uint16_t qtrcal_max_ref;
uint16_t qtrcal_min_ref;
uint16_t position;
uint8_t qtrcal_counter = 0;

// Ultrasonic sensor setup
Ultrasonic RANGEFINDER(ULTSONIC_SIG_PIN);

// Motor setup
const int offset_motorA = 1;
const int offset_motorB = 1;
Motor MotorA_Left = Motor(DRIVER_AIN1, DRIVER_AIN2, DRIVER_PWMA, offset_motorA, DRIVER_STBY);
Motor MotorB_Right = Motor(DRIVER_BIN1, DRIVER_BIN2, DRIVER_PWMB, offset_motorB, DRIVER_STBY);

// Vehicle controller setup
const uint8_t MODE_IDLE = 0;
const uint8_t MODE_QTRCAL = 1;
const uint8_t MODE_COMBAT = 2;
const uint8_t SM_TORNADO = 0;
uint8_t mode = MODE_IDLE;
uint8_t search = SM_TORNADO;
uint8_t arm_motor = 0;
int16_t speed_limit = 230;
int16_t left_motor = 0;
int16_t right_motor = 0;
long rdg_distance = 0;

long min_tgt_distance = 0;
int16_t ram_speed = 0;
int16_t tornado_search_speed = 0;
uint16_t ring_threshold = 0;

// Blinker setup
EventLoop LEDQtrCalBlinker;
EventLoop LEDActiveBlinker;
EventLoop LEDIdleBlinker;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    dev_connected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    dev_connected = false;
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rx_value = pCharacteristic->getValue();
    if (rx_value.length() > 0) {
      parseCommand(String(rx_value.c_str()));
    }
  }
};

void setup() {

  // Initialize serial and pin modes
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  // Load data
  NVS.begin("gabulbot", false);

  // Initialize blinkers
  LEDQtrCalBlinker.onRepeat(100, [] () {
      static bool state = false;
      digitalWrite(LED, state = !state);
  });
  LEDActiveBlinker.onRepeat(200, [] () {
      static bool state = false;
      digitalWrite(LED, state = !state);
  });
  LEDIdleBlinker.onRepeat(500, [] () {
      static bool state = false;
      digitalWrite(LED, state = !state);
  });

  BLEDevice::init("GabulBOT");

  // Create the BLE Server
  Server = BLEDevice::createServer();
  Server->setCallbacks(new ServerCallbacks());
  BLEService *Service = Server->createService(SERVICE_UUID);

  // BLEChar for telemetry data
  CharTX = Service->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  // BLEChar for receiving commands
  CharRX = Service->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_WRITE
  );

  CharRX->setCallbacks(new CommandCallbacks());
  CharTX->addDescriptor(new BLE2902());
  CharRX->addDescriptor(new BLE2902());
  Service->start();
  Server->getAdvertising()->start();
  digitalWrite(LED, HIGH);

  // Initialize QTR sensor
  QTRLeft.setTypeRC();
  QTRRight.setTypeRC();
  QTRLeft.setSensorPins(QTR_LEFT_PINS, SENSOR_COUNT);
  QTRRight.setSensorPins(QTR_RIGHT_PINS, SENSOR_COUNT);
  QTRLeft.setEmitterPin(QTR_LEFT_EMITTER_PIN);
  QTRRight.setEmitterPin(QTR_RIGHT_EMITTER_PIN);
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    qtrcal_min_values_left[i] = 0;
    qtrcal_max_values_left[i] = 0;
    qtrcal_min_values_right[i] = 0;
    qtrcal_max_values_right[i] = 0;
  }

  // Load calibration data
  speed_limit = NVS.getUShort("speed_limit", 230);
  min_tgt_distance = NVS.getUShort("min_tgt_distance", 40);
  tornado_search_speed = NVS.getUShort("tornado_search_speed", 130);
  qtrcal_min_ref = NVS.getUShort("qtrcal_min_ref", 200);
  qtrcal_max_ref = NVS.getUShort("qtrcal_max_ref", 900);

}

void loop() {

  // On connect
  if (dev_connected) {
    
    setLedMode(mode);

    // QTR calibration mode
    if (mode == MODE_QTRCAL) {
      for (uint16_t i = 0; i < 400; i++)
      {
        qtrcal_counter = i;
        QTRLeft.calibrate();
        QTRRight.calibrate();
        sendTelemetry();
      }
      qtrcal_counter = uint8_t(400);
      for (uint8_t i = 0; i < SENSOR_COUNT; i++)
      {
        qtrcal_min_values_left[i] = QTRLeft.calibrationOn.minimum[i];
        qtrcal_max_values_left[i] = QTRLeft.calibrationOn.maximum[i];
        qtrcal_min_values_right[i] = QTRRight.calibrationOn.minimum[i];
        qtrcal_max_values_right[i] = QTRRight.calibrationOn.maximum[i];
        sendTelemetry();
      }
      uint16_t qtrcal_max_ref_left = *std::min_element(qtrcal_max_values_left, qtrcal_max_values_left + (sizeof(qtrcal_max_values_left) / sizeof(qtrcal_max_values_left[0])));
      uint16_t qtrcal_min_ref_left = *std::min_element(qtrcal_min_values_left, qtrcal_min_values_left + (sizeof(qtrcal_min_values_left) / sizeof(qtrcal_min_values_left[0])));
      uint16_t qtrcal_max_ref_right = *std::min_element(qtrcal_max_values_right, qtrcal_max_values_right + (sizeof(qtrcal_max_values_right) / sizeof(qtrcal_max_values_right[0])));
      uint16_t qtrcal_min_ref_right = *std::min_element(qtrcal_min_values_right, qtrcal_min_values_right + (sizeof(qtrcal_min_values_right) / sizeof(qtrcal_min_values_right[0])));
      if (qtrcal_max_ref_left > qtrcal_max_ref_right) {
        qtrcal_max_ref = qtrcal_max_ref_left;
      } else {
        qtrcal_max_ref = qtrcal_max_ref_right;
      }
      if (qtrcal_min_ref_left > qtrcal_min_ref_right) {
        qtrcal_min_ref = qtrcal_min_ref_left;
      } else {
        qtrcal_min_ref = qtrcal_min_ref_right;
      }
      ring_threshold = (qtrcal_min_ref + qtrcal_max_ref) / 2 - (qtrcal_min_ref * 0.75);

      NVS.putUShort("qtrcal_max_ref", qtrcal_max_ref);
      NVS.putUShort("qtrcal_min_ref", qtrcal_min_ref);

      sendTelemetry();
      mode = MODE_IDLE;

      delay(500);
    }

    if (mode == MODE_COMBAT) {

      QTRLeft.read(sensor_values_left);
      QTRRight.read(sensor_values_right);
      /*
        sensor_values_left[0] -> rear left
        sensor_values_left[1] -> front left
        sensor_values_right[0] -> rear right
        sensor_values_right[1] -> front right
      */

      // if front sensors detect the ring, move backwards
      while (sensor_values_left[1] < ring_threshold || 
             sensor_values_right[1] < ring_threshold)
      {
        motorDrive(-speed_limit, -speed_limit);
      }

      // if rear sensors detect the ring, move forward
      while (sensor_values_left[0] < ring_threshold || 
             sensor_values_right[0] < ring_threshold)
      {
        motorDrive(speed_limit, speed_limit);
      }

      if (search = SM_TORNADO) {
        searchTornado();
      }

      sendTelemetry();
    }

    if (mode == MODE_IDLE) {
      // Safe all motors
      left_motor = 0;
      right_motor = 0;
      MotorA_Left.brake();
      MotorB_Right.brake();

      sendTelemetry();

      delay(500);
    }

  }

  // On disconnect
  if (!dev_connected && last_dev_connected) {

    // Reset mode and stop motors
    mode = MODE_IDLE;

    // Give the BLE stack the chance to get things ready and advertise again
    delay(500);
    Server->startAdvertising(); 
    Serial.println("Advertising again...");
    last_dev_connected = dev_connected;

    digitalWrite(LED, HIGH);

  }

  // While connecting
  if (dev_connected && !last_dev_connected) {

    last_dev_connected = dev_connected;
    digitalWrite(LED, HIGH);

  }

}

void parseCommand(String command) {
  command.trim();
  if (command.startsWith("mode ")) {
    String mode_value = command.substring(5);
    if (mode_value == "cb") {
      mode = MODE_COMBAT;
    } else if (mode_value == "qc") {
      mode = MODE_QTRCAL;
    } else if (mode_value == "id") {
      mode = MODE_IDLE;
    } else {
      Serial.println("Unknown mode: " + mode_value + ". Available modes: id, qc, cb");
    }
  } else if (command.startsWith("set tss ")) {
    String tss = command.substring(7);
    NVS.putUShort("tornado_search_speed", tss.toInt());
    tornado_search_speed = tss.toInt(); 
  } else if (command.startsWith("set mtd ")) {
    String mtd = command.substring(7);
    NVS.putUShort("min_tgt_distance", mtd.toInt());
    min_tgt_distance = mtd.toInt();
  } else if (command.startsWith("set sl ")) {
    String sl = command.substring(7);
    NVS.putUShort("speed_limit", sl.toInt());
    speed_limit = sl.toInt();
  } else if (command.startsWith("set rt ")) {
    String rt = command.substring(7);
    NVS.putUShort("ring_threshold", rt.toInt());
    ring_threshold = rt.toInt();
  } else if (command.startsWith("arm md")) {
    arm_motor = 1;
  } else if (command.startsWith("safe md")) {
    arm_motor = 0;
  } else {
    Serial.println("Unknown command: " + command);
  }
}

void sendTelemetry() {
  if (!dev_connected) return;
  if (mode == MODE_IDLE)
  {
    tx_idle = String(mode) + "," + 
              String(arm_motor) + "," + 
              String(min_tgt_distance) + "," + 
              String(tornado_search_speed) + "," + 
              String(speed_limit);
    CharTX->setValue(tx_idle.c_str());
    CharTX->notify();
  } else if (mode == MODE_QTRCAL) {
    tx_qtrcal = String(mode) + "," +
                  String(qtrcal_counter) + "," +
                  String(qtrcal_min_values_left[0]) + "," +
                  String(qtrcal_min_values_left[1]) + "," +
                  String(qtrcal_min_values_right[0]) + "," +
                  String(qtrcal_min_values_right[1]) + "," +
                  String(qtrcal_max_values_left[0]) + "," +
                  String(qtrcal_max_values_left[1]) + "," +
                  String(qtrcal_max_values_right[0]) + "," +
                  String(qtrcal_max_values_right[1]);
      CharTX->setValue(tx_qtrcal.c_str());
      CharTX->notify();
  } else if (mode == MODE_COMBAT) {
    tx_combat = String(mode) + "," +
                  String(left_motor) + "," + 
                  String(right_motor) + "," +
                  String(sensor_values_left[0]) + "," +
                  String(sensor_values_left[1]) + "," +
                  String(sensor_values_right[0]) + "," +
                  String(sensor_values_right[1]) + "," +
                  String(rdg_distance);
    CharTX->setValue(tx_combat.c_str());
    CharTX->notify();
  }
}

void setLedMode(uint8_t mode) {

  if (mode == MODE_IDLE) {
    LEDIdleBlinker.tick();
  } else if (mode == MODE_QTRCAL) {
    LEDQtrCalBlinker.tick();
  } else if (mode == MODE_COMBAT) {
    LEDActiveBlinker.tick();
  }

}

void motorDrive(int left, int right) {
  
  left_motor = left;
  right_motor = right;
  if (arm_motor) {
    MotorA_Left.drive(left);
    MotorB_Right.drive(right);
  } else {
    MotorA_Left.brake();
    MotorB_Right.brake();
  }

}

void searchTornado() {

  rdg_distance = RANGEFINDER.MeasureInCentimeters();

  if (rdg_distance > min_tgt_distance) {
    // Spin around until target is found with ultrasonic sensor
    motorDrive(tornado_search_speed, -tornado_search_speed);
  } else {
    // Target found, move towards it
    ramTarget();

    // Check if the target is lost while moving towards it
    if (RANGEFINDER.MeasureInCentimeters() > min_tgt_distance) {
      // Target lost, spin again to search for it
      motorDrive(tornado_search_speed, -tornado_search_speed);
    }
  }

}

void ramTarget() {
  // Move towards the target with full speed
  motorDrive(speed_limit, speed_limit);
}