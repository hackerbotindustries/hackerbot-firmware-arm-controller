/****************************************************************************** 
HackerBot Industries, LLC
Ian Bernstein
December 2024
Updated: 2025.01.05

This sketch is written for the "Arm Controller" PCB and interfaces the main
controller to the myCobot 280 arm and Hackerbot gripper.
*********************************************************************************/

#include <Dynamixel2Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SerialCmd.h>
#include <Wire.h>

// Set up variables and constants for dynamixel control
#define DXL_SERIAL   Serial1

const int DXL_DIR_PIN = 2;
const uint8_t DXL_GRIPPER_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

unsigned long startMillis;
unsigned long currentMillis;
byte RxArray[32];

const float CALIBRATION_CURRENT = 35.0;
const float CALIBRATION_OFFSET = 200.0;
const double CALIBRATION_MIN = -1000;
const double CALIBRATION_MAX = 100000;
double homeCalibration;

// Set up the onboard neopixel
Adafruit_NeoPixel onboard_pixel(1, PIN_NEOPIXEL);

// Set up the serial command processor
SerialCmd mySerCmd(Serial);


// -------------------------------------------------------
// I2C Rx Handler
// -------------------------------------------------------
void I2C_RxHandler(int numBytes) {
  Serial.print("[INFO] I2C Byte Received... ");
  for (int i = 0; i < numBytes; i++) {
    RxArray[i] = Wire.read();
    Serial.print("0x");
    Serial.print(RxArray[i], HEX);
    Serial.print(" ");
  }

  Serial.println();

  // Parse incoming commands
  switch (RxArray[0]) {
    case 0x01: // Set_IDLE Command - Params(0 = Off, 1 = On)
      Serial.println("Set_IDLE command received");
      if (RxArray[1] == 0x00) {
        //idle = 0;
      } else {
        //idle = 1;
      }
      break;
    case 0x02: // Set_Direction Command - Params(rotation h, rotation l, pitch h, pitch l, speed)
      Serial.println("Set_Direction command received");
      //position_turn = ((RxArray[1] << 8) + RxArray[2]) * 0.1;
      //position_speed = RxArray[5];
      //newMovement = 1;
      break;
  }
}

// -------------------------------------------------------
// Command User Functions
// -------------------------------------------------------
void sendOK(void) {
  mySerCmd.Print((char *) "OK \r\n");
}


// -------------------------------------------------------
// setup()
// -------------------------------------------------------
void setup() {
  // Set up the debug USB C serial port
  unsigned long serialTimout = millis();

  Serial.begin(115200);
  while(!Serial && millis() - serialTimout <= 5000);

  // Set up the dynamixel serial port
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Set up the I2C port
  Wire.begin(91); // Initialize I2C (Slave Mode: address=0x5B)
  Wire.onReceive(I2C_RxHandler);

  // Configure the gripper dynamixel
  dxl.ping(DXL_GRIPPER_ID);

  dxl.torqueOff(DXL_GRIPPER_ID);
  dxl.setOperatingMode(DXL_GRIPPER_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_GRIPPER_ID);
  
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_GRIPPER_ID, 10);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_GRIPPER_ID, 40);

  // Configure the onboard neopixel
  onboard_pixel.begin();
  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 0, 5));
  onboard_pixel.show();

  // Start the application
  Serial.println("INFO: Starting application...");

  dxl.setGoalVelocity(DXL_GRIPPER_ID, 30, UNIT_RPM);
}


// ----------------------- loop() ------------------------
void loop() {
  double current_reading;
  //dxl.setGoalPosition(DXL_GRIPPER_ID, -2000, UNIT_DEGREE);

  delay(50);

  //Serial.println(dxl.getPresentPosition(DXL_GRIPPER_ID, UNIT_DEGREE));

  current_reading = abs(dxl.getPresentCurrent(DXL_GRIPPER_ID, UNIT_MILLI_AMPERE));

  Serial.println(current_reading);

  if (current_reading >= CALIBRATION_CURRENT) {
    dxl.setGoalVelocity(DXL_GRIPPER_ID, 0, UNIT_RPM);

    dxl.torqueOff(DXL_GRIPPER_ID);
    dxl.setOperatingMode(DXL_GRIPPER_ID, OP_EXTENDED_POSITION);
    dxl.torqueOn(DXL_GRIPPER_ID);

    homeCalibration = dxl.getPresentPosition(DXL_GRIPPER_ID, UNIT_DEGREE) - CALIBRATION_OFFSET;

    dxl.setGoalPosition(DXL_GRIPPER_ID, homeCalibration, UNIT_DEGREE);

    delay(10000);

    while(1) {
      dxl.setGoalPosition(DXL_GRIPPER_ID, homeCalibration - CALIBRATION_MAX, UNIT_DEGREE);

      delay(60000);

      dxl.setGoalPosition(DXL_GRIPPER_ID, homeCalibration + CALIBRATION_MIN, UNIT_DEGREE);

      delay(60000);
    }
  }
}
