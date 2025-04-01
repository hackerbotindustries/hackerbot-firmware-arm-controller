/****************************************************************************** 
HackerBot Industries, LLC
Created By: Ian Bernstein
Created:    December 2024
Updated:    2025.03.31

This sketch is written for the "Arm Controller" PCB and interfaces the main
controller to the myCobot 280 arm and Hackerbot gripper.

Special thanks to the following for their code contributions to this codebase:
Randy  - https://github.com/rbeiter
*********************************************************************************/

#include <Dynamixel2Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MyCobotBasic.h>
#include <SerialCmd.h>
#include <Wire.h>
#include "Hackerbot_Shared.h"
#include "SerialCmd_Helper.h"

// Arm Controller software version
#define VERSION_NUMBER 5

// Set up variables and constants for dynamixel control
#define DXL_SERIAL     Serial1

const int DXL_DIR_PIN = 2;
const uint8_t DXL_GRIPPER_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

const float CALIBRATION_CURRENT = 60.0;
const float CALIBRATION_OFFSET = 550.0;
const double CALIBRATION_MIN = -1000;
const double CALIBRATION_MAX = 100000;
double homeCalibration;

// Timing variables
unsigned long startMillis;
unsigned long currentMillis;

// Set up the onboard neopixel
Adafruit_NeoPixel onboard_pixel(1, PIN_NEOPIXEL);

// Other defines and variables
byte I2CRxArray[16];
byte I2CTxArray[16];
byte cmd = 0;
byte incomingI2CFlag = 0;

// Set up the myCobot280
Uart Serial2 (&sercom2, 9, 10, SERCOM_RX_PAD_1, UART_TX_PAD_2);
MyCobotBasic myCobot;

// Set up the serial command processor
SerialCmdHelper mySerCmd(Serial);
int8_t ret;


// -------------------------------------------------------
// I2C Rx Handler
// -------------------------------------------------------
void I2C_RxHandler(int numBytes) {
  for (int i = 0; i < numBytes; i++) {
    I2CRxArray[i] = Wire.read();
  }

  // Parse incoming commands
  switch (I2CRxArray[0]) {
    case I2C_COMMAND_PING: // Ping
      cmd = I2C_COMMAND_PING;
      I2CTxArray[0] = 0x01;
      break;
    case I2C_COMMAND_VERSION: // Version
      cmd = I2C_COMMAND_VERSION;
      I2CTxArray[0] = VERSION_NUMBER;
      break;
    case I2C_COMMAND_A_CAL: // Cal
      cmd = I2C_COMMAND_A_CAL;
      incomingI2CFlag = 1;
      break;
    case I2C_COMMAND_A_OPEN: // Open
      cmd = I2C_COMMAND_A_OPEN;
      incomingI2CFlag = 1;
      break;
    case I2C_COMMAND_A_CLOSE: // Close
      cmd = I2C_COMMAND_A_CLOSE;
      incomingI2CFlag = 1;
      break;
    case I2C_COMMAND_A_ANGLE: // Set_ANGLE Command - Params(joint, angle h, angle l, speed)
      cmd = I2C_COMMAND_A_ANGLE;
      incomingI2CFlag = 1;
      break;
    case I2C_COMMAND_A_ANGLES: // Set_ANGLE Command - Params(joint, angle h, angle l, speed)
      cmd = I2C_COMMAND_A_ANGLES;
      incomingI2CFlag = 1;
      break;
  }
}


// -------------------------------------------------------
// I2C Tx Handler
// -------------------------------------------------------
void I2C_TxHandler(void) {
  switch (cmd) {
    case I2C_COMMAND_PING: // Ping
      Wire.write(I2CTxArray[0]);
      break;
    case I2C_COMMAND_VERSION: // Version
      Wire.write(I2CTxArray[0]);
      break;
  }
}


// -------------------------------------------------------
// Serial2 SERCOM Handler
// -------------------------------------------------------
void SERCOM2_Handler() {
  Serial2.IrqHandler();
}


// -------------------------------------------------------
// User Functions
// -------------------------------------------------------
void sendOK(void) {
  mySerCmd.Print((char *) "OK\r\n");
}


// -------------------------------------------------------
// Functions for SerialCmd
// -------------------------------------------------------
void send_PING(void) {
  sendOK();
}


// Reports the current fw version
// Example - "VERSION"
void Get_Version(void) {
  mySerCmd.Print((char *) "INFO: Arm Controller Firmware (v");
  mySerCmd.Print(VERSION_NUMBER);
  mySerCmd.Print((char *) ".0)\r\n");

  sendOK();
}


// OPEN
void set_OPEN(void) {
  double current_reading = 0.0;

  dxl.setGoalPosition(DXL_GRIPPER_ID, homeCalibration - 10000, UNIT_DEGREE);

  delay (500);

  while(dxl.readControlTableItem(MOVING, DXL_GRIPPER_ID)) {
    current_reading = abs(dxl.getPresentCurrent(DXL_GRIPPER_ID, UNIT_MILLI_AMPERE));
    mySerCmd.Print((char *) "STATUS: Current reading ");
    mySerCmd.Print(current_reading);
    mySerCmd.Print((char *) "mA\r\n");

    if (current_reading >= 200.0) {
      dxl.setGoalPosition(DXL_GRIPPER_ID, dxl.getPresentPosition(DXL_GRIPPER_ID, UNIT_DEGREE), UNIT_DEGREE);
      break;
    }

    delay(50);
  }

  sendOK();
}

// CLOSE
void set_CLOSE(void) {
  double current_reading = 0.0;

  dxl.setGoalPosition(DXL_GRIPPER_ID, homeCalibration, UNIT_DEGREE);

  delay (500);

  while(dxl.readControlTableItem(MOVING, DXL_GRIPPER_ID)) {
    current_reading = abs(dxl.getPresentCurrent(DXL_GRIPPER_ID, UNIT_MILLI_AMPERE));
    mySerCmd.Print((char *) "STATUS: Current reading ");
    mySerCmd.Print(current_reading);
    mySerCmd.Print((char *) "mA\r\n");

    if (current_reading >= 150.0) {
      dxl.setGoalPosition(DXL_GRIPPER_ID, dxl.getPresentPosition(DXL_GRIPPER_ID, UNIT_DEGREE), UNIT_DEGREE);
      break;
    }

    delay(50);
  }

  sendOK();
}

// ANGLE
void set_ANGLE(void) {
  uint8_t jointParam = 0;
  float angleParam = 0.0;
  uint8_t speedParam = 0;

  if (!mySerCmd.ReadNextUInt8(&jointParam) || !mySerCmd.ReadNextFloat(&angleParam) || !mySerCmd.ReadNextUInt8(&speedParam)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  jointParam = constrain(jointParam, 1, 6);
  if (jointParam != 6) {
    angleParam = constrain(angleParam, -165.0, 165);
  } else {
    angleParam = constrain(angleParam, -175.0, 175);
  }
  speedParam  = constrain(speedParam, 0, 100);

  char buf[128] = {0};
  sprintf(buf, "STATUS: Setting the angle of joint %d to %0.1f degrees at speed %d\r\n", jointParam, angleParam, speedParam);
  mySerCmd.Print(buf);

  myCobot.writeAngle(jointParam, angleParam, speedParam);

  sendOK();
}

// ANGLES
void set_ANGLES(void) {
  float joint1Param = 0.0;
  float joint2Param = 0.0;
  float joint3Param = 0.0;
  float joint4Param = 0.0;
  float joint5Param = 0.0;
  float joint6Param = 0.0;
  uint8_t speedParam = 0;

  if (!mySerCmd.ReadNextFloat(&joint1Param) || 
      !mySerCmd.ReadNextFloat(&joint2Param) || 
      !mySerCmd.ReadNextFloat(&joint3Param) || 
      !mySerCmd.ReadNextFloat(&joint4Param) || 
      !mySerCmd.ReadNextFloat(&joint5Param) || 
      !mySerCmd.ReadNextFloat(&joint6Param) || 
      !mySerCmd.ReadNextUInt8(&speedParam)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  joint1Param = constrain(joint1Param, -165.0, 165.0);
  joint2Param = constrain(joint2Param, -165.0, 165.0);
  joint3Param = constrain(joint3Param, -165.0, 165.0);
  joint4Param = constrain(joint4Param, -165.0, 165.0);
  joint5Param = constrain(joint5Param, -165.0, 165.0);
  joint6Param = constrain(joint6Param, -175.0, 175.0);
  speedParam  = constrain(speedParam, 0, 100);

  Angles angles = {joint1Param, joint2Param, joint3Param, joint4Param, joint5Param, joint6Param};

  mySerCmd.Print((char *) "STATUS: Setting the angle of the joints to (1) ");
  mySerCmd.Print(joint1Param);
  mySerCmd.Print((char *) ", (2) ");
  mySerCmd.Print(joint2Param);
  mySerCmd.Print((char *) ", (3) ");
  mySerCmd.Print(joint3Param);
  mySerCmd.Print((char *) ", (4) ");
  mySerCmd.Print(joint4Param);
  mySerCmd.Print((char *) ", (5) ");
  mySerCmd.Print(joint5Param);
  mySerCmd.Print((char *) ", (6) ");
  mySerCmd.Print(joint6Param);
  mySerCmd.Print((char *) " degrees at speed ");
  mySerCmd.Print((int)speedParam);
  mySerCmd.Print((char *) "\r\n");

  myCobot.writeAngles(angles, speedParam);

  sendOK();
}

// CAL
void run_CALIBRATION(void) {
  double current_reading = 0.0;

  dxl.torqueOff(DXL_GRIPPER_ID);
  dxl.setOperatingMode(DXL_GRIPPER_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_GRIPPER_ID);

  dxl.setGoalVelocity(DXL_GRIPPER_ID, 30, UNIT_RPM);

  while (current_reading <= CALIBRATION_CURRENT) {
    current_reading = abs(dxl.getPresentCurrent(DXL_GRIPPER_ID, UNIT_MILLI_AMPERE));
    mySerCmd.Print((char *) "STATUS: Current reading ");
    mySerCmd.Print(current_reading);
    mySerCmd.Print((char *) "mA\r\n");
    delay(50);
  }

  dxl.setGoalVelocity(DXL_GRIPPER_ID, 0, UNIT_RPM);

  dxl.torqueOff(DXL_GRIPPER_ID);
  dxl.setOperatingMode(DXL_GRIPPER_ID, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_GRIPPER_ID);

  homeCalibration = dxl.getPresentPosition(DXL_GRIPPER_ID, UNIT_DEGREE) - CALIBRATION_OFFSET;

  dxl.setGoalPosition(DXL_GRIPPER_ID, homeCalibration, UNIT_DEGREE);

  mySerCmd.Print((char *) "INFO: Calibration complete!\r\n");
  sendOK();
}


// -------------------------------------------------------
// setup()
// -------------------------------------------------------
void setup() {
  // Set up the debug USB C serial port
  unsigned long serialTimout = millis();

  Serial.begin(115200);
  while(!Serial && millis() - serialTimout <= 5000);

  // Define serial commands
  mySerCmd.AddCmd("PING", SERIALCMD_FROMALL, send_PING);
  mySerCmd.AddCmd("VERSION", SERIALCMD_FROMALL, Get_Version);
  mySerCmd.AddCmd("A_CAL", SERIALCMD_FROMALL, run_CALIBRATION);
  mySerCmd.AddCmd("A_OPEN", SERIALCMD_FROMALL, set_OPEN);
  mySerCmd.AddCmd("A_CLOSE", SERIALCMD_FROMALL, set_CLOSE);
  mySerCmd.AddCmd("A_ANGLE", SERIALCMD_FROMALL, set_ANGLE);
  mySerCmd.AddCmd("A_ANGLES", SERIALCMD_FROMALL, set_ANGLES);

  // Set up the dynamixel serial port
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Initialize I2C (Slave Mode: address=0x5C)
  Wire.begin(ARM_I2C_ADDRESS);
  Wire.onReceive(I2C_RxHandler);
  Wire.onRequest(I2C_TxHandler);

  // Configure the gripper dynamixel
  dxl.ping(DXL_GRIPPER_ID);
  
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_GRIPPER_ID, 10);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_GRIPPER_ID, 40);

  // Finish Serial2 setup and configure the myCobot280
  PortGroup *port = digitalPinToPort(9);
  port->OUTSET.reg = g_APinDescription[9].ulPin ;
  port->PINCFG[g_APinDescription[9].ulPin].reg |= PORT_PINCFG_PULLEN;

  myCobot.setup(&Serial2);
  delay(100);
  myCobot.powerOn();
  delay(100);

  // Configure the onboard neopixel
  onboard_pixel.begin();
  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 0, 5));
  onboard_pixel.show();

  // Start the application
  mySerCmd.Print((char *) "INFO: Starting application...\r\n");

  // Calibrate the gripper
  mySerCmd.Print((char *) "INFO: Calibrating the gripper...\r\n");
  run_CALIBRATION();

  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 5, 0));
  onboard_pixel.show();
}


// -------------------------------------------------------
// loop()
// -------------------------------------------------------
void loop() {
  String query = String();
  char CharArray[32];

  if (incomingI2CFlag) {
    mySerCmd.Print((char *) "INFO: Processing I2C Byte Received...\r\n");

    switch (cmd) {
      case I2C_COMMAND_A_CAL: // CAL
        mySerCmd.Print((char *) "INFO: run_CALIBRATION command received\r\n");
        ret = mySerCmd.ReadString((char *) "A_CAL");
        incomingI2CFlag = 0;
        break;
      case I2C_COMMAND_A_OPEN: // Open
        mySerCmd.Print((char *) "INFO: set_OPEN command received\r\n");
        ret = mySerCmd.ReadString((char *) "A_OPEN");
        incomingI2CFlag = 0;
        break;
      case I2C_COMMAND_A_CLOSE: // Close
        mySerCmd.Print((char *) "INFO: set_CLOSE command received\r\n");
        ret = mySerCmd.ReadString((char *) "A_CLOSE");
        incomingI2CFlag = 0;
        break;
      case I2C_COMMAND_A_ANGLE: // Set_ANGLE Command - Params(joint, angle h, angle l, speed)
        mySerCmd.Print((char *) "INFO: Set_ANGLE command received\r\n");
        query = "A_ANGLE," + (String)(I2CRxArray[1]) + "," + (String)((((I2CRxArray[2] << 8) + I2CRxArray[3]) * 0.1) - 165.0) + "," + (String)(I2CRxArray[4]);
        query.toCharArray(CharArray, query.length() + 1);
        ret = mySerCmd.ReadString(CharArray);
        incomingI2CFlag = 0;
        break;
      case I2C_COMMAND_A_ANGLES: // Set_ANGLE Command - Params(joint, angle h, angle l, speed)
        mySerCmd.Print((char *) "INFO: Set_ANGLES command received\r\n");
        query = "A_ANGLES," + 
        (String)((((I2CRxArray[1] << 8) + I2CRxArray[2]) * 0.1) - 165.0) + "," + 
        (String)((((I2CRxArray[3] << 8) + I2CRxArray[4]) * 0.1) - 165.0) + "," + 
        (String)((((I2CRxArray[5] << 8) + I2CRxArray[6]) * 0.1) - 165.0) + "," + 
        (String)((((I2CRxArray[7] << 8) + I2CRxArray[8]) * 0.1) - 165.0) + "," + 
        (String)((((I2CRxArray[9] << 8) + I2CRxArray[10]) * 0.1) - 165.0) + "," + 
        (String)((((I2CRxArray[11] << 8) + I2CRxArray[12]) * 0.1) - 175.0) + "," + 
        (String)(I2CRxArray[13]);
        query.toCharArray(CharArray, query.length() + 1);
        ret = mySerCmd.ReadString(CharArray);
        incomingI2CFlag = 0;
        break;
    }
  }

  ret = mySerCmd.ReadSer();
  if (ret == 0) {
    mySerCmd.Print((char *) "ERROR: Unrecognized command\r\n");
  }
}
