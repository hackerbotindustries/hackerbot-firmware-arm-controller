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
#include <MyCobotBasic.h>
#include <SerialCmd.h>
#include <Wire.h>
#include "HackerbotShared.h"
#include "HackerbotSerialCmd.h"

// Arm Controller software version
#define VERSION_NUMBER 2

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
HackerbotSerialCmd mySerCmd(Serial);
int8_t ret;


// -------------------------------------------------------
// I2C Rx Handler
// -------------------------------------------------------
void I2C_RxHandler(int numBytes) {
  //mySerCmd.Print((char *) "INFO: I2C Byte Received... \r\n");
  for (int i = 0; i < numBytes; i++) {
    I2CRxArray[i] = Wire.read();
    //mySerCmd.Print((char *) "0x");
    //Serial.print(I2CRxArray[i], HEX);
    //mySerCmd.Print((char *) " ");
  }

  //mySerCmd.Print((char *) "\r\n");

  // Parse incoming commands
  switch (I2CRxArray[0]) {
    case 0x01: // Ping
      cmd = 0x01;
      I2CTxArray[0] = 0x01;
      break;
    case 0x02: // Version
      cmd = 0x02;
      I2CTxArray[0] = VERSION_NUMBER;
      break;
    case 0x20: // Cal
      cmd = 0x20;
      incomingI2CFlag = 1;
      break;
    case 0x21: // Open
      cmd = 0x21;
      incomingI2CFlag = 1;
      break;
    case 0x22: // Close
      cmd = 0x22;
      incomingI2CFlag = 1;
      break;
    case 0x25: // Set_ANGLE Command - Params(joint, angle h, angle l, speed)
      cmd = 0x25;
      incomingI2CFlag = 1;
      break;
    case 0x26: // Set_ANGLE Command - Params(joint, angle h, angle l, speed)
      cmd = 0x26;
      incomingI2CFlag = 1;
      break;
  }
}


// -------------------------------------------------------
// I2C Tx Handler
// -------------------------------------------------------
void I2C_TxHandler(void) {
  switch (cmd) {
    case 0x01: // Ping
      Wire.write(I2CTxArray[0]);
      break;
    case 0x02: // Version
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

  if (!mySerCmd.ReadNextUInt8(&jointParam) ||
      !mySerCmd.ReadNextFloat(&angleParam) ||
      !mySerCmd.ReadNextUInt8(&speedParam)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range
  jointParam = constrain(jointParam, 1, 6);
  float limit = LIMIT_FOR_ARM_JOINT(jointParam);
  angleParam = constrain(angleParam, -1 * limit, limit);
  speedParam = constrain(speedParam, 0, 100);

  char buf[128] = {0};
  sprintf(buf, "STATUS: Setting the angle of joint %d to %0.1f degrees at speed %d\r\n", jointParam, angleParam, speedParam);
  mySerCmd.Print(buf);

  myCobot.writeAngle(jointParam, angleParam, speedParam);

  sendOK();
}

// ANGLES
void set_ANGLES(void) {
  float jointParam[6] = {0.0};
  uint8_t speedParam = 0;

  if (!mySerCmd.ReadNextFloat(&jointParam[0]) ||
      !mySerCmd.ReadNextFloat(&jointParam[1]) ||
      !mySerCmd.ReadNextFloat(&jointParam[2]) ||
      !mySerCmd.ReadNextFloat(&jointParam[3]) ||
      !mySerCmd.ReadNextFloat(&jointParam[4]) ||
      !mySerCmd.ReadNextFloat(&jointParam[5]) ||
      !mySerCmd.ReadNextUInt8(&speedParam)) {
    mySerCmd.Print((char *) "ERROR: Missing parameter\r\n");
    return;
  }

  // Constrain values to acceptable range
  for (int ndx=0; ndx<6; ndx++) {
    float limit = LIMIT_FOR_ARM_JOINT(ndx+1);
    jointParam[ndx] = constrain(jointParam[ndx], -1 * limit, limit);
  }
  speedParam = constrain(speedParam, 0, 100);

  Angles angles;
  std::copy(std::begin(jointParam), std::end(angles), angles.begin()); // fill Angles std::array with data from scalar jointParam

  char buf[160] = {0};
  char *pos=buf;
  pos += sprintf(pos, "STATUS: Setting the angle of the joints to");
  for (size_t ndx = 0; ndx < angles.size(); ndx++) {
    pos += sprintf(pos, " (%d) %.1f%s", ndx+1, angles[ndx], (ndx < 5 ? "," : ""));
  }
  sprintf(pos, " degrees at speed %d\r\n", speedParam);
  mySerCmd.Print(buf);

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
  mySerCmd.AddCmd("CAL", SERIALCMD_FROMALL, run_CALIBRATION);
  mySerCmd.AddCmd("OPEN", SERIALCMD_FROMALL, set_OPEN);
  mySerCmd.AddCmd("CLOSE", SERIALCMD_FROMALL, set_CLOSE);
  mySerCmd.AddCmd("ANGLE", SERIALCMD_FROMALL, set_ANGLE);
  mySerCmd.AddCmd("ANGLES", SERIALCMD_FROMALL, set_ANGLES);

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
  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 5, 0));
  onboard_pixel.show();

  // Start the application
  mySerCmd.Print((char *) "INFO: Starting application...\r\n");

  // Calibrate the gripper
  mySerCmd.Print((char *) "INFO: Calibrating the gripper...\r\n");
  run_CALIBRATION();

  onboard_pixel.setPixelColor(0, onboard_pixel.Color(0, 0, 5));
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
      case 0x20: // CAL
        mySerCmd.Print((char *) "INFO: run_CALIBRATION command received\r\n");
        ret = mySerCmd.ReadString((char *) "CAL");
        incomingI2CFlag = 0;
        break;
      case 0x21: // Open
        mySerCmd.Print((char *) "INFO: set_OPEN command received\r\n");
        ret = mySerCmd.ReadString((char *) "OPEN");
        incomingI2CFlag = 0;
        break;
      case 0x22: // Close
        mySerCmd.Print((char *) "INFO: set_CLOSE command received\r\n");
        ret = mySerCmd.ReadString((char *) "CLOSE");
        incomingI2CFlag = 0;
        break;
      case 0x25: // Set_ANGLE Command - Params(joint, angle h, angle l, speed)
        mySerCmd.Print((char *) "INFO: Set_ANGLE command received\r\n");

         // divide by 10 and subtract half of uint16_t's max value to convert unsigned uint16_t representation back to signed float
        query = "ANGLE," + (String)(I2CRxArray[1]) + "," + (String)(hb_btof(&I2CRxArray[2])) + "," + (String)(I2CRxArray[4]);
        query.toCharArray(CharArray, query.length() + 1);
        ret = mySerCmd.ReadString(CharArray);
        incomingI2CFlag = 0;
        break;
      case 0x26: // Set_ANGLE Command - Params(joint, angle h, angle l, speed)
        mySerCmd.Print((char *) "INFO: Set_ANGLES command received\r\n");
         // divide by 10 and subtract half of uint16_t's max value to convert unsigned uint16_t representation back to signed float
        query = "ANGLES," +
        (String)(hb_btof(&I2CRxArray[1])) + "," +
        (String)(hb_btof(&I2CRxArray[3])) + "," +
        (String)(hb_btof(&I2CRxArray[5])) + "," +
        (String)(hb_btof(&I2CRxArray[7])) + "," +
        (String)(hb_btof(&I2CRxArray[9])) + "," +
        (String)(hb_btof(&I2CRxArray[11])) + "," +
        (String)(I2CRxArray[13]);
        query.toCharArray(CharArray, query.length() + 1);
        // mySerCmd.Print((char *) CharArray);
        // mySerCmd.Print((char *) "\r\n");
        ret = mySerCmd.ReadString(CharArray);
        incomingI2CFlag = 0;
        break;
    }
  }

  ret = mySerCmd.ReadSer();
  if (ret == 0) {
    mySerCmd.Print((char *) "ERROR: Urecognized command\r\n");
  }
}
