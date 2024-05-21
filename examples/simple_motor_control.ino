#include "Arduino.h"
#include "mcp_can.h"
#include "TMotor_ServoConnection.h"

// define constants
#define CAN0_INT 2 // set interrupt pin for incoming CAN messages
#define ID_MOTOR_1 1

// init global vars
MCP_CAN CAN0(10); // set SPI select pin
TMotor_ServoConnection servo_conn(CAN0);  // create CAN Servo Connection object

void setup() {
  // establisehd serial connection to PC
  while (!Serial);
  Serial.begin(115200);
  // establish connection between Arduino and MCP2515
  while(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK){ // check if conenction could be establisehd; retry if not
    Serial.println("Error Initializing MCP2515...");
    delay(100);
  } 
  Serial.println("MCP2515 Initialized Successfully!");
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  servo_conn.set_origin(ID_MOTOR_1, 0);

}

void loop() {
  // put your main code here, to run repeatedly:

  // use position controller and move motor by 180° with low speed and low acceleration
  servo_conn.set_pos_spd(ID_MOTOR_1, 180, 1000, 1000);

  delay(5);
  // check for incoming messages
  if (!digitalRead(CAN0_INT)) {
    // get all messages in waiting queue
    while (CAN_MSGAVAIL == CAN0.checkReceive()) {
      // receive and process messages
      servo_conn.can_receive();
    }
  }
  // print received data to Serial output
  servo_conn.print_motor_vars(ID_MOTOR_1);
}