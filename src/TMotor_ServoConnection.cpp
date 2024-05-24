#include "Arduino.h"
#include "TMotor_ServoConnection.h"
#include <iostream>
#include <limits>

typedef enum {
  CAN_PACKET_SET_DUTY = 0, // Duty Cycle Mode
  CAN_PACKET_SET_CURRENT, // Current Loop Mode
  CAN_PACKET_SET_CURRENT_BRAKE, // Current Brake Mode
  CAN_PACKET_SET_RPM, // Speed Mode
  CAN_PACKET_SET_POS, // Position Mode
  CAN_PACKET_SET_ORIGIN_HERE, // Set Origin Mode
  CAN_PACKET_SET_POS_SPD, // Position-Speed Loop Mode
  CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;

TMotor_ServoConnection::TMotor_ServoConnection(MCP_CAN &can_connection):_can_connection(can_connection){
  motor_constants.Kt = 0.113;
  motor_constants.reduction_ratio = 6;
}

void TMotor_ServoConnection::send_CAN_message(uint32_t id, bool extended, byte data[8], uint8_t len) {
  byte sndStat = _can_connection.sendMsgBuf(id, extended, len, data);
  if(sndStat != CAN_OK){
    Serial.println("Error Sending Message...");
    Serial.println(sndStat);
  }
}

void TMotor_ServoConnection::set_duty_cycle(long unsigned int id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  send_CAN_message(id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), 1, buffer, send_index);
}

void TMotor_ServoConnection::set_current(long unsigned int id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  send_CAN_message(id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), 1, buffer, send_index);
}

void TMotor_ServoConnection::set_break(long unsigned int id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  send_CAN_message(id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), 1, buffer, send_index);
}

void TMotor_ServoConnection::set_origin(long unsigned int id, uint8_t set_origin_mode) {
  int32_t send_index = 0;
  uint8_t buffer;
  buffer = set_origin_mode;
  send_CAN_message(id | ((uint32_t) CAN_PACKET_SET_ORIGIN_HERE << 8), 1, &buffer, send_index);
  delay(500);
  Serial.println("Zero pos set");
}

void TMotor_ServoConnection::set_pos(long unsigned int id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  send_CAN_message(id | ((uint32_t)CAN_PACKET_SET_POS << 8), 1, buffer, send_index);
}

void TMotor_ServoConnection::set_speed(long unsigned int id, float speed) {
int32_t send_index = 0;
uint8_t buffer[4];
buffer_append_int32(buffer, (int32_t)speed, &send_index);
send_CAN_message(id | ((uint32_t)CAN_PACKET_SET_RPM << 8), 1, buffer, send_index);
}

void TMotor_ServoConnection::set_pos_spd(long unsigned int id, float pos, int16_t spd, int16_t RPA ) {
  int32_t send_index = 0;
  int16_t send_index1 = 4;
  uint8_t buffer[8];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(buffer,spd/10.0, & send_index1);
  buffer_append_int16(buffer,RPA/10.0, & send_index1);
  send_CAN_message(id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), 1, buffer, send_index1);
}

void TMotor_ServoConnection::can_receive(void){
  long unsigned int rxId = 0;
  unsigned char len = 0;
  unsigned char rxBuf[8];
  byte isMessage = _can_connection.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
  parse_CAN_data(rxId, rxBuf);
}

void TMotor_ServoConnection::parse_CAN_data(long unsigned int id, unsigned char rx_message[8]){
  int16_t pos_int = (rx_message[0] << 8 | rx_message[1]);
  int16_t spd_int = (rx_message[2] << 8 | rx_message[3]);
  int16_t cur_int = (rx_message[4] << 8 | rx_message[5]);
  float motor_pos= (float)( pos_int * 0.1f); // Motor Position
  float motor_spd= (float)( spd_int * 10.0f);// Motor Speed
  float motor_cur= (float) ( cur_int * 0.01f);// Motor Current
  int motor_temp= rx_message[6];// Motor Temperature
  int motor_error= rx_message[7];// Motor Error Code

  // id ist only in the last two bytes
  int motor_id = id & 0x000000FF;

  // check if id is already in map
  motor_data* const data = get_motor_data_reference((int)motor_id);

  // create id if not already in map
  if (data == NULL){
    motor_data motor;
    motor.position = motor_pos;
    motor.velocity = motor_spd;
    motor.current = motor_cur;
    motor.temp = motor_temp;
    motor.error = motor_error;
    motors[motor_id] = motor;
  } else { // change values of existing entry
    data->position = motor_pos;
    data->velocity = motor_spd;
    data->current = motor_cur;
    data->temp = motor_temp;
    data->error = motor_error;
  }
}

motor_data* const TMotor_ServoConnection::get_motor_data_reference(int id){
  if (motors.count(id) == 0) {
    Serial.print("Your Key:");
    Serial.print(id);
    Serial.print("...");
    Serial.print("ID not found! Possible IDs: ");
    for(auto const& imap: motors){
      Serial.print(imap.first);
      Serial.print(", ");
    }
    Serial.println("");
    return NULL;
  }
  return &motors[id];
}

void TMotor_ServoConnection::print_motor_vars(int id){
  motor_data* data = get_motor_data_reference(id);
  if (data == NULL) return;
  motor_data motor = *data;
  char msgString[128];    
  sprintf(msgString, " | CAN ID: %02X, Pos: %.2f, Vel: %.2f, Current: %.2f, Temp: %1d, Error: %1d", id, motor.position, motor.velocity, motor.current, motor.temp, motor.error);
  Serial.println(msgString);
}

float TMotor_ServoConnection::get_pos(int id){
  motor_data* data = get_motor_data_reference(id);
  if (data == NULL) return std::numeric_limits<float>::infinity();
  motor_data motor = *data;
  return motor.position;
}

float TMotor_ServoConnection::get_speed(int id){
  motor_data* data = get_motor_data_reference(id);
  if (data == NULL) return std::numeric_limits<float>::infinity();
  motor_data motor = *data;
  return motor.velocity;
}

float TMotor_ServoConnection::get_current(int id){
  motor_data* data = get_motor_data_reference(id);
  if (data == NULL) return std::numeric_limits<float>::infinity();
  motor_data motor = *data;
  return motor.current;
}

int TMotor_ServoConnection::get_temp(int id){
  motor_data* data = get_motor_data_reference(id);
  if (data == NULL) return -1;
  motor_data motor = *data;
  return motor.temp;
}

int TMotor_ServoConnection::get_error(int id){
  motor_data* data = get_motor_data_reference(id);
  if (data == NULL) return -1;
  motor_data motor = *data;
  return motor.error;
}

float TMotor_ServoConnection::current_to_torque(float current){
  return current * motor_constants.Kt * motor_constants.reduction_ratio;
}

float TMotor_ServoConnection::torque_to_current(float torque){
  return torque / (motor_constants.Kt * motor_constants.reduction_ratio);
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}



