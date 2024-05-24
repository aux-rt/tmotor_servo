#ifndef TMotor_Servo_h
#define TMotor_Servo_h

#include "Arduino.h"
#include "mcp_can.h"
#include <map>

struct motor_constants_struct {
  float Kt;
  int reduction_ratio;
};

struct motor_data {
  float position;
  float velocity;
  float current;
  int temp;
  int error;
};

class TMotor_ServoConnection
{
    public:
    motor_constants_struct motor_constants;

    // Constructs ServoConection with an already established connection to a MCP2515 CAN Board.
    // As the connection with multiple motors is possible the id of the motor must be used in every "set" function call.
    TMotor_ServoConnection(MCP_CAN &can_connection);
    // Sets Duty Cycle of the motor in percent. Range: -1 to 1.
    void set_duty_cycle(long unsigned int id, float duty);
    // Sets Motor Current. Range: -60A to 60A.
    void set_current(long unsigned int id, float current);
    // Sets zero point of motor for position control. set_origin_mode = 0 sets current position as a temporary origin until power-off.
    // set_origin_mode = 1 sets the current positions as permanent origin even after power-off.
    void set_origin(long unsigned int id, uint8_t set_origin_mode);
    // Drive motor to given position with max speed and and max acceleration. Range -36000° to 36000°.
    void set_pos(long unsigned int id, float pos);
    // Drive motor with given speed with max acceleration. Range: -100000 to 100000 electrical RPM. 
    void set_speed(long unsigned int id, float speed);
    // Drive motor to given position with max speed spd and max acceleration RPM. Range
    // pos: -36000° to 36000°. spd: -327680 to 327670 electircal RPM. RPA: 0 - 327670 electrical RPM/s^2. 
    void set_pos_spd(long unsigned int id, float pos,int16_t spd, int16_t RPA);
    // sets motor brake with given current. Range 0A - 60A.
    void set_break(long unsigned int id, float current);
    // helper to receive data from all connected ids.
    void can_receive(void);
    // parse received CAN data and save it in motors map.
    void parse_CAN_data(long unsigned int id, unsigned char rx_message[8]);
    // print motor data of given id
    void print_motor_vars(int id);
    // get current motor position in °; Range: -3200° - 3200°; std::numeric_limits<float>::infinity() if no motor with this id is found
    float get_pos(int id);
    // current motor speed in electical rpm; Range: -320000 electrical RPM - 320000° electrical RPM; std::numeric_limits<float>::infinity() if no motor with this id is found
    float get_speed(int id);
    // get motor current in ampere; Range: -60A - 60A ; std::numeric_limits<float>::infinity() if no motor with this id is found
    float get_current(int id);
    // get motor temperature in °C; Range: 20°C - 127°C; -1 if no motor with this id is found
    int get_temp(int id);
    // get motor error code; Error Types: -1: motor if not found; 0: no fault; 1: motor over-temperature fault; 2: over-current fault; 3: over-voltage fault; 4: under-voltage fault; 5: encoder fault; 6: MOSFET over-temperature fault; 7: motor stall
    int get_error(int id);

    // get pointer to most recent motor data, print warning if id is not in the map
    motor_data* const get_motor_data_reference(int id);

    // convert motor current to motor torque and vice versa
    float current_to_torque(float current);
    float torque_to_current(float torque);

    private:
	// reference to mcp can connection
    MCP_CAN &_can_connection;
    // map of structs that holds the most recent motor data, key: id; value: motor_data struct
    std::map<int, motor_data> motors;
    // helper to send CAN message
    void send_CAN_message(uint32_t id, bool extended, byte data[8], uint8_t len);
};

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index);

#endif