//
// /*
//  * Sensor_values.h
//  *
//  * Created: 4/20/2017 3:58:01 PM
//  *  Author: gusst967
//  */ 


#ifndef SENSOR_VALUES_H_
#define SENSOR_VALUES_H_

extern int left_distance;
extern int right_distance;
extern bool right_side_detected;
extern bool left_side_detected;
extern uint8_t angle;
extern uint8_t gyro_rotation_speed;
extern float front_distance;
extern float travel_distance;
extern uint16_t wheel_sensor_counter;
extern uint8_t standing_still_counter;
extern uint8_t velocity;
extern uint8_t LIDAR_angle;
extern uint8_t LIDAR_rotation_speed;
extern uint8_t LIDAR_rotation_turns;
extern bool autonomous;

void IR_conversion(bool, uint8_t);

void Left_side_detectable();

void Right_side_detectable();

void Set_front_distance(uint16_t);

void Distance_calculation(uint8_t);

void Distance_travelled();

void Calculate_wheel_sensor_counter(uint8_t);

bool Standing_still();

#endif /* SENSOR_VALUES_H_ */