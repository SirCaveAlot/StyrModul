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
extern int forward_IR_distance;
extern bool right_side_detected;
extern bool left_side_detected;
extern bool forward_IR_detected;
extern int16_t angle;
extern int16_t angle_to_rotate;
extern int16_t gyro_rotation_speed;
extern int32_t distance_until_stop;
extern int32_t stop_distance;
extern int32_t travel_distance;
extern uint16_t wheel_sensor_counter;
extern uint8_t standing_still_counter;
extern uint8_t velocity;
extern uint8_t LIDAR_angle;
extern uint8_t LIDAR_rotation_speed;
extern uint8_t LIDAR_rotated_turns;
extern uint8_t LIDAR_turns;
extern bool line_detected;
extern float iteration_time;
extern bool first_detection;

void IR_conversion(char, uint8_t);

void Left_side_detectable();

void Right_side_detectable();

void Forward_IR_detectable();

void Gyro_calculation();

void Angle_calculation();

void Set_angle_to_rotate(uint8_t);

void Set_rotation_distance(uint8_t);

void Set_LIDAR_turns(uint8_t);

void Set_distance_until_stop(uint8_t);

void Distance_travelled();

void Calculate_wheel_sensor_counter(uint8_t);

bool Standing_still();

void Line_detection(uint8_t);

#endif /* SENSOR_VALUES_H_ */