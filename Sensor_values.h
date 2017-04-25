/*
 * Sensor_values.h
 *
 * Created: 4/20/2017 3:58:01 PM
 *  Author: gusst967
 */ 


#ifndef SENSOR_VALUES_H_
#define SENSOR_VALUES_H_

extern uint8_t left_distance;
extern uint8_t right_distance;
extern uint8_t angle;
extern uint8_t gyro_rotation_speed;
extern uint8_t forward_distance;
extern uint8_t velocity;
extern uint8_t travelled_distance;
extern uint8_t LIDAR_angle;
extern uint8_t LIDAR_rotation_speed;
extern bool autonomous;

void IR_conversion(bool, uint8_t);

//void IR_conversion_right(uint8_t);

bool Left_side_detectable();

bool Right_side_detectable();

#endif /* SENSOR_VALUES_H_ */