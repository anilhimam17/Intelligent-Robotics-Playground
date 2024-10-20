/*
 * File:          epuck_final.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 6.28

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  // Initialising all the Distance Sensors of the Robot
  int i ;
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  
  for (i=0; i<8; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  
  // Initialising all the motors of the robot
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data */
    double ps_values[8];
    for (i=0; i<8; i++) {
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    }
    
    // Obstacle Detection
    bool right_obstacle = 
      ps_values[0] >= 70.0 ||
      ps_values[1] >= 70.0 ||
      ps_values[2] >= 70.0;
      
    bool left_obstacle = 
      ps_values[7] >= 70.0 ||
      ps_values[6] >= 70.0 ||
      ps_values[5] >= 70.0;
      
    // Setting the speed of the motor based on the detection of obstacles
    double right_speed = 0.5 * MAX_SPEED;
    double left_speed = 0.5 * MAX_SPEED;
    
    if (right_obstacle) {
      left_speed -= 0.5 * MAX_SPEED;
      right_speed += 0.5 * MAX_SPEED;
    } 
    else if (left_obstacle) {
      left_speed += 0.5 * MAX_SPEED;
      right_speed -= 0.5 * MAX_SPEED;
    }
    
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
    
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
