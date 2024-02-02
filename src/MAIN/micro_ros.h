#include <Arduino.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "config.h"
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>



//------------------SUBS--------------------

#define vel_linear_topic "joy/controler/ps4/cmd_vel/linear"
#define vel_angular_topic "joy/controler/ps4/cmd_vel/angular"
#define emergency_break_topic "joy/controler/ps4/break"
#define battery_level_topic "joy/controler/ps4/battery"
#define controler_connected_topic "joy/controler/connected"
#define circle_topic "joy/controler/ps4/circle"
#define cross_topic "joy/controler/ps4/cross"
#define triangle_topic "joy/controler/ps4/triangle"

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

geometry_msgs__msg__Twist cmd_vel_msg;
rcl_publisher_t ticks_right_t;
std_msgs__msg__Int32 ticks_right_m;

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  for (int i = 0; i<10; i++)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP.restart();
}

void ros_init(){

  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_control_node", "", &support));



  //create publisher for encoder ticks right 
  RCCHECK(rclc_publisher_init_default(
    &ticks_right_t,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    ticks_right_topic));


RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}