#include <Arduino.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "config.h"
#include <sensor_msgs/msg/joy.h>
#include <Arduino.h>



//------------------SUBS--------------------

#define joy_topic "joy/controler/ps4"
#define battery_level_topic "joy/controler/ps4/battery"
#define controler_connected_topic "joy/controler/connected"


rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t joy_publisher;
sensor_msgs__msg__Joy joy_msg;

#define LED_PIN 2
#define JOY_MSG_SIZE 4  // Number of elements in the axes array
#define BOTTON_MSG_SIZE 12

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

  // Initialize Micro-ROS transports
    set_microros_transports();

    // Initialize ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "joy_node", "", &support));

    // Create Joy publisher
    RCCHECK(rclc_publisher_init_default(
        &joy_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        "joy"));

    // Initialize the Joy message
    joy_msg.header.frame_id.data = "joy_frame";
    joy_msg.header.frame_id.size = 20;  // Length of the string "joy_frame"
    joy_msg.axes.size = JOY_MSG_SIZE;
    joy_msg.axes.data = (float *)malloc(sizeof(float) * JOY_MSG_SIZE);
    joy_msg.buttons.size = BOTTON_MSG_SIZE;
    joy_msg.buttons.data = (int32_t *)malloc(sizeof(int32_t) * BOTTON_MSG_SIZE);

    // Set up Joy message values (example values)
    for (size_t i = 0; i < JOY_MSG_SIZE; ++i) {
        joy_msg.axes.data[i] = 0.1 * i;  // Example: Assign values to axes
        joy_msg.buttons.data[i] = i % 2;  // Example: Assign binary values to buttons
    }


    for (size_t j = 0; j < BOTTON_MSG_SIZE; ++j) {
        
        joy_msg.buttons.data[j] = j % 2;  // Example: Assign binary values to buttons
    }

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
   

    // Wait for everything to initialize
    delay(1000);
}

void ros_loop(int cmd_vel_linear, float cmd_vel_angular, int emergency_break,int triangle,int circle, int battery_level, bool connected){
    // This callback is called when a new Joy message is received
    const sensor_msgs__msg__Joy *msg = (const sensor_msgs__msg__Joy *)msgin;

    // Process the Joy message data as needed
    // For example, you can print the values of axes and buttons:
    joy_msg.axes.data[0] = cmd_vel_linear ;
    joy_msg.axes.data[1] = 0 ; 
    joy_msg.axes.data[2] = 0 ;
    joy_msg.axes.data[3] = 0 ;

    for (size_t i = 0; i < msg->buttons.size; ++i) {
        Serial.print("Button ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(msg->buttons.data[i]);
    }
}