#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/bool.h>
#include <Arduino.h>


#define LED_PIN 2
#define JOY_MSG_SIZE 4  // Number of elements in the axes array
#define BOTTON_MSG_SIZE 4


//------------------PUBS--------------------

#define joy_topic "joy/controller/ps4"
#define battery_level_topic "joy/controller/ps4/battery"
#define controler_connected_topic "joy/controller/connected"


rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t joy_publisher;
sensor_msgs__msg__Joy joy_msg;

rcl_publisher_t bat_publisher ; 
std_msgs__msg__Int16 bat_msg;

rcl_publisher_t connect_publisher ; 
std_msgs__msg__Bool connect_msg;



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
  
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "joy_node", "", &support));


    RCCHECK(rclc_publisher_init_default(
      &bat_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      battery_level_topic));

    bat_msg.data = 0 ;

    RCCHECK(rclc_publisher_init_default(
      &connect_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      controler_connected_topic));

    connect_msg.data = 0 ;

    // Create Joy publisher
    RCCHECK(rclc_publisher_init_default(
        &joy_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        joy_topic));

    // Initialize the Joy message
    // joy_msg.header.frame_id.data = "joy_frame";
    joy_msg.header.frame_id.size = 20;  // Length of the string "joy_frame"
    joy_msg.axes.size = JOY_MSG_SIZE;
    joy_msg.axes.data = (float *)malloc(sizeof(float) * JOY_MSG_SIZE);
    joy_msg.buttons.size = BOTTON_MSG_SIZE;
    joy_msg.buttons.data = (int32_t *)malloc(sizeof(int32_t) * BOTTON_MSG_SIZE);

    // Set up Joy message values (example values)
    for (size_t i = 0; i < JOY_MSG_SIZE; ++i) {
        joy_msg.axes.data[i] = 0;  // Example: Assign values to axes
        joy_msg.buttons.data[i] = 0;  // Example: Assign binary values to buttons
    }

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
   

    // Wait for everything to initialize
    delay(1000);
}

void ros_loop( int cmd_vel_linear, int  cmd_vel_angular, int emergency_break,int triangle,int circle, int battery_level, bool connected){
  
    bat_msg.data = battery_level*10;

    connect_msg.data = connected;

    // JOYSTICKS 
    joy_msg.axes.data[0] = cmd_vel_linear; //cmd_vel_linear ; //Y left stick 
    joy_msg.axes.data[1] = 0 ; // //X left stick 
    joy_msg.axes.data[2] = cmd_vel_angular ;//cmd_vel_angular; //Y right stick 
    joy_msg.axes.data[3] = 0 ;//X right stick 

    // BOTTONS  

    joy_msg.buttons.data[0] = emergency_break ; //emergency_break; // X 
    joy_msg.buttons.data[1] = circle ; //circle; //  O 
    joy_msg.buttons.data[2] = triangle ;//triangle;// triangulo 
    joy_msg.buttons.data[3] = 0 ; // no button
    for (size_t j = 3; j < BOTTON_MSG_SIZE; ++j) {
        
        joy_msg.buttons.data[j] = 0;    
    }

    RCCHECK(rcl_publish(&connect_publisher, &connect_msg, NULL));

    RCCHECK(rcl_publish(&joy_publisher, &joy_msg, NULL));

    RCCHECK(rcl_publish(&bat_publisher, &bat_msg, NULL));

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(100);
    
}