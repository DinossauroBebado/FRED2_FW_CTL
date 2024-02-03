#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joy.h>
#include <Arduino.h>

rclc_executor_t executor;
rcl_node_t node;
rcl_publisher_t joy_publisher;
sensor_msgs__msg__Joy joy_msg;

#define JOY_MSG_SIZE 4  // Number of elements in the axes array
#define BOTTON_MSG_SIZE 12
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

#define LED_PIN 2

void error_loop(){
  for (int i = 0; i<10; i++)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP.restart();

}

// void joy_callback()
// {
//     // This callback is called when a new Joy message is received
//     const sensor_msgs__msg__Joy *msg = (const sensor_msgs__msg__Joy *)msgin;

//     // Process the Joy message data as needed
//     // For example, you can print the values of axes and buttons:
//     for (size_t i = 0; i < msg->axes.size; ++i) {
//         Serial.print("Axis ");
//         Serial.print(i);
//         Serial.print(": ");
//         Serial.println(msg->axes.data[i]);
//     }

//     for (size_t i = 0; i < msg->buttons.size; ++i) {
//         Serial.print("Button ");
//         Serial.print(i);
//         Serial.print(": ");
//         Serial.println(msg->buttons.data[i]);
//     }
// }

void setup()
{
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
int h = 0 ;
float H = 0.0;

void loop()

{   // Set up Joy message values (example values)
    h = h + 1 ; 
    H = H + 0.1; 
    for (size_t i = 0; i < JOY_MSG_SIZE; ++i) {
        joy_msg.axes.data[i] = H;  // Example: Assign values to axes
        joy_msg.buttons.data[i] = h;  // Example: Assign binary values to buttons
    }

    // Publish the Joy message periodically
    RCCHECK(rcl_publish(&joy_publisher, &joy_msg, NULL));
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(100);

}
