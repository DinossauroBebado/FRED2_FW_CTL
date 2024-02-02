#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>


// Subscribers ------------

//Publisher 


//debug

ros::NodeHandle  nh;

//------------------SUBS--------------------

#define vel_linear_topic "joy/controler/ps4/cmd_vel/linear"
#define vel_angular_topic "joy/controler/ps4/cmd_vel/angular"
#define emergency_break_topic "joy/controler/ps4/break"
#define battery_level_topic "joy/controler/ps4/battery"
#define controler_connected_topic "joy/controler/connected"
#define circle_topic "joy/controler/ps4/circle"
#define cross_topic "joy/controler/ps4/cross"
#define triangle_topic "joy/controler/ps4/triangle"


//-----------------PUBS-------------------------


std_msgs::Int16 cmd_vel_linear_msg ;
ros::Publisher sub_cmd_vel_linear(vel_linear_topic, &cmd_vel_linear_msg);

std_msgs::Int16 cmd_vel_angular_msg ;
ros::Publisher sub_cmd_vel_angular(vel_angular_topic, &cmd_vel_angular_msg);

std_msgs::Int16 emergency_break_msg ;
ros::Publisher sub_emergency_break(emergency_break_topic, &emergency_break_msg);

std_msgs::Int16 battery_level_msg ;
ros::Publisher sub_battery_level(battery_level_topic, &battery_level_msg);

std_msgs::Bool controler_connected_msg ;
ros::Publisher sub_controler_connected(controler_connected_topic, &controler_connected_msg);

std_msgs::Int16 circle_msg ;
ros::Publisher sub_circle(circle_topic, &circle_msg);

std_msgs::Int16 triangle_msg ;
ros::Publisher sub_triangle(triangle_topic, &triangle_msg);

std_msgs::Int16 cross_msg ;
ros::Publisher sub_cross(cross_topic, &cross_msg);


// debug




bool rosConnected(ros::NodeHandle  nh,bool _connect){
    bool connected = nh.connected();
    if(_connect != connected){
        _connect = connected;
        // digitalWrite(LED_BUILD_IN,!connected);
       
    }
    
    return connected;
}


void ros_init(){

  nh.initNode(); 
    
  nh.advertise(sub_cmd_vel_linear);
  nh.advertise(sub_cmd_vel_angular);
  nh.advertise(sub_emergency_break);
  nh.advertise(sub_battery_level);
  nh.advertise(sub_controler_connected);
  nh.advertise(sub_circle);
  nh.advertise(sub_cross);
  nh.advertise(sub_triangle);

}

void ros_loop(int cmd_vel_linear, float cmd_vel_angular, int emergency_break,int triangle,int circle, int battery_level, bool connected){

    //vel linear
    cmd_vel_linear_msg.data = cmd_vel_linear;
    sub_cmd_vel_linear.publish(&cmd_vel_linear_msg);

    //vel angular
    cmd_vel_angular_msg.data = cmd_vel_angular;
    sub_cmd_vel_angular.publish(&cmd_vel_angular_msg);

    //emergency break pressed 
    emergency_break_msg.data = emergency_break;
    sub_emergency_break.publish(&emergency_break_msg);

    circle_msg.data = circle;
    sub_circle.publish(&circle_msg);

    cross_msg.data = emergency_break;
    sub_cross.publish(&cross_msg);

    triangle_msg.data = triangle;
    sub_triangle.publish(&triangle_msg);

    //show current controler level 
    battery_level_msg.data = battery_level;
    sub_battery_level.publish(&battery_level_msg);

    controler_connected_msg.data = connected;
    sub_controler_connected.publish(&controler_connected_msg);

 


}