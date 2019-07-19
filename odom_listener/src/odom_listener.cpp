#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h> 
#include <nav_msgs/Odometry.h>

class odom_listener{ 
    private : 
        ros::Subscriber sub; 
        nav_msgs::Odometry odom; 

    public : 
        odom_listener(); 
        void odom_loop(); 

    private : 
        void odomCallback(const nav_msgs::Odometry &odom_msg); 

};


odom_listener::odom_listener(){ 

    ros::NodeHandle n; 
    sub = n.subscribe("odom", 1, &odom_listener::odomCallback, this);

} 

void odom_listener::odom_loop() { 
    ros::Rate loop_rate(20); //20Hzのループ 

    while(ros::ok()) { 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 

    return; 
} 


void odom_listener::odomCallback(const nav_msgs::Odometry &odom_msg) { 

    ROS_INFO("odom : x %lf  : y %lf\n", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y); 

    return; 

} 


int main(int argc, char **argv) { 

    ros::init(argc, argv, "odom_listener"); 
    odom_listener odometry; 
    odometry.odom_loop(); 

    return (0); 

}

