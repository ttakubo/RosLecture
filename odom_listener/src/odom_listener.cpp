#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <vector>
#include <fstream>
#include <string>

class odom_listener{ 
private : 
  ros::Subscriber sub; 
  nav_msgs::Odometry odom; 
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped finish_pose_;
  geometry_msgs::Pose pre_pose;
  std::string filename_;
  std::string world_frame_;
  int start;

public : 
  odom_listener(); 
  void odom_loop(); 
  void save();        
  
private : 
  void odomCallback(const nav_msgs::Odometry &odom_msg); 
  
};

odom_listener::odom_listener():filename_("waypoints.yaml"){ 

    ros::NodeHandle n; 
    sub = n.subscribe("odom", 1, &odom_listener::odomCallback, this);

    start=0;
    pre_pose.position.x = 0.0;
    pre_pose.position.y = 0.0;
    pre_pose.position.z = 0.0;
    pre_pose.orientation.x = 0.0;
    pre_pose.orientation.y = 0.0;
    pre_pose.orientation.z = 0.0;
    pre_pose.orientation.w = 0.0;

} 

void odom_listener::odom_loop() { 

  ros::Rate loop_rate(2); //2Hz 
  
  while(ros::ok()) { 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 

  if(start != 0){  //save waypoints
    ROS_INFO_STREAM("start write");
    save();
  }
  
    return; 

} 

void odom_listener::odomCallback(const nav_msgs::Odometry &odom_msg) { 

  ROS_INFO("odom : x %lf  : y %lf\n", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y); 

  if(pow(odom_msg.pose.pose.position.x - pre_pose.position.x, 2.0) + pow(odom_msg.pose.pose.position.y - pre_pose.position.y, 2.0) + pow(odom_msg.pose.pose.position.z - pre_pose.position.z, 2.0) >= 0.5) {// check distance

    if(start==0) start=1;
    waypoints.push_back(odom_msg.pose.pose);//collect position

    ROS_INFO_STREAM("save current position");

    pre_pose.position.x = odom_msg.pose.pose.position.x;
    pre_pose.position.y = odom_msg.pose.pose.position.y;
    pre_pose.position.z = odom_msg.pose.pose.position.z;
    pre_pose.orientation.x = odom_msg.pose.pose.orientation.x;
    pre_pose.orientation.y = odom_msg.pose.pose.orientation.y;
    pre_pose.orientation.z = odom_msg.pose.pose.orientation.z;
    pre_pose.orientation.w = odom_msg.pose.pose.orientation.w;
  }

  // update finish position
  finish_pose_.header.seq = 0;
  //finish_pose_.header.stamp = 0.0;//pre_timestamp;
  finish_pose_.header.frame_id = world_frame_;
  finish_pose_.pose.position.x = odom_msg.pose.pose.position.x;
  finish_pose_.pose.position.y = odom_msg.pose.pose.position.y;
  finish_pose_.pose.position.z = odom_msg.pose.pose.position.z;
  finish_pose_.pose.orientation.x = odom_msg.pose.pose.orientation.x;
  finish_pose_.pose.orientation.y = odom_msg.pose.pose.orientation.y;
  finish_pose_.pose.orientation.z = odom_msg.pose.pose.orientation.z;
  finish_pose_.pose.orientation.w = odom_msg.pose.pose.orientation.w;
  
  return; 

} 

void odom_listener::save(){// save waypoints
  std::ofstream ofs(filename_.c_str(), std::ios::out);
  
  ofs << "waypoints:" << std::endl;
  for(int i=0; i < waypoints.size(); i++){
    ofs << "    " << "- position:" << std::endl;
    ofs << "        x: " << waypoints[i].position.x << std::endl;
    ofs << "        y: " << waypoints[i].position.y << std::endl;
    ofs << "        z: " << waypoints[i].position.z << std::endl;
    ofs << "        qx: "<< waypoints[i].orientation.x << std::endl;
    ofs << "        qy: "<< waypoints[i].orientation.y << std::endl;
    ofs << "        qz: "<< waypoints[i].orientation.z << std::endl;
    ofs << "        qw: "<< waypoints[i].orientation.w << std::endl;
  }
  
  ofs << "finish_pose:"           << std::endl;
  ofs << "    header:"            << std::endl;
  ofs << "        seq: "          << finish_pose_.header.seq << std::endl;
  ofs << "        stamp: "        << finish_pose_.header.stamp << std::endl;
  ofs << "        frame_id: "     << finish_pose_.header.frame_id << std::endl;;
  ofs << "    pose:"              << std::endl;
  ofs << "        position:"      << std::endl;
  ofs << "            x: "        << finish_pose_.pose.position.x << std::endl;
  ofs << "            y: "        << finish_pose_.pose.position.y << std::endl;
  ofs << "            z: "        << finish_pose_.pose.position.z << std::endl;
  ofs << "        orientation:"   << std::endl;
  ofs << "            x: "        << finish_pose_.pose.orientation.x << std::endl;
  ofs << "            y: "        << finish_pose_.pose.orientation.y << std::endl;
  ofs << "            z: "        << finish_pose_.pose.orientation.z << std::endl;
  ofs << "            w: "        << finish_pose_.pose.orientation.w << std::endl;
  
  ofs.close();
  
  std::cout << "write success"<<std::endl;

}


int main(int argc, char **argv) { 

    ros::init(argc, argv, "odom_listener"); 
    odom_listener odometry; 
    odometry.odom_loop(); 

    return (0); 

}
