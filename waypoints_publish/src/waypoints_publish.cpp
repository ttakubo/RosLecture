#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>
#include <math.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>//for visual marker on rviz

#define MAKE_ORIENTATION

template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}

class Waypoints_publish {
  
private :
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  std::vector<geometry_msgs::PoseStamped>::iterator current_waypoint_;
  geometry_msgs::Pose finish_pose_;
  ros::Rate rate;
  ros::Publisher marker_pub_;//marker for rviz
  
  
public :
  Waypoints_publish() :
    move_base_action_("move_base", true),
    rate(10)
  {
    
    /* connect move_base */
    while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
      {
 ROS_INFO("Waiting for the move_base action");
      }

    ros::NodeHandle nh;
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
    
    /* read .yaml */
    ros::NodeHandle private_nh("~");
    std::string filename = "";
    private_nh.param("filename", filename, filename);
    if(filename != ""){
      ROS_INFO_STREAM("Read waypoints data from " << filename);
      if(!readFile(filename)) {
 ROS_ERROR("Failed loading waypoints file");
      }
      current_waypoint_ = waypoints_.begin();
    } else {
      ROS_ERROR("waypoints file doesn't have name");
    }
    
  }
  
  
  void run() {
    
    bool has_activate_ = true;
    
    ros::Rate rate(10);
    
    while(ros::ok() && has_activate_){
      has_activate_ = sendPoint();
      while(!navigationFinished() && ros::ok()) rate.sleep();
      publishMarkers();
    }
    
    return;
  }
  
  
private :
  bool readFile(const std::string &filename){
    waypoints_.clear();
    try{
      std::ifstream ifs(filename.c_str(), std::ifstream::in);
      if(ifs.good() == false){
 return false;
      }
      
      YAML::Node node;
      
      node = YAML::Load(ifs);

      const YAML::Node &wp_node_tmp = node["waypoints"];
      const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
      
      if(wp_node != NULL){
 for(int i=0; i < wp_node->size(); i++){
   geometry_msgs::PoseStamped pose;
   
   (*wp_node)[i]["position"]["x"] >> pose.pose.position.x;
   (*wp_node)[i]["position"]["y"] >> pose.pose.position.y;
   (*wp_node)[i]["position"]["z"] >> pose.pose.position.z;
   (*wp_node)[i]["position"]["qx"] >> pose.pose.orientation.x;
   (*wp_node)[i]["position"]["qy"] >> pose.pose.orientation.y;
   (*wp_node)[i]["position"]["qz"] >> pose.pose.orientation.z;
   (*wp_node)[i]["position"]["qw"] >> pose.pose.orientation.w;
   
   waypoints_.push_back(pose);
   
 }
      }else{
 return false;
      }
      
      const YAML::Node &fp_node_tmp = node["finish_pose"];
      const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;
      
      if(fp_node != NULL){
 (*fp_node)["pose"]["position"]["x"] >> finish_pose_.position.x;
 (*fp_node)["pose"]["position"]["y"] >> finish_pose_.position.y;
 (*fp_node)["pose"]["position"]["z"] >> finish_pose_.position.z;
 
 (*fp_node)["pose"]["orientation"]["x"] >> finish_pose_.orientation.x;
 (*fp_node)["pose"]["orientation"]["y"] >> finish_pose_.orientation.y;
 (*fp_node)["pose"]["orientation"]["z"] >> finish_pose_.orientation.z;
 (*fp_node)["pose"]["orientation"]["w"] >> finish_pose_.orientation.w;
      }else{
 return false;
      }
      
    }catch(YAML::ParserException &e){
      return false;
      
    }catch(YAML::RepresentationException &e){
      return false;
    }
    
    return true;
  }
  
  
  bool sendPoint() {
    
    move_base_msgs::MoveBaseGoal move_base_goal;
    
    move_base_goal.target_pose.header.stamp = ros::Time::now();
    move_base_goal.target_pose.header.frame_id = "/map";
    
    if(current_waypoint_ == waypoints_.end()-1) {

#ifdef MAKE_ORIENTATION
      double goal_direction = atan2(finish_pose_.position.y - current_waypoint_->pose.position.y,
     finish_pose_.position.x - current_waypoint_->pose.position.x);
      move_base_goal.target_pose.pose = current_waypoint_->pose;
      move_base_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_direction);
#else
      move_base_goal.target_pose.pose = current_waypoint_->pose;
#endif

      current_waypoint_++;
      
      ROS_INFO("Sending goal");
      move_base_action_.sendGoal(move_base_goal);
      
    } else if (current_waypoint_ < waypoints_.end() - 1) {

#ifdef MAKE_ORIENTATION      
      double goal_direction = atan2((current_waypoint_+1)->pose.position.y - current_waypoint_->pose.position.y,
     (current_waypoint_+1)->pose.position.x - current_waypoint_->pose.position.x);
      move_base_goal.target_pose.pose = current_waypoint_->pose;
      move_base_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_direction);
#else
      move_base_goal.target_pose.pose = current_waypoint_->pose;
#endif
 
      current_waypoint_++;
      
      ROS_INFO("Sending goal");
      move_base_action_.sendGoal(move_base_goal);
      
    } else {
      
      move_base_goal.target_pose.pose = finish_pose_;
      
      ROS_INFO("Sending goal");
      move_base_action_.sendGoal(move_base_goal);
      
      return false;
    }
    
    return true;
    
  }
  
  
  bool navigationFinished(){
    return move_base_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
  }
  

  void publishMarkers(){
    visualization_msgs::MarkerArray markers_array;
    for(int i=0; i < waypoints_.size(); i++){
      visualization_msgs::Marker marker, label;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.pose.position.z = marker.scale.z / 2.0;
      marker.color.r = 0.8f;
      marker.color.g = 0.2f;
      marker.color.b = 0.2f;
      
      std::stringstream name;
      name << "waypoint " << i;
      marker.ns = name.str();
      marker.id = i;
      marker.pose.position.x = waypoints_[i].pose.position.x;
      marker.pose.position.y = waypoints_[i].pose.position.y;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.color.a = 1.0f;
      markers_array.markers.push_back(marker);
      
      //ROS_INFO_STREAM("waypoints \n" << waypoints_[i]);
    }
    marker_pub_.publish(markers_array);
  }
  
  
};


int main(int argc, char** argv){

    ros::init(argc, argv, "waypoints_publish_tf");

    Waypoints_publish waypoints_publish;

    waypoints_publish.run();

  return 0;
}
