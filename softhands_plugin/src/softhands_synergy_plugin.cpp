#include <softhands_plugin/softhands_synergy_plugin.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <string>
#include <ros/ros.h>

using namespace gazebo;
using namespace std;

/*


*/

void softhandsSynergyPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  int argc = 0;
  char **argv;
  ros::init(argc, argv, "softhands_synergy_Plugin");

  this->model = _parent;

  // Make sure the ROS node for Gazebo has already been initialized                                                                                    
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Initialize filtered input
  this->hand_syn_filt.data = 0;
  this->hand_syn_filt_pitch.data = 0;

  // Retrieve namespace and filter constant from urdf tags
  this->ns_name =_sdf->GetElement("namespace")->Get<string>();
  this->alpha =_sdf->GetElement("close_rate")->Get<double>();
  this->close_type =_sdf->GetElement("close_type")->Get<string>();

  // Everything-is-fine message
  std::string ok_msg = "Softhands's synergy plugin started!";
  ROS_WARN_STREAM(ok_msg);

  // Compose subscriber string topic name
  cmd_sub_name = ns_name + "/synergy/command";

  // Compose publishers string topic names for each joint of the fingers
  // thumb finger
  cmd_pub_thumb_abd = ns_name + "/thumb_abd_joint/command";
  cmd_pub_thumb_inn = ns_name + "/thumb_inner_joint/command";
  cmd_pub_thumb_out = ns_name + "/thumb_outer_joint/command";
  // index finger
  cmd_pub_index_abd = ns_name + "/index_abd_joint/command";
  cmd_pub_index_inn = ns_name + "/index_inner_joint/command";
  cmd_pub_index_mid = ns_name + "/index_middle_joint/command";
  cmd_pub_index_out = ns_name + "/index_outer_joint/command";
  // middle finger
  cmd_pub_middle_abd = ns_name + "/middle_abd_joint/command";
  cmd_pub_middle_inn = ns_name + "/middle_inner_joint/command";
  cmd_pub_middle_mid = ns_name + "/middle_middle_joint/command";
  cmd_pub_middle_out = ns_name + "/middle_outer_joint/command";
  // ring finger
  cmd_pub_ring_abd = ns_name + "/ring_abd_joint/command";
  cmd_pub_ring_inn = ns_name + "/ring_inner_joint/command";
  cmd_pub_ring_mid = ns_name + "/ring_middle_joint/command";
  cmd_pub_ring_out = ns_name + "/ring_outer_joint/command";
  // little finger
  cmd_pub_little_abd = ns_name + "/little_abd_joint/command";
  cmd_pub_little_inn = ns_name + "/little_inner_joint/command";
  cmd_pub_little_mid = ns_name + "/little_middle_joint/command";
  cmd_pub_little_out = ns_name + "/little_outer_joint/command";

  // Subscribers and Publishers for the joint states and command
  sub = n.subscribe(cmd_sub_name, 10, &softhandsSynergyPlugin::getSyn_callback, this);
  // for thumb finger
  pub_thumb_abd = n.advertise<std_msgs::Float64>(cmd_pub_thumb_abd, 500);
  pub_thumb_inn = n.advertise<std_msgs::Float64>(cmd_pub_thumb_inn, 500);
  pub_thumb_out = n.advertise<std_msgs::Float64>(cmd_pub_thumb_out, 500);
  // for index finger
  pub_index_abd = n.advertise<std_msgs::Float64>(cmd_pub_index_abd, 500);
  pub_index_inn = n.advertise<std_msgs::Float64>(cmd_pub_index_inn, 500);
  pub_index_mid = n.advertise<std_msgs::Float64>(cmd_pub_index_mid, 500);
  pub_index_out = n.advertise<std_msgs::Float64>(cmd_pub_index_out, 500);
  // for middle finger
  pub_middle_abd = n.advertise<std_msgs::Float64>(cmd_pub_middle_abd, 500);
  pub_middle_inn = n.advertise<std_msgs::Float64>(cmd_pub_middle_inn, 500);
  pub_middle_mid = n.advertise<std_msgs::Float64>(cmd_pub_middle_mid, 500);
  pub_middle_out = n.advertise<std_msgs::Float64>(cmd_pub_middle_out, 500);
  // for ring finger
  pub_ring_abd = n.advertise<std_msgs::Float64>(cmd_pub_ring_abd, 500);
  pub_ring_inn = n.advertise<std_msgs::Float64>(cmd_pub_ring_inn, 500);
  pub_ring_mid = n.advertise<std_msgs::Float64>(cmd_pub_ring_mid, 500);
  pub_ring_out = n.advertise<std_msgs::Float64>(cmd_pub_ring_out, 500);
  // for little finger
  pub_little_abd = n.advertise<std_msgs::Float64>(cmd_pub_little_abd, 500);
  pub_little_inn = n.advertise<std_msgs::Float64>(cmd_pub_little_inn, 500);
  pub_little_mid = n.advertise<std_msgs::Float64>(cmd_pub_little_mid, 500);
  pub_little_out = n.advertise<std_msgs::Float64>(cmd_pub_little_out, 500);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&softhandsSynergyPlugin::OnUpdate, this, _1));  

}

// Subscriber callbacks references
void softhandsSynergyPlugin::getSyn_callback(const std_msgs::Float64& val)
{
    this->hand_syn = val;
}

// Simple low pass filter for the synergy input
void softhandsSynergyPlugin::lpFilter(double data, double alpha)
{
  this->hand_syn_filt.data = this->hand_syn_filt.data - alpha*(this->hand_syn_filt.data - data);
  this->hand_syn_filt_pitch.data = this->hand_syn_filt_pitch.data - 2*alpha*(this->hand_syn_filt_pitch.data - data);
}

// Main update function
void softhandsSynergyPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
 
  softhandsSynergyPlugin::lpFilter(this->hand_syn.data, this->alpha);

  // Compute joint values and assign to variables to be published
  if (this->close_type == "power")
  {
    // for thumb finger
    this->thumb_abd.data = this->hand_syn_filt.data*thumb_syn[0];
    this->thumb_inn.data = this->hand_syn_filt.data*thumb_syn[1];
    this->thumb_out.data = this->hand_syn_filt.data*thumb_syn[2];
    // for index finger
    this->index_abd.data = this->hand_syn_filt.data*index_syn[0];
    this->index_inn.data = this->hand_syn_filt.data*index_syn[1];
    this->index_mid.data = this->hand_syn_filt.data*index_syn[2];
    this->index_out.data = this->hand_syn_filt.data*index_syn[3];
    // for middle finger
    this->middle_abd.data = this->hand_syn_filt.data*middle_syn[0];
    this->middle_inn.data = this->hand_syn_filt.data*middle_syn[1];
    this->middle_mid.data = this->hand_syn_filt.data*middle_syn[2];
    this->middle_out.data = this->hand_syn_filt.data*middle_syn[3];
    // for ring finger
    this->ring_abd.data = this->hand_syn_filt.data*ring_syn[0];
    this->ring_inn.data = this->hand_syn_filt.data*ring_syn[1];
    this->ring_mid.data = this->hand_syn_filt.data*ring_syn[2];
    this->ring_out.data = this->hand_syn_filt.data*ring_syn[3];
    // for little finger
    this->little_abd.data = this->hand_syn_filt.data*little_syn[0];
    this->little_inn.data = this->hand_syn_filt.data*little_syn[1];
    this->little_mid.data = this->hand_syn_filt.data*little_syn[2];
    this->little_out.data = this->hand_syn_filt.data*little_syn[3];
  }
  else if (this->close_type == "pinch")
  {
    double scale_thumb_4_pitch_closure = 0.7;
    // for thumb finger
    this->thumb_abd.data = this->hand_syn_filt.data*thumb_syn[0]*scale_thumb_4_pitch_closure;
    this->thumb_inn.data = this->hand_syn_filt.data*thumb_syn[1]*scale_thumb_4_pitch_closure;
    this->thumb_out.data = this->hand_syn_filt.data*thumb_syn[2];
    // for index finger
    this->index_abd.data = this->hand_syn_filt.data*index_syn[0];
    this->index_inn.data = this->hand_syn_filt.data*index_syn[1];
    this->index_mid.data = this->hand_syn_filt.data*index_syn[2];
    this->index_out.data = this->hand_syn_filt.data*index_syn[3];
    // for middle finger
    this->middle_abd.data = this->hand_syn_filt_pitch.data*middle_syn[0];
    this->middle_inn.data = this->hand_syn_filt_pitch.data*middle_syn[1];
    this->middle_mid.data = this->hand_syn_filt_pitch.data*middle_syn[2];
    this->middle_out.data = this->hand_syn_filt_pitch.data*middle_syn[3];
    // for ring finger
    this->ring_abd.data = this->hand_syn_filt_pitch.data*ring_syn[0];
    this->ring_inn.data = this->hand_syn_filt_pitch.data*ring_syn[1];
    this->ring_mid.data = this->hand_syn_filt_pitch.data*ring_syn[2];
    this->ring_out.data = this->hand_syn_filt_pitch.data*ring_syn[3];
    // for little finger
    this->little_abd.data = this->hand_syn_filt_pitch.data*little_syn[0];
    this->little_inn.data = this->hand_syn_filt_pitch.data*little_syn[1];
    this->little_mid.data = this->hand_syn_filt_pitch.data*little_syn[2];
    this->little_out.data = this->hand_syn_filt_pitch.data*little_syn[3];
  }

  // Publish values
  // for thumb finger
  pub_thumb_abd.publish(this->thumb_abd);
  pub_thumb_inn.publish(this->thumb_inn);
  pub_thumb_out.publish(this->thumb_out);
  // for index finger
  pub_index_abd.publish(this->index_abd);
  pub_index_inn.publish(this->index_inn);
  pub_index_mid.publish(this->index_mid);
  pub_index_out.publish(this->index_out);
  // for middle finger
  pub_middle_abd.publish(this->middle_abd);
  pub_middle_inn.publish(this->middle_inn);
  pub_middle_mid.publish(this->middle_mid);
  pub_middle_out.publish(this->middle_out);
  // for ring finger
  pub_ring_abd.publish(this->ring_abd);
  pub_ring_inn.publish(this->ring_inn);
  pub_ring_mid.publish(this->ring_mid);
  pub_ring_out.publish(this->ring_out);
  // for little finger
  pub_little_abd.publish(this->little_abd);
  pub_little_inn.publish(this->little_inn);
  pub_little_mid.publish(this->little_mid);
  pub_little_out.publish(this->little_out);

}