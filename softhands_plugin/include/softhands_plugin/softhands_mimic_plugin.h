#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo{
  
  class softhandsPluginMimic : public ModelPlugin {
  
  public:
    // Constructor
    softhandsPluginMimic() : ModelPlugin(){}

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void OnUpdate(const common::UpdateInfo & /*_info*/);

  private:

    // Joint name and namespace retrieved from the urdf
    std::string joint_name;
    std::string ns_name;

    // Pointer to the model 
    physics::ModelPtr model;

    // Pointer to output shaft joint 
    physics::JointPtr joint, joint_mimic;

    // Pointer to the update event connection 
    event::ConnectionPtr updateConnection;    

    // Define node handle
    ros::NodeHandle n;
    
    // Reference messages
    std_msgs::Float64 joint_ref;

    // Temporarly messages to publish
    std_msgs::Float64 joint_tau;

    // Define the joint variables
    double K;         // [Nm/rad]
    double Damp = 0.025;    // [Nms/rad] 
    double q, q_mimic, dq_mimic;

  };

  GZ_REGISTER_MODEL_PLUGIN(softhandsPluginMimic)
}