#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo{
  
  class softhandsPlugin : public ModelPlugin {
  
  public:
    // Constructor
    softhandsPlugin() : ModelPlugin(){}

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void OnUpdate(const common::UpdateInfo & /*_info*/);

    void getRef_callback(const std_msgs::Float64& val);

  private:

    // Command name to create a specific publisher and subscribers
    std::string cmd_sub_name;
    std::string cmd_pub_name;

    // Joint name and namespace retrieved from the urdf
    std::string joint_name;
    std::string ns_name;

    // Pointer to the model 
    physics::ModelPtr model;

    // Pointer to output shaft joint 
    physics::JointPtr joint;

    // Pointer to the update event connection 
    event::ConnectionPtr updateConnection;    

    // Define publisher and subscriber
    ros::Publisher pub;     // for link position
    ros::Subscriber sub;    // for joint position reference
    ros::NodeHandle n;
    
    // Reference messages
    std_msgs::Float64 joint_ref;

    // Temporarly messages to publish
    std_msgs::Float64 joint_tau;

    // Define the joint variables
    double K;         // [Nm/rad]
    double Damp = 0.05;    // [Nms/rad] 
    double q, dq;

  };

  GZ_REGISTER_MODEL_PLUGIN(softhandsPlugin)
}