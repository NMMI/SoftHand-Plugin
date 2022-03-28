#include <softhands_plugin/softhands_mimic_plugin.h>
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

void softhandsPluginMimic::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  int argc = 0;
  char **argv;
  ros::init(argc, argv, "softhands_Plugin_mimic");

  this->model = _parent;

  // Make sure the ROS node for Gazebo has already been initialized                                                                                    
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Retrieve joint identifier and control mode from urdf tags
  this->joint_name =_sdf->GetElement("joint")->Get<string>();
  this->ns_name =_sdf->GetElement("namespace")->Get<string>();
  this->K =_sdf->GetElement("stiffness")->Get<double>();

  // Everything-is-fine message
  std::string ok_msg = "Softhands mimic plugin on " + joint_name + " started!";
  ROS_WARN_STREAM(ok_msg);

  // Retrieve joint
  this->joint = this->model->GetJoint(joint_name);
  // remove the suffix _mimic from the joint name (this supposes you to have the right name for the couple of joints!)
  int len_name = joint_name.length();
  std::string joint_name_wo_mimic = joint_name.erase(len_name-6, 6);
  this->joint_mimic = this->model->GetJoint(joint_name_wo_mimic);

/*  std::cout << "String org " << joint_name << " string erase " << joint_name_wo_mimic << std::endl;
*/
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&softhandsPluginMimic::OnUpdate, this, _1));  

}


// Main update function
void softhandsPluginMimic::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  // Retrieve joint actual position of the original and mimic joints
  this->q = this->joint->GetAngle(0).Radian();
  this->q_mimic = this->joint_mimic->GetAngle(0).Radian();
  this->dq_mimic = this->joint_mimic->GetVelocity(0);

  // Compute the elastic torque
  this->joint_tau.data = this->K*(this->q_mimic - this->q) - this->Damp*this->dq_mimic;

  // Set to the joint elastic torque
  this->joint->SetForce(0, this->joint_tau.data);

/*  std::cout << "Joint " << this->q << "\t Mimic " << this->q_mimic << std::endl;
*/
}
