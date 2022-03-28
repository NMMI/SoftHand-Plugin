#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo{
  
  class softhandsSynergyPlugin : public ModelPlugin {
  
  public:
    // Constructor
    softhandsSynergyPlugin() : ModelPlugin(){}

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void OnUpdate(const common::UpdateInfo & /*_info*/);

    void getSyn_callback(const std_msgs::Float64& val);

    void lpFilter(double data, double alpha);

  private:

    // Command name to create a specific publisher and subscribers
    std::string cmd_sub_name;
    // for thumb finger
    std::string cmd_pub_thumb_abd, cmd_pub_thumb_inn, cmd_pub_thumb_out;
    // for index finger
    std::string cmd_pub_index_abd, cmd_pub_index_inn, cmd_pub_index_mid, cmd_pub_index_out;
    // for middle finger
    std::string cmd_pub_middle_abd, cmd_pub_middle_inn, cmd_pub_middle_mid, cmd_pub_middle_out;
    // for ring finger
    std::string cmd_pub_ring_abd, cmd_pub_ring_inn, cmd_pub_ring_mid, cmd_pub_ring_out;
    // for little finger
     std::string cmd_pub_little_abd, cmd_pub_little_inn, cmd_pub_little_mid, cmd_pub_little_out;
        
    // Namespace retrieved from the urdf
    std::string ns_name;
    std::string close_type;

    // Pointer to the model 
    physics::ModelPtr model;

    // Pointer to output shaft joint 
    physics::JointPtr joint;

    // Pointer to the update event connection 
    event::ConnectionPtr updateConnection;    

    // Define publisher and subscriber
    ros::Subscriber sub;                // for synergy input
    // for thumb finger
    ros::Publisher pub_thumb_abd;       
    ros::Publisher pub_thumb_inn;
    ros::Publisher pub_thumb_out;
    // for index finger    
    ros::Publisher pub_index_abd;       
    ros::Publisher pub_index_inn;
    ros::Publisher pub_index_mid;
    ros::Publisher pub_index_out;
    // for middle finger
    ros::Publisher pub_middle_abd;       
    ros::Publisher pub_middle_inn;
    ros::Publisher pub_middle_mid;
    ros::Publisher pub_middle_out;
    // for ring finger  
    ros::Publisher pub_ring_abd;       
    ros::Publisher pub_ring_inn;
    ros::Publisher pub_ring_mid;
    ros::Publisher pub_ring_out; 
    // for little finger  
    ros::Publisher pub_little_abd;       
    ros::Publisher pub_little_inn;
    ros::Publisher pub_little_mid;
    ros::Publisher pub_little_out; 

    ros::NodeHandle n;
    
    // Reference messages
    std_msgs::Float64 hand_syn;
    std_msgs::Float64 hand_syn_filt, hand_syn_filt_pitch;
    double alpha;

    // Fingers command variables
    // for thumb finger
    std_msgs::Float64 thumb_abd, thumb_inn, thumb_out;
    // for index finger
    std_msgs::Float64 index_abd, index_inn, index_mid, index_out;
    // for middle finger
    std_msgs::Float64 middle_abd, middle_inn, middle_mid, middle_out;
    // for ring finger
    std_msgs::Float64 ring_abd, ring_inn, ring_mid, ring_out;
    // for little finger
    std_msgs::Float64 little_abd, little_inn, little_mid, little_out;

    // Define the synergies gain values for the four fingers (taken from pisa-iit-softhand original package)
    double thumb_syn[3] = {1.3, 1.3, 1.0};
    double index_syn[4] = {0.0, 1.0, 1.0, 1.0};
    double middle_syn[4] = {0.1, 1.0, 1.0, 1.0};
    double ring_syn[4] = {0.2, 1.0, 1.0, 1.0};
    double little_syn[4] = {0.3, 1.0, 1.0, 1.0};

  };

  GZ_REGISTER_MODEL_PLUGIN(softhandsSynergyPlugin)
}