#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> 
#include <sensor_msgs/JointState.h>

#define INITIALIZE_PARAMETER_FROM_TAG( TYPE, VAR, SDF_NAME, NAME, DEF_VALUE ) if ((SDF_NAME)->HasElement(NAME)) {(VAR) = ((SDF_NAME)->GetElement((NAME))->Get<TYPE>()); }else{ (VAR) = (DEF_VALUE); }

namespace gazebo{

  class SoftHandMimicPlugin : public ModelPlugin {

    class DCMotor{

      public:
        DCMotor(const double & _J = 0.0233, //kg m^2
                const double & _D = 0.2698, //N m /(m/s)
                const double & _tauMax = 6.0, // N m
                const double & _maxVel = 6.0, // rad/s
                const double & _minPos = -std::numeric_limits<double>::infinity(),
                const double & _maxPos = std::numeric_limits<double>::infinity(),
                const double & _tauFric = 0.5 // N m  (to check)
                ): J(_J), D(_D), tauMax(_tauMax), maxVel(_maxVel), minPos(_minPos), maxPos(_maxPos), tauFric(_tauFric)
                {
                pos = 0.0;
                vel = 0.0;
                w = 0.0;
                };

        // update function to be called once per step
        void OnUpdate(const double & dT, double & tauMot, const double & tauLoad);

        //state
        double pos; // actual output shaft position
        double vel; // actual output shaft velocity
        double w; // friction equilibrium state

        //parameters
        double maxVel; // max output shaft velocity
        double minPos; // min output shaft position
        double maxPos; // max output shaft position
        double tauFric; // output shaft friction

        double J; // motor shaft inertia
        double D; // motor shaft damping
        double tauMax; // max input torque

        double effort; // total effort to the motor output shaft

      };

    class PIDController{

      public:
        PIDController(const double & _P = 10.0,
                const double & _I = 0.0,
                const double & _D = 2.0
                ): P(_P), I(_I), D(_D)
                {
                  effort = 0.0;
                  posRef = 0.0;
                  velRef = 0.0;
                  errInt = 0.0;
                };

        // update function to be called once per step
        void OnUpdate(const double & dT, const double & pos, const double & vel);

        //state
        double effort; // controller output

        double posRef; // position reference
        double velRef; // velocity reference
        double errInt; // error Integral

        //parameters
        double P; // proportional constant
        double I; // integral constant
        double D; // derivative constant

      };

  public:
    // Constructor
    SoftHandMimicPlugin() : ModelPlugin(){} 

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);  

    void OnUpdate(const common::UpdateInfo & info);  

    void getRef_callback(const std_msgs::Float64& val_1);  
    void getThumbCmd_callback(const std_msgs::Float64MultiArray& val);
    void getIndexCmd_callback(const std_msgs::Float64MultiArray& val);
    void getMiddleCmd_callback(const std_msgs::Float64MultiArray& val);
    void getRingCmd_callback(const std_msgs::Float64MultiArray& val);
    void getLittleCmd_callback(const std_msgs::Float64MultiArray& val);
    void getExtTau_callback(const std_msgs::Float64& tauExt); 
    static double saturate(double val, double max_val);

  protected:

    // initialize motors
    void InitParams(sdf::ElementPtr _sdf); // change to change actuator
    
    // motors
    DCMotor mot;
    sensor_msgs::JointState mot_state;

    // controllers
    PIDController ctrl;
    PIDController pid_thumb[3], pid_index[4], pid_middle[4], pid_ring[4], pid_little[4];
    std::vector<std::vector<PIDController>> pid_fingers, pid_fingers_mimic; 

    // status
    common::Time oldTime;
    double ref = 0;
    double ext_tau;
    double T_sample;

    enum OperationModes {SoftSynergies = 0 , AdaptiveSynergiesTorque = 1, AdaptiveSynergiesPosition = 2, FingerTorques = 3, FingerPositions = 4, FingerEqPositions = 5};
    OperationModes operationMode;
    std::vector<std::string> mode_names = {"SoftSynergies", "AdaptiveSynergiesTorque", "AdaptiveSynergiesPosition", "FingerTorques", "FingerPositions", "FingerEquilibriumPositions"};

	  void OnUpdateSoftSyn(const common::UpdateInfo & info);
	  void OnUpdateAdaptSynTau(const common::UpdateInfo & info);
	  void OnUpdateAdaptSynPos(const common::UpdateInfo & info);
	  void OnUpdateFingTau(const common::UpdateInfo & info);
	  void OnUpdateFingPos(const common::UpdateInfo & info);
    void OnUpdateFingEqPos(const common::UpdateInfo & info);

    // helper functions
    void Publish(double tauFing[][4], double tauFingMimic[][3]);
    void PublishMotor();
    void topicNames(std::string ns_name, std::string joint_name);
    void DisplayJointPos();
    void GetFingersState();
	  void SetFingersTorque(double tauFingers[][4], double tauMimicFingers[][3]);
	  void SetPalmTorque(double tauPalm[][2], double tauMimicPalm[][2]);
	  double ComputeActualSyn();
    // ROS handle
    ros::NodeHandle n;

    // publishers
    ros::Publisher pub_mot_state;   // for motor state
    ros::Publisher pub_thumb_state, pub_index_state, pub_middle_state, pub_ring_state, pub_little_state, pub_palm_state;   	// for fingers state

    // subscribers
    ros::Subscriber sub_cmd;  // for references
    ros::Subscriber sub_ext;// for eternal torque
    ros::Subscriber sub_thumb_cmd, sub_index_cmd, sub_middle_cmd, sub_ring_cmd, sub_little_cmd;				// for fingers command (in case of full actuation modes)

    // Pointer to the model
    physics::ModelPtr model;
    physics::LinkPtr link;

    // Pointer to output shaft joint
    std::vector<std::vector<physics::JointPtr>> fingers_joint, fingers_mimic_joint, palm_joint, palm_mimic_joint;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Command name to create a specific publisher and subscribers
    std::string cmd_pub_name, cmd_ref_name;
    std::string cmd_thumb_name, cmd_index_name, cmd_middle_name, cmd_ring_name, cmd_little_name, cmd_palm_name;
    std::string pub_thumb_name, pub_index_name, pub_middle_name, pub_ring_name, pub_little_name, pub_palm_name;

    // String names of the topic from which retrieve the desired motor variables
    std::string link_pub_name;

    // Joint name and namespace retrieved from the urdf
    std::string joint_name;
    std::string ns_name;

    // Enable publish and subscribe to specific topics (default "false")
    bool flag_pub_el_tau = true;
    bool flag_pub_state = true;
    bool flag_sub_ext_tau = true;

    std::vector<std::vector<double>> q_palm, dq_palm;   // thumb (first row), index (second row), middle (third row), ring (forth row), little (fifth row)
    std::vector<std::vector<double>> qMimic_palm, dqMimic_palm;   // thumb (first row), index (second row), middle (third row), ring (forth row), little (fifth row)
    // Fingers state
    std::vector<std::vector<double>> q_fingers, dq_fingers;   // thumb (first row), index (second row), middle (third row), ring (forth row), little (fifth row)
    std::vector<std::vector<double>> qMimic_fingers, dqMimic_fingers;   // thumb (first row), index (second row), middle (third row), ring (forth row), little (fifth row)
    sensor_msgs::JointState thumb_state, index_state, middle_state, ring_state, little_state;
    std::vector<sensor_msgs::JointState> fingers_state;

    // Fingers command vectors
    std::vector<std_msgs::Float64MultiArray> cmd_fingers;

    // Finger string names
    std::vector<std::string> palm_names = {"palm_central"};
    std::vector<std::string> palm_part_names = {"thumb", "little"};

    std::vector<std::string> finger_names = {"thumb", "index", "middle", "ring", "little"};
    std::vector<std::string> finger_part_names = {"knuckle", "proximal", "middle", "distal"};

    // Define the synergy constants for the adaptive case
    double r_finger, R_pulley, n_param;

    double fingers_syn_S[5][4], fingers_syn_Rt[5][4];
    double palm_syn_S[1][2];

    // Elastic value of the spring of each finger (TODO: modify this with different springs values for the fingers)
    double spring_k, spring_k_mimic, spring_k_tendon;
    double spring_k_each[5][4], spring_k_mimic_each[5][4];
    double spring_k_each_palm[1][2], spring_k_mimic_each_palm[1][2];

    // PID gains for each finger (supposed equal)
    double kP_fing, kI_fing, kD_fing;

    // Palm
    bool wide;
    // std::vector<std::string> palm_names = {"thumb", "little"};
    // sensor_msgs::JointState palm_state;

  };

  GZ_REGISTER_MODEL_PLUGIN(SoftHandMimicPlugin)
}
