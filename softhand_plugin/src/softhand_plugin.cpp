#include <softhand_plugin/softhand_plugin.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> 
#include <sensor_msgs/JointState.h>

using namespace gazebo;
using namespace std;

/*
  TO ADD DESCRIPTION
*/

// Saturation for input torque
double SoftHandPlugin::saturate(double val, double max_val)
{
    if (val > max_val) {
        val = max_val;
    }
    if (val < -max_val) {
        val = -max_val;
    }
    return val;
}

// Update function of the motor class
void SoftHandPlugin::DCMotor::OnUpdate(const double & dT, double & tauMot, const double & tauLoad)
{
    tauMot = saturate(tauMot, tauMax);

    const double K = 1e3;

    if  (pos - w < -tauFric/K){
        w = pos + tauFric/K;
    }
    else if (pos - w > tauFric/K){
        w = pos - tauFric/K;
    }
    // else w = w;

    double tauFricNow = K *(pos - w);

    effort = - D*vel + tauMot + tauLoad + tauFricNow;
    double acc = effort / J;

    vel = vel + acc * dT;

    pos = pos + vel * dT;

    if(pos < minPos){
        pos = minPos;
        vel = 0;
        effort = std::numeric_limits<double>::quiet_NaN();
    }
    else if(pos > maxPos){
        pos = maxPos;
        vel = 0;
        effort = std::numeric_limits<double>::quiet_NaN();
    }
}

// Update function of the PID controller class
void SoftHandPlugin::PIDController::OnUpdate(const double & dT, const double & pos, const double & vel)
{
    double e = posRef - pos;
    double de = velRef - vel;
    errInt += I * e * dT;

    effort = errInt + P * e + D * de;
}

// Subscriber callbacks references
void SoftHandPlugin::getRef_callback(const std_msgs::Float64& val)
{

    ref = val.data;
}

// Subscribers fingers commands (in case of fully actuation modes)
void SoftHandPlugin::getThumbCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_fingers[0] = val;
}
void SoftHandPlugin::getIndexCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_fingers[1] = val;
}
void SoftHandPlugin::getMiddleCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_fingers[2] = val;
}
void SoftHandPlugin::getRingCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_fingers[3] = val;
}
void SoftHandPlugin::getLittleCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_fingers[4] = val;
}

// Subscriber callback for external torque
void SoftHandPlugin::getExtTau_callback(const std_msgs::Float64& e_tau){
    
    ext_tau = e_tau.data;
}

// Function to  retrieve the joint finger position and velocity
void SoftHandPlugin::GetFingersState(){

    int finger_idx = 0;
    for (auto i = fingers_joint.begin(); i < fingers_joint.end(); i++)    // fingers
    {
      int joint_idx = 0;
      // std::cout << "\rFingers n. " << finger_idx << " (" + finger_names[finger_idx] + ")\n";
      // std::cout << " pos = [\t";
      for (auto j = i->begin(); j < i->end(); j++)                        // joints
      {
        q_fingers[finger_idx][joint_idx] = fingers_joint[finger_idx][joint_idx]->Position(0);
        dq_fingers[finger_idx][joint_idx] = fingers_joint[finger_idx][joint_idx]->GetVelocity(0);
        // std::cout << q_fingers[finger_idx][joint_idx] << "\t";
        joint_idx++;
      }
      // std::cout << "\n";
      // std::cout << std::flush;
      finger_idx++;
    }
    // std::cout << std::flush;
}

// Function to set the fingers torque in Gazebo
void SoftHandPlugin::SetFingersTorque(double tauFingers[][4])
{

    int finger_idx = 0;
    for (auto i = fingers_joint.begin(); i < fingers_joint.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                        // joints
      {
        fingers_joint[finger_idx][joint_idx]->SetForce(0, tauFingers[finger_idx][joint_idx]);
        joint_idx++;
      }
      finger_idx++;
    }
}

// Compute current synergies value from the finger positions
double SoftHandPlugin::ComputeActualSyn()
{
    double synActualPos = 0;

    int finger_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                      // joints
      {
        synActualPos += fingers_syn_Rt[finger_idx][joint_idx]*q_fingers[finger_idx][joint_idx];
        joint_idx++;
      }
      finger_idx++;
    }
    return synActualPos;
}

// Soft Synergies control update function
void SoftHandPlugin::OnUpdateSoftSyn(const common::UpdateInfo & info)
{

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // assign reference to synergy motor position
    double qR = ref;

    // compute old elastic torque & update joint effort
    double tauEl[5][4];

    int finger_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                      // joints
      {
        tauEl[finger_idx][joint_idx] = spring_k_each[finger_idx][joint_idx]*(fingers_syn_S[finger_idx][joint_idx]*qR - q_fingers[finger_idx][joint_idx]);
        joint_idx++;
      }
      finger_idx++;
    }
    SetFingersTorque(tauEl);

    // publish relevant topics 
    SoftHandPlugin::Publish(tauEl);
    mot.pos = ComputeActualSyn();
    mot.effort = 0;
    SoftHandPlugin::PublishMotor();

    // update timer
    oldTime = t;
}

// Adaptive Synergies control update function,  with torque reference
void SoftHandPlugin::OnUpdateAdaptSynTau(const common::UpdateInfo & info)
{

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // assign reference to synergy motor torque
    double tauM = ref;
    tauM = saturate(tauM, mot.tauMax);

    // compute old elastic torque & update joint effort
    double tauFing[5][4];

    int finger_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                      // joints
      {
        tauFing[finger_idx][joint_idx] = fingers_syn_Rt[finger_idx][joint_idx]*tauM - spring_k_each[finger_idx][joint_idx]*q_fingers[finger_idx][joint_idx];
        joint_idx++;
      }
      finger_idx++;
    }
    SetFingersTorque(tauFing);

    // publish relevant topics
    mot.pos = ComputeActualSyn();
    mot.effort = tauM;
    SoftHandPlugin::Publish(tauFing);
    SoftHandPlugin::PublishMotor();

    // update timer
    oldTime = t;
}

// Adaptive Synergies control update function, with position reference 
void SoftHandPlugin::OnUpdateAdaptSynPos(const common::UpdateInfo & info)
{

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // update controllers
    ctrl.posRef = saturate(ref, 25); // TODO: understand why we should impose this limit
    ctrl.OnUpdate(dT, mot.pos, mot.vel);
    double synMot = saturate(ctrl.effort, mot.tauMax);
    
    // compute singery value from finger position
    double synActualPos = ComputeActualSyn();

    // compute synergy motor torque
    double tauM = spring_k_tendon*(mot.pos - synActualPos); 
    
    // compute old elastic torque & update joint effort
    double tauFing[5][4];

    int finger_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                      // joints
      {
        tauFing[finger_idx][joint_idx] = fingers_syn_Rt[finger_idx][joint_idx]*tauM - spring_k_each[finger_idx][joint_idx]*q_fingers[finger_idx][joint_idx];
        joint_idx++;
      }
      finger_idx++;
    }
    SetFingersTorque(tauFing);

    //update motor
    mot.OnUpdate(dT, synMot, tauM);

    // publish relevant topics 
    SoftHandPlugin::Publish(tauFing);
    SoftHandPlugin::PublishMotor();

    // update timer
    oldTime = t;
}

// Direct Torque control update function
void SoftHandPlugin::OnUpdateFingTau(const common::UpdateInfo & info)
{

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // compute old elastic torque & update joint effort
    double tauFingers[5][4];
    int finger_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                      // joints
      {
        tauFingers[finger_idx][joint_idx] = cmd_fingers[finger_idx].data[joint_idx];
        joint_idx++;
      }
      finger_idx++;
    }
    SetFingersTorque(tauFingers);

    // publish relevant topics 
    SoftHandPlugin::Publish(tauFingers);

    // update timer
    oldTime = t;  
}

// Position control update function
void SoftHandPlugin::OnUpdateFingPos(const common::UpdateInfo & info)
{

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // compute fingers torque using the PID controllers
    double tauFingers[5][4]; 
    int finger_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                      // joints
      {
        pid_fingers[finger_idx][joint_idx].posRef = cmd_fingers[finger_idx].data[joint_idx];
        pid_fingers[finger_idx][joint_idx].OnUpdate(dT, q_fingers[finger_idx][joint_idx], dq_fingers[finger_idx][joint_idx]);
        tauFingers[finger_idx][joint_idx] = pid_fingers[finger_idx][joint_idx].effort;
        joint_idx++;
      }
      finger_idx++;
    }
    SetFingersTorque(tauFingers);

    // publish relevant topics 
    SoftHandPlugin::Publish(tauFingers);

    // update timer
    oldTime = t;  
}

// Position control update function
void SoftHandPlugin::OnUpdateFingEqPos(const common::UpdateInfo & info)
{

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // compute fingers torque using the PID controllers
    double tauFingers[5][4]; 
    int finger_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                      // joints
      {
        tauFingers[finger_idx][joint_idx] = spring_k_each[finger_idx][joint_idx]*( cmd_fingers[finger_idx].data[joint_idx] - q_fingers[finger_idx][joint_idx] );
        joint_idx++;
      }
      finger_idx++;
    }
    SetFingersTorque(tauFingers);

    // publish relevant topics 
    SoftHandPlugin::Publish(tauFingers);

    // update timer
    oldTime = t;  
}

// Main update function
void SoftHandPlugin::OnUpdate(const common::UpdateInfo & info)
{
    switch(operationMode){
    case(SoftSynergies):              // operationMode = 0
        OnUpdateSoftSyn(info);
        break;
    case(AdaptiveSynergiesTorque):    // operationMode = 1
        OnUpdateAdaptSynTau(info);
        break;
    case(AdaptiveSynergiesPosition):  // operationMode = 2
        OnUpdateAdaptSynPos(info);
        break;
    case(FingerTorques):              // operationMode = 3
        OnUpdateFingTau(info);
        break;
    case(FingerPositions):            // operationMode = 4 
        OnUpdateFingPos(info);
        break;
    case(FingerEqPositions):          // operationMode = 5 
        OnUpdateFingEqPos(info);
        break;
    }
}

// Publish information relative to the fingers
void SoftHandPlugin::Publish(double tauFing[][4])
{
  // Publish whole joint state
  if (flag_pub_state)
  {
    int finger_idx = 0;
    for (auto i = pid_fingers.begin(); i < pid_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                      // joints
      {
        fingers_state[finger_idx].name[joint_idx] = finger_names[finger_idx] + "_" + finger_part_names[joint_idx];
        fingers_state[finger_idx].position[joint_idx] = q_fingers[finger_idx][joint_idx];
        fingers_state[finger_idx].velocity[joint_idx] = dq_fingers[finger_idx][joint_idx];
        fingers_state[finger_idx].effort[joint_idx] = tauFing[finger_idx][joint_idx];
        joint_idx++;
      }
      finger_idx++;
    }
  }
  pub_thumb_state.publish(fingers_state[0]);
  pub_index_state.publish(fingers_state[1]);
  pub_middle_state.publish(fingers_state[2]);
  pub_ring_state.publish(fingers_state[3]);
  pub_little_state.publish(fingers_state[4]);
}

// Publish information relative to the synergy motor
void SoftHandPlugin::PublishMotor()
{
  mot_state.position[0] = mot.pos;
  mot_state.velocity[0] = mot.vel;
  mot_state.effort[0] = mot.effort;
  pub_mot_state.publish(mot_state);
}

// Initialize useful parameters
void SoftHandPlugin::InitParams(sdf::ElementPtr _sdf)
{
  // Retrieve mechanical parameters from tags
  INITIALIZE_PARAMETER_FROM_TAG( double, r_finger, _sdf, "r_finger", 0.0033 ); // radius of the finger pulleys [m] (taken from qbRobotics info!)
  INITIALIZE_PARAMETER_FROM_TAG( double, R_pulley, _sdf, "R_pulley", 0.0053 ); // radius of the motor pulley [m] (taken from qbRobotics info!)
  INITIALIZE_PARAMETER_FROM_TAG( double, n_param, _sdf, "n_param", (4*r_finger)/(2*R_pulley) ); // synergy parameter 

  // Retrieve spring values from tags (if inserted)
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k, _sdf, "spring_k", 0.5 ) // fingers spring rate [N m /rad]
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_tendon, _sdf, "spring_k_tendon", 100 ) // fingers spring rate [N m /rad]
  
  // ------------------------------ spring rates --------------------------------------  
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[0][0], _sdf, "k_thumb_j1", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[0][1], _sdf, "k_thumb_j2", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[0][2], _sdf, "k_thumb_j3", spring_k )

  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[1][0], _sdf, "k_index_j1", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[1][1], _sdf, "k_index_j2", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[1][2], _sdf, "k_index_j3", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[1][3], _sdf, "k_index_j4", spring_k )

  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[2][0], _sdf, "k_middle_j1", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[2][1], _sdf, "k_middle_j2", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[2][2], _sdf, "k_middle_j3", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[2][3], _sdf, "k_middle_j4", spring_k )

  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[3][0], _sdf, "k_ring_j1", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[3][1], _sdf, "k_ring_j2", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[3][2], _sdf, "k_ring_j3", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[3][3], _sdf, "k_ring_j4", spring_k )

  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[4][0], _sdf, "k_little_j1", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[4][1], _sdf, "k_little_j2", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[4][2], _sdf, "k_little_j3", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each[4][3], _sdf, "k_little_j4", spring_k )      
  
  // ------------------- coefficents for R_transp synergy matrix ------------------------
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[0][0], _sdf, "syn_thumb_j1", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[0][1], _sdf, "syn_thumb_j2", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[0][2], _sdf, "syn_thumb_j3", n_param )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][0], _sdf, "syn_index_j1", 0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][1], _sdf, "syn_index_j2", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][2], _sdf, "syn_index_j3", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][3], _sdf, "syn_index_j4", n_param )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][0], _sdf, "syn_middle_j1", 0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][1], _sdf, "syn_middle_j2", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][2], _sdf, "syn_middle_j3", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][3], _sdf, "syn_middle_j4", n_param )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][0], _sdf, "syn_ring_j1", 0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][1], _sdf, "syn_ring_j2", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][2], _sdf, "syn_ring_j3", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][3], _sdf, "syn_ring_j4", n_param )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][0], _sdf, "syn_little_j1", 0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][1], _sdf, "syn_little_j2", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][2], _sdf, "syn_little_j3", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][3], _sdf, "syn_little_j4", n_param )    

  // ------------------------ coefficents for S synergy matrix --------------------------
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[0][0], _sdf, "syn_thumb_j1", 1.3 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[0][1], _sdf, "syn_thumb_j2", 1.3 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[0][2], _sdf, "syn_thumb_j3", 1.0 )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][0], _sdf, "syn_index_j1", 0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][1], _sdf, "syn_index_j2", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][2], _sdf, "syn_index_j3", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][3], _sdf, "syn_index_j4", 1.0 )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][0], _sdf, "syn_middle_j1", 0.1 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][1], _sdf, "syn_middle_j2", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][2], _sdf, "syn_middle_j3", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][3], _sdf, "syn_middle_j4", 1.0 )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][0], _sdf, "syn_ring_j1", 0.2 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][1], _sdf, "syn_ring_j2", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][2], _sdf, "syn_ring_j3", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][3], _sdf, "syn_ring_j4", 1.0 )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][0], _sdf, "syn_little_j1", 0.3 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][1], _sdf, "syn_little_j2", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][2], _sdf, "syn_little_j3", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][3], _sdf, "syn_little_j4", 1.0 )    

  // Maximum motor torque
  INITIALIZE_PARAMETER_FROM_TAG( double, mot.tauMax, _sdf, "tauMax", 1.5);

  // Initialize variables
  // sizing fingers
  fingers_joint.resize(5);        // Gazebo structures for the finger joints
  q_fingers.resize(5);            // positions of the fingers
  dq_fingers.resize(5);           // velocities of the fingers
  fingers_state.resize(5);        // fingers states
  cmd_fingers.resize(5);          // fingers command for fully actiation modes
  pid_fingers.resize(5);          // PID variables

  // sizing joints
  int joints_dim[5] = {3, 4, 4, 4, 4};
  for (int i = 0; i < 5; ++i)
   {
     fingers_joint[i].resize(joints_dim[i]);
     q_fingers[i].resize(joints_dim[i]);
     dq_fingers[i].resize(joints_dim[i]);

     fingers_state[i].name.resize(joints_dim[i]);
     fingers_state[i].position.resize(joints_dim[i]);
     fingers_state[i].velocity.resize(joints_dim[i]);
     fingers_state[i].effort.resize(joints_dim[i]);
     cmd_fingers[i].data.resize(joints_dim[i]);
     pid_fingers[i].resize(joints_dim[i]);         
   } 

  // Synergy motor variables
  mot_state.position.resize(1);
  mot_state.velocity.resize(1);
  mot_state.effort.resize(1);

  // Retrieve PID gains of the finger controllers (used ONLY in fully actuation mode)
  INITIALIZE_PARAMETER_FROM_TAG( double, kP_fing, _sdf, "kP_fing", 10 ) 
  INITIALIZE_PARAMETER_FROM_TAG( double, kI_fing, _sdf, "kI_fing", 0 )  
  INITIALIZE_PARAMETER_FROM_TAG( double, kD_fing, _sdf, "kD_fing", 0.5 )
  // and assign them to all the finger controllers
  int finger_idx = 0;
  for (auto i = pid_fingers.begin(); i < pid_fingers.end(); i++)    // fingers
  {
    int joint_idx = 0;
    for (auto j = i->begin(); j < i->end(); j++)                      // joints
    {
      pid_fingers[finger_idx][joint_idx].P = kP_fing;
      pid_fingers[finger_idx][joint_idx].I = kI_fing;
      pid_fingers[finger_idx][joint_idx].D = kD_fing;
      joint_idx++;
    }
    finger_idx++;
  }
}

// Compose topics' names according to the type of actuators
void SoftHandPlugin::topicNames(std::string ns_name, std::string joint_name)
{
  // Compose string name for the publishers 
  pub_thumb_name = ns_name + "/thumb_state";
  pub_index_name = ns_name + "/index_state";
  pub_middle_name = ns_name + "/middle_state";
  pub_ring_name = ns_name + "/ring_state";
  pub_little_name = ns_name + "/little_state";

  // Compose string name for the state publisher
  link_pub_name = ns_name + "/fingers_state";

  switch(operationMode){
    case(SoftSynergies):
        cmd_ref_name = ns_name + "/synergy_command";
        break;
    case(AdaptiveSynergiesTorque):
        cmd_ref_name = ns_name + "/synergy_tau_command";
        break;
    case(AdaptiveSynergiesPosition):
        cmd_ref_name = ns_name + "/synergy_pos_command";
        break;
    case(FingerTorques):
    case(FingerPositions):
    case(FingerEqPositions):
        cmd_thumb_name = ns_name + "/thumb_command";
        cmd_index_name = ns_name + "/index_command";
        cmd_middle_name = ns_name + "/middle_command";
        cmd_ring_name = ns_name + "/ring_command";
        cmd_little_name = ns_name + "/little_command";
        break;
  }
}

// Main load function of the plugin
void SoftHandPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  int argc = 0;
  char **argv;
  ros::init(argc, argv, "SoftHand_Plugin");

  model = _parent;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // // Retrieve namespace and control mode from a configuration file
  // ros::param::get("namespace", ns_name);
  // ros::param::get("T_sample", T_sample);

  // // Retrieve joint identifier and control mode from urdf tags
  // //act_type =_sdf->GetElement("actuator_type")->Get<string>();
  // flag_pub_el_tau =_sdf->GetElement("pub_eltau")->Get<bool>();
  // flag_pub_state =_sdf->GetElement("pub_state")->Get<bool>();
  // flag_sub_ext_tau =_sdf->GetElement("sub_ext_tau")->Get<bool>();

  // Retrieve operation mode and robot namespace from urdf tags
  operationMode = (OperationModes) _sdf->GetElement("operation_mode")->Get<int>();
  INITIALIZE_PARAMETER_FROM_TAG( std::string, ns_name, _sdf, "namespace", "toBEAssigned" ) // fingers spring rate [N m /rad]

  // Everything-is-fine message
  std::string ok_msg = "SoftHandPlugin on " + ns_name + " started with " + mode_names[operationMode] + " control, mode n. [" + std::to_string(operationMode) + "]!";
  ROS_WARN_STREAM(ok_msg);

  // // Initialize motors, controllers and other parameters
  InitParams(_sdf);

  // Retrieve fingers joint
  int finger_idx = 0;
  for (auto i = fingers_joint.begin(); i < fingers_joint.end(); i++)    // fingers
  {
    int joint_idx = 0;
    std::cout << "Fingers number " << finger_idx << std::endl;
    for (auto j = i->begin(); j < i->end(); j++)                        // joints
    {
      std::string temp_name;
      if (finger_idx == 0 && joint_idx == 2){
        temp_name = ns_name + "_" + finger_names[finger_idx] + "_" + finger_part_names[joint_idx+1] + "_joint";
      }else{
        temp_name = ns_name + "_" + finger_names[finger_idx] + "_" + finger_part_names[joint_idx] + "_joint";
      }
      fingers_joint[finger_idx][joint_idx] = model->GetJoint(temp_name);
      std::cout << "Joints number " << joint_idx << "\t name = " << temp_name << "\n";
      joint_idx++;
    }
    std::cout << std::endl;
    finger_idx++;
  }

  // Compose the topic names
  SoftHandPlugin::topicNames(ns_name, joint_name);

  // Subscribers
  if ( (operationMode == SoftSynergies) ||  (operationMode == AdaptiveSynergiesTorque) ||  (operationMode == AdaptiveSynergiesPosition))
  {
    sub_cmd = n.subscribe(cmd_ref_name, 10, &SoftHandPlugin::getRef_callback, this);
  }

  if ( (operationMode == FingerTorques) ||  (operationMode == FingerPositions) ||  (operationMode == FingerEqPositions) )
  {
    sub_thumb_cmd = n.subscribe(cmd_thumb_name, 10, &SoftHandPlugin::getThumbCmd_callback, this);
    sub_index_cmd = n.subscribe(cmd_index_name, 10, &SoftHandPlugin::getIndexCmd_callback, this);
    sub_middle_cmd = n.subscribe(cmd_middle_name, 10, &SoftHandPlugin::getMiddleCmd_callback, this);
    sub_ring_cmd = n.subscribe(cmd_ring_name, 10, &SoftHandPlugin::getRingCmd_callback, this);
    sub_little_cmd = n.subscribe(cmd_little_name, 10, &SoftHandPlugin::getLittleCmd_callback, this);
  }

  // // Publishers 
  if ( (operationMode == SoftSynergies) || (operationMode == AdaptiveSynergiesPosition) || (operationMode == AdaptiveSynergiesTorque))
  {
      std::string motor_state_name = ns_name + "/motor_state";
      pub_mot_state = n.advertise<sensor_msgs::JointState>(motor_state_name, 500);
  }

  if (flag_pub_state)
  {
    // pubL_state = n.advertise<softhand_plugin::state_info>(link_pub_name, 500);
    pub_thumb_state = n.advertise<sensor_msgs::JointState>(pub_thumb_name, 500);
    pub_index_state = n.advertise<sensor_msgs::JointState>(pub_index_name, 500);
    pub_middle_state = n.advertise<sensor_msgs::JointState>(pub_middle_name, 500);
    pub_ring_state = n.advertise<sensor_msgs::JointState>(pub_ring_name, 500);
    pub_little_state = n.advertise<sensor_msgs::JointState>(pub_little_name, 500);
  }

  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SoftHandPlugin::OnUpdate, this, _1));

}