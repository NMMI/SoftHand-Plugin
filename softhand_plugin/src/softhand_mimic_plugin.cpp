#include <softhand_plugin/softhand_mimic_plugin.h>
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
double SoftHandMimicPlugin::saturate(double val, double max_val)
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
void SoftHandMimicPlugin::DCMotor::OnUpdate(const double & dT, double & tauMot, const double & tauLoad)
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
void SoftHandMimicPlugin::PIDController::OnUpdate(const double & dT, const double & pos, const double & vel)
{
    double e = posRef - pos;
    double de = velRef - vel;
    errInt += I * e * dT;

    effort = errInt + P * e + D * de;
}

// Subscriber callbacks references
void SoftHandMimicPlugin::getRef_callback(const std_msgs::Float64& val)
{

    ref = val.data;
}

// Subscribers fingers commands (in case of fully actuation modes)
void SoftHandMimicPlugin::getThumbCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_fingers[0] = val;
}
void SoftHandMimicPlugin::getIndexCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_fingers[1] = val;
}
void SoftHandMimicPlugin::getMiddleCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_fingers[2] = val;
}
void SoftHandMimicPlugin::getRingCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_fingers[3] = val;
}
void SoftHandMimicPlugin::getLittleCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_fingers[4] = val;
}

// Subscriber callback for external torque
void SoftHandMimicPlugin::getExtTau_callback(const std_msgs::Float64& e_tau){
    
    ext_tau = e_tau.data;
}

// Funcrion to display the joint positions with basic and mimic
void SoftHandMimicPlugin::DisplayJointPos(){
    /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
    // std::cout << "----------------------------- Basic joint ---------------------------" << std::endl;
    int fing_basic_idx = 0;
    for (auto i = fingers_joint.begin(); i < fingers_joint.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                        // joints
      {
        // std::cout << q_fingers[fing_basic_idx][joint_idx];
        // std::cout << "\t";
        joint_idx++;
      }
      fing_basic_idx++;
      // std::cout << std::endl;
    }
    /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
    // std::cout << "----------------------------- Mimic joint ---------------------------" << std::endl;
    int fing_mimic_idx = 0;
    for (auto i = fingers_mimic_joint.begin(); i < fingers_mimic_joint.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                                    // joints
      {
        // std::cout << std::to_string(qMimic_fingers[fing_mimic_idx][joint_idx]);
        // std::cout << "\t";
        joint_idx++;
      }
      fing_mimic_idx++;
      // std::cout << std::endl;
    }
    // std::cout << std::endl;
}

// Function to  retrieve the joint finger position and velocity
void SoftHandMimicPlugin::GetFingersState(){
    /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
    int fing_basic_idx = 0;
    for (auto i = fingers_joint.begin(); i < fingers_joint.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                        // joints
      {
        q_fingers[fing_basic_idx][joint_idx] = fingers_joint[fing_basic_idx][joint_idx]->Position(0);
        dq_fingers[fing_basic_idx][joint_idx] = fingers_joint[fing_basic_idx][joint_idx]->GetVelocity(0);

        joint_idx++;
      }
      fing_basic_idx++;
    }
    /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
    int fing_mimic_idx = 0;
    for (auto i = fingers_mimic_joint.begin(); i < fingers_mimic_joint.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                                    // joints
      {
        qMimic_fingers[fing_mimic_idx][joint_idx] = fingers_mimic_joint[fing_mimic_idx][joint_idx]->Position(0);
        dqMimic_fingers[fing_mimic_idx][joint_idx] = fingers_mimic_joint[fing_mimic_idx][joint_idx]->GetVelocity(0);

        joint_idx++;
      }
      fing_mimic_idx++;
    }

  if(wide){
        /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
    int palm_basic_idx = 0;
    int joint_idx = 0;
    for (int i = 0; i < 2; ++i)                        // joints
    {
      q_palm[palm_basic_idx][joint_idx] = palm_joint[palm_basic_idx][joint_idx]->Position(0);
      dq_palm[palm_basic_idx][joint_idx] = palm_joint[palm_basic_idx][joint_idx]->GetVelocity(0);

      joint_idx++;
    }
    /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
    int palm_mimic_idx = 0;
    int mimic_joint_idx = 0;
    for (int i = 0; i < 2; ++i)                        // joints
    {
      qMimic_palm[0][mimic_joint_idx] = palm_mimic_joint[0][mimic_joint_idx]->Position(0);
      dqMimic_palm[0][mimic_joint_idx] = palm_mimic_joint[0][mimic_joint_idx]->GetVelocity(0);

      mimic_joint_idx++;
    }
  }
}

// Function to set the fingers torque in Gazebo
void SoftHandMimicPlugin::SetFingersTorque(double tauFingers[][4], double tauMimicFingers[][3]){
    /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
    int fing_basic_idx = 0;
    for (auto i = fingers_joint.begin(); i < fingers_joint.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                        // joints
      {
        fingers_joint[fing_basic_idx][joint_idx]->SetForce(0, tauFingers[fing_basic_idx][joint_idx]);
        joint_idx++;
      }
      fing_basic_idx++;
    }
    /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
    int fing_mimic_idx = 0;
    for (auto i = fingers_mimic_joint.begin(); i < fingers_mimic_joint.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                                    // joints
      {
        fingers_mimic_joint[fing_mimic_idx][joint_idx]->SetForce(0, tauMimicFingers[fing_mimic_idx][joint_idx]);
        joint_idx++;
      }
      fing_mimic_idx++;
    }
}



// Function to set the fingers torque in Gazebo
void SoftHandMimicPlugin::SetPalmTorque(double tauPalm[][2], double tauMimicPalm[][2]){
    /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
    int palm_basic_idx = 0;
    for (auto i = palm_joint.begin(); i < palm_joint.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                        // joints
      {
        palm_joint[palm_basic_idx][joint_idx]->SetForce(0, tauPalm[palm_basic_idx][joint_idx]);
        joint_idx++;
      }
      palm_basic_idx++;
    }
    /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
    int palm_mimic_idx = 0;
    for (auto i = palm_mimic_joint.begin(); i < palm_mimic_joint.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                                    // joints
      {
        palm_mimic_joint[palm_mimic_idx][joint_idx]->SetForce(0, tauMimicPalm[palm_mimic_idx][joint_idx]);
        joint_idx++;
      }
      palm_mimic_idx++;
    }
}

// Compute current synergies value from the finger positions
double SoftHandMimicPlugin::ComputeActualSyn(){
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
void SoftHandMimicPlugin::OnUpdateSoftSyn(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // assign reference to synergy motor position
    double qR = ref;

    // Compute coupling torques between basic and mimic joints
    double tauCoupling[5][3];
    for (int i = 0; i < 5; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        tauCoupling[i][j] = spring_k_mimic_each[i][j]*(q_fingers[i][j+1] - qMimic_fingers[i][j]);
      }
    }

    // compute old elastic torque & update joint effort
    double tauEl[5][4], tauEl_mimic[5][3];
    /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
    int fing_basic_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                // joints
      {
        if (joint_idx == 0){          // knuckle element does not have mimic joint!
          tauEl[fing_basic_idx][joint_idx] = spring_k_each[fing_basic_idx][joint_idx]*(fingers_syn_S[fing_basic_idx][joint_idx]*qR - q_fingers[fing_basic_idx][joint_idx]);
        }else{
          tauEl[fing_basic_idx][joint_idx] = spring_k_each[fing_basic_idx][joint_idx]*(fingers_syn_S[fing_basic_idx][joint_idx]*qR - q_fingers[fing_basic_idx][joint_idx]) - tauCoupling[fing_basic_idx][joint_idx-1];  
        }
        
        joint_idx++;
      }
      fing_basic_idx++;
    }
    /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
    int fing_mimic_idx = 0;
    for (auto i = qMimic_fingers.begin(); i < qMimic_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                          // joints
      {
        tauEl_mimic[fing_mimic_idx][joint_idx] = spring_k_each[fing_mimic_idx][joint_idx]*(fingers_syn_S[fing_mimic_idx][joint_idx]*qR - qMimic_fingers[fing_mimic_idx][joint_idx]) + tauCoupling[fing_mimic_idx][joint_idx];
        joint_idx++;
      }
      fing_mimic_idx++;
    }

    //Case for the Wide hand
    if(wide){
      double tauCoupling_palm[1][2];
      for (int i = 0; i < 2; ++i)
      {
        tauCoupling_palm[0][i] = spring_k_mimic_each_palm[0][i]*(q_palm[0][i+1] - qMimic_palm[0][i]);
      }

          // compute old elastic torque & update joint effort
      double tauEl_palm[1][2], tauEl_mimic_palm[1][2];
      int palm_basic_idx = 0;
      int joint_idx = 0;
      for (int i = 0; i < 2; ++i)    // fingers
      {
        // /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
        tauEl_palm[palm_basic_idx][joint_idx] = spring_k_each_palm[palm_basic_idx][joint_idx]*(palm_syn_S[palm_basic_idx][joint_idx]*qR - q_palm[palm_basic_idx][joint_idx]) - tauCoupling_palm[palm_basic_idx][joint_idx-1];  
        // /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
        tauEl_mimic_palm[palm_basic_idx][joint_idx] = spring_k_each_palm[palm_basic_idx][joint_idx]*(palm_syn_S[palm_basic_idx][joint_idx]*qR - qMimic_palm[palm_basic_idx][joint_idx]) + tauCoupling_palm[palm_basic_idx][joint_idx];
        
        joint_idx++;
      }
      SetPalmTorque(tauEl_palm, tauEl_mimic_palm);

    }

    SetFingersTorque(tauEl, tauEl_mimic);
    DisplayJointPos();

    // publish relevant topics 
    Publish(tauEl, tauEl_mimic);
    mot.pos = ComputeActualSyn();
    mot.effort = 0;
    PublishMotor();

    // update timer
    oldTime = t;
}

// Adaptive Synergies control update function,  with torque reference
void SoftHandMimicPlugin::OnUpdateAdaptSynTau(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // assign reference to synergy motor torque
    double tauM = ref;
    tauM = saturate(tauM, mot.tauMax);

    // Compute coupling torques between basic and mimic joints
    double tauCoupling[5][3];
    for (int i = 0; i < 5; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        tauCoupling[i][j] = spring_k_mimic*(q_fingers[i][j+1] - qMimic_fingers[i][j]);
      }
    }

    // compute old elastic torque & update joint effort
    double tauEl[5][4], tauEl_mimic[5][3];
    /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
    int fing_basic_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                // joints
      {
        if (joint_idx == 0){          // knuckle element does not have mimic joint!
          tauEl[fing_basic_idx][joint_idx] = fingers_syn_Rt[fing_basic_idx][joint_idx]*tauM - spring_k_each[fing_basic_idx][joint_idx]*q_fingers[fing_basic_idx][joint_idx];
        }else{
          tauEl[fing_basic_idx][joint_idx] = fingers_syn_Rt[fing_basic_idx][joint_idx]*tauM - spring_k_each[fing_basic_idx][joint_idx]*q_fingers[fing_basic_idx][joint_idx] - tauCoupling[fing_basic_idx][joint_idx-1];  
        }
        
        joint_idx++;
      }
      fing_basic_idx++;
    }
    /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
    int fing_mimic_idx = 0;
    for (auto i = qMimic_fingers.begin(); i < qMimic_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                          // joints
      {
        tauEl_mimic[fing_mimic_idx][joint_idx] = fingers_syn_Rt[fing_mimic_idx][joint_idx]*tauM - spring_k_mimic_each[fing_mimic_idx][joint_idx]*qMimic_fingers[fing_mimic_idx][joint_idx] + tauCoupling[fing_mimic_idx][joint_idx];
        joint_idx++;
      }
      fing_mimic_idx++;
    }
    SetFingersTorque(tauEl, tauEl_mimic);

    // publish relevant topics
    mot.pos = ComputeActualSyn();
    mot.effort = tauM;
    Publish(tauEl, tauEl_mimic);
    PublishMotor();

    // update timer
    oldTime = t;
}

// Adaptive Synergies control update function, with position reference 
void SoftHandMimicPlugin::OnUpdateAdaptSynPos(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // update controllers
    ctrl.posRef = saturate(ref, 7); // TODO: understand why we should impose this limit
    ctrl.OnUpdate(dT, mot.pos, mot.vel);
    double synMot = saturate(ctrl.effort, mot.tauMax);

    // compute singery value from finger position
    double synActualPos = ComputeActualSyn();
    
    // compute synergy motor torque
    double tauM = spring_k_tendon*(mot.pos - synActualPos);

    // Compute coupling torques between basic and mimic joints
    double tauCoupling[5][3];
    for (int i = 0; i < 5; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        tauCoupling[i][j] = spring_k_mimic*(q_fingers[i][j+1] - qMimic_fingers[i][j]);
      }
    }

    // compute old elastic torque & update joint effort
    double tauEl[5][4], tauEl_mimic[5][3];
    /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
    int fing_basic_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                // joints
      {
        if (joint_idx == 0){          // knuckle element does not have mimic joint!
          tauEl[fing_basic_idx][joint_idx] = fingers_syn_Rt[fing_basic_idx][joint_idx]*tauM - spring_k_each[fing_basic_idx][joint_idx]*q_fingers[fing_basic_idx][joint_idx];
        }else{
          tauEl[fing_basic_idx][joint_idx] = fingers_syn_Rt[fing_basic_idx][joint_idx]*tauM/2 - spring_k_each[fing_basic_idx][joint_idx]*q_fingers[fing_basic_idx][joint_idx] - tauCoupling[fing_basic_idx][joint_idx-1];  
        }
        joint_idx++;
      }
      fing_basic_idx++;
    }
    /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
    int fing_mimic_idx = 0;
    for (auto i = qMimic_fingers.begin(); i < qMimic_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                          // joints
      {
        tauEl_mimic[fing_mimic_idx][joint_idx] = fingers_syn_Rt[fing_mimic_idx][joint_idx]*tauM/2 - spring_k_mimic_each[fing_mimic_idx][joint_idx]*qMimic_fingers[fing_mimic_idx][joint_idx] + tauCoupling[fing_mimic_idx][joint_idx];
        joint_idx++;
      }
      fing_mimic_idx++;
    }
    SetFingersTorque(tauEl, tauEl_mimic);

    // publish relevant topics
    //mot.pos = ComputeActualSyn();
    //mot.effort = tauM;
    Publish(tauEl, tauEl_mimic);
    PublishMotor();

    //update motor
    mot.OnUpdate(dT, synMot, tauM);

    // update timer
    oldTime = t;
}

// Direct Torque control update function
void SoftHandMimicPlugin::OnUpdateFingTau(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // Compute coupling torques between basic and mimic joints
    double tauCoupling[5][3];
    for (int i = 0; i < 5; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        tauCoupling[i][j] = spring_k*(q_fingers[i][j+1] - qMimic_fingers[i][j]);
      }
    }

    // compute old elastic torque & update joint effort
    double tauEl[5][4], tauEl_mimic[5][3];
    /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
    int fing_basic_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                // joints
      {
        if (joint_idx == 0){          // knuckle element does not have mimic joint!
          tauEl[fing_basic_idx][joint_idx] = cmd_fingers[fing_basic_idx].data[joint_idx] ;
        }else{
          tauEl[fing_basic_idx][joint_idx] = cmd_fingers[fing_basic_idx].data[joint_idx]/2 - tauCoupling[fing_basic_idx][joint_idx-1];  
        }
        
        joint_idx++;
      }
      fing_basic_idx++;
    }
    /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
    int fing_mimic_idx = 0;
    for (auto i = qMimic_fingers.begin(); i < qMimic_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                          // joints
      {
        tauEl_mimic[fing_mimic_idx][joint_idx] = cmd_fingers[fing_mimic_idx].data[joint_idx+1]/2 + tauCoupling[fing_mimic_idx][joint_idx];
        joint_idx++;
      }
      fing_mimic_idx++;
    }
    SetFingersTorque(tauEl, tauEl_mimic);

    // publish relevant topics 
    Publish(tauEl, tauEl_mimic);

    // update timer
    oldTime = t;  
}

// Position control update function
void SoftHandMimicPlugin::OnUpdateFingPos(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    double tauFingers[5][4], tauFingers_mimic[5][3]; 
    /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
    int fing_basic_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                      // joints
      {
        pid_fingers[fing_basic_idx][joint_idx].posRef = cmd_fingers[fing_basic_idx].data[joint_idx]/2;
        pid_fingers[fing_basic_idx][joint_idx].OnUpdate(dT, q_fingers[fing_basic_idx][joint_idx], dq_fingers[fing_basic_idx][joint_idx]);
        tauFingers[fing_basic_idx][joint_idx] = pid_fingers[fing_basic_idx][joint_idx].effort;
        joint_idx++;
      }
      fing_basic_idx++;
    }
    /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
    int fing_mimic_idx = 0;
    for (auto i = qMimic_fingers.begin(); i < qMimic_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                      // joints
      {
        pid_fingers_mimic[fing_mimic_idx][joint_idx].posRef = cmd_fingers[fing_mimic_idx].data[joint_idx+1]/2;
        pid_fingers_mimic[fing_mimic_idx][joint_idx].OnUpdate(dT, qMimic_fingers[fing_mimic_idx][joint_idx], dqMimic_fingers[fing_mimic_idx][joint_idx]);
        tauFingers_mimic[fing_mimic_idx][joint_idx] = pid_fingers_mimic[fing_mimic_idx][joint_idx].effort;
        joint_idx++;
      }
      fing_mimic_idx++;
    }
    SetFingersTorque(tauFingers, tauFingers_mimic);

    // publish relevant topics 
    Publish(tauFingers, tauFingers_mimic);

    // update timer
    oldTime = t;  
}

// Position control update function
void SoftHandMimicPlugin::OnUpdateFingEqPos(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // compute old elastic torque & update joint effort
    double tauEl[5][4], tauEl_mimic[5][3];
    /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
    int fing_basic_idx = 0;
    for (auto i = q_fingers.begin(); i < q_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                // joints
      {
        if (joint_idx == 0){          // knuckle element does not have mimic joint!
          tauEl[fing_basic_idx][joint_idx] = spring_k_each[fing_basic_idx][joint_idx]*(cmd_fingers[fing_basic_idx].data[joint_idx] - q_fingers[fing_basic_idx][joint_idx]);
        }else{
          tauEl[fing_basic_idx][joint_idx] = spring_k_each[fing_basic_idx][joint_idx]*(cmd_fingers[fing_basic_idx].data[joint_idx]/2 - q_fingers[fing_basic_idx][joint_idx]);  
        }
        
        joint_idx++;
      }
      fing_basic_idx++;
    }
    /* --------------------------------------- MIMIC JOINTS ------------------------------------------------------- */
    int fing_mimic_idx = 0;
    for (auto i = qMimic_fingers.begin(); i < qMimic_fingers.end(); i++)    // fingers
    {
      int joint_idx = 0;
      for (auto j = i->begin(); j < i->end(); j++)                          // joints
      {
        tauEl_mimic[fing_mimic_idx][joint_idx] = spring_k_each[fing_mimic_idx][joint_idx]*(cmd_fingers[fing_mimic_idx].data[joint_idx+1]/2 - qMimic_fingers[fing_mimic_idx][joint_idx]);
        joint_idx++;
      }
      fing_mimic_idx++;
    }
    SetFingersTorque(tauEl, tauEl_mimic);

    // publish relevant topics 
    Publish(tauEl, tauEl_mimic);

    // update timer
    oldTime = t;  
}

// Main update function
void SoftHandMimicPlugin::OnUpdate(const common::UpdateInfo & info){
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
void SoftHandMimicPlugin::Publish(double tauFing[][4], double tauFingMimic[][3])
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
        /* --------------------------------------- BASIC JOINTS ------------------------------------------------------- */
        fingers_state[finger_idx].name[joint_idx] = finger_names[finger_idx] + "_" + finger_part_names[joint_idx];
        fingers_state[finger_idx].position[joint_idx] = q_fingers[finger_idx][joint_idx];
        fingers_state[finger_idx].velocity[joint_idx] = dq_fingers[finger_idx][joint_idx];
        fingers_state[finger_idx].effort[joint_idx] = tauFing[finger_idx][joint_idx];
        /* --------------------------------------- MIMIC JOINTS (only if different from knuckle) ---------------------- */
        if (joint_idx > 0)
        {
            int fing_elements = pid_fingers[finger_idx].size();
            fingers_state[finger_idx].name[fing_elements+joint_idx-1] = finger_names[finger_idx] + "_" + finger_part_names[joint_idx] + "_mimic";
            fingers_state[finger_idx].position[fing_elements+joint_idx-1] = qMimic_fingers[finger_idx][joint_idx-1];
            fingers_state[finger_idx].velocity[fing_elements+joint_idx-1] = dqMimic_fingers[finger_idx][joint_idx-1];
            fingers_state[finger_idx].effort[fing_elements+joint_idx-1] = tauFingMimic[finger_idx][joint_idx-1];
        }
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

  //if wide hand
  
}

// Publish information relative to the synergy motor
void SoftHandMimicPlugin::PublishMotor()
{
  mot_state.position[0] = mot.pos;
  mot_state.velocity[0] = mot.vel;
  mot_state.effort[0] = mot.effort;
  pub_mot_state.publish(mot_state);
}

// Initialize useful parameters
void SoftHandMimicPlugin::InitParams(sdf::ElementPtr _sdf)
{
  // Retrieve mechanical parameters from tags
  INITIALIZE_PARAMETER_FROM_TAG( double, r_finger, _sdf, "r_finger", 0.0033 ); // radius of the finger pulleys [m] (taken from qbRobotics info!)
  INITIALIZE_PARAMETER_FROM_TAG( double, R_pulley, _sdf, "R_pulley", 0.0053 ); // radius of the motor pulley [m] (taken from qbRobotics info!)
  INITIALIZE_PARAMETER_FROM_TAG( double, n_param, _sdf, "n_param", (2*r_finger)/(2*R_pulley) ); // synergy parameter 

  // Retrieve spring values from tags (if inserted)
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k, _sdf, "spring_k", 0.5 ) // fingers spring rate [N m /rad]
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic, _sdf, "spring_k_mimic", 0.5 ) // fingers spring rate [N m /rad]
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_tendon, _sdf, "spring_k_tendon", 50 ) // fingers spring rate [N m /rad]
  
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
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[0][0], _sdf, "synR_thumb_j1", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[0][1], _sdf, "synR_thumb_j2", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[0][2], _sdf, "synR_thumb_j3", n_param )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][0], _sdf, "synR_index_j1", 0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][1], _sdf, "synR_index_j2", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][2], _sdf, "synR_index_j3", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][3], _sdf, "synR_index_j4", n_param )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][0], _sdf, "synR_middle_j1", 0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][1], _sdf, "synR_middle_j2", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][2], _sdf, "synR_middle_j3", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][3], _sdf, "synR_middle_j4", n_param )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][0], _sdf, "synR_ring_j1", 0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][1], _sdf, "synR_ring_j2", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][2], _sdf, "synR_ring_j3", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][3], _sdf, "synR_ring_j4", n_param )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][0], _sdf, "synR_little_j1", 0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][1], _sdf, "synR_little_j2", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][2], _sdf, "synR_little_j3", n_param )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][3], _sdf, "synR_little_j4", n_param )    

  // ------------------------ coefficents for S synergy matrix --------------------------
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[0][0], _sdf, "synS_thumb_j1", 1.3 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[0][1], _sdf, "synS_thumb_j2", 1.3 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[0][2], _sdf, "synS_thumb_j3", 1.0 )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][0], _sdf, "synS_index_j1", 0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][1], _sdf, "synS_index_j2", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][2], _sdf, "synS_index_j3", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][3], _sdf, "synS_index_j4", 1.0 )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][0], _sdf, "synS_middle_j1", 0.1 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][1], _sdf, "synS_middle_j2", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][2], _sdf, "synS_middle_j3", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][3], _sdf, "synS_middle_j4", 1.0 )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][0], _sdf, "synS_ring_j1", 0.2 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][1], _sdf, "synS_ring_j2", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][2], _sdf, "synS_ring_j3", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][3], _sdf, "synS_ring_j4", 1.0 )

  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][0], _sdf, "synS_little_j1", 0.3 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][1], _sdf, "synS_little_j2", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][2], _sdf, "synS_little_j3", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][3], _sdf, "synS_little_j4", 1.0 )    
 
   // ------------------------------ spring rates --------------------------------------  
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[0][0], _sdf, "k_thumb_j1_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[0][1], _sdf, "k_thumb_j2_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[0][2], _sdf, "k_thumb_j3_mimic", spring_k_mimic )

  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[1][0], _sdf, "k_index_j1_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[1][1], _sdf, "k_index_j2_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[1][2], _sdf, "k_index_j3_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[1][3], _sdf, "k_index_j4_mimic", spring_k_mimic )

  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[2][0], _sdf, "k_middle_j1_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[2][1], _sdf, "k_middle_j2_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[2][2], _sdf, "k_middle_j3_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[2][3], _sdf, "k_middle_j4_mimic", spring_k_mimic )

  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[3][0], _sdf, "k_ring_j1_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[3][1], _sdf, "k_ring_j2_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[3][2], _sdf, "k_ring_j3_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[3][3], _sdf, "k_ring_j4_mimic", spring_k_mimic )

  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[4][0], _sdf, "k_little_j1_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[4][1], _sdf, "k_little_j2_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[4][2], _sdf, "k_little_j3_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each[4][3], _sdf, "k_little_j4_mimic", spring_k_mimic )     

  //Palm joints
    
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each_palm[0][0], _sdf, "k_palm_j1", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_each_palm[0][1], _sdf, "k_palm_j2", spring_k )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each_palm[0][0], _sdf, "k_palm_j1_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k_mimic_each_palm[0][1], _sdf, "k_palm_j2_mimic", spring_k_mimic )
  INITIALIZE_PARAMETER_FROM_TAG( double, palm_syn_S[0][0], _sdf, "synS_palm_j1", 1.0 )
  INITIALIZE_PARAMETER_FROM_TAG( double, palm_syn_S[0][1], _sdf, "synS_palm_j2", 1.0 )   

  // // ------------------- coefficents for R_transp synergy matrix ------------------------
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[0][0], _sdf, "syn_thumb_j1", n_param )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[0][1], _sdf, "syn_thumb_j2", n_param )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[0][2], _sdf, "syn_thumb_j3", n_param )

  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][0], _sdf, "syn_index_j1", 0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][1], _sdf, "syn_index_j2", n_param )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][2], _sdf, "syn_index_j3", n_param )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[1][3], _sdf, "syn_index_j4", n_param )

  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][0], _sdf, "syn_middle_j1", 0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][1], _sdf, "syn_middle_j2", n_param )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][2], _sdf, "syn_middle_j3", n_param )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[2][3], _sdf, "syn_middle_j4", n_param )

  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][0], _sdf, "syn_ring_j1", 0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][1], _sdf, "syn_ring_j2", n_param )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][2], _sdf, "syn_ring_j3", n_param )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[3][3], _sdf, "syn_ring_j4", n_param )

  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][0], _sdf, "syn_little_j1", 0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][1], _sdf, "syn_little_j2", n_param )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][2], _sdf, "syn_little_j3", n_param )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_Rt[4][3], _sdf, "syn_little_j4", n_param )    

  // // ------------------------ coefficents for S synergy matrix --------------------------
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[0][0], _sdf, "syn_thumb_j1", 1.3 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[0][1], _sdf, "syn_thumb_j2", 1.3 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[0][2], _sdf, "syn_thumb_j3", 1.0 )

  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][0], _sdf, "syn_index_j1", 0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][1], _sdf, "syn_index_j2", 1.0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][2], _sdf, "syn_index_j3", 1.0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[1][3], _sdf, "syn_index_j4", 1.0 )

  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][0], _sdf, "syn_middle_j1", 0.1 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][1], _sdf, "syn_middle_j2", 1.0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][2], _sdf, "syn_middle_j3", 1.0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[2][3], _sdf, "syn_middle_j4", 1.0 )

  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][0], _sdf, "syn_ring_j1", 0.2 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][1], _sdf, "syn_ring_j2", 1.0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][2], _sdf, "syn_ring_j3", 1.0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[3][3], _sdf, "syn_ring_j4", 1.0 )

  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][0], _sdf, "syn_little_j1", 0.3 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][1], _sdf, "syn_little_j2", 1.0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][2], _sdf, "syn_little_j3", 1.0 )
  // INITIALIZE_PARAMETER_FROM_TAG( double, fingers_syn_S[4][3], _sdf, "syn_little_j4", 1.0 )    

  // Maximum motor torque
  INITIALIZE_PARAMETER_FROM_TAG( double, mot.tauMax, _sdf, "tauMax", 1.5);

  INITIALIZE_PARAMETER_FROM_TAG( bool, wide, _sdf, "wide", false);

  // Initialize variables
  // sizing fingers
 
  fingers_state.resize(5);        // fingers states
  cmd_fingers.resize(5);          // fingers command for fully actiation modes
  pid_fingers.resize(5);          // PID variables
  pid_fingers_mimic.resize(5);    // PID variables for mimic joints

  // basic
  fingers_joint.resize(5);        // Gazebo structures for the finger joints
  q_fingers.resize(5);            // positions of the fingers
  dq_fingers.resize(5);           // velocities of the fingers
  // mimic
  fingers_mimic_joint.resize(5);  // Gazebo structures for the finger mimic joints
  qMimic_fingers.resize(5);       // positions of the mimic fingers
  dqMimic_fingers.resize(5);      // velocities of the mimic fingers

  // sizing joints
  int joints_dim[5] = {3, 4, 4, 4, 4};
  for (int i = 0; i < 5; ++i)
   {
     // basic joints
     fingers_joint[i].resize(joints_dim[i]);
     q_fingers[i].resize(joints_dim[i]);
     dq_fingers[i].resize(joints_dim[i]);
     // mimic joints
     fingers_mimic_joint[i].resize(joints_dim[i]-1);
     qMimic_fingers[i].resize(joints_dim[i]-1);
     dqMimic_fingers[i].resize(joints_dim[i]-1);

     fingers_state[i].name.resize(joints_dim[i]*2 - 1);     // include also the mimic as a unique vector for simplicity. So the size is joints_dim*2 - 1
     fingers_state[i].position.resize(joints_dim[i]*2 - 1); // idem up
     fingers_state[i].velocity.resize(joints_dim[i]*2 - 1); // idem up
     fingers_state[i].effort.resize(joints_dim[i]*2 - 1);   // idem up
     cmd_fingers[i].data.resize(joints_dim[i]);
     pid_fingers[i].resize(joints_dim[i]);  
     pid_fingers_mimic[i].resize(joints_dim[i]-1);         
   } 
    if(wide){
      palm_joint.resize(1);           // Gazebo structures for the finger joints
      palm_mimic_joint.resize(1);  // Gazebo structures for the finger mimic joints
      qMimic_palm.resize(1);       // positions of the mimic fingers
      dqMimic_palm.resize(1);      // velocities of the mimic fingers
      q_palm.resize(1);            // positions of the fingers
      dq_palm.resize(1);           // velocities of the fingers


      q_palm[0].resize(2);
      dq_palm[0].resize(2);
      palm_joint[0].resize(2); // il palmo è uno e ha quattro giunti
      palm_mimic_joint[0].resize(2);
      qMimic_palm[0].resize(2);
      dqMimic_palm[0].resize(2);
    }

  // Synergy motor variables
  mot_state.position.resize(1);
  mot_state.velocity.resize(1);
  mot_state.effort.resize(1);

  // Retrieve PID gains of the finger controllers (used ONLY in fully actuation mode)
  INITIALIZE_PARAMETER_FROM_TAG( double, kP_fing, _sdf, "kP_fing", 0.1 ) 
  INITIALIZE_PARAMETER_FROM_TAG( double, kI_fing, _sdf, "kI_fing", 0 )  
  INITIALIZE_PARAMETER_FROM_TAG( double, kD_fing, _sdf, "kD_fing", 0 )
  // and assign them to all the finger controllers
  int finger_basic_idx = 0;
  for (auto i = pid_fingers.begin(); i < pid_fingers.end(); i++)    // fingers
  {
    int joint_idx = 0;
    for (auto j = i->begin(); j < i->end(); j++)                      // joints
    {
      pid_fingers[finger_basic_idx][joint_idx].P = kP_fing;
      pid_fingers[finger_basic_idx][joint_idx].I = kI_fing;
      pid_fingers[finger_basic_idx][joint_idx].D = kD_fing;
      joint_idx++;
    }
    finger_basic_idx++;
  }
  // Use the same PID gains for the MIMIC fingers
  int finger_mimic_idx = 0;
  for (auto i = pid_fingers_mimic.begin(); i < pid_fingers_mimic.end(); i++)    // fingers
  {
    int joint_idx = 0;
    for (auto j = i->begin(); j < i->end(); j++)                      // joints
    {
      pid_fingers_mimic[finger_mimic_idx][joint_idx].P = kP_fing;
      pid_fingers_mimic[finger_mimic_idx][joint_idx].I = kI_fing;
      pid_fingers_mimic[finger_mimic_idx][joint_idx].D = kD_fing;
      joint_idx++;
    }
    finger_mimic_idx++;
  }
}

// Compose topics' names according to the type of actuators
void SoftHandMimicPlugin::topicNames(std::string ns_name, std::string joint_name)
{
  // Compose string name for the publishers 
  pub_thumb_name = ns_name + "/thumb_state";
  pub_index_name = ns_name + "/index_state";
  pub_middle_name = ns_name + "/middle_state";
  pub_ring_name = ns_name + "/ring_state";
  pub_little_name = ns_name + "/little_state";

  if(wide) pub_palm_name = ns_name + "/palm_state";

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
void SoftHandMimicPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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
  std::string ok_msg = "SoftHandMimicPlugin on " + ns_name + " started with " + mode_names[operationMode] + " control, mode n. [" + std::to_string(operationMode) + "]!";
  ROS_WARN_STREAM(ok_msg);

  // // Initialize motors, controllers and other parameters
  InitParams(_sdf);

  // Retrieve fingers joint
  int finger_idx = 0;
  for (auto i = fingers_joint.begin(); i < fingers_joint.end(); i++)    // fingers
  {
    // basic joints
    int jnt_basic_idx = 0;
    // std::cout << "Fingers number " << finger_idx << std::endl;
    // std::cout << "--------------- basic ---------------- " << std::endl;
    for (auto j = i->begin(); j < i->end(); j++)                        // joints
    {
      std::string temp_name;
      if (finger_idx == 0 && jnt_basic_idx == 2){
        temp_name = ns_name + "_" + finger_names[finger_idx] + "_" + finger_part_names[jnt_basic_idx+1] + "_joint";
      }else{
        temp_name = ns_name + "_" + finger_names[finger_idx] + "_" + finger_part_names[jnt_basic_idx] + "_joint";
      }
      fingers_joint[finger_idx][jnt_basic_idx] = model->GetJoint(temp_name);
      // std::cout << "Joints number " << jnt_basic_idx << "\t name = " << temp_name << "\n";
      jnt_basic_idx++;
    }
    // std::cout << "--------------- mimic ---------------- " << std::endl;
    // basic joints
    int jnt_mimic_idx = 0;
    for (auto j = i->begin(); j < i->end()-1; j++)                        // joints
    {
      std::string temp_name;
      if (finger_idx == 0 && jnt_mimic_idx == 1){
        temp_name = ns_name + "_" + finger_names[finger_idx] + "_" + finger_part_names[jnt_mimic_idx+2] + "_virtual_joint";
      }else{
        temp_name = ns_name + "_" + finger_names[finger_idx] + "_" + finger_part_names[jnt_mimic_idx+1] + "_virtual_joint";
      }
      fingers_mimic_joint[finger_idx][jnt_mimic_idx] = model->GetJoint(temp_name);
      // std::cout << "Joints number " << jnt_mimic_idx << "\t name = " << temp_name << "\n";
      jnt_mimic_idx++;
    }
    // std::cout << std::endl;
    finger_idx++;
  }


  // Retrieve palm joint
  if(wide){
    int palm_idx = 0;
      // basic joints
      int palm_jnt_basic_idx = 0;
      // std::cout << "Palm  " << std::endl;
      // std::cout << "--------------- basic ---------------- " << std::endl;
      for (int i = 0 ; i < 2; i++)                        // joints
      {
        std::string palm_temp_name;
        palm_temp_name = ns_name + "_" + palm_names[palm_idx] + "_" + palm_part_names[palm_jnt_basic_idx] + "_joint";
        
        palm_joint[palm_idx][palm_jnt_basic_idx] = model->GetJoint(palm_temp_name);
        // std::cout << "Joints number " << palm_jnt_basic_idx << "\t name = " << palm_temp_name << "\n";
        palm_jnt_basic_idx++;
      }
      // std::cout << "--------------- mimic ---------------- " << std::endl;
      // basic joints
      int palm_jnt_mimic_idx = 0;
      for (int i = 0 ; i < 2; i++)                        // joints
      {
        std::string palm_temp_name;

        palm_temp_name = ns_name + "_" + palm_names[palm_idx] + "_" + palm_part_names[palm_jnt_mimic_idx] + "_virtual_joint";
        
        palm_mimic_joint[palm_idx][palm_jnt_mimic_idx] = model->GetJoint(palm_temp_name);
        // std::cout << "Joints number " << palm_jnt_mimic_idx << "\t name = " << palm_temp_name << "\n";
        palm_jnt_mimic_idx++;
      }
    // std::cout << std::endl;
    
  } 


  // Compose the topic names
  SoftHandMimicPlugin::topicNames(ns_name, joint_name);

  // Subscribers
  if ( (operationMode == SoftSynergies) ||  (operationMode == AdaptiveSynergiesTorque) ||  (operationMode == AdaptiveSynergiesPosition))
  {
    sub_cmd = n.subscribe(cmd_ref_name, 10, &SoftHandMimicPlugin::getRef_callback, this);
  }

  if ( (operationMode == FingerTorques) ||  (operationMode == FingerPositions) ||  (operationMode == FingerEqPositions) )
  {
    sub_thumb_cmd = n.subscribe(cmd_thumb_name, 10, &SoftHandMimicPlugin::getThumbCmd_callback, this);
    sub_index_cmd = n.subscribe(cmd_index_name, 10, &SoftHandMimicPlugin::getIndexCmd_callback, this);
    sub_middle_cmd = n.subscribe(cmd_middle_name, 10, &SoftHandMimicPlugin::getMiddleCmd_callback, this);
    sub_ring_cmd = n.subscribe(cmd_ring_name, 10, &SoftHandMimicPlugin::getRingCmd_callback, this);
    sub_little_cmd = n.subscribe(cmd_little_name, 10, &SoftHandMimicPlugin::getLittleCmd_callback, this);
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
    if(wide) pub_palm_state = n.advertise<sensor_msgs::JointState>(pub_palm_name, 500);

  }

  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SoftHandMimicPlugin::OnUpdate, this, _1));

}
