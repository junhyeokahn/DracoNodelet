#include "DracoNodelet/DracoNodelet.hpp"

using namespace apptronik_ros_utils;
namespace draco_nodelet
{
  DracoNodelet::DracoNodelet() {
  }

  DracoNodelet::~DracoNodelet()
  {
    m_spin_thread->join();

    for (int i = 0; i < numJoint; ++i) {
        delete jPosList[i];
        delete jVelList[i];
        delete jTrqList[i];
        delete jPosCmdList[i];
        delete jVelCmdList[i];
        delete jTrqCmdList[i];
    }
  }

  void DracoNodelet::onInit()
  {
    m_nh = getNodeHandle();
    m_spin_thread.reset(new boost::thread(boost::bind(&DracoNodelet::spinThread, this)));
  }

  void DracoNodelet::spinThread()
  {
    _initialize();
    _preprocess();

    //set up controller
    m_sync.reset(new apptronik_ros_utils::Synchronizer(true, "draco_nodelet"));
    m_sync->connect();
    apptronik_ros_utils::enableRT(99, 2);



    // bool previously_faulted = false;
    for(std::size_t i = 0; i < slaveNames.size(); ++i) {
      m_sync->clearFaults(slaveNames[i]);
    }

    //main control loop
    while(m_sync->ok())
    {
      //wait for bus transaction
      m_sync->awaitNextControl();
      //TODO : _copyData();
      if ((m_sync->getAllFaults()).any()) {
          //TODO : _sendSafeCmd();
      } else {
          // TODO : Interface->getCommand(SensorData, CommandData);
           //TODO : _copyCommand();
      }

      m_sync->logger->captureLine();

      //indicate that we're done
      m_sync->finishControl();
    }

    m_sync->awaitShutdownComplete();
  }

  void DracoNodelet::_initialize() {
    numJoint = 10;
    jPos = Eigen::VectorXd::Zero(numJoint);
    jVel = Eigen::VectorXd::Zero(numJoint);
    jTrq = Eigen::VectorXd::Zero(numJoint);
    jPosList.resize(numJoint);
    jVelList.resize(numJoint);
    jTrqList.resize(numJoint);
    jPosCmd = Eigen::VectorXd::Zero(numJoint);
    jVelCmd = Eigen::VectorXd::Zero(numJoint);
    jTrqCmd = Eigen::VectorXd::Zero(numJoint);
    jPosCmdList.resize(numJoint);
    jVelCmdList.resize(numJoint);
    jTrqCmdList.resize(numJoint);
    medullaName = "Medulla_V2";
    slaveNames.resize(numJoint+1);
    slaveNames = {"rHipYaw", "rHipRoll", "rHipPitch", "rKnee", "rAnkle",
                  "lHipYaw", "lHipRoll", "lHipPitch", "lKnee", "lAnkle",
                  medullaName };
    // TODO : construct interface, data, cmd

  }

  void DracoNodelet::_preprocess() {
    for (int i = 0; i < numJoint; ++i) {
        // Set Run Mode
        m_sys->setRunMode("JOINT_IMPEDANCE", slaveNames[i]);

        // Register State
        jPosList[i] = new double(0.);
        m_sys->registerStatePtr(jPosList[i], "js__joint__position__rad", slaveNames[i]);
        jVelList[i] = new double(0.);
        m_sys->registerStatePtr(jVelList[i], "js__joint__velocity__radps", slaveNames[i]);
        jTrqList[i] = new double(0.);
        m_sys->registerStatePtr(jTrqList[i], "js__joint__effort__Nm", slaveNames[i]);

        // Register Command
        jPosCmdList[i] = new double(0.);
        m_sys->registerCommandPtr(jPosCmdList[i], "cmd__joint__position__rad", slaveNames[i]);
        jVelCmdList[i] = new double(0.);
        m_sys->registerCommandPtr(jVelCmdList[i], "cmd__joint__position__rad", slaveNames[i]);
        jTrqCmdList[i] = new double(0.);
        m_sys->registerCommandPtr(jTrqCmdList[i], "cmd__joint__position__rad", slaveNames[i]);

        // Service call
        callSetService(m_nh, slaveNames[i], "joint_kp","/Control__Joint__Impedance__KP/set");
        callSetService(m_nh, slaveNames[i], "joint_kd","/Control__Joint__Impedance__KD/set");
        callSetService(m_nh, slaveNames[i], "torque_kp","/Control__Actuator__Effort__KP/set");
        callSetService(m_nh, slaveNames[i], "torque_kd","/Control__Actuator__Effort__KD/set");
        callSetService(m_nh, slaveNames[i], "current_limit","/Limits__Motor__Current_Max_A/set");
        callSetService(m_nh, slaveNames[i], "enable_dob", "/Control__Actuator__Effort__EN_DOB/set");
    }
  }

  void DracoNodeLet::_copyData() {
    for (int i = 0; i < numJoint; ++i) {
        jPos[i] = *(jPosList[i]);
        jVel[i] = *(jVelList[i]);
        jTrq[i] = *(jTrqList[i]);
    }
    // TODO : copy to sensor data
  }

  void DracoNodelet::_copyCommand() {
    // TODO : copy from command
    for (int i = 0; i < numJoint; ++i) {
        *(jPosCmdList[i]) = jPosCmd[i];
        *(jVelCmdList[i]) = jVelCmd[i];
        *(jTrqCmdList[i]) = jTrqCmd[i];
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(draco_nodelet::DracoNodelet, nodelet::Nodelet)
