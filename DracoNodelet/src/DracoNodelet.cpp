#include "DracoNodelet/DracoNodelet.hpp"
#include "Configuration.h"

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
        delete temperatureList[i];
        delete motorCurrentList[i];
        delete jPosCmdList[i];
        delete jVelCmdList[i];
        delete jTrqCmdList[i];
    }

    delete interface;
    delete sensor_data;
    delete cmd;
  }

  void DracoNodelet::onInit()
  {
    m_nh = getNodeHandle();
    m_spin_thread.reset(new boost::thread(boost::bind(&DracoNodelet::spinThread, this)));
  }

  void DracoNodelet::spinThread()
  {

    //set up controller
    m_sync.reset(new apptronik_ros_utils::Synchronizer(true, "draco_nodelet"));
    m_sync->connect();
    apptronik_ros_utils::enableRT(99, 2);

    _initialize();
    _preprocess();

    // bool previously_faulted = false;
    for(std::size_t i = 0; i < slaveNames.size(); ++i) {
      m_sync->clearFaults(slaveNames[i]);
    }

    //main control loop
    while(m_sync->ok())
    {
      //wait for bus transaction
      m_sync->awaitNextControl();
      _copyData();
      if ((m_sync->getAllFaults()).any()) {
          _setDefaultCmd();
      } else {
          interface->getCommand(sensor_data, cmd);
          _checkSafety();
      }
      _copyCommand();

      m_sync->logger->captureLine();

      //indicate that we're done
      m_sync->finishControl();
    }

    m_sync->awaitShutdownComplete();
  }

  void DracoNodelet::_setDefaultCmd() {
    jPosCmd = defaultPosition;
    jVelCmd.setZero();
    jTrqCmd.setZero();
  }

  void DracoNodelet::_checkSafety() {
    // toggle go_safe_config based on sensored jpos, jvel, jtrq,
    // temperature, and print out which causes that

    // TODO : Do Toggle

    if (go_safe_config) {
        std::cout << "Going Back to Safe Configuration!" << std::endl;
        static double time_duration(1.0);
        static double d_ary[10];
        static int count(0);
        double t(count*SERVO_RATE);

        if (!is_safety_spline_generated) {
            double ini[30]; double fin[30]; double **middle_pt;
            for (int i = 0; i < 30; ++i) {
                ini[i] = 0.0;
                fin[i] = 0.0;
            }
            for (int i = 0; i < 10; ++i) {
                ini[i] = jPos[i];
                fin[i] = defaultPosition[i];
            }
            for (int i = 10; i < 20; ++i) {
                ini[i] = jVel[i-10];
            }
            safety_spline.SetParam(ini, fin, middle_pt, time_duration);
            is_safety_spline_generated = true;
        }

        if (t < time_duration) {
            safety_spline.getCurvePoint(t, d_ary);
            for (int i = 0; i < 10; ++i) jPosCmd[i] = d_ary[i];
            safety_spline.getCurveDerPoint(t, 1, d_ary);
            for (int i = 0; i < 10; ++i) jVelCmd[i] = d_ary[i];
            jTrqCmd.setZero();
        } else {
            _setDefaultCmd();
        }
        ++count;
    }
  }

  void DracoNodelet::_initialize() {
    numJoint = 10;
    jPos = Eigen::VectorXd::Zero(numJoint);
    jVel = Eigen::VectorXd::Zero(numJoint);
    jTrq = Eigen::VectorXd::Zero(numJoint);
    jPosList.resize(numJoint);
    jVelList.resize(numJoint);
    jTrqList.resize(numJoint);
    temperatureList.resize(numJoint);
    motorCurrentList.resize(numJoint);
    jPosCmd = Eigen::VectorXd::Zero(numJoint);
    jVelCmd = Eigen::VectorXd::Zero(numJoint);
    jTrqCmd = Eigen::VectorXd::Zero(numJoint);
    jPosCmdList.resize(numJoint);
    jVelCmdList.resize(numJoint);
    jTrqCmdList.resize(numJoint);
    medullaName = "Medulla_V2";
    slaveNames.resize(numJoint+1);
    slaveNames = {"lHipYaw", "lHipRoll", "lHipPitch", "lKnee", "lAnkle",
                  "rHipYaw", "rHipRoll", "rHipPitch", "rKnee", "rAnkle",
                  medullaName };

    interface = new FixedDracoInterface();
    sensor_data = new FixedDracoSensorData();
    cmd = new FixedDracoCommand();
    go_safe_config = false;
    is_safety_spline_generated = false;
    // TODO : read defaultPosition from yaml

  }

  void DracoNodelet::_preprocess() {
    for (int i = 0; i < numJoint; ++i) {
        // Set Run Mode
        m_sync->setRunMode("JOINT_IMPEDANCE", slaveNames[i]);

        // Register State
        jPosList[i] = new float(0.);
        m_sync->registerStatePtr(jPosList[i], "js__joint__position__rad", slaveNames[i]);
        jVelList[i] = new float(0.);
        m_sync->registerStatePtr(jVelList[i], "js__joint__velocity__radps", slaveNames[i]);
        jTrqList[i] = new float(0.);
        m_sync->registerStatePtr(jTrqList[i], "js__joint__effort__Nm", slaveNames[i]);
        temperatureList[i] = new float(0.);
        m_sync->registerStatePtr(temperatureList[i], "motor__core_temp_est__C", slaveNames[i]);
        motorCurrentList[i] = new float(0.);
        m_sync->registerStatePtr(motorCurrentList[i], "motor__current__A", slaveNames[i]);

        // Register Command
        jPosCmdList[i] = new float(0.);
        m_sync->registerCommandPtr(jPosCmdList[i], "cmd__joint__position__rad", slaveNames[i]);
        jVelCmdList[i] = new float(0.);
        m_sync->registerCommandPtr(jVelCmdList[i], "cmd__joint__position__rad", slaveNames[i]);
        jTrqCmdList[i] = new float(0.);
        m_sync->registerCommandPtr(jTrqCmdList[i], "cmd__joint__position__rad", slaveNames[i]);

        // Service call
        _callFloat32Service(m_nh, slaveNames[i], "joint_kp","/Control__Joint__Impedance__KP/set");
        _callFloat32Service(m_nh, slaveNames[i], "joint_kd","/Control__Joint__Impedance__KD/set");
        _callFloat32Service(m_nh, slaveNames[i], "torque_kp","/Control__Actuator__Effort__KP/set");
        _callFloat32Service(m_nh, slaveNames[i], "torque_kd","/Control__Actuator__Effort__KD/set");
        _callFloat32Service(m_nh, slaveNames[i], "current_limit","/Limits__Motor__Current_Max_A/set");
        _callInt16Service(m_nh, slaveNames[i], "enable_dob", "/Control__Actuator__Effort__EN_DOB/set");
    }
  }

  void DracoNodelet::_copyData() {
    for (int i = 0; i < numJoint; ++i) {
        jPos[i] = *(jPosList[i]);
        jVel[i] = *(jVelList[i]);
        jTrq[i] = *(jTrqList[i]);
        temperature[i] = *(temperatureList[i]);
        motorCurrent[i] = *(motorCurrentList[i]);
    }
    sensor_data->q = jPos;
    sensor_data->qdot = jVel;
    sensor_data->jtrq = jTrq;
    // TODO : add temperature in sensor data
  }

  void DracoNodelet::_copyCommand() {
    jPosCmd = cmd->q;
    jVelCmd = cmd->qdot;
    jTrqCmd = cmd->jtrq;
    for (int i = 0; i < numJoint; ++i) {
        *(jPosCmdList[i]) = jPosCmd[i];
        *(jVelCmdList[i]) = jVelCmd[i];
        *(jTrqCmdList[i]) = jTrqCmd[i];
    }
  }

  void DracoNodelet::_callFloat32Service(const ros::NodeHandle & nh,
                                         const std::string & slave_name,
                                         const std::string & parameter_name,
                                         const std::string & service_name){

      double value;
      nh.param(slave_name + "/" + parameter_name, value, 0.);

      apptronik_srvs::Float32 srv_float;
      srv_float.request.set_data = value;

      if(ros::service::exists
              ("/" + slave_name + service_name, false)) {
          ros::service::call("/" + slave_name + service_name, srv_float);
      } else {
          ROS_WARN("Could not find service %s",("/" + slave_name + service_name).c_str());
      }
  }

    void DracoNodelet::_callInt16Service(const ros::NodeHandle & nh,
                                         const std::string & slave_name,
                                         const std::string & parameter_name,
                                         const std::string & service_name) {

      int value;
      nh.param(slave_name + "/" + parameter_name, value, 0);

      apptronik_srvs::UInt16 srv_int;
      srv_int.request.set_data = value;

      if(ros::service::exists
              ("/" + slave_name + service_name, false)) {
          ros::service::call("/" + slave_name + service_name, srv_int);
      } else {
          ROS_WARN("Could not find service %s",("/" + slave_name + service_name).c_str());
      }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(draco_nodelet::DracoNodelet, nodelet::Nodelet)
