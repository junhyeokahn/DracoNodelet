#include "DracoNodelet/DracoNodelet.hpp"
#include "Configuration.h"
#include "Utils/Utilities.hpp"

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
      //if ((m_sync->getAllFaults()).any()) {
      if (m_sync->printFaults()) {
          _setCurrentPositionCmd();
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

  void DracoNodelet::_setCurrentPositionCmd() {
      jPosCmd = jPos;
      jVelCmd.setZero();
      jTrqCmd.setZero();
  }

  void DracoNodelet::_setHomePositionCmd() {
    jPosCmd = homePosition;
    jVelCmd.setZero();
    jTrqCmd.setZero();
  }

  void DracoNodelet::_turnOff() {
    for (int i = 0; i < numJoint; ++i) {
        m_sync->setRunMode("OFF", slaveNames[i]);
    }
  }

  void DracoNodelet::_checkSafety() {
      jPosCmd = cmd->q;
      jVelCmd = cmd->qdot;
      jTrqCmd = cmd->jtrq;
    // toggle go_safe_config based on sensored jpos, jvel, jtrq,
    // temperature, and print out which causes that

    if (!(myUtils::isInBoundingBox(minPosition, jPos, maxPosition))) {
        std::cout << "Measured Joint Position Hits the Limit" << std::endl;
        std::cout << jPos << std::endl;
        go_safe_config = true;
        _turnOff();
    } else if (!(myUtils::isInBoundingBox(minPosition, jPosCmd, maxPosition))) {
        std::cout << "Commanded Joint Position Hits the Limit" << std::endl;
        go_safe_config = true;
    } else if (!(myUtils::isInBoundingBox(Eigen::VectorXd::Constant(numJoint, -50.), temperature, maxTemperature))) {
        std::cout << "Temperature Hits the Limit" << std::endl;
        go_safe_config = true;
    } else if (!(myUtils::isInBoundingBox(-maxVelocity, jVelCmd, maxVelocity))) {
        std::cout << "Commanded Joint Velocity Hits the Limit" << std::endl;
        go_safe_config = true;
    } else if (!(myUtils::isInBoundingBox(-maxVelocity, jVel, maxVelocity))) {
        std::cout << "Measured Joint Velocity Hits the Limit" << std::endl;
        go_safe_config = true;
    } else if (!(myUtils::isInBoundingBox(-maxTrq, jTrqCmd, maxTrq))) {
        std::cout << "Commanded Joint Torque Hits the Limit" << std::endl;
        go_safe_config = true;
    } else if (!(myUtils::isInBoundingBox(-maxTrq, jTrq, maxTrq))) {
        std::cout << "Measured Joint Torque Hits the Limit" << std::endl;
        go_safe_config = true;
    } else {
        // Do Nothing
    }

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
                fin[i] = homePosition[i];
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
            _turnOff();
            //_setHomePositionCmd();
        }
        ++count;
    } else {
        // do nothing
    }
  }

  void DracoNodelet::_initialize() {
    numJoint = 10;
    jPos = Eigen::VectorXd::Zero(numJoint);
    jVel = Eigen::VectorXd::Zero(numJoint);
    jTrq = Eigen::VectorXd::Zero(numJoint);
    temperature = Eigen::VectorXd::Zero(numJoint);
    motorCurrent = Eigen::VectorXd::Zero(numJoint);
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
    YAML::Node safety_cfg =
        YAML::LoadFile(THIS_COM"Config/Draco/SAFETY.yaml");
    myUtils::readParameter(safety_cfg, "home_position", homePosition);
    myUtils::readParameter(safety_cfg, "max_temperature", maxTemperature);
    myUtils::readParameter(safety_cfg, "max_position", maxPosition);
    myUtils::readParameter(safety_cfg, "min_position", minPosition);
    myUtils::readParameter(safety_cfg, "max_velocity", maxVelocity);
    myUtils::readParameter(safety_cfg, "max_trq", maxTrq);
    maxPosition += Eigen::VectorXd::Constant(numJoint, 0.2);
    minPosition -= Eigen::VectorXd::Constant(numJoint, 0.2);
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
        m_sync->registerCommandPtr(jVelCmdList[i], "cmd__joint__velocity__radps", slaveNames[i]);
        jTrqCmdList[i] = new float(0.);
        m_sync->registerCommandPtr(jTrqCmdList[i], "cmd__joint__effort__nm", slaveNames[i]);

    }
    _parameterSetting();
  }
  void DracoNodelet::_parameterSetting() {
    YAML::Node ll_config =
        YAML::LoadFile(THIS_COM"Config/Draco/LOW_LEVEL_CONFIG.yaml");
    Eigen::VectorXd jp_kp, jp_kd, t_kp, t_kd, current_limit;
    Eigen::VectorXi en_auto_kd, en_dob;
    myUtils::readParameter(ll_config, "jp_kp", jp_kp);
    myUtils::readParameter(ll_config, "jp_kd", jp_kd);
    myUtils::readParameter(ll_config, "t_kp", t_kp);
    myUtils::readParameter(ll_config, "t_kd", t_kd);
    myUtils::readParameter(ll_config, "current_limit", current_limit);
    myUtils::readParameter(ll_config, "en_auto_kd", en_auto_kd);
    myUtils::readParameter(ll_config, "en_dob", en_dob);
    for (int i = 0; i < numJoint; ++i) {
        apptronik_srvs::Float32 srv_float;
        apptronik_srvs::UInt16 srv_int;
        srv_float.request.set_data = jp_kp[i];
        callSetService(slaveNames[i], "Control__Joint__Impedance__KP", srv_float);
        srv_float.request.set_data = jp_kd[i];
        callSetService(slaveNames[i], "Control__Joint__Impedance__KD", srv_float);
        srv_float.request.set_data = t_kp[i];
        callSetService(slaveNames[i], "Control__Actuator__Effort__KP", srv_float);
        srv_float.request.set_data = t_kd[i];
        callSetService(slaveNames[i], "Control__Actuator__Effort__KD", srv_float);
        srv_float.request.set_data = current_limit[i];
        callSetService(slaveNames[i], "Limits__Motor__Current_Max_A", srv_float);
        srv_int.request.set_data = en_dob[i];
        callSetService(slaveNames[i], "Control__Actuator__Effort__EN_DOB", srv_int);
        srv_int.request.set_data = en_auto_kd[i];
        callSetService(slaveNames[i], "Control__Actuator__Effort__AutoKD", srv_int);
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
    for (int i = 0; i < numJoint; ++i) {
        *(jPosCmdList[i]) = jPosCmd[i];
        *(jVelCmdList[i]) = jVelCmd[i];
        *(jTrqCmdList[i]) = jTrqCmd[i];
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(draco_nodelet::DracoNodelet, nodelet::Nodelet)
