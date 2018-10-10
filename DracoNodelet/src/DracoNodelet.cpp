#include "DracoNodelet/DracoNodelet.hpp"
#include "Configuration.h"
#include "Utils/Utilities.hpp"
#include <stdexcept>

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
        delete busVoltageList[i];
        delete busCurrentList[i];
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

    // Initialize
    _initialize();
    _preprocess();
    // Initialize

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
      _setCurrentPositionCmd();
      if (m_sync->printFaults()) {

      } else {
          interface->getCommand(sensor_data, cmd);
          jPosCmd = cmd->q; jVelCmd = cmd->qdot; jTrqCmd = cmd->jtrq;
      }
      _copyCommand();

      m_sync->logger->captureLine();

      //indicate that we're done
      m_sync->finishControl();

      ++mCount;
      prevJPos = jPos;
      prevJVel = jVel;
      prevJTrq = jTrq;
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
      if (mCount > 400) {
          for (int i = 0; i < numJoint; ++i) {
              m_sync->changeMode("OFF", slaveNames[i]);
          }
      } else {
          std::cout << "Replace data from previous one, which is" << std::endl;
          std::cout << prevJPos << std::endl;
          jPos = prevJPos;
          jVel = prevJVel;
          jTrq = prevJTrq;
      }

  }

  void DracoNodelet::_initialize() {
    mCount = 0;
    numJoint = 10;
    jPos = Eigen::VectorXd::Zero(numJoint);
    jVel = Eigen::VectorXd::Zero(numJoint);
    jTrq = Eigen::VectorXd::Zero(numJoint);
    prevJPos = Eigen::VectorXd::Zero(numJoint);
    prevJVel = Eigen::VectorXd::Zero(numJoint);
    prevJTrq = Eigen::VectorXd::Zero(numJoint);
    temperature = Eigen::VectorXd::Zero(numJoint);
    motorCurrent = Eigen::VectorXd::Zero(numJoint);
    busVoltage = Eigen::VectorXd::Zero(numJoint);
    busCurrent = Eigen::VectorXd::Zero(numJoint);
    jPosList.resize(numJoint);
    jVelList.resize(numJoint);
    jTrqList.resize(numJoint);
    temperatureList.resize(numJoint);
    motorCurrentList.resize(numJoint);
    busVoltageList.resize(numJoint);
    busCurrentList.resize(numJoint);
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
    _parameterSetting();

    interface = new FixedDracoInterface();
    sensor_data = new FixedDracoSensorData();
    cmd = new FixedDracoCommand();
    go_safe_config = false;
    is_safety_spline_generated = false;
    try {
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
    }catch(std::runtime_error& e) {
        std::cout << "Error Reading Parameter [" << e.what() << "[" << std::endl;
    }
  }

  void DracoNodelet::_preprocess() {
    for (int i = 0; i < numJoint; ++i) {
        // Set Run Mode
        m_sync->changeMode("JOINT_IMPEDANCE", slaveNames[i]);

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
        busVoltageList[i] = new float(0.);
        m_sync->registerStatePtr(busVoltageList[i], "energetics__bus_voltage__V", slaveNames[i]);
        busCurrentList[i] = new float(0.);
        m_sync->registerStatePtr(busCurrentList[i], "energetics__bus_current__A", slaveNames[i]);

        // Register Command
        jPosCmdList[i] = new float(0.);
        m_sync->registerCommandPtr(jPosCmdList[i], "cmd__joint__position__rad", slaveNames[i]);
        jVelCmdList[i] = new float(0.);
        m_sync->registerCommandPtr(jVelCmdList[i], "cmd__joint__velocity__radps", slaveNames[i]);
        jTrqCmdList[i] = new float(0.);
        m_sync->registerCommandPtr(jTrqCmdList[i], "cmd__joint__effort__nm", slaveNames[i]);

        //TODO
        //m_sync->logger->addMISOChannel("actuator__position__m", slaveNames[i]);
        //m_sync->logger->addMISOChannel("motor__position__Rad", slaveNames[i]);
        //m_sync->logger->addMISOChannel("linear__joint__pos__rad", slaveNames[i]);
        //m_sync->logger->addMISOChannel("motor_dPosition_Rad", slaveNames[i]);
        //m_sync->logger->addMISOChannel("init_motor_position_aps_rad", slaveNames[i]);
        //m_sync->logger->addMISOChannel("init_motor_dposition_rad", slaveNames[i]);
        //m_sync->logger->addMISOChannel("actuator_pot_position_filt_v", slaveNames[i]);
        //m_sync->logger->addMISOChannel("actuator_pot_position_v", slaveNames[i]);
        //m_sync->logger->addMISOChannel("linear_pot_yintercept", slaveNames[i]);
        //m_sync->logger->addMISOChannel("linear_pot_slope", slaveNames[i]);
        //m_sync->logger->addMISOChannel("my_actuator_position_m", slaveNames[i]);
        //m_sync->logger->addMISOChannel("my_linear_pot_slope", slaveNames[i]);
        //m_sync->logger->addMISOChannel("my_linear_pot_yintercept", slaveNames[i]);
        //m_sync->logger->addMISOChannel("my_const_actuator_position_m", slaveNames[i]);
        //m_sync->logger->addMISOChannel("my_actuator_pot_position_filt_v", slaveNames[i]);
        //TODO
    }
  }
  void DracoNodelet::_parameterSetting() {

      Eigen::VectorXd jp_kp, jp_kd, t_kp, t_kd, current_limit;
      Eigen::VectorXi en_auto_kd, en_dob;
      try {
          YAML::Node ll_config =
              YAML::LoadFile(THIS_COM"Config/Draco/LOW_LEVEL_CONFIG.yaml");
          myUtils::readParameter(ll_config, "jp_kp", jp_kp);
          myUtils::readParameter(ll_config, "jp_kd", jp_kd);
          myUtils::readParameter(ll_config, "t_kp", t_kp);
          myUtils::readParameter(ll_config, "t_kd", t_kd);
          myUtils::readParameter(ll_config, "current_limit", current_limit);
          myUtils::readParameter(ll_config, "en_auto_kd", en_auto_kd);
          myUtils::readParameter(ll_config, "en_dob", en_dob);
      } catch(std::runtime_error& e) {
          std::cout << "Error Reading Parameter [" << e.what() << "[" << std::endl;
      }

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

  void DracoNodelet::_checkSensorData() {
      if (!(myUtils::isInBoundingBox(minPosition, jPos, maxPosition))) {
          std::cout << "Measured Joint Position Hits the Limit at " << mCount << std::endl;
          std::cout << jPos << std::endl;
          _turnOff();
      } else if (!(myUtils::isInBoundingBox(Eigen::VectorXd::Constant(numJoint, -50.), temperature, maxTemperature))) {
          std::cout << "Temperature Hits the Limit at " << mCount << std::endl;
          std::cout << temperature << std::endl;
          _turnOff();
      } else if (!(myUtils::isInBoundingBox(-maxVelocity, jVel, maxVelocity))) {
          std::cout << "Measured Joint Velocity Hits the Limit at " << mCount << std::endl;
          std::cout << jVel << std::endl;
          _turnOff();
      } else if (!(myUtils::isInBoundingBox(-maxTrq, jTrq, maxTrq))) {
          std::cout << "Measured Joint Torque Hits the Limit at " << mCount << std::endl;
          go_safe_config = true;
          std::cout << jTrq << std::endl;
          _turnOff();
      } else {
          // Do Nothing
      }
    /*
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
    */
  }

  void DracoNodelet::_checkCommand() {
    if (!(myUtils::isInBoundingBox(minPosition, jPosCmd, maxPosition))) {
        std::cout << "Commanded Joint Position Hits the Limit at " << mCount << std::endl;
        std::cout << jPosCmd << std::endl;
        _turnOff();
    } else if (!(myUtils::isInBoundingBox(-maxVelocity, jVelCmd, maxVelocity))) {
        std::cout << "Commanded Joint Velocity Hits the Limit at " << mCount << std::endl;
        std::cout << jVelCmd << std::endl;
        _turnOff();
    } else if (!(myUtils::isInBoundingBox(-maxTrq, jTrqCmd, maxTrq))) {
        std::cout << "Commanded Joint Torque Hits the Limit at " << mCount << std::endl;
        std::cout << jTrqCmd << std::endl;
        _turnOff();
    }
  }

  void DracoNodelet::_copyData() {
    for (int i = 0; i < numJoint; ++i) {
        jPos[i] = static_cast<double> (*(jPosList[i]));
        jVel[i] = static_cast<double> (*(jVelList[i]));
        jTrq[i] = static_cast<double> (*(jTrqList[i]));
        temperature[i] = static_cast<double> (*(temperatureList[i]));
        motorCurrent[i] = static_cast<double> (*(motorCurrentList[i]));
        busVoltage[i] = static_cast<double> (*(busVoltageList[i]));
        busCurrent[i] = static_cast<double> (*(busCurrentList[i]));
    }
    _checkSensorData();
    sensor_data->q = jPos;
    sensor_data->qdot = jVel;
    sensor_data->jtrq = jTrq;
    sensor_data->motorCurrent = motorCurrent;
    sensor_data->busVoltage = busVoltage;
    sensor_data->busCurrent = busCurrent;
    sensor_data->temperature = temperature;
  }

  void DracoNodelet::_copyCommand() {
    _checkCommand();
    for (int i = 0; i < numJoint; ++i) {
        *(jPosCmdList[i]) = static_cast<float>(jPosCmd[i]);
        *(jVelCmdList[i]) = static_cast<float>(jVelCmd[i]);
        *(jTrqCmdList[i]) = static_cast<float>(jTrqCmd[i]);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(draco_nodelet::DracoNodelet, nodelet::Nodelet)
