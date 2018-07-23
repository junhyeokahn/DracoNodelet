#include <AnkleKneeNodelet/ankle_knee_nodelet.hpp>

namespace ankle_knee_nodelet
{
  using namespace apptronik_system;

  // For Nodelets, use onInit() method instead of constructor
  AnkleKneeNodelet::AnkleKneeNodelet():numJoint(2) {

      jPos = Eigen::VectorXd::Zero(numJoint);
      jVel = Eigen::VectorXd::Zero(numJoint);
      jTrq = Eigen::VectorXd::Zero(numJoint);
      chirp = Eigen::VectorXd::Zero(numJoint);
      chirpInput = Eigen::VectorXd::Zero(numJoint);
      chirpOutput = Eigen::VectorXd::Zero(numJoint);
      busVoltage= Eigen::VectorXd::Zero(numJoint);
      coreTemp = Eigen::VectorXd::Zero(numJoint);
      jPosList.resize(numJoint);
      jVelList.resize(numJoint);
      jTrqList.resize(numJoint);
      chirpList.resize(numJoint);
      chirpInputList.resize(numJoint);
      chirpOutputList.resize(numJoint);
      nanosecondList.resize(numJoint);
      busVoltageList.resize(numJoint);
      coreTempList.resize(numJoint);
      jPosCmd = Eigen::VectorXd::Zero(numJoint);
      jVelCmd = Eigen::VectorXd::Zero(numJoint);
      jTrqCmd = Eigen::VectorXd::Zero(numJoint);
      jPosCmdList.resize(numJoint);
      jVelCmdList.resize(numJoint);
      jTrqCmdList.resize(numJoint);
      jNameIdxMap["rKnee"] = 0;
      jNameIdxMap["rAnkle"] = 1;
      slaveNames.resize(numJoint+1);
      medullaName = "Medulla_V2";
      slaveNames = {"rKnee", "rAnkle", medullaName};
      Interface = std::make_unique<AnkleKneeInterface>();
      SensorData = std::make_shared<AnkleKneeSensorData>();
      CommandData = std::make_shared<AnkleKneeCommand>();
  }


  AnkleKneeNodelet::~AnkleKneeNodelet()
  {
    m_sys->stop();
    m_system_thread->join();
    ros::Duration(0.1).sleep(); //make sure everyone's done using our pointers

    for (int i = 0; i < numJoint; ++i) {
        delete jPosList[i];
        delete jVelList[i];
        delete jTrqList[i];
        delete chirpList[i];
        delete chirpInputList[i];
        delete chirpOutputList[i];
        delete busVoltageList[i];
        delete nanosecondList[i];
        delete jPosCmdList[i];
        delete jVelCmdList[i];
        delete jTrqCmdList[i];
    }
  }

  // onInit() function should not block. It should initialize and then return.
  void AnkleKneeNodelet::onInit()
  {
    m_system_thread.reset(new boost::thread(boost::bind(&AnkleKneeNodelet::systemThread, this)));
  }

  //set up ctrl interface here
  void AnkleKneeNodelet::systemThread()
  {
    ros::NodeHandle nh = getPrivateNodeHandle();
    nh.param("env/mode/controlMode", mControlMode, 0);
    nh.param("env/mode/enable_spring", mEnableSpring, 0);
    std::cout << "enable spring : " << mEnableSpring << std::endl;

    // User calls SystemLoop Constructor:
    m_sys.reset(new SystemLoop(boost::bind(&AnkleKneeNodelet::loop, this, _1, _2), nh, slaveNames, true));


    _ClearFaults(slaveNames);


    // User sets run mode for each slave:
    if (mControlMode == 0) {
        m_sys->setRunMode(JOINT_IMPEDANCE, "rKnee");
        m_sys->setRunMode(JOINT_IMPEDANCE, "rAnkle");
    } else if (mControlMode == 1) {
        m_sys->setRunMode(MOTOR_CURRENT, "rKnee");
        m_sys->setRunMode(MOTOR_CURRENT, "rAnkle");
    } else {
        m_sys->setRunMode(MOTOR_POSITION, "rKnee");
        m_sys->setRunMode(MOTOR_POSITION, "rAnkle");
    }

    // User registers a state ptr for each MISO topic with desired state info
    for (int i = 0; i < numJoint; ++i) {
        jPosList[i] = new double(0.);
        m_sys->registerStatePtr(jPosList[i], "js__joint__position__rad",
                slaveNames[i]);
        jVelList[i] = new double(0.);
        m_sys->registerStatePtr(jVelList[i], "js__joint__velocity__radps",
                slaveNames[i]);
        jTrqList[i] = new double(0.);
        m_sys->registerStatePtr(jTrqList[i], "js__joint__effort__Nm",
                slaveNames[i]);
        nanosecondList[i] = new unsigned int(0);
        m_sys->registerStatePtr(nanosecondList[i], "timestamp__ns",
                slaveNames[i]);
        busVoltageList[i] = new double(0.);
        m_sys->registerStatePtr(busVoltageList[i], "energetics__bus_voltage__V",
                slaveNames[i]);
        coreTempList[i] = new double(0.);
        m_sys->registerStatePtr(coreTempList[i], "motor__core_temp_est__C",
                slaveNames[i]);
        if (mControlMode == 0) {
            chirpList[i] = new double(0.);
            m_sys->registerStatePtr(chirpList[i], "js__joint__position__rad",
                    slaveNames[i]);
            chirpInputList[i] = new double(0.);
            m_sys->registerStatePtr(chirpInputList[i], "mirrored__joint__position__cmd",
                    slaveNames[i]);
            if (mEnableSpring == 0) {
                chirpOutputList[i] = new double(0.);
                if (i == 0) {
                    m_sys->registerStatePtr(chirpOutputList[i], "joint__position__with__spring__rad",
                            slaveNames[i]);
                } else {
                    m_sys->registerStatePtr(chirpOutputList[i], "js__joint__position__rad",
                            slaveNames[i]);
                }
            } else {
                chirpOutputList[i] = new double(0.);
                m_sys->registerStatePtr(chirpOutputList[i], "js__joint__position__rad",
                        slaveNames[i]);
            }
        } else if (mControlMode == 1) {
            chirpList[i] = new double(0.);
            m_sys->registerStatePtr(chirpList[i], "motor__current__A",
                    slaveNames[i]);
            chirpInputList[i] = new double(0.);
            m_sys->registerStatePtr(chirpInputList[i], "motor__current__A",
                    slaveNames[i]);
            chirpOutputList[i] = new double(0.);
            m_sys->registerStatePtr(chirpOutputList[i], "actuator__force__N",
                    slaveNames[i]);
        } else {
            chirpList[i] = new double(0.);
            m_sys->registerStatePtr(chirpList[i], "motor__position__Rad",
                    slaveNames[i]);
            chirpInputList[i] = new double(0.);
            m_sys->registerStatePtr(chirpInputList[i], "motor__current__A",
                    slaveNames[i]);
            chirpOutputList[i] = new double(0.);
            m_sys->registerStatePtr(chirpOutputList[i], "actuator__force__N",
                    slaveNames[i]);
        }
    }

    // User registers a command ptr for each MOSI topic corresponding to the desired mode
    for (int i = 0; i < numJoint; ++i) {
        if (mControlMode == 0) {
            jPosCmdList[i] = new double(0.);
            m_sys->registerCommandPtr(jPosCmdList[i], "cmd__joint__position__rad",
                    slaveNames[i]);
            jVelCmdList[i] = new double(0.);
            m_sys->registerCommandPtr(jVelCmdList[i], "cmd__joint__velocity__radps",
                    slaveNames[i]);
            jTrqCmdList[i] = new double(0.);
            m_sys->registerCommandPtr(jTrqCmdList[i], "cmd__joint__effort__nm",
                    slaveNames[i]);
        } else if (mControlMode == 1) {
            jPosCmdList[i] = new double(0.);
            jVelCmdList[i] = new double(0.);
            jTrqCmdList[i] = new double(0.);
            m_sys->registerCommandPtr(jTrqCmdList[i], "cmd__motor__effort__a",
                    slaveNames[i]);
        } else {
            jPosCmdList[i] = new double(0.);
            m_sys->registerCommandPtr(jPosCmdList[i], "cmd__motor__position__rad",
                    slaveNames[i]);
            jVelCmdList[i] = new double(0.);
            jTrqCmdList[i] = new double(0.);
        }
    }

    // Parameter Setting
    for(int i(0); i<numJoint; ++i ){
        _CallFloat32Service(nh, slaveNames[i], "joint_kp","/Control__Joint__Impedance__KP/set");
        _CallFloat32Service(nh, slaveNames[i], "joint_kd","/Control__Joint__Impedance__KD/set");
        _CallFloat32Service(nh, slaveNames[i], "torque_kp","/Control__Actuator__Effort__KP/set");
        _CallFloat32Service(nh, slaveNames[i], "torque_kd","/Control__Actuator__Effort__KD/set");
        _CallFloat32Service(nh, slaveNames[i], "current_limit", "/Limits__Motor__Current_Max_A/set");
        _CallInt16Service(nh, slaveNames[i], "enable_dob","/Control__Actuator__Effort__EN_DOB/set");
        _CallInt16Service(nh, slaveNames[i], "jPosFeedbackSrc", "/Sensing__LinearPos__Feedback_Source/set");
        _CallInt16Service(nh, slaveNames[i], "enable_spring", "/Sensing__ActuatorPos__Include_Spring/set");
    }

    // Must call start to start loop
    m_sys->start();
  }


  //define control behavior here
  void AnkleKneeNodelet::loop(const double& time, const dBitset& fault_bitmap)
  {

    _CopyData();

    if(fault_bitmap.any())
    {
        //_SetSafeCmd();
    }
    else
    {
        Interface->getCommand(SensorData, CommandData);
        _CopyCommand();
    }
  }

  void AnkleKneeNodelet::_SetSafeCmd() {
      jPosCmd = SensorData->q;
      jVelCmd.setZero();
      jTrqCmd.setZero();
      for (int i = 0; i < numJoint; ++i) {
          *(jPosCmdList[i]) = jPosCmd[i];
          *(jVelCmdList[i]) = jVelCmd[i];
          *(jTrqCmdList[i]) = jTrqCmd[i];
      }
  }

  void AnkleKneeNodelet::_CopyData() {
    for (int i = 0; i < numJoint; ++i) {
        jPos[i] = *(jPosList[i]);
        jVel[i] = *(jVelList[i]);
        jTrq[i] = *(jTrqList[i]);
        chirp[i] = *(chirpList[i]);
        chirpInput[i] = *(chirpInputList[i]);
        chirpOutput[i] = *(chirpOutputList[i]);
        busVoltage[i] = *(busVoltageList[i]);
        coreTemp[i] = *(coreTempList[i]);
    }
    SensorData->q = jPos;
    SensorData->qdot = jVel;
    SensorData->jtrq = jTrq;
    SensorData->chirp = chirp;
    SensorData->chirpInput = chirpInput;
    SensorData->chirpOutput = chirpOutput;
    SensorData->nanosecondKnee = *(nanosecondList[0]);
    SensorData->nanosecondAnkle = *(nanosecondList[1]);
    SensorData->busVoltage = busVoltage;
    SensorData->coreTemp = coreTemp;
  }

  void AnkleKneeNodelet::_CopyCommand() {
      if (mControlMode == 0) {
          jPosCmd = CommandData->q;
          jVelCmd = CommandData->qdot;
          jTrqCmd = CommandData->jtrq;
          for (int i = 0; i < numJoint; ++i) {
              *(jPosCmdList[i]) = jPosCmd[i];
              *(jVelCmdList[i]) = jVelCmd[i];
              *(jTrqCmdList[i]) = jTrqCmd[i];
          }
      } else if (mControlMode == 1) {
          jTrqCmd = CommandData->jtrq;
          for (int i = 0; i < numJoint; ++i) {
              *(jTrqCmdList[i]) = jTrqCmd[i];
          }
      } else {
          jPosCmd = CommandData->q;
          for (int i = 0; i < numJoint; ++i) {
              *(jPosCmdList[i]) = jPosCmd[i];
          }
      }
  }

  void AnkleKneeNodelet::_CallInt16Service(const ros::NodeHandle & nh,
                                           const std::string & slave_name,
                                           const std::string & parameter_name,
                                           const std::string & service_name){

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

  void AnkleKneeNodelet::_CallFloat32Service(const ros::NodeHandle & nh,
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

  void AnkleKneeNodelet::_ClearFaults(std::vector<std::string> slave_names)
  {
    ros::NodeHandle nh = getPrivateNodeHandle();
    for (int i = 0; i < slave_names.size(); i++)
    {
      std::string service_name = "/" + slave_names[i] + "/clear_faults/set";
      ros::ServiceClient client = nh.serviceClient<apptronik_srvs::UInt16>(service_name);
      apptronik_srvs::UInt16 srv;
      srv.request.set_data = 1;
      if (client.call(srv))
      {
        NODELET_INFO_STREAM("Successfully called service: " << service_name);
      }
      else
      {
        NODELET_INFO_STREAM("ERROR: Failed to call service: " << service_name);
      }
    }
  }

}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ankle_knee_nodelet::AnkleKneeNodelet, nodelet::Nodelet)
