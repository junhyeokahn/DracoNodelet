#include <HipNodelet/hip_nodelet.hpp>

namespace hip_nodelet
{
  using namespace apptronik_system;

  // For Nodelets, use onInit() method instead of constructor
  HipNodelet::HipNodelet():numJoint(6) {

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
      jNameIdxMap["rHipFlexion"] = 0;
      jNameIdxMap["rHipAbduction"] = 1;
      jNameIdxMap["rHipRotation"] = 2;
      jNameIdxMap["lHipRotation"] = 3;
      jNameIdxMap["lHipAbduction"] = 4;
      jNameIdxMap["lHipFlexion"] = 5;
      slaveNames.resize(numJoint+1);
      medullaName = "Medulla_V2";
      slaveNames = {"rHipFlexion", "rHipAbduction", "rHipRotation",
          "lHipRotation", "lHipAbduction", "lHipFlexion", medullaName};
      Interface = std::make_unique<HipInterface>();
      SensorData = std::make_shared<HipSensorData>();
      CommandData = std::make_shared<HipCommand>();
  }


  HipNodelet::~HipNodelet()
  {
    m_sys->stop();
    m_system_thread->join();
    ros::Duration(0.1).sleep(); //make sure everyone's done using our pointers

    for (int i = 0; i < numJoint; ++i) {
        delete jPosList[i];
        delete jVelList[i];
        delete jTrqList[i];
        delete jPosCmdList[i];
        delete jVelCmdList[i];
        delete jTrqCmdList[i];
    }
  }

  // onInit() function should not block. It should initialize and then return.
  void HipNodelet::onInit()
  {
    m_system_thread.reset(new boost::thread(boost::bind(&HipNodelet::systemThread, this)));
  }

  //set up ctrl interface here
  void HipNodelet::systemThread()
  {
      ros::NodeHandle nh = getPrivateNodeHandle();
    // User calls SystemLoop Constructor:
    m_sys.reset(new SystemLoop(boost::bind(&HipNodelet::loop, this, _1, _2), nh, slaveNames, true));

    _ClearFaults(slaveNames);


    // User sets run mode for each slave:
    for (int i = 0; i < numJoint; ++i) {
        m_sys->setRunMode(JOINT_IMPEDANCE, slaveNames[i]);
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
    }

    // User registers a command ptr for each MOSI topic corresponding to the desired mode
    for (int i = 0; i < numJoint; ++i) {
        jPosCmdList[i] = new double(0.);
        m_sys->registerCommandPtr(jPosCmdList[i], "cmd__joint__position__rad",
                slaveNames[i]);
        jVelCmdList[i] = new double(0.);
        m_sys->registerCommandPtr(jVelCmdList[i], "cmd__joint__velocity__radps",
                slaveNames[i]);
        jTrqCmdList[i] = new double(0.);
        m_sys->registerCommandPtr(jTrqCmdList[i], "cmd__joint__effort__nm",
                slaveNames[i]);
    }

    // Parameter Setting
    for(int i(0); i<numJoint; ++i ){
        _CallFloat32Service(nh, slaveNames[i], "joint_kp","/Control__Joint__Impedance__KP/set");
        _CallFloat32Service(nh, slaveNames[i], "joint_kd","/Control__Joint__Impedance__KD/set");
        _CallFloat32Service(nh, slaveNames[i], "torque_kp","/Control__Actuator__Effort__KP/set");
        _CallFloat32Service(nh, slaveNames[i], "torque_kd","/Control__Actuator__Effort__KD/set");
        _CallFloat32Service(nh, slaveNames[i], "current_limit", "/Limits__Motor__Current_Max_A/set");
        _CallInt16Service(nh, slaveNames[i], "enable_dob","/Control__Actuator__Effort__EN_DOB/set");
    }

    // Must call start to start loop
    m_sys->start();
  }


  //define control behavior here
  void HipNodelet::loop(const double& time, const dBitset& fault_bitmap)
  {

    _CopyData();
        //Interface->getCommand(SensorData, CommandData);
        //_CopyCommand();

    if(fault_bitmap.any())
    {

    }
    else
    {
        Interface->getCommand(SensorData, CommandData);
        _CopyCommand();
    }
  }

  void HipNodelet::_CopyData() {
    for (int i = 0; i < numJoint; ++i) {
        jPos[i] = *(jPosList[i]);
        jVel[i] = *(jVelList[i]);
        jTrq[i] = *(jTrqList[i]);
    }
    SensorData->q = jPos;
    SensorData->qdot = jVel;
    SensorData->jtrq = jTrq;
  }

  void HipNodelet::_CopyCommand() {
      jPosCmd = CommandData->q;
      jVelCmd = CommandData->qdot;
      jTrqCmd = CommandData->jtrq;
      for (int i = 0; i < numJoint; ++i) {
          *(jPosCmdList[i]) = jPosCmd[i];
          *(jVelCmdList[i]) = jVelCmd[i];
          *(jTrqCmdList[i]) = jTrqCmd[i];
      }
  }

  void HipNodelet::_CallInt16Service(const ros::NodeHandle & nh,
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

  void HipNodelet::_CallFloat32Service(const ros::NodeHandle & nh,
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

  void HipNodelet::_ClearFaults(std::vector<std::string> slave_names)
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
PLUGINLIB_EXPORT_CLASS(hip_nodelet::HipNodelet, nodelet::Nodelet)
