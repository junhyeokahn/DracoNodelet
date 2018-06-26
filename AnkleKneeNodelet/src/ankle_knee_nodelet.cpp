#include <AnkleKneeNodelet/ankle_knee_nodelet.hpp>

namespace ankle_knee_nodelet
{
  using namespace apptronik_system;

  // For Nodelets, use onInit() method instead of constructor
  AnkleKneeNodelet::AnkleKneeNodelet():numJoint(2) {

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
      jNameIdxMap["lknee"] = 0;
      jNameIdxMap["lankle"] = 1;
      slaveNames.resize(numJoint+1);
      medullaName = "Medulla_V2";
      slaveNames = {"lknee", "lankle", medullaName};
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
        delete jPosCmdList[i];
        delete jVelCmdList[i];
        delete jTrqCmdList[i];
    }
    delete kneeMJPos;
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

    // User calls SystemLoop Constructor:
    m_sys.reset(new SystemLoop(boost::bind(&AnkleKneeNodelet::loop, this, _1, _2), nh, slaveNames, true));


    _ClearFaults(slaveNames);


    // User sets run mode for each slave:
    m_sys->setRunMode(JOINT_IMPEDANCE, "lknee");
    m_sys->setRunMode(JOINT_IMPEDANCE, "lankle");

    // User registers a state ptr for each MISO topic with desired state info
    kneeMJPos = new double(0.);
    m_sys->registerStatePtr(kneeMJPos, "joint_position2_rad__",
            slaveNames[0]);
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
        _CallFloat32Service(nh, slaveNames[i], "kp","/Control__Joint__Impedance__KP/set");
        _CallFloat32Service(nh, slaveNames[i], "kd","/Control__Joint__Impedance__KD/set");
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
        _SetSafeCmd();
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
    }
    SensorData->kneeMJPos = *(kneeMJPos);
    SensorData->q = jPos;
    SensorData->qdot = jVel;
    SensorData->jtrq = jTrq;
  }

  void AnkleKneeNodelet::_CopyCommand() {
      jPosCmd = CommandData->q;
      jVelCmd = CommandData->qdot;
      jTrqCmd = CommandData->jtrq;
    for (int i = 0; i < numJoint; ++i) {
        *(jPosCmdList[i]) = jPosCmd[i];
        *(jVelCmdList[i]) = jVelCmd[i];
        *(jTrqCmdList[i]) = jTrqCmd[i];
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
