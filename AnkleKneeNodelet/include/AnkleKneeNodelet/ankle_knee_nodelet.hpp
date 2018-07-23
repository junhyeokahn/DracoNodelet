#pragma once

#include <memory>
#include <apptronik_system/system.hpp>
#include <AnkleKneeInterface/AnkleKneeInterface.hpp>
#include <apptronik_srvs/UInt16.h>
#include <Eigen/Dense>

/**  A clean and empty example Nodelet Controller for you to copy and customize. */
namespace ankle_knee_nodelet
{
  using namespace apptronik_system;

  /**  An empty Nodelet controller that does nothing. */
  class AnkleKneeNodelet: public nodelet::Nodelet
  {
  public:
    boost::shared_ptr<SystemLoop> m_sys;
    boost::shared_ptr<boost::thread> m_system_thread;


    // States
    unsigned int m_fault_status;
    int mControlMode;
    int mEnableSpring;
    Eigen::VectorXd jPos;
    Eigen::VectorXd jVel;
    Eigen::VectorXd jTrq;
    Eigen::VectorXd chirp;
    Eigen::VectorXd chirpInput;
    Eigen::VectorXd chirpOutput;
    Eigen::VectorXd busVoltage;
    Eigen::VectorXd coreTemp;

    std::vector<double*> jPosList;
    std::vector<double*> jVelList;
    std::vector<double*> jTrqList;
    std::vector<double*> chirpList;
    std::vector<double*> chirpInputList;
    std::vector<double*> chirpOutputList;
    std::vector<unsigned int*> nanosecondList;
    std::vector<double*> busVoltageList;
    std::vector<double*> coreTempList;

    // Commands
    Eigen::VectorXd jTrqCmd;
    Eigen::VectorXd jPosCmd;
    Eigen::VectorXd jVelCmd;
    std::vector<double*> jPosCmdList;
    std::vector<double*> jVelCmdList;
    std::vector<double*> jTrqCmdList;

    int numJoint;
    std::map<std::string, int> jNameIdxMap;
    std::vector<std::string> slaveNames;
    std::string medullaName;
    std::shared_ptr<AnkleKneeInterface> Interface;
    std::shared_ptr<AnkleKneeSensorData> SensorData;
    std::shared_ptr<AnkleKneeCommand> CommandData;


    AnkleKneeNodelet();
    ~AnkleKneeNodelet();
    void onInit();
    void systemThread();
    void loop(const double& time, const dBitset& fault_bitmap );
    void _CallFloat32Service(const ros::NodeHandle & nh,
                             const std::string & slave_name,
                             const std::string & parameter_name,
                             const std::string & service_name);

    void _CallInt16Service(const ros::NodeHandle & nh,
                           const std::string & slave_name,
                           const std::string & parameter_name,
                           const std::string & service_name);
    void _CopyData();
    void _CopyCommand();
    void _ClearFaults(std::vector<std::string> slave_names);
    void _SetSafeCmd();
  };
}
