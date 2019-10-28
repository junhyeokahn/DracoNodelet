#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <rt_utils/synchronizer.hpp>

#include <cassert>

#include <Eigen/Dense>

#include <apptronik_srvs/UInt16.h>
#include <apptronik_srvs/Float32.h>

#include "PnC/DracoPnC/DracoInterface.hpp"

namespace draco_nodelet
{
  class DracoNodelet: public nodelet::Nodelet
  {
  public:
    void spinThread();
    void onInit();
    DracoNodelet();
    ~DracoNodelet();

  private:
    ros::NodeHandle m_nh;
    boost::shared_ptr<aptk::core::Synchronizer> m_sync;
    boost::shared_ptr<boost::thread> m_spin_thread;

    template <class SrvType>
    void callGetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj);
    template <class SrvType>
    void callSetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj);

    // States
    Eigen::VectorXd jPos;
    Eigen::VectorXd jVel;
    Eigen::VectorXd jTrq;
    Eigen::VectorXd temperature;
    Eigen::VectorXd motorCurrent;
    Eigen::VectorXd busVoltage;
    Eigen::VectorXd busCurrent;
    Eigen::VectorXd imuAngVel; // x, y, z
    Eigen::VectorXd imuAcc;  // x, y, z
    Eigen::VectorXd rotorInertia;
    bool rFootContact;
    bool lFootContact;
    Eigen::VectorXd rFootATI;
    Eigen::VectorXd lFootATI;
    std::vector<float*> jPosList;
    std::vector<float*> jVelList;
    std::vector<float*> jTrqList;
    std::vector<float*> temperatureList;
    std::vector<float*> motorCurrentList;;
    std::vector<float*> busVoltageList;
    std::vector<float*> busCurrentList;
    std::vector<float*> imuAngVelList;
    std::vector<float*> imuAccList;
    std::vector<float*> rotorInertiaList;
    std::vector<float*> rFootATIList; // Tx, Ty, Tz, Fx, Fy, Fz
    std::vector<float*> lFootATIList; // Tx, Ty, Tz, Fx, Fy, Fz

    // Commands
    Eigen::VectorXd jPosCmd;
    Eigen::VectorXd jVelCmd;
    Eigen::VectorXd jTrqCmd;
    std::vector<float*> jPosCmdList;
    std::vector<float*> jVelCmdList;
    std::vector<float*> jTrqCmdList;

    int numJoint;
    std::vector<std::string> slaveNames;
    std::string medullaName;

    // Safety Factors
    Eigen::VectorXd maxTemperature;
    Eigen::VectorXd maxPosition;
    Eigen::VectorXd minPosition;
    Eigen::VectorXd maxVelocity;
    Eigen::VectorXd maxTrq;
    void _turnOff();

    DracoInterface* interface;
    DracoSensorData* sensor_data;
    DracoCommand* cmd;

    void _initialize();
    void _preprocess();
    void _checkSensorData();
    void _checkCommand();
    void _setCurrentPositionCmd();
    void _copyData();
    void _copyCommand();
    void _parameterSetting();
    void _InterfaceInitialize();
    void _checkContact();

    int mCount;
  };

  template <class SrvType>
  void DracoNodelet::callSetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj)
  {
    std::string full_set_service = "/" + slave_name + "/" + srv_name + "/" + "set";
    ros::NodeHandle nh = getPrivateNodeHandle();  // for Nodelets

    ros::ServiceClient client = nh.serviceClient<SrvType>(full_set_service);

    if (client.call(srv_obj))
    {
      NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/" << srv_name.c_str()); // for Nodelets
    }
    else
    {
      NODELET_INFO_STREAM("Failed to call service: " << full_set_service.c_str()); // for Nodelets
    }
  }

  template <class SrvType>
  void DracoNodelet::callGetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj)
  {
    std::string full_get_service = "/" + slave_name + "/" + srv_name + "/" + "get";
    ros::NodeHandle nh = getPrivateNodeHandle();  // for Nodelets

    ros::ServiceClient client = nh.serviceClient<SrvType>(full_get_service);

    if (client.call(srv_obj))
    {
      NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/" << srv_name.c_str()); // for Nodelets
    }
    else
    {
      NODELET_INFO_STREAM("Failed to call service: " << full_get_service.c_str()); // for Nodelets
    }
  }

}
