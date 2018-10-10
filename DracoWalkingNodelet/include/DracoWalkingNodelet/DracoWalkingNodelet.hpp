#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <apptronik_ros_utils/synchronizer.hpp>

#include <cassert>

#include <Eigen/Dense>

#include <apptronik_srvs/UInt16.h>
#include <apptronik_srvs/Float32.h>

#include <DracoBip_Controller/DracoBip_DynaCtrl_Definition.h>

class interface;

namespace draco_walking_nodelet
{
  class DracoWalkingNodelet: public nodelet::Nodelet
  {
  public:
    void spinThread();
    void onInit();
    DracoWalkingNodelet();
    ~DracoWalkingNodelet();

  private:
    uint64_t m_last_rt_lnx_time_ns, m_last_ec_bus_time_ns;
    uint64_t m_rt_lnx_dt_ns,        m_ec_bus_dt_ns;

    ros::NodeHandle m_nh;
    boost::shared_ptr<apptronik_ros_utils::Synchronizer> m_sync;
    boost::shared_ptr<boost::thread> m_spin_thread;

    template <class SrvType>
    void callGetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj);
    template <class SrvType>
    void callSetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj);

    std::vector<float*> jPosList;
    std::vector<float*> jVelList;
    std::vector<float*> jTrqList;
    std::vector<float*> temperatureList;
    std::vector<float*> motorCurrentList;;
    std::vector<float*> busVoltageList;
    std::vector<float*> busCurrentList;


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

    void _turnOff();

    interface* interface_;
    DracoBip_SensorData* sensor_data;
    DracoBip_Command* cmd;

    void _initialize();
    void _preprocess();
    void _setCurrentPositionCmd();
    void _copyData();
    void _copyCommand();
    void _parameterSetting();

    int mCount;
    Eigen::VectorXd prevJPos;
    Eigen::VectorXd prevJVel;
    Eigen::VectorXd prevJTrq;
  };

  template <class SrvType>
  void DracoWalkingNodelet::callSetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj)
  {
    std::string full_set_service = "/" + slave_name + "/" + srv_name + "/" + "set";
    //ros::NodeHandle nh("~"); // for Nodes
    ros::NodeHandle nh = getPrivateNodeHandle();  // for Nodelets

    ros::ServiceClient client = nh.serviceClient<SrvType>(full_set_service);

    if (client.call(srv_obj))
    {
      // ROS_INFO("Called /%s/%s/set", slave_name.c_str(), srv_name.c_str()); // for Nodes
      NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/" << srv_name.c_str()); // for Nodelets
    }
    else
    {
      // ROS_ERROR("Failed to call service: %s", full_get_service.c_str()); // for Nodes
      NODELET_INFO_STREAM("Failed to call service: " << full_set_service.c_str()); // for Nodelets
    }
  }

  template <class SrvType>
  void DracoWalkingNodelet::callGetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj)
  {
    std::string full_get_service = "/" + slave_name + "/" + srv_name + "/" + "get";
    //ros::NodeHandle nh("~"); // for Nodes
    ros::NodeHandle nh = getPrivateNodeHandle();  // for Nodelets

    ros::ServiceClient client = nh.serviceClient<SrvType>(full_get_service);

    if (client.call(srv_obj))
    {
      // ROS_INFO("Called /%s/%s/set", slave_name.c_str(), srv_name.c_str()); // for Nodes
      NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/" << srv_name.c_str()); // for Nodelets
    }
    else
    {
      // ROS_ERROR("Failed to call service: %s", full_get_service.c_str()); // for Nodes
      NODELET_INFO_STREAM("Failed to call service: " << full_get_service.c_str()); // for Nodelets
    }
  }

}
