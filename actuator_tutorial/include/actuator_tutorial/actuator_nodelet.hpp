#pragma once

#include <apptronik_srvs/Float32.h>
#include <cortex_utils/debug_interfacer.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <rt_utils/synchronizer.hpp>

#include <cassert>

#include <Eigen/Dense>

namespace actuator_nodelet {

namespace control_modes {
constexpr int kOff = 0;
constexpr int kCurrentControlMode = 1;
constexpr int kJointImpedanceControlMode = 2;
} // namespace control_modes

class ActuatorNodelet : public nodelet::Nodelet {
public:
  void spinThread();
  void onInit();
  ActuatorNodelet();
  ~ActuatorNodelet();

private:
  ros::NodeHandle nh_;
  boost::shared_ptr<aptk::comm::Synchronizer> sync_;
  boost::shared_ptr<boost::thread> spin_thread_;
  boost::shared_ptr<aptk::util::DebugInterfacer> interfacer_; // for plot

  std::vector<std::string> slave_names_;
  std::string medulla_name_;

  int count_;
  int n_joint_;

  int control_mode_;

  // data
  std::vector<float *> data_joint_positions_;
  std::vector<float *> data_joint_velocities_;

  // cmd
  std::vector<float *> cmd_joint_current_;
  std::vector<float *> cmd_joint_positions_;
  std::vector<float *> cmd_joint_velocities_;
  std::vector<float *> cmd_joint_efforts_;

  void RegisterData();
  void CopyData();
  void CopyCommand();
  void SetServices();

  template <class SrvType>
  void CallGetService(const std::string &slave_name,
                      const std::string &srv_name, SrvType &srv_obj);
  template <class SrvType>
  void CallSetService(const std::string &slave_name,
                      const std::string &srv_name, SrvType &srv_obj);

  Eigen::Vector2d plot_state_;
  double plot_cmd_joint_current_;
  Eigen::Vector3d plot_cmd_joint_impedance_;

  ros::ServiceServer mode_handler_;
  ros::ServiceServer current_control_handler_;
  ros::ServiceServer joint_impedance_control_handler_;

  bool ModeChange(apptronik_srvs::Float32::Request &req,
                  apptronik_srvs::Float32::Response &res);
  bool CurrentControlHandler(apptronik_srvs::Float32::Request &req,
                             apptronik_srvs::Float32::Response &res);
  bool JointImpedanceControlHandler(apptronik_srvs::Float32::Request &req,
                                    apptronik_srvs::Float32::Response &res);
  void SetSafeCommand();
  void SetCurrentCommand();
  void SetJointImpedanceCommand();
  void ClearFaults() {
    for (std::size_t i = 0; i < n_joint_; ++i) {
      sync_->clearFaults(slave_names_[i]);
    }
  };

  double sin_mid_;
  double sin_amp_;
  double curr_time_; // TODO: how to get it
  double start_time_;
};

} // namespace actuator_nodelet
