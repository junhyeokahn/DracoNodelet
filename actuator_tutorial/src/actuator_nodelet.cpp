#include <stdexcept>

#include <actuator_tutorial/actuator_nodelet.hpp>

using namespace aptk::comm;
namespace actuator_nodelet {
ActuatorNodelet::ActuatorNodelet() {
  count_ = 0;
  slave_names_ = {"QDM9_3"};
  n_joint_ = 1;

  // data
  data_joint_positions_.resize(n_joint_);
  data_joint_velocities_.resize(n_joint_);

  // current control mode
  cmd_joint_current_.resize(n_joint_);

  // joint impedance mode
  cmd_joint_positions_.resize(n_joint_);
  cmd_joint_velocities_.resize(n_joint_);
  cmd_joint_efforts_.resize(n_joint_);

  // for plotting
  plot_state_.setZero();
  plot_cmd_joint_current_ = 0.;
  plot_cmd_joint_impedance_.setZero();

  // control mode
  control_mode_ = control_modes::kOff;

  // control params
  sin_mid_ = 0.;
  sin_amp_ = 0.;
  start_time_ = 0.;
  curr_time_ = 0.;
}

ActuatorNodelet::~ActuatorNodelet() {
  spin_thread_->join();
  for (int i = 0; i < n_joint_; ++i) {
    delete data_joint_positions_[i];
    delete data_joint_velocities_[i];

    delete cmd_joint_current_[i];

    delete cmd_joint_positions_[i];
    delete cmd_joint_velocities_[i];
    delete cmd_joint_efforts_[i];
  }
}

void ActuatorNodelet::onInit() {
  nh_ = getNodeHandle();
  spin_thread_.reset(
      new boost::thread(boost::bind(&ActuatorNodelet::spinThread, this)));

  mode_handler_ =
      nh_.advertiseService("/mode_handler", &ActuatorNodelet::ModeChange, this);
  current_control_handler_ =
      nh_.advertiseService("/current_control_handler",
                           &ActuatorNodelet::CurrentControlHandler, this);
  joint_impedance_control_handler_ = nh_.advertiseService(
      "/joint_impedance_control_handler",
      &ActuatorNodelet::JointImpedanceControlHandler, this);
}

bool ActuatorNodelet::ModeChange(apptronik_srvs::Float32::Request &req,
                                 apptronik_srvs::Float32::Response &res) {
  this->ClearFaults();
  double data = static_cast<double>(req.set_data);
  std::cout << "Data Received : " << data << std::endl;
  if (data == 0) {
    std::cout << "change to off mode" << std::endl;
    for (int i = 0; i < n_joint_; ++i) {
      sync_->changeMode("OFF", slave_names_[i]);
      control_mode_ = control_modes::kOff;
    }
  } else if (data == 1) {
    std::cout << "change to motor current control mode" << std::endl;
    for (int i = 0; i < n_joint_; ++i) {
      sync_->changeMode("MOTOR_CURRENT", slave_names_[i]);
      control_mode_ = control_modes::kCurrentControlMode;
    }
  } else if (data == 2) {
    std::cout << "change to motor joint impedance control mode" << std::endl;
    for (int i = 0; i < n_joint_; ++i) {
      sync_->changeMode("JOINT_IMPEDANCE", slave_names_[i]);
      control_mode_ = control_modes::kJointImpedanceControlMode;
    }
  } else {
    std::cout << "wrong data received" << std::endl;
    return false;
  }
  return true;
}

bool ActuatorNodelet::CurrentControlHandler(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  if (control_mode_ == control_modes::kCurrentControlMode) {
    plot_cmd_joint_current_ = static_cast<double>(req.set_data);
    std::cout << "target amp : " << plot_cmd_joint_current_ << std::endl;
    return true;
  } else {
    std::cout << "current mode " << control_mode_
              << " is not current control mode" << std::endl;
    return false;
  }
}

bool ActuatorNodelet::JointImpedanceControlHandler(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  if (control_mode_ == control_modes::kJointImpedanceControlMode) {
    sin_amp_ = static_cast<double>(req.set_data);
    sin_mid_ = plot_state_[0];
    start_time_ = curr_time_;
    std::cout << "sin amp : " << sin_amp_ << std::endl;
    std::cout << "sin mid : " << sin_mid_ << std::endl;
    std::cout << "start time : " << start_time_ << std::endl;
    return true;
  } else {
    std::cout << "current mode " << control_mode_
              << " is not joint impedance control mode" << std::endl;
    return false;
  }
}

void ActuatorNodelet::SetSafeCommand() {
  // set zero command
  plot_cmd_joint_impedance_.setZero();
  plot_cmd_joint_impedance_[0] = plot_state_[0];
  plot_cmd_joint_current_ = 0.;
}

void ActuatorNodelet::spinThread() {
  // set up controller
  sync_.reset(new aptk::comm::Synchronizer(true, "actuator_nodelet"));
  sync_->connect();
  interfacer_.reset(new aptk::util::DebugInterfacer(
      "actuator", sync_->getNodeHandle(), sync_->getLogger()));
  // interfacer_->addEigen(&plot_state_, "/actuator_state", {"x", "xdot"});
  // interfacer_->addPrimitive(&plot_cmd_joint_current_, "double");

  aptk::comm::enableRT(5, 2);

  // Initialize
  RegisterData();
  SetServices();
  ClearFaults();

  // main control loop
  while (sync_->ok()) {

    // wait for bus transaction
    sync_->awaitNextControl();

    CopyData();
    if (sync_->printIndicatedFaults()) {
      // Faulted
      SetSafeCommand();
    } else {
      // Compute Commands : Update plot_cmd_joint_impedance_ variable
      if (control_mode_ == control_modes::kOff) {
        SetSafeCommand();
      } else if (control_mode_ == control_modes::kCurrentControlMode) {
        SetCurrentCommand();
      } else if (control_mode_ == control_modes::kJointImpedanceControlMode) {
        SetJointImpedanceCommand();
      }

      CopyCommand();
    }

    sync_->getLogger()->captureLine();

    // indicate that we're done
    sync_->finishControl();

    ++count_;
    curr_time_ += 1. / 500.;
    // interfacer_->updateDebug();
  }

  sync_->awaitShutdownComplete();
}

void ActuatorNodelet::RegisterData() {
  for (int i = 0; i < n_joint_; ++i) {
    // Register State
    data_joint_positions_[i] = new float(0.);
    sync_->registerMISOPtr(data_joint_positions_[i], "js__joint__position__rad",
                           slave_names_[i], false);
    data_joint_velocities_[i] = new float(0.);
    sync_->registerMISOPtr(data_joint_velocities_[i],
                           "js__joint__velocity__radps", slave_names_[i],
                           false);

    // Register Command
    cmd_joint_current_[i] = new float(0.);
    sync_->registerMOSIPtr(cmd_joint_current_[i], "cmd__motor__effort__a",
                           slave_names_[i], false);

    cmd_joint_positions_[i] = new float(0.);
    sync_->registerMOSIPtr(cmd_joint_positions_[i], "cmd__joint__position__rad",
                           slave_names_[i], false);
    cmd_joint_velocities_[i] = new float(0.);
    sync_->registerMOSIPtr(cmd_joint_positions_[i],
                           "cmd__joint__velocity__radps", slave_names_[i],
                           false);
    cmd_joint_efforts_[i] = new float(0.);
    sync_->registerMOSIPtr(cmd_joint_positions_[i], "cmd__joint__effort__nm",
                           slave_names_[i], false);
  }

  // interfacer_->addPrimitive(data_joint_positions_[0], "jpos");
  // interfacer_->addPrimitive(data_joint_velocities_[0], "jvel");
}

void ActuatorNodelet::CopyData() {
  plot_state_ << *(data_joint_positions_[0]), *(data_joint_velocities_[0]);
}

void ActuatorNodelet::CopyCommand() {
  // Copy command from Command
  for (int i = 0; i < n_joint_; ++i) {
    *(cmd_joint_current_[i]) = plot_cmd_joint_current_;
    *(cmd_joint_positions_[i]) = plot_cmd_joint_impedance_[0];
    *(cmd_joint_velocities_[i]) = plot_cmd_joint_impedance_[1];
    *(cmd_joint_efforts_[i]) = 0.;
  }
}

template <class SrvType>
void ActuatorNodelet::CallSetService(const std::string &slave_name,
                                     const std::string &srv_name,
                                     SrvType &srv_obj) {
  std::string full_set_service =
      "/" + slave_name + "/" + srv_name + "/" + "set";
  ros::NodeHandle nh = getPrivateNodeHandle(); // for Nodelets

  ros::ServiceClient client = nh.serviceClient<SrvType>(full_set_service);

  if (client.call(srv_obj)) {
    NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/"
                                   << srv_name.c_str()); // for Nodelets
  } else {
    NODELET_INFO_STREAM(
        "Failed to call service: " << full_set_service.c_str()); // for Nodelets
  }
}

template <class SrvType>
void ActuatorNodelet::CallGetService(const std::string &slave_name,
                                     const std::string &srv_name,
                                     SrvType &srv_obj) {
  std::string full_get_service =
      "/" + slave_name + "/" + srv_name + "/" + "get";
  ros::NodeHandle nh = getPrivateNodeHandle(); // for Nodelets

  ros::ServiceClient client = nh.serviceClient<SrvType>(full_get_service);

  if (client.call(srv_obj)) {
    NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/"
                                   << srv_name.c_str()); // for Nodelets
  } else {
    NODELET_INFO_STREAM(
        "Failed to call service: " << full_get_service.c_str()); // for Nodelets
  }
}

void ActuatorNodelet::SetServices() {
  apptronik_srvs::Float32 srv_float;
  // TODO Determine from UI
  srv_float.request.set_data = 0.;
  for (int i = 0; i < n_joint_; ++i) {
    CallSetService(slave_names_[i], "Control__Joint__Impedance__KP", srv_float);
  }
  // TODO Determine from UI
  srv_float.request.set_data = 0.;
  for (int i = 0; i < n_joint_; ++i) {
    CallSetService(slave_names_[i], "Control__Joint__Impedance__KD", srv_float);
  }
}

void ActuatorNodelet::SetCurrentCommand() {
  // zero out joint impedance
  plot_cmd_joint_impedance_.setZero();
  plot_cmd_joint_impedance_[0] = plot_state_[0];
}

void ActuatorNodelet::SetJointImpedanceCommand() {
  // set joint impedance
  plot_cmd_joint_impedance_.setZero();
  // TODO set sinusoidal pos, vel cmd
  // zero out joint impedance
  plot_cmd_joint_current_ = 0.;
}

} // namespace actuator_nodelet

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(actuator_nodelet::ActuatorNodelet, nodelet::Nodelet)
