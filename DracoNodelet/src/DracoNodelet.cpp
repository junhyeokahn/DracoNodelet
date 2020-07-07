#include "DracoNodelet/DracoNodelet.hpp"
#include <stdexcept>
#include "Configuration.h"
#include "ExternalSource/myYaml/include/myYaml/node/node.h"
#include "Utils/General/Clock.hpp"
#include "Utils/IO/IOUtilities.hpp"
#include "Utils/Math/MathUtilities.hpp"

using namespace aptk::core;
namespace draco_nodelet {
DracoNodelet::DracoNodelet() {}

DracoNodelet::~DracoNodelet() {
    m_spin_thread->join();

    for (int i = 0; i < numJoint; ++i) {
        delete jPosList[i];
        delete jVelList[i];
        delete jTrqList[i];
        delete temperatureList[i];
        delete motorCurrentList[i];
        delete busVoltageList[i];
        delete busCurrentList[i];
        delete rotorInertiaList[i];
        delete jPosCmdList[i];
        delete jVelCmdList[i];
        delete jTrqCmdList[i];
    }
    for (int i = 0; i < 3; ++i) {
        delete imuAccList[i];
        delete imuAngVelList[i];
        delete imuMagList[i];
    }
    for (int i = 0; i < 6; ++i) {
        delete rFootATIList[i];
        delete lFootATIList[i];
    }

    delete interface;
    delete sensor_data;
    delete cmd;
}

void DracoNodelet::onInit() {
    m_nh = getNodeHandle();
    m_spin_thread.reset(
        new boost::thread(boost::bind(&DracoNodelet::spinThread, this)));
}

void DracoNodelet::spinThread() {
    // set up controller
    m_sync.reset(new aptk::core::Synchronizer(true, "draco_nodelet"));
    m_sync->connect();
    aptk::core::enableRT(5, 2);

    // Initialize
    _initialize();
    _preprocess();

    // bool previously_faulted = false;
    for (std::size_t i = 0; i < slaveNames.size(); ++i) {
        m_sync->clearFaults(slaveNames[i]);
    }

    Clock clock_;
    // main control loop
    while (m_sync->ok()) {
        // clock_.start();

        // wait for bus transaction
        m_sync->awaitNextControl();

        _copyData();
        _setCurrentPositionCmd();

        if (m_sync->printIndicatedFaults()) {
            // interface->getCommand(sensor_data, cmd);
        } else {
            interface->getCommand(sensor_data, cmd);
            jPosCmd = cmd->q;
            jVelCmd = cmd->qdot;
            jTrqCmd = cmd->jtrq;
        }
        _copyCommand();

        // printf("time: %f\n", clock_.stop());
        // m_sync->logger->captureLine();

        // indicate that we're done
        m_sync->finishControl();

        ++mCount;
    }

    m_sync->awaitShutdownComplete();
}

void DracoNodelet::_setCurrentPositionCmd() {
    jPosCmd = jPos;
    jVelCmd.setZero();
    jTrqCmd.setZero();
}

void DracoNodelet::_turnOff() {
    if (mCount > 400) {
        for (int i = 0; i < numJoint; ++i) {
            m_sync->changeMode("OFF", slaveNames[i]);
        }
    }
}

void DracoNodelet::_initialize() {
    mCount = 0;
    numJoint = 10;
    jPos = Eigen::VectorXd::Zero(numJoint);
    jVel = Eigen::VectorXd::Zero(numJoint);
    jTrq = Eigen::VectorXd::Zero(numJoint);
    temperature = Eigen::VectorXd::Zero(numJoint);
    motorCurrent = Eigen::VectorXd::Zero(numJoint);
    busVoltage = Eigen::VectorXd::Zero(numJoint);
    busCurrent = Eigen::VectorXd::Zero(numJoint);
    imuAngVel = Eigen::VectorXd::Zero(3);
    imuAcc = Eigen::VectorXd::Zero(3);
    imuMag = Eigen::VectorXd::Zero(3);
    rotorInertia = Eigen::VectorXd::Zero(numJoint);
    rFootContact = false;
    lFootContact = false;
    rFootATI = Eigen::VectorXd::Zero(6);
    lFootATI = Eigen::VectorXd::Zero(6);

    jPosList.resize(numJoint);
    jVelList.resize(numJoint);
    jTrqList.resize(numJoint);
    temperatureList.resize(numJoint);
    motorCurrentList.resize(numJoint);
    busVoltageList.resize(numJoint);
    busCurrentList.resize(numJoint);
    imuAngVelList.resize(3);
    imuAccList.resize(3);
    imuMagList.resize(3);
    rotorInertiaList.resize(numJoint);
    rFootATIList.resize(6);
    lFootATIList.resize(6);

    jPosCmd = Eigen::VectorXd::Zero(numJoint);
    jVelCmd = Eigen::VectorXd::Zero(numJoint);
    jTrqCmd = Eigen::VectorXd::Zero(numJoint);

    jPosCmdList.resize(numJoint);
    jVelCmdList.resize(numJoint);
    jTrqCmdList.resize(numJoint);

    medullaName = "Medulla_V3";
    slaveNames.resize(numJoint + 1);
    slaveNames = {"lHipYaw", "lHipRoll", "lHipPitch", "lKnee",
                  "lAnkle",  "rHipYaw",  "rHipRoll",  "rHipPitch",
                  "rKnee",   "rAnkle",   "Medulla_V3"};
    _parameterSetting();

    interface = new DracoInterface();
    sensor_data = new DracoSensorData();
    cmd = new DracoCommand();
    _InterfaceInitialize();
    try {
        YAML::Node ll_config =
            YAML::LoadFile(THIS_COM "Config/Draco/LOW_LEVEL_CONFIG.yaml");
        YAML::Node safety_turn_off_cfg = ll_config["safety_turn_off"];
        myUtils::readParameter(safety_turn_off_cfg, "max_temperature",
                               maxTemperature);
        myUtils::readParameter(safety_turn_off_cfg, "max_position",
                               maxPosition);
        myUtils::readParameter(safety_turn_off_cfg, "min_position",
                               minPosition);
        myUtils::readParameter(safety_turn_off_cfg, "max_velocity",
                               maxVelocity);
        myUtils::readParameter(safety_turn_off_cfg, "max_trq", maxTrq);
    } catch (std::runtime_error& e) {
        std::cout << "Error Reading Parameter [" << e.what() << "]"
                  << std::endl;
    }
}

void DracoNodelet::_InterfaceInitialize() {
    cmd->q = Eigen::VectorXd::Zero(10);
    cmd->qdot = Eigen::VectorXd::Zero(10);
    cmd->jtrq = Eigen::VectorXd::Zero(10);
}

void DracoNodelet::_preprocess() {
    for (int i = 0; i < numJoint; ++i) {
        // Set Run Mode
        m_sync->changeMode("JOINT_IMPEDANCE", slaveNames[i]);

        // Register State
        jPosList[i] = new float(0.);
        m_sync->registerMISOPtr(jPosList[i], "js__joint__position__rad",
                                slaveNames[i], false);
        jVelList[i] = new float(0.);
        m_sync->registerMISOPtr(jVelList[i], "js__joint__velocity__radps",
                                slaveNames[i], false);
        jTrqList[i] = new float(0.);
        m_sync->registerMISOPtr(jTrqList[i], "js__joint__effort__Nm",
                                slaveNames[i], false);
        temperatureList[i] = new float(0.);
        m_sync->registerMISOPtr(temperatureList[i], "motor__core_temp_est__C",
                                slaveNames[i], false);
        motorCurrentList[i] = new float(0.);
        m_sync->registerMISOPtr(motorCurrentList[i], "motor__current__A",
                                slaveNames[i], false);
        busVoltageList[i] = new float(0.);
        m_sync->registerMISOPtr(busVoltageList[i], "energetics__bus_voltage__V",
                                slaveNames[i], false);
        busCurrentList[i] = new float(0.);
        // m_sync->registerMISOPtr(busCurrentList[i],
        // "energetics__bus_current__A", slaveNames[i], false);
        rotorInertiaList[i] = new float(0.);
        m_sync->registerMISOPtr(rotorInertiaList[i],
                                "joint__reflected__motor__inertia__kgm2",
                                slaveNames[i]);

        // Register Command
        jPosCmdList[i] = new float(0.);
        m_sync->registerMOSIPtr(jPosCmdList[i], "cmd__joint__position__rad",
                                slaveNames[i], false);
        jVelCmdList[i] = new float(0.);
        m_sync->registerMOSIPtr(jVelCmdList[i], "cmd__joint__velocity__radps",
                                slaveNames[i], false);
        jTrqCmdList[i] = new float(0.);
        m_sync->registerMOSIPtr(jTrqCmdList[i], "cmd__joint__effort__nm",
                                slaveNames[i], false);
    }
    for (int i = 0; i < 3; ++i) {
        imuAngVelList[i] = new float(0.);
        imuAccList[i] = new float(0.);
        imuMagList[i] = new float(0.);
    }
    m_sync->registerMISOPtr(imuAngVelList[0], "gyro__x__angularRate__radps",
                            medullaName, false);
    m_sync->registerMISOPtr(imuAngVelList[1], "gyro__y__angularRate__radps",
                            medullaName, false);
    m_sync->registerMISOPtr(imuAngVelList[2], "gyro__z__angularRate__radps",
                            medullaName, false);
    m_sync->registerMISOPtr(imuAccList[0],
                            "accelerometer__x__acceleration__mps2", medullaName,
                            false);
    m_sync->registerMISOPtr(imuAccList[1],
                            "accelerometer__y__acceleration__mps2", medullaName,
                            false);
    m_sync->registerMISOPtr(imuAccList[2],
                            "accelerometer__z__acceleration__mps2", medullaName,
                            false);
    // TODO: Name
    m_sync->registerMISOPtr(imuMagList[0], "magnetometer__x__field__gauss",
                            medullaName, false);
    m_sync->registerMISOPtr(imuMagList[1], "magnetometer__y__field__gauss",
                            medullaName, false);
    m_sync->registerMISOPtr(imuMagList[2], "magnetometer__z__field__gauss",
                            medullaName, false);

    for (int i = 0; i < 6; ++i) {
        rFootATIList[i] = new float(0.);
        lFootATIList[i] = new float(0.);
    }

    m_sync->registerMISOPtr(lFootATIList[0], "ati__Tx__filt__Nm", "lAnkle",
                            false);
    m_sync->registerMISOPtr(lFootATIList[1], "ati__Ty__filt__Nm", "lAnkle",
                            false);
    m_sync->registerMISOPtr(lFootATIList[2], "ati__Tz__filt__Nm", "lAnkle",
                            false);
    m_sync->registerMISOPtr(lFootATIList[3], "ati__Fx__filt__N", "lAnkle",
                            false);
    m_sync->registerMISOPtr(lFootATIList[4], "ati__Fy__filt__N", "lAnkle",
                            false);
    m_sync->registerMISOPtr(lFootATIList[5], "ati__Fz__filt__N", "lAnkle",
                            false);

    m_sync->registerMISOPtr(rFootATIList[0], "ati__Tx__filt__Nm", "rAnkle",
                            false);
    m_sync->registerMISOPtr(rFootATIList[1], "ati__Ty__filt__Nm", "rAnkle",
                            false);
    m_sync->registerMISOPtr(rFootATIList[2], "ati__Tz__filt__Nm", "rAnkle",
                            false);
    m_sync->registerMISOPtr(rFootATIList[3], "ati__Fx__filt__N", "rAnkle",
                            false);
    m_sync->registerMISOPtr(rFootATIList[4], "ati__Fy__filt__N", "rAnkle",
                            false);
    m_sync->registerMISOPtr(rFootATIList[5], "ati__Fz__filt__N", "rAnkle",
                            false);
}

void DracoNodelet::_parameterSetting() {
    Eigen::VectorXd jp_kp, jp_kd, t_kp, t_kd, current_limit, temperature_limit,
        rotor_inertia;
    Eigen::VectorXi en_auto_kd, en_dob;
    try {
        YAML::Node ll_config =
            YAML::LoadFile(THIS_COM "Config/Draco/LOW_LEVEL_CONFIG.yaml");
        YAML::Node service_call_cfg = ll_config["service_call"];
        myUtils::readParameter(service_call_cfg, "jp_kp", jp_kp);
        myUtils::readParameter(service_call_cfg, "jp_kd", jp_kd);
        myUtils::readParameter(service_call_cfg, "t_kp", t_kp);
        myUtils::readParameter(service_call_cfg, "t_kd", t_kd);
        myUtils::readParameter(service_call_cfg, "current_limit",
                               current_limit);
        myUtils::readParameter(service_call_cfg, "en_auto_kd", en_auto_kd);
        myUtils::readParameter(service_call_cfg, "en_dob", en_dob);
        myUtils::readParameter(service_call_cfg, "temperature_limit",
                               temperature_limit);
        myUtils::readParameter(service_call_cfg, "rotor_inertia",
                               rotor_inertia);
    } catch (std::runtime_error& e) {
        std::cout << "Error Reading Parameter [" << e.what() << "["
                  << std::endl;
    }

    for (int i = 0; i < numJoint; ++i) {
        apptronik_srvs::Float32 srv_float;
        apptronik_srvs::UInt16 srv_int;
        srv_float.request.set_data = jp_kp[i];
        callSetService(slaveNames[i], "Control__Joint__Impedance__KP",
                       srv_float);
        srv_float.request.set_data = jp_kd[i];
        callSetService(slaveNames[i], "Control__Joint__Impedance__KD",
                       srv_float);
        srv_float.request.set_data = t_kp[i];
        callSetService(slaveNames[i], "Control__Actuator__Effort__KP",
                       srv_float);
        srv_float.request.set_data = t_kd[i];
        callSetService(slaveNames[i], "Control__Actuator__Effort__KD",
                       srv_float);
        srv_float.request.set_data = current_limit[i];
        callSetService(slaveNames[i], "Limits__Motor__Current_Max_A",
                       srv_float);
        srv_float.request.set_data = rotor_inertia[i];
        callSetService(slaveNames[i], "Actuator__Sprung_mass_kg", srv_float);
        srv_float.request.set_data = temperature_limit[i];
        callSetService(slaveNames[i], "Limits__Motor__Max_winding_temp_C",
                       srv_float);
        srv_int.request.set_data = en_dob[i];
        callSetService(slaveNames[i], "Control__Actuator__Effort__EN_DOB",
                       srv_int);
        srv_int.request.set_data = en_auto_kd[i];
        callSetService(slaveNames[i], "Control__Actuator__Effort__AutoKD",
                       srv_int);
    }
}

void DracoNodelet::_checkSensorData() {
    if (!(myUtils::isInBoundingBox(jPos, minPosition, maxPosition))) {
        std::cout << "Measured Joint Position Hits the Limit" << std::endl;
        _turnOff();
    } else if (!(myUtils::isInBoundingBox(
                   temperature, Eigen::VectorXd::Constant(numJoint, -50.),
                   maxTemperature))) {
        std::cout << "Temperature Hits the Limit" << std::endl;
        _turnOff();
    } else if (!(myUtils::isInBoundingBox(jVel, -maxVelocity, maxVelocity))) {
        std::cout << "Measured Joint Velocity Hits the Limit" << std::endl;
        _turnOff();
    } else if (!(myUtils::isInBoundingBox(jTrq, -maxTrq, maxTrq))) {
        std::cout << "Measured Joint Torque Hits the Limit" << std::endl;
        _turnOff();
    } else {
        // Do Nothing
    }

    if (fabs(imuAcc[1]) < 0.00001) {
        if (mCount % 1000 == 1) {
            myUtils::pretty_print(imuAcc, std::cout, "imu_acc");
            _turnOff();
        }
    }
}

void DracoNodelet::_checkCommand() {
    if (!(myUtils::isInBoundingBox(jPosCmd, minPosition, maxPosition))) {
        std::cout << "Commanded Joint Position Hits the Limit" << std::endl;
        _turnOff();
    } else if (!(myUtils::isInBoundingBox(jVelCmd, -maxVelocity,
                                          maxVelocity))) {
        std::cout << "Commanded Joint Velocity Hits the Limit" << std::endl;
        _turnOff();
    } else if (!(myUtils::isInBoundingBox(jTrqCmd, -maxTrq, maxTrq))) {
        std::cout << "Commanded Joint Torque Hits the Limit" << std::endl;
        _turnOff();
    }
}

void DracoNodelet::_copyData() {
    for (int i = 0; i < numJoint; ++i) {
        jPos[i] = static_cast<double>(*(jPosList[i]));
        jVel[i] = static_cast<double>(*(jVelList[i]));
        jTrq[i] = static_cast<double>(*(jTrqList[i]));
        temperature[i] = static_cast<double>(*(temperatureList[i]));
        motorCurrent[i] = static_cast<double>(*(motorCurrentList[i]));
        busVoltage[i] = static_cast<double>(*(busVoltageList[i]));
        busCurrent[i] = static_cast<double>(*(busCurrentList[i]));
        rotorInertia[i] = static_cast<double>(*(rotorInertiaList[i]));
    }
    for (int i = 0; i < 3; ++i) {
        imuAngVel[i] = static_cast<double>(*(imuAngVelList[i]));
        imuAcc[i] = static_cast<double>(*(imuAccList[i]));
        imuMag[i] = static_cast<double>(*(imuMagList[i]));
    }
    for (int i = 0; i < 6; ++i) {
        rFootATI[i] = static_cast<double>(*(rFootATIList[i]));
        lFootATI[i] = static_cast<double>(*(lFootATIList[i]));
    }
    _checkSensorData();
    sensor_data->q = jPos;
    sensor_data->qdot = jVel;
    sensor_data->jtrq = jTrq;
    sensor_data->motor_current = motorCurrent;
    sensor_data->bus_voltage = busVoltage;
    sensor_data->temperature = temperature;
    sensor_data->bus_current = busCurrent;
    sensor_data->imu_ang_vel = imuAngVel;
    sensor_data->imu_acc = imuAcc;
    sensor_data->imu_mag = imuMag;
    sensor_data->rotor_inertia = rotorInertia;
    sensor_data->rfoot_contact = rFootContact;
    sensor_data->lfoot_contact = lFootContact;
    sensor_data->rf_wrench = rFootATI;
    sensor_data->lf_wrench = lFootATI;
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
