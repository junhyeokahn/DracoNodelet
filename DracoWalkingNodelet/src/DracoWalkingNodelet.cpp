#include "DracoWalkingNodelet/DracoWalkingNodelet.hpp"
#include <stdexcept>
#include <DracoBip_Controller/DracoBip_interface.hpp>

using namespace apptronik_ros_utils;
namespace draco_walking_nodelet
{
    DracoWalkingNodelet::DracoWalkingNodelet() {
    }

    DracoWalkingNodelet::~DracoWalkingNodelet()
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
        delete interface_;
        delete sensor_data;
        delete cmd;
    }

    void DracoWalkingNodelet::onInit()
    {
        m_nh = getNodeHandle();
        m_spin_thread.reset(new boost::thread(boost::bind(&DracoWalkingNodelet::spinThread, this)));
    }

    void DracoWalkingNodelet::spinThread()
    {

        //set up controller
        m_sync.reset(new apptronik_ros_utils::Synchronizer(true, "draco_walking_nodelet"));
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
                interface_->GetCommand(sensor_data, cmd);
            }
            _copyCommand();

            m_sync->logger->captureLine();

            //indicate that we're done
            m_sync->finishControl();

            ++mCount;
        }

        m_sync->awaitShutdownComplete();
    }

    void DracoWalkingNodelet::_setCurrentPositionCmd() {
        for (int i = 0; i < numJoint; ++i) {
            *(jPosCmdList[i]) = (*(jPosList[i]));
            *(jVelCmdList[i]) = 0.;
            *(jTrqCmdList[i]) = 0.;
        }
    }

    void DracoWalkingNodelet::_turnOff() {
        for (int i = 0; i < numJoint; ++i) {
            m_sync->changeMode("OFF", slaveNames[i]);
        }
    }

    void DracoWalkingNodelet::_initialize() {
        mCount = 0;
        numJoint = 10;
        jPosList.resize(numJoint);
        jVelList.resize(numJoint);
        jTrqList.resize(numJoint);
        temperatureList.resize(numJoint);
        motorCurrentList.resize(numJoint);
        busVoltageList.resize(numJoint);
        busCurrentList.resize(numJoint);
        jPosCmdList.resize(numJoint);
        jVelCmdList.resize(numJoint);
        jTrqCmdList.resize(numJoint);
        medullaName = "Medulla_V2";
        slaveNames.resize(numJoint+1);
        slaveNames = {"lHipYaw", "lHipRoll", "lHipPitch", "lKnee", "lAnkle",
            "rHipYaw", "rHipRoll", "rHipPitch", "rKnee", "rAnkle",
            medullaName };
        _parameterSetting();

        interface_ = new DracoBip_interface();
        sensor_data = new DracoBip_SensorData();
        cmd = new DracoBip_Command();
    }

    void DracoWalkingNodelet::_preprocess() {
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

        }
    }
    void DracoWalkingNodelet::_parameterSetting() {

        //Eigen::VectorXd jp_kp, jp_kd, t_kp, t_kd, current_limit;
        //Eigen::VectorXi en_auto_kd, en_dob;
        //try {
        //YAML::Node ll_config =
        //YAML::LoadFile(DracoBipConfigPath"LOW_LEVEL_CONFIG.yaml");
        //myUtils::readParameter(ll_config, "jp_kp", jp_kp);
        //myUtils::readParameter(ll_config, "jp_kd", jp_kd);
        //myUtils::readParameter(ll_config, "t_kp", t_kp);
        //myUtils::readParameter(ll_config, "t_kd", t_kd);
        //myUtils::readParameter(ll_config, "current_limit", current_limit);
        //myUtils::readParameter(ll_config, "en_auto_kd", en_auto_kd);
        //myUtils::readParameter(ll_config, "en_dob", en_dob);
        //} catch(std::runtime_error& e) {
        //std::cout << "Error Reading Parameter [" << e.what() << "[" << std::endl;
        //}

        //for (int i = 0; i < numJoint; ++i) {
        //apptronik_srvs::Float32 srv_float;
        //apptronik_srvs::UInt16 srv_int;
        //srv_float.request.set_data = jp_kp[i];
        //callSetService(slaveNames[i], "Control__Joint__Impedance__KP", srv_float);
        //srv_float.request.set_data = jp_kd[i];
        //callSetService(slaveNames[i], "Control__Joint__Impedance__KD", srv_float);
        //srv_float.request.set_data = t_kp[i];
        //callSetService(slaveNames[i], "Control__Actuator__Effort__KP", srv_float);
        //srv_float.request.set_data = t_kd[i];
        //callSetService(slaveNames[i], "Control__Actuator__Effort__KD", srv_float);
        //srv_float.request.set_data = current_limit[i];
        //callSetService(slaveNames[i], "Limits__Motor__Current_Max_A", srv_float);
        //srv_int.request.set_data = en_dob[i];
        //callSetService(slaveNames[i], "Control__Actuator__Effort__EN_DOB", srv_int);
        //srv_int.request.set_data = en_auto_kd[i];
        //callSetService(slaveNames[i], "Control__Actuator__Effort__AutoKD", srv_int);
        //}
    }

    void DracoWalkingNodelet::_copyData() {
        for (int i = 0; i < numJoint; ++i) {
            sensor_data->jpos[i] = static_cast<double> (*(jPosList[i]));
            sensor_data->jvel[i] = static_cast<double> (*(jVelList[i]));
            sensor_data->torque[i] = static_cast<double> (*(jTrqList[i]));

            sensor_data->temperature[i] = static_cast<double> (*(temperatureList[i]));
            sensor_data->motor_current[i] = static_cast<double> (*(motorCurrentList[i]));
            sensor_data->bus_voltage[i] = static_cast<double> (*(busVoltageList[i]));
            sensor_data->bus_current[i] = static_cast<double> (*(busCurrentList[i]));

            // TODO
            sensor_data->rotor_inertia[i] = 0.;
        }
    }

    void DracoWalkingNodelet::_copyCommand() {
        for (int i = 0; i < numJoint; ++i) {
            *(jPosCmdList[i]) = static_cast<float>(cmd->jpos_cmd[i]);
            *(jVelCmdList[i]) = static_cast<float>(cmd->jvel_cmd[i]);
            *(jTrqCmdList[i]) = static_cast<float>(cmd->jtorque_cmd[i]);   
        }
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(draco_walking_nodelet::DracoWalkingNodelet, nodelet::Nodelet)
