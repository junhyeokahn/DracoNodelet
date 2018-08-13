#pragma once

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <vector>

class AnkleKneeCommand
{
public:
    Eigen::Vector2d q;
    Eigen::Vector2d qdot;
    Eigen::Vector2d jtrq;
};

class AnkleKneeSensorData
{
public:
    Eigen::Vector2d q;
    Eigen::Vector2d qdot;
    Eigen::Vector2d jtrq;
    Eigen::Vector2d chirp;
    Eigen::Vector2d chirpInput;
    Eigen::Vector2d chirpOutput;
    Eigen::Vector2d busVoltage;
    Eigen::Vector2d currentMsr;
    Eigen::Vector2d coreTemp;
    unsigned int nanosecondKnee;
    unsigned int nanosecondAnkle;
};

class AnkleKneeInterface
{
public:
    AnkleKneeInterface ();
    virtual ~AnkleKneeInterface ();

    void getCommand(std::shared_ptr<AnkleKneeSensorData> data,
                    std::shared_ptr<AnkleKneeCommand> cmd);
private:
    double mTime;
    int mCount;
    double mInitTime;
    double mServoRate;
    int mNumJoint;
    Eigen::Vector2d mInitQ;

    //Chirp related
    double m_high_freq;
    double m_low_freq;
    double m_rate;
    double m_amp;
    double m_offset;
    double m_start_dur;
    double m_end_dur;
    double m_last_signal;
    double m_last_velocity;
    std::vector<double> m_save_input;
    std::vector<double> m_save_output;
    std::vector<double> m_save_time;
    std::vector<double> m_save_temp;
    std::vector<double> m_debug_knee_q;
    std::vector<double> m_debug_knee_qdot;
    std::vector<double> m_debug_knee_effort;
    std::vector<double> m_debug_bus_voltage;
    std::vector<double> m_debug_currentMsr;
    bool m_is_saved;
    bool m_is_saved_cmd;
    int mMode; // 0 : joint impedance, 1 : currrent, 2: motor pos
    int mLinearChirp;

    void _initialize(std::shared_ptr<AnkleKneeSensorData> data,
                     std::shared_ptr<AnkleKneeCommand> cmd);
    void _maintainInitialPosition(std::shared_ptr<AnkleKneeSensorData> data,
                                  std::shared_ptr<AnkleKneeCommand> cmd);
    void _sinusoidalPosition(std::shared_ptr<AnkleKneeSensorData> data,
                             std::shared_ptr<AnkleKneeCommand> cmd);
    void _chirpSignal(std::shared_ptr<AnkleKneeSensorData> data,
                      std::shared_ptr<AnkleKneeCommand> cmd);
    void _stepSignal(std::shared_ptr<AnkleKneeSensorData> data,
                     std::shared_ptr<AnkleKneeCommand> cmd);
    void _bangControl(std::shared_ptr<AnkleKneeSensorData> data,
                      std::shared_ptr<AnkleKneeCommand> cmd);

    Eigen::Vector2d mJPosAct;
    Eigen::Vector2d mJVelAct;
    Eigen::Vector2d mJEffAct;
    Eigen::Vector2d mJPosDes;
    Eigen::Vector2d mJVelDes;
    Eigen::Vector2d mJEffDes;
    Eigen::MatrixXd mA;
    double mMass;
    double mGrav;
    double mRx;
    double mRy;
};
