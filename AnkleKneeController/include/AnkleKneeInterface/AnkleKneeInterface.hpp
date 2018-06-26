#pragma once

#include <iostream>
#include <memory>
#include <Eigen/Dense>

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
    double kneeMJPos;
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

    void _initialize(std::shared_ptr<AnkleKneeSensorData> data,
                     std::shared_ptr<AnkleKneeCommand> cmd);
    void _maintainInitialPosition(std::shared_ptr<AnkleKneeSensorData> data,
                                  std::shared_ptr<AnkleKneeCommand> cmd);
    void _sinusoidalPosition(std::shared_ptr<AnkleKneeSensorData> data,
                             std::shared_ptr<AnkleKneeCommand> cmd);

    Eigen::Vector2d mJPosAct;
    Eigen::Vector2d mJVelAct;
    Eigen::Vector2d mJEffAct;
    Eigen::Vector2d mJPosDes;
    Eigen::Vector2d mJVelDes;
    Eigen::Vector2d mJEffDes;
    double mKneeMJPos;
    Eigen::MatrixXd mA;
    double mMass;
    double mGrav;
    double mRx;
    double mRy;
};
