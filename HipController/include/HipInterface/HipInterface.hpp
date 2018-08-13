#pragma once

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <vector>

class HipCommand
{
public:
    Eigen::Matrix<double,6,1> q;
    Eigen::Matrix<double,6,1> qdot;
    Eigen::Matrix<double,6,1> jtrq;
};

class HipSensorData
{
public:
    Eigen::Matrix<double,6,1> q;
    Eigen::Matrix<double,6,1> qdot;
    Eigen::Matrix<double,6,1> jtrq;
};

class HipInterface
{
public:
    HipInterface ();
    virtual ~HipInterface ();

    void getCommand(std::shared_ptr<HipSensorData> data,
                    std::shared_ptr<HipCommand> cmd);
private:
    double mTime;
    int mCount;
    double mInitTime;
    double mServoRate;
    int mNumJoint;
    Eigen::Matrix<double,6,1> mInitQ;

    void _initialize(std::shared_ptr<HipSensorData> data,
                     std::shared_ptr<HipCommand> cmd);
    void _maintainInitialPosition(std::shared_ptr<HipSensorData> data,
                                  std::shared_ptr<HipCommand> cmd);
    void _sinusoidalPosition(std::shared_ptr<HipSensorData> data,
                             std::shared_ptr<HipCommand> cmd);

    Eigen::Matrix<double,6,1> mJPosAct;
    Eigen::Matrix<double,6,1> mJVelAct;
    Eigen::Matrix<double,6,1> mJEffAct;
    Eigen::Matrix<double,6,1> mJPosDes;
    Eigen::Matrix<double,6,1> mJVelDes;
    Eigen::Matrix<double,6,1> mJEffDes;
    Eigen::MatrixXd mA;
    double mGrav;
};
