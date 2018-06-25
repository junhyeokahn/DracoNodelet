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
};

class AnkleKneeInterface
{
public:
    AnkleKneeInterface ();
    virtual ~AnkleKneeInterface ();

    void getCommand(std::shared_ptr<AnkleKneeSensorData> data,
                    std::shared_ptr<AnkleKneeCommand> & cmd);
private:
    double mTime;
    int mCount;
    double mInitTime;
    double mServoRate;
    int mNumJoint;

    void _initialize(std::shared_ptr<AnkleKneeSensorData> data,
                     std::shared_ptr<AnkleKneeCommand> & cmd);
    void _mainTainCurrentPosition(std::shared_ptr<AnkleKneeSensorData> data,
                                  std::shared_ptr<AnkleKneeCommand> cmd);
};
