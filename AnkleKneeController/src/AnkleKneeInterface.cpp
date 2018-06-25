#include <AnkleKneeInterface/AnkleKneeInterface.hpp>

AnkleKneeInterface::AnkleKneeInterface() : mTime(0.0),
                                           mCount(0),
                                           mInitTime(0.01),
                                           mServoRate(1.0/1500),
                                           mNumJoint(2) {}

AnkleKneeInterface::~AnkleKneeInterface() {

}

void AnkleKneeInterface::getCommand(std::shared_ptr<AnkleKneeSensorData> data,
                                    std::shared_ptr<AnkleKneeCommand> cmd) {
    if (mTime < mInitTime) {
        _initialize(data, cmd);
    } else {
        _maintainInitialPosition(data, cmd);
        //_sinusoidalPosition(data, cmd);
    }
    mTime += mServoRate;
    mCount += 1;
}

void AnkleKneeInterface::_initialize(std::shared_ptr<AnkleKneeSensorData> data,
                                     std::shared_ptr<AnkleKneeCommand> cmd) {
    cmd->q = data->q;
    mInitQ = data->q;
    cmd->jtrq.setZero();
    cmd->qdot.setZero();
}

void AnkleKneeInterface::_maintainInitialPosition(std::shared_ptr<AnkleKneeSensorData> data,
                                                  std::shared_ptr<AnkleKneeCommand> cmd) {
    cmd->q = mInitQ;
    cmd->jtrq.setZero();
    cmd->qdot.setZero();
}

void AnkleKneeInterface::_sinusoidalPosition(std::shared_ptr<AnkleKneeSensorData> data,
                                             std::shared_ptr<AnkleKneeCommand> cmd) {
}
