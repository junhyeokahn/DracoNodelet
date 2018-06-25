#include <AnkleKneeInterface/AnkleKneeInterface.hpp>

AnkleKneeInterface::AnkleKneeInterface() : mTime(0.0),
                                           mCount(0),
                                           mInitTime(0.01),
                                           mServoRate(1.0/1500),
                                           mNumJoint(2) {}

AnkleKneeInterface::~AnkleKneeInterface() {

}

void AnkleKneeInterface::getCommand(std::shared_ptr<AnkleKneeSensorData> data,
                                    std::shared_ptr<AnkleKneeCommand> & cmd) {
    if (mTime < mInitTime) {
        _initialize(data, cmd);
    } else {
        _mainTainCurrentPosition(data, cmd);
    }
    mTime += mServoRate;
    mCount += 1;
    //if (mCount % 1000 == 0) {
        //for (int i = 0; i < 2; ++i) {
            //std::cout << i << " th Joint ============" << std::endl;
            //std::cout << data->q[i] << std::endl;
            //std::cout << data->qdot[i] << std::endl;
            //std::cout << data->jtrq[i] << std::endl;
        //}
    //}

}

void AnkleKneeInterface::_initialize(std::shared_ptr<AnkleKneeSensorData> data,
                                     std::shared_ptr<AnkleKneeCommand> & cmd) {
    for (int i = 0; i < mNumJoint; ++i) {
        cmd->jtrq[i] = 0.0;
        cmd->q[i] = data->q[i];
        cmd->qdot[i] = 0.0;
    }
}

void AnkleKneeInterface::_mainTainCurrentPosition(std::shared_ptr<AnkleKneeSensorData> data,
                                                  std::shared_ptr<AnkleKneeCommand> cmd) {
    for (int i = 0; i < mNumJoint; ++i) {
        cmd->jtrq[i] = 0.0;
        cmd->q[i] = data->q[i];
        cmd->qdot[i] = 0.0;
    }
}
