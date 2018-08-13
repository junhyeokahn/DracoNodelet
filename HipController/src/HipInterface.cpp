#include <HipInterface/HipInterface.hpp>
#include <utils/BSplineBasic.h>
#include <utils/ParameterFetcher.hpp>
#include <utils/utilities.hpp>
#include <cmath>
#include <data_manager/DataManager.hpp>
#include <nodelet/nodelet.h>

HipInterface::HipInterface() : mTime(0.0),
                               mCount(0),
                               mInitTime(0.01),
                               mServoRate(1.0/1000),
                               mNumJoint(2),
                               mGrav(9.81) {

    mJPosAct.setZero();
    mJVelAct.setZero();
    mJEffAct.setZero();
    mJPosDes.setZero();
    mJVelDes.setZero();
    mJEffDes.setZero();
    DataManager::getDataManager()->RegisterData(&mTime, DOUBLE, "time", 1);
    DataManager::getDataManager()->RegisterData(&mJPosAct, VECT2, "JPosAct", 2);
    DataManager::getDataManager()->RegisterData(&mJVelAct, VECT2, "JVelAct", 2);
    DataManager::getDataManager()->RegisterData(&mJEffAct, VECT2, "JEffAct", 2);
    DataManager::getDataManager()->RegisterData(&mJPosDes, VECT2, "JPosDes", 2);
    DataManager::getDataManager()->RegisterData(&mJVelDes, VECT2, "JVelDes", 2);
    DataManager::getDataManager()->RegisterData(&mJEffDes, VECT2, "JEffDes", 2);

    //ros::NodeHandle nh("/high_level_system/model/com");
    //mA = Eigen::MatrixXd::Zero(2, 2);
    //mMass = 0.0;
    //mRx = 0.0;
    //mRy = 0.0;
    //ParameterFetcher::searchReqParam(nh, "mass", mMass);
    //ParameterFetcher::searchReqParam(nh, "rx", mRx);
    //ParameterFetcher::searchReqParam(nh, "ry", mRy);
    //mA(0, 0) = mMass * mRx * mRx;
}

HipInterface::~HipInterface() {

}

void HipInterface::getCommand(std::shared_ptr<HipSensorData> data,
                                    std::shared_ptr<HipCommand> cmd) {
    if (mTime < mInitTime) {
        _initialize(data, cmd);
    } else {
        _maintainInitialPosition(data, cmd);
        //_sinusoidalPosition(data, cmd);
    }
    mTime += mServoRate;
    mCount += 1;
    mJPosAct = data->q;
    mJVelAct = data->qdot;
    mJEffAct = data->jtrq;
    mJPosDes = cmd->q;
    mJVelDes = cmd->qdot;
    mJEffDes = cmd->jtrq;
}

void HipInterface::_initialize(std::shared_ptr<HipSensorData> data,
                                     std::shared_ptr<HipCommand> cmd) {
    cmd->q = data->q;
    mInitQ = data->q;
    cmd->jtrq.setZero();
    cmd->qdot.setZero();
    DataManager::getDataManager()->start();
}

void HipInterface::_maintainInitialPosition(std::shared_ptr<HipSensorData> data,
                                                  std::shared_ptr<HipCommand> cmd) {
    cmd->q = mInitQ;
    cmd->qdot.setZero();
    cmd->jtrq.setZero();
}

void HipInterface::_sinusoidalPosition(std::shared_ptr<HipSensorData> data,
                                             std::shared_ptr<HipCommand> cmd) {
    static bool isFirst(true);
    static BS_Basic<6, 3, 0, 2, 2> spline;
    static double transDur(0.0);
    static double initTime(mTime);
    static std::vector<double> mid(6);
    static std::vector<double> amp(6);
    static std::vector<double> freq(6);
    static double d_ary[6];
    if (isFirst) {
        // Do Plan
        ros::NodeHandle nh("/high_level_system/planner/sinusoidal");
        ParameterFetcher::searchReqParam(nh, "transDur", transDur);
        ParameterFetcher::searchReqParam(nh, "mid", mid);
        ParameterFetcher::searchReqParam(nh, "amp", amp);
        ParameterFetcher::searchReqParam(nh, "freq", freq);
        double ini[18] = {data->q[0], data->q[1], data->q[2], data->q[3], data->q[4], data->q[5],
                          0., 0., 0., 0., 0., 0.,
                          0., 0., 0., 0., 0., 0.};
        double fin[18] = {mid[0], mid[1], mid[2], mid[3], mid[4], mid[5],
                          amp[0]*2*M_PI*freq[0], amp[1]*2*M_PI*freq[1], amp[2]*2*M_PI*freq[2], amp[3]*2*M_PI*freq[3], amp[4]*2*M_PI*freq[4], amp[5]*2*M_PI*freq[5],
                          0., 0., 0., 0., 0., 0.};
        double **middle_pt;
        spline.SetParam(ini, fin, middle_pt, transDur);
        isFirst = false;
    } else {
        // Get Trajectory
        if (mTime < initTime + transDur) {
            spline.getCurvePoint(mTime - initTime, d_ary);
            for (int i = 0; i < 6; ++i) cmd->q[i] = d_ary[i];
            spline.getCurveDerPoint(mTime - initTime, 1, d_ary);
            for (int i = 0; i < 6; ++i) cmd->qdot[i] = d_ary[i];
            spline.getCurveDerPoint(mTime - initTime, 2, d_ary);
            for (int i = 0; i < 6; ++i) cmd->jtrq[i] = d_ary[i];
            //cmd->jtrq = mA * cmd->jtrq; // feedforword
            //cmd->jtrq[0] = cmd->jtrq[0] + (mMass * mGrav * mRx * cos(M_PI - data->q[0])); // gravity
            cmd->jtrq.setZero();
        } else {
            for (int i = 0; i < 2; ++i) {
                cmd->q[i] =
                    amp[i] * sin(2*M_PI*freq[i]* (mTime - transDur)) + mid[i];
                cmd->qdot[i] =
                    amp[i]*2*M_PI*freq[i]*cos(2*M_PI*freq[i]*(mTime - transDur));
                cmd->jtrq[i] =
                    -amp[i]*2*M_PI*freq[i]*2*M_PI*freq[i]*sin(2*M_PI*freq[i]*(mTime - transDur));
            }
            //cmd->jtrq = mA*cmd->jtrq; // feedforword
            //cmd->jtrq[0] = cmd->jtrq[0] + (mMass * mGrav * mRx * cos(M_PI - data->q[0])); // gravity
            cmd->jtrq.setZero();
        }
    }
}
