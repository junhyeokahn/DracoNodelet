#ifndef DATA_PYTHON_SENDER
#define DATA_PYTHON_SENDER

#include <iostream>
#include <Python.h>
#include <vector>
#include <Eigen/Dense>

class DataPythonSender{
 public:
  DataPythonSender();
  ~DataPythonSender();

  void SendData(double time,
                const Eigen::VectorXd & jpos_act,
                const Eigen::VectorXd & jvel_act,
                const Eigen::VectorXd & jeff_act,
                const Eigen::VectorXd & jpos_des,
                const Eigen::VectorXd & jvel_des,
                const Eigen::VectorXd & jeff_des,
                const Eigen::VectorXd & motor_current,
                const Eigen::VectorXd & temperature,
                const Eigen::VectorXd & ee_act,
                const Eigen::VectorXd & ee_des);
 protected:
  // Python
  PyObject *pName, *pModule, *pDict,
    *pFunc_send, *pFunc_update, *pArgs;
};

#endif
