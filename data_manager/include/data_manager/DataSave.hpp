#ifndef DATA_SAVE
#define DATA_SAVE

#include <utils/Sejong_Thread.hpp>
#include <data_manager/data_protocol.h>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <list>

static std::list< std::string > gs_fileName_string; //global & static

class DataSave: public Sejong_Thread{
public:
  DataSave();
  virtual ~DataSave();
  virtual void run (void );

  // Save Data
  void saveVectorSequence(const std::vector<Eigen::VectorXd> & seq,
                          std::string _name, bool b_param = false);
  void saveVector(const Eigen::VectorXd & _vec, std::string _name, bool b_param = false);
  void saveVector(const std::vector<double> & _vec, std::string _name, bool b_param = false);
  void saveVector(double * _vec, std::string _name, int size, bool b_param = false);
  void saveQuaternion(const Eigen::Quaterniond & qq, std::string _name, bool b_param = false);
  void saveValue(double _value, std::string _name, bool b_param = false);
  void cleaning_file(std::string _file_name, std::string & ret_file, bool b_param);

private:
  void _ShowDataSetup(const DATA_Protocol::DATA_SETUP & data_setup);
  int socket1_;
  int socket2_;
};

#endif
