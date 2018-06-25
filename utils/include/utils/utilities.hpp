#ifndef UTILITIES
#define UTILITIES

#include <utils/wrap_eigen.hpp>

#define SAFE_DELETE(p)			if(p) { delete (p); (p) = NULL; }

namespace sejong{
  static std::list< std::string > gs_fileName_string; //global & static

  // Box Muller Transform
  double generator_white_noise(double mean, double var);
  double generator_truncated_white_noise(double mean, double var, double min, double max);
  // min, var > 0
  // mean = a * b
  // variance = a * b * b
  double generator_gamma_noise(double a, double b);

  std::string pretty_string(double vv);
  std::string pretty_string(sejong::Vector const & vv);
  std::string pretty_string(sejong::Quaternion const & qq);
  std::string pretty_string(sejong::Matrix const & mm, std::string const & prefix);

  void pretty_print(sejong::Vector const & vv, std::ostream & os,
                    std::string const & title,
                    std::string const & prefix="", bool nonl = false);
  void pretty_print(sejong::Vect3 const & vv, std::ostream & os,
                    std::string const & title, std::string const & prefix="", bool nonl = false);
  void pretty_print(const std::vector<double> & _vec, const char* title);
  void pretty_print(const std::vector<int> & _vec, const char* title);
  void pretty_print(const double * _vec, const char* title, int size);
  void pretty_print(sejong::Quaternion const & qq, std::ostream & os,
                    std::string const & title,
                    std::string const & prefix="", bool nonl = false);
  void pretty_print(sejong::Matrix const & mm, std::ostream & os,
                    std::string const & title,
                    std::string const & prefix ="",
                    bool vecmode = false, bool nonl = false);
  void printVectorSequence(const std::vector<sejong::Vector> & seq, std::string _name);


  // Smooth Changing
  double smooth_changing(double ini, double end,
                         double moving_duration, double curr_time);
  double smooth_changing_vel(double ini, double end,
                             double moving_duration, double curr_time);
  double smooth_changing_acc(double ini, double end,
                             double moving_duration, double curr_time);

  double MinMaxBound(double value, double min, double max);
  bool MinMaxCheck(double value, double min, double max, bool b_exit = false);
  void sqrtm(const sejong::Matrix & mt, sejong::Matrix & sqrt_mt);

  //jh
  void read_file(std::string file_name_, std::vector<std::string> & _vec);
  void split_string(std::string* str_array, std::string strTarget, std::string strTok );
  sejong::Matrix crossmat(sejong::Vect3);
  void hStack(const Eigen::MatrixXd a,
              const Eigen::MatrixXd b,
              Eigen::MatrixXd & ab);
  void vStack(const Eigen::MatrixXd a,
              const Eigen::MatrixXd b,
              Eigen::MatrixXd & ab);
  void vStack(const Eigen::VectorXd a,
              const Eigen::VectorXd b,
              Eigen::VectorXd & ab);

  //jh
}

#endif
