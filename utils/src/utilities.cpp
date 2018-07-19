#include <utils/utilities.hpp>
#include <iostream>
#include <stdio.h>

#include <fstream>
#include <algorithm>
#include <Eigen/Eigenvalues>
#include <string>

using namespace std;

namespace sejong{
  std::string pretty_string(sejong::Vector const & vv)
  {
    ostringstream os;
    pretty_print(vv, os, "", "", true);
    return os.str();
  }


  void saveVector(const Eigen::VectorXd & vec_, std::string name_, bool b_param){
        std::string file_name;
        cleaningFile(name_, file_name, b_param);

        std::ofstream savefile(file_name.c_str(), std::ios::app);
        for (int i(0); i < vec_.rows(); ++i){
            savefile<<vec_(i)<< "\t";
        }
        savefile<<"\n";
        savefile.flush();
    }

    void saveVector(double * _vec, std::string _name, int size, bool b_param){
        std::string file_name;
        cleaningFile(_name, file_name, b_param);
        std::ofstream savefile(file_name.c_str(), std::ios::app);

        for (int i(0); i < size; ++i){
            savefile<<_vec[i]<< "\t";
        }
        savefile<<"\n";
        savefile.flush();
    }

    void saveVector(const std::vector<double> & _vec, std::string _name, bool b_param){
        std::string file_name;
        cleaningFile(_name, file_name, b_param);
        std::ofstream savefile(file_name.c_str(), std::ios::app);
        for (int i(0); i < _vec.size(); ++i){
            savefile<<_vec[i]<< "\t";
        }
        savefile<<"\n";
        savefile.flush();
    }

    void cleaningFile(std::string  _file_name, std::string & _ret_file, bool b_param){
        if(b_param)
            _ret_file += "asdf";
        else
            _ret_file += "/home/apptronik/ros/DracoNodelet/AnkleKneeController/ExperimentData/";

        _ret_file += _file_name;
        _ret_file += ".txt";

        std::list<std::string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
        if(gs_fileName_string.end() == iter){
            gs_fileName_string.push_back(_file_name);
            remove(_ret_file.c_str());
        }
    }



  std::string pretty_string(sejong::Quaternion const & qq)
  {
    ostringstream os;
    pretty_print(qq, os, "", "", true);
    return os.str();
  }


  std::string pretty_string(sejong::Matrix const & mm, std::string const & prefix)
  {
    ostringstream os;
    pretty_print(mm, os, "", prefix);
    return os.str();
  }


  void pretty_print(sejong::Vector const & vv, std::ostream & os,
                    std::string const & title, std::string const & prefix,
                    bool nonl)
  {
    pretty_print((sejong::Matrix const &) vv, os, title, prefix, true, nonl);
  }

  void pretty_print(sejong::Vect3 const & vv, std::ostream & os,
                    std::string const & title, std::string const & prefix, bool nonl){
    pretty_print((sejong::Matrix const &) vv, os, title, prefix, true, nonl);
  }


  void pretty_print(sejong::Quaternion const & qq, std::ostream & os,
                    std::string const & title, std::string const & prefix,
                    bool nonl)
  {
    pretty_print(qq.coeffs(), os, title, prefix, true, nonl);
  }


  std::string pretty_string(double vv)
  {
    static int const buflen(32);
    static char buf[buflen];
    memset(buf, 0, sizeof(buf));
    /*#ifndef WIN32
      if (isinf(vv)) {
      snprintf(buf, buflen-1, " inf    ");
      }
      else if (isnan(vv)) {
      snprintf(buf, buflen-1, " nan    ");
      }
      else if (fabs(fmod(vv, 1)) < 1e-9) {
      snprintf(buf, buflen-1, "%- 7d  ", static_cast<int>(rint(vv)));
      }
      else {
      snprintf(buf, buflen-1, "% 6.6f  ", vv);
      }
      #else // WIN32*/
    snprintf(buf, buflen-1, "% 6.6f  ", vv);
    //#endif // WIN32
    string str(buf);
    return str;
  }

  void pretty_print(const std::vector<double> & _vec, const char* title){
    printf("%s: ", title);
    for( int i(0); i< _vec.size(); ++i){
      printf("% 6.4f, \t", _vec[i]);
    }
    printf("\n");
  }
  void pretty_print(const std::vector<int> & _vec, const char* title){
    printf("%s: ", title);
    for( int i(0); i< _vec.size(); ++i){
      printf("%d, \t", _vec[i]);
    }
    printf("\n");
  }

  void pretty_print(const double * _vec, const char* title, int size){
    printf("%s: ", title);
    for(int i(0); i< size; ++i){
      printf("% 6.4f, \t", _vec[i]);
    }
    printf("\n");
  }
  void pretty_print(sejong::Matrix const & mm, std::ostream & os,
                    std::string const & title, std::string const & prefix,
                    bool vecmode, bool nonl)
  {
    char const * nlornot("\n");
    if (nonl) {
      nlornot = "";
    }
    if ( ! title.empty()) {
      os << title << nlornot;
    }
    if ((mm.rows() <= 0) || (mm.cols() <= 0)) {
      os << prefix << " (empty)" << nlornot;
    }
    else {
      // if (mm.cols() == 1) {
      //   vecmode = true;
      // }

      if (vecmode) {
        if ( ! prefix.empty())
          os << prefix;
        for (int ir(0); ir < mm.rows(); ++ir) {
          os << pretty_string(mm.coeff(ir, 0));
        }
        os << nlornot;

      }
      else {

        for (int ir(0); ir < mm.rows(); ++ir) {
          if ( ! prefix.empty())
            os << prefix;
          for (int ic(0); ic < mm.cols(); ++ic) {
            os << pretty_string(mm.coeff(ir, ic));
          }
          os << nlornot;
        }

      }
    }
  }

  void printVectorSequence(const std::vector<sejong::Vector> & seq, string _name){
    std::cout<<_name<<":\n";
    for (int i(0); i< seq.size(); ++i){
      for (int j(0); j< seq[i].rows(); ++j){
        printf("% 6.4f, \t", seq[i][j]);
      }
      std::cout<<"\n";
    }
  }

  double generator_truncated_white_noise(double mean, double var, double min, double max){
    double ret;
    do{
      ret = generator_white_noise(mean, var);
    } while (ret <min || max < ret );

    return ret;
  }

  double generator_white_noise(double mean, double var){

    static bool hasSpare = false;
    static double rand1, rand2;

    if(hasSpare){
      hasSpare = false;
      return sqrt(var*rand1)*sin(rand2) + mean;
    }
    hasSpare = true;

    rand1 = rand() / ((double ) RAND_MAX);
    if(rand1 < 1e-100) rand1 = 1e-100;
    rand1 = -2*log(rand1);
    rand2 = rand() / ((double ) RAND_MAX) * M_PI * 2.;

    return mean + sqrt(var*rand1)*cos(rand2);
  }
  double generator_gamma_noise(double a, double b){
    /* assume a > 0 */
     if (a < 1)      {
       double u = rand()/((double)RAND_MAX);
       return generator_gamma_noise (1.0 + a, b) * pow (u, 1.0 / a);
     }

     {
       double x, v, u;
       double d = a - 1.0 / 3.0;
       double c = (1.0 / 3.0) / sqrt (d);

       while (true){
         do{
           x = generator_white_noise (0., 1.0);
           v = 1.0 + c * x;
         }
         while (v <= 0);

         v = v * v * v;
         u = rand()/((double)RAND_MAX);

         if (u < 1 - 0.0331 * x * x * x * x)  break;
         if (log (u) < 0.5 * x * x + d * (1 - v + log (v)))  break;
       }
       return b * d * v;
     }
  }

  void sqrtm(const sejong::Matrix & mt, sejong::Matrix & sqrt_mt){
    Eigen::EigenSolver<sejong::Matrix> es(mt, true);
    Eigen::MatrixXcd V = es.eigenvectors();
    Eigen::VectorXcd Dv = es.eigenvalues();
    Eigen::MatrixXcd sqrtD(mt.cols(), mt.rows());
    sqrtD = Dv.cwiseSqrt().asDiagonal();
    Eigen::MatrixXcd tmp_sqrt_mt = V*sqrtD*V.inverse();

    sqrt_mt = tmp_sqrt_mt.real();
  }


  double smooth_changing(double ini, double end, double moving_duration, double curr_time){
    double ret;
    ret = ini + (end - ini)*0.5*(1-cos(curr_time/moving_duration * M_PI));
    if(curr_time>moving_duration){
      ret = end;
    }
    return ret;
  }

  double smooth_changing_vel(double ini, double end, double moving_duration, double curr_time){
    double ret;
    ret = (end - ini)*0.5*(M_PI/moving_duration)*sin(curr_time/moving_duration*M_PI);
    if(curr_time>moving_duration){
      ret = 0.0;
    }
    return ret;
  }
  double smooth_changing_acc(double ini, double end, double moving_duration, double curr_time){
    double ret;
    ret = (end - ini)*0.5*(M_PI/moving_duration)*(M_PI/moving_duration)*cos(curr_time/moving_duration*M_PI);
    if(curr_time>moving_duration){
      ret = 0.0;
    }
    return ret;
  }

  double MinMaxBound(double value, double min, double max){
    if(value > max){
      return max;
    }
    else if (value < min){
      return min;
    }
    return value;
  }
  bool MinMaxCheck(double value, double min, double max, bool b_exit){
    if(value > max){
      if(b_exit) exit(0);
      return true;
    }
    else if (value < min){
      if(b_exit) exit(0);
      return true;
    }
    return false;
  }

  //jh
  void read_file(std::string _file_name, std::vector<std::string> & _vec){
    ifstream  InputFile(_file_name.c_str());
    std::string tempstring;
    if(!InputFile.is_open()){
      cout << "Data file load error... check the data file" << endl;
      exit(0);
    }
    else{
      while(!InputFile.eof()){
        InputFile.clear();
        getline(InputFile,tempstring);
        _vec.push_back(tempstring);
      }
      InputFile.close();
    }
  }

  void split_string(std::string* str_array, std::string strTarget, std::string strTok ){
    int nCutPos = 0;
    int nIndex = 0;
    while ((nCutPos = strTarget.find_first_of(strTok)) != strTarget.npos){
      if (nCutPos > 0){
        str_array[nIndex++] = strTarget.substr(0, nCutPos);
      }
      strTarget = strTarget.substr(nCutPos+1);
    }
    if(strTarget.length() > 0){
      str_array[nIndex++] = strTarget.substr(0, nCutPos);
    }
  }

  sejong::Matrix crossmat(sejong::Vect3 r){
      sejong::Matrix ret(3, 3);
      ret << 0          , -1 * r[2]   , r[1],
             r[2]       ,  0          , -1 * r[0],
             -1 * r[1]  , r[0]        , 0;

      return ret;
  }

  void hStack(const Eigen::MatrixXd a,
              const Eigen::MatrixXd b,
              Eigen::MatrixXd & ab){
      if ( a.rows() != b.rows() ) {
          std::cout << "[hStack] Matrix Size is Wrong" << std::endl;
          exit(0);
      }
      ab.resize(a.rows(), a.cols() + b.cols());
      ab << a, b;
  }

  void vStack(const Eigen::MatrixXd a,
              const Eigen::MatrixXd b,
              Eigen::MatrixXd & ab){
      if ( a.cols() != b.cols() ) {
          std::cout << "[vStack] Matrix Size is Wrong" << std::endl;
          exit(0);
      }
      ab.resize(a.rows() + b.rows(), a.cols());
      ab << a, b;
  }

  void vStack(const Eigen::VectorXd a,
              const Eigen::VectorXd b,
              Eigen::VectorXd & ab){
      ab.resize(a.size() + b.size());
      ab << a, b;
  }
  //jh
}

