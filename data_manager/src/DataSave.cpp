#include <data_manager/DataSave.hpp>

#include <iostream>
#include <fstream>

#include <utils/comm_udp.hpp>
#include <data_manager/DataManager.hpp>
#include <data_manager/DataManager_Configuration.h>

DataSave::DataSave():Sejong_Thread(), socket1_(0), socket2_(0){}
DataSave::~DataSave(){}

void DataSave::run(void ) {
  DATA_Protocol::DATA_SETUP data_setup;


  COMM::receive_data(socket1_, PORT_DATA_SETUP, &data_setup, sizeof(DATA_Protocol::DATA_SETUP), IP_ADDR);
  _ShowDataSetup(data_setup);

  double* data = new double[data_setup.tot_num_array_data];
  long long iter(0);
  while (true){
    COMM::receive_data(socket2_, PORT_DATA_RECEIVE, data, data_setup.tot_num_array_data*sizeof(double), IP_ADDR);

    int st_idx(0);
    int display_freq(300);
    for (int i(0); i < data_setup.num_data; ++i){
      if(iter % display_freq == 0){
        printf("%s : ", data_setup.data_name[i]);
        for(int j(0); j < data_setup.num_array_data[i]; ++j){
          std::cout<< data[st_idx + j]<<", ";
        }
        printf("\n");
      }
      saveVector(&data[st_idx],
                 data_setup.data_name[i],
                 data_setup.num_array_data[i]);
      st_idx += data_setup.num_array_data[i];
    }
    ++iter;
  }

  delete [] data;
}


void DataSave::_ShowDataSetup(const DATA_Protocol::DATA_SETUP & data_setup){
  printf("Number of Data: %i \n", data_setup.num_data);
  printf("Total number of values: %i \n", data_setup.tot_num_array_data);
  for(int i(0); i<data_setup.num_data; ++i){
    printf("%i th -  data name: %s, number of value: %i \n", i, data_setup.data_name[i], data_setup.num_array_data[i]);
  }
}


////////////////////////////////////////////////////
//           Save Data
////////////////////////////////////////////////////
void DataSave::saveVector(const Eigen::VectorXd & _vec, std::string _name, bool b_param){
  std::string file_name;
  cleaning_file(_name, file_name, b_param);

  std::ofstream savefile(file_name.c_str(), std::ios::app);
  for (int i(0); i < _vec.rows(); ++i){
    savefile<<_vec(i)<< "\t";
  }
  savefile<<"\n";
  savefile.flush();
}

void DataSave::saveVector(double * _vec, std::string _name, int size, bool b_param){
 std::string file_name;
  cleaning_file(_name, file_name, b_param);
  std::ofstream savefile(file_name.c_str(), std::ios::app);

  for (int i(0); i < size; ++i){
    savefile<<_vec[i]<< "\t";
  }
  savefile<<"\n";
  savefile.flush();
}

void DataSave::saveVector(const std::vector<double> & _vec, std::string _name, bool b_param){
 std::string file_name;
  cleaning_file(_name, file_name, b_param);
  std::ofstream savefile(file_name.c_str(), std::ios::app);
  for (int i(0); i < _vec.size(); ++i){
    savefile<<_vec[i]<< "\t";
  }
  savefile<<"\n";
  savefile.flush();
}
void DataSave::saveQuaternion(const Eigen::Quaterniond & qq, std::string _name, bool b_param){
 std::string file_name;
  cleaning_file(_name, file_name, b_param);
  std::ofstream savefile(file_name.c_str(), std::ios::app);
  savefile<<qq.w()<< "\t";
  savefile<<qq.x()<< "\t";
  savefile<<qq.y()<< "\t";
  savefile<<qq.z()<< "\t";
  savefile<<"\n";
  savefile.flush();
}


void DataSave::saveValue(double _value, std::string _name, bool b_param){
 std::string file_name;
  cleaning_file(_name, file_name, b_param);
  std::ofstream savefile(file_name.c_str(), std::ios::app);

  savefile<<_value <<"\n";
  savefile.flush();
}

void DataSave::saveVectorSequence(const std::vector<Eigen::VectorXd> & seq, std::string _name, bool b_param){
  std::string file_name;
  cleaning_file(_name, file_name, b_param);

  std::ofstream savefile(file_name.c_str(), std::ios::app);

  for (int i(0); i< seq.size(); ++i){
    for (int j(0); j< seq[i].rows(); ++j){
      savefile<<seq[i](j)<<"\t";
    }
    savefile<<"\n";
  }
  savefile.flush();
}

void DataSave::cleaning_file(std::string  _file_name, std::string & _ret_file, bool b_param){
  if(b_param)
    _ret_file += DATA_LOCATION"parameter_data/";
  else
    _ret_file += DATA_LOCATION"experiment_data/";

  _ret_file += _file_name;
  _ret_file += ".txt";

  std::list<std::string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
  if(gs_fileName_string.end() == iter){
    gs_fileName_string.push_back(_file_name);
    remove(_ret_file.c_str());
  }
}
////////////////////////////////////////////////////
//           End of Save Data
////////////////////////////////////////////////////
