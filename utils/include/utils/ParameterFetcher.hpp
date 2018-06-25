#ifndef PARMETER_FETCHER_HPP
#define PARMETER_FETCHER_HPP

#include <ros/ros.h>

class ParameterFetcher {
private:

public:
  ParameterFetcher ();
  virtual ~ParameterFetcher ();

  template<typename T>
  static void searchReqParam(const ros::NodeHandle & nh, const std::string & param_name, T& var_out)
  {
    std::string temp_key;
    if (nh.searchParam(param_name, temp_key))
    {
      nh.getParam(temp_key, var_out);
    }
    else
    {
      ROS_ERROR("Could not find required parameter '%s' in any parent namespace", param_name.c_str());
      ros::shutdown();
      exit(1);
    }
  }


  template<typename T>
  static void getReqParam(const ros::NodeHandle & nh, const std::string & param_name, T& var_out)
  {
    if(!nh.hasParam(param_name))
    {
      ROS_ERROR("Did not supply required '%s' parameter.  Exiting...", param_name.c_str());
      ros::shutdown();
      exit(1);
    }
    nh.getParam(param_name, var_out);
  }




};






#endif  // PARMETER_FETCHER_HPP
