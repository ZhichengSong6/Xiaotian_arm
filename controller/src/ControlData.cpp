#include "ControlData.h"
#include "ParamHandler.hpp"
#include <iostream>

bool ArmParameter::getParamsFromYAML(const char *filename)
{
  ParamHandler param_handler((std::string(filename)));
  bool _successful = true;

  _successful &= param_handler.getVector(std::string("jointlimit"), jointlimit);
  _successful &= param_handler.getVector(std::string("Kp"), Kp);
  _successful &= param_handler.getVector(std::string("Kd"), Kd);
  std::cout << "joint limit &  Kp, Kd load succeed" << std::endl;

  _successful &= param_handler.getVector(std::string("des_translation"), des_translation);
  _successful &= param_handler.getVector(std::string("des_rotation"), des_rotation);
  _successful &= param_handler.getValue(std::string("gripper_des_q"), gripper_des_q);
  std::cout << "desired position load succeed" << std::endl;

  if (!_successful)
  {
    throw std::runtime_error("init param failed ...");
  }

  return _successful;
}