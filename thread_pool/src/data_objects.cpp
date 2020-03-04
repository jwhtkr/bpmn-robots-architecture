/**
 * @File: data_objects.cpp
 * @Date: March 2020
 * @Author: James Swedeen
 **/

/* Local Headers */
#include"thread_pool/data_objects.hpp"

/* C++ Headers */
#include<string>

namespace thread_pool
{
  ResourceMsgs::ResourceMsgs(const std::string& name) noexcept
   : name(name)
  {}

  bool ResourceMsgs::operator==(const ResourceMsgs& rhs) const noexcept
  {
    return this->name == rhs.name;
  }

  bool ResourceMsgs::operator!=(const ResourceMsgs& rhs) const noexcept
  {
    return !(*this == rhs);
  }

  bool ResourceMsgs::operator>=(const ResourceMsgs& rhs) const noexcept
  {
    return this->name >= rhs.name;
  }

  bool ResourceMsgs::operator<=(const ResourceMsgs& rhs) const noexcept
  {
    return this->name <= rhs.name;
  }

  bool ResourceMsgs::operator>(const ResourceMsgs& rhs) const noexcept
  {
    return !(*this <= rhs);
  }

  bool ResourceMsgs::operator<(const ResourceMsgs& rhs) const noexcept
  {
    return !(*this >= rhs);
  }



  RobotServices::RobotServices(const std::string& name) noexcept
   : name(name)
  {}

  bool RobotServices::operator==(const RobotServices& rhs) const noexcept
  {
    return this->name == rhs.name;
  }

  bool RobotServices::operator!=(const RobotServices& rhs) const noexcept
  {
    return !(*this == rhs);
  }

  bool RobotServices::operator>=(const RobotServices& rhs) const noexcept
  {
    return this->name >= rhs.name;
  }

  bool RobotServices::operator<=(const RobotServices& rhs) const noexcept
  {
    return this->name <= rhs.name;
  }

  bool RobotServices::operator>(const RobotServices& rhs) const noexcept
  {
    return !(*this <= rhs);
  }

  bool RobotServices::operator<(const RobotServices& rhs) const noexcept
  {
    return !(*this >= rhs);
  }
} // thread_pool

/* data_objects.cpp */
