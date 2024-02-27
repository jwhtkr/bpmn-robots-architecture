/**
 * @File: extend_lock.cpp
 * @Date: 26 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * A class to describe a extend lock request.
 **/

/* Local Headers */
#include"camunda_c_api/extend_lock.hpp"
#include"camunda_c_api/json.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>
#include<stdexcept>

namespace camunda
{
  ExtendLock::ExtendLock(const web::json::value& other) noexcept
   : Json::Json(other)
  {}

  ExtendLock::ExtendLock(const std::string& workerId, const uint32_t newDuration) noexcept
   : ExtendLock(web::json::value(workerId), web::json::value(newDuration))
  {}

  ExtendLock::ExtendLock(const web::json::value& workerId, const web::json::value& newDuration) noexcept
  {
    this->addWorkerId(workerId);
    this->addNewDuration(newDuration);
  }

  void ExtendLock::addWorkerId(const std::string& workerId) noexcept
  {
    this->addWorkerId(web::json::value(workerId));
  }

  void ExtendLock::addWorkerId(const web::json::value& workerId) noexcept
  {
    this->add("workerId", workerId);
  }

  void ExtendLock::addNewDuration(const uint32_t newDuration) noexcept
  {
    this->addNewDuration(web::json::value(newDuration));
  }

  void ExtendLock::addNewDuration(const web::json::value& newDuration) noexcept
  {
    this->add("newDuration", newDuration);
  }

  const web::json::value& ExtendLock::getWorkerId() const
  {
    try
    {
      return this->at("workerId");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ExtendLock::getWorkerId error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& ExtendLock::getNewDuration() const
  {
    try
    {
      return this->at("newDuration");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ExtendLock::getNewDuration error, " + static_cast<std::string>(e.what()));
    }
  }

  bool ExtendLock::is_safe() const noexcept
  {
    if(this->Json::is_safe()               &&
       this->has_field("workerId")         &&
       this->has_field("newDuration")      &&
       this->at("workerId")   .is_string() &&
       this->at("newDuration").is_number())
    {
      return true;
    }
    return false;
  }

  const web::json::value& ExtendLock::cgetExtendLockRequest() const
  {
    if(this->is_safe())
    {
      return this->cget();
    }
    throw std::runtime_error("ExtendLock::cgetExtendLockRequest error, object missing fields");
  }

  web::json::value& ExtendLock::getExtendLockRequest()
  {
    if(this->is_safe())
    {
      return this->get();
    }
    throw std::runtime_error("ExtendLock::getExtendLockRequest error, object missing fields");
  }
}// camunda

/* extend_lock.cpp */

