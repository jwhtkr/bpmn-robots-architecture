/**
 * @File: lock_response.cpp
 * @Date: 20 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * A raper implementation around a json based on the response body
 * of a fetch and lock rest call.
 **/

/* Local Headers */
#include"camunda_c_api/json.hpp"
#include"camunda_c_api/lock_response.hpp"

/* Rest Headers */
#include<cpprest/json.h>
#include<cpprest/http_client.h>

/* C++ Headers */
#include<string>
#include<stdexcept>

namespace camunda
{
  LockResponse::LockResponse(const web::http::http_response& http_in)
   : LockResponse(http_in.extract_json())
  {}

  LockResponse::LockResponse(const pplx::task<web::json::value>& task_in)
   : LockResponse(task_in.get())
  {}

  LockResponse::LockResponse(const web::json::value& json_in) noexcept
   : Json::Json(json_in)
  {}

  const web::json::value& LockResponse::getId(const size_t array_index) const
  {
    try
    {
      return this->at(array_index).at("id");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockResponse::getId error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& LockResponse::getWorkerId(const size_t array_index) const
  {
    try
    {
      return this->at(array_index).at("workerId");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockResponse::getWorkerId error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& LockResponse::getRetries(const size_t array_index) const
  {
    try
    {
      if(!this->at(array_index).at("retries").is_number())
      {
        const_cast<web::json::value&>(this->at(array_index).at("retries")) = web::json::value(0);
      }

      return this->at(array_index).at("retries");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockResponse::getRetries error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& LockResponse::getVariables(const size_t array_index) const
  {
    try
    {
      return this->at(array_index).at("variables");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockResponse::getVariables error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& LockResponse::getPriority(const size_t array_index) const
  {
    try
    {
      return this->at(array_index).at("priority");
    }
    catch(const std::exception& ex)
    {
      throw std::runtime_error("LockResponse::getPriority error, " + static_cast<std::string>(ex.what()));
    }
  }

  const web::json::value& LockResponse::getActivityId(const size_t array_index) const
  {
    try
    {
      return this->at(array_index).at("activityId");
    }
    catch(const std::exception& ex)
    {
      throw std::runtime_error("LockResponse::getActivityId error, " + static_cast<std::string>(ex.what()));
    }
  }

}; // camunda

/* lock_response.cpp */

