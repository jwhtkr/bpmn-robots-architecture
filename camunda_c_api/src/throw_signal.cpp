/**
 * @File: throw_signal.cpp
 * @Date: 27 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * A base class to describe a Throw Signal request.
 **/

/* Local Headers */
#include"camunda_c_api/throw_signal.hpp"
#include"camunda_c_api/json.hpp"
#include"camunda_c_api/variables.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>
#include<stdexcept>

namespace camunda
{
  ThrowSignal::ThrowSignal(const web::json::value& other) noexcept
   : Json::Json(other)
  {}

  ThrowSignal::ThrowSignal(const std::string& name,
                           const Variables&   variables) noexcept
   : ThrowSignal(web::json::value(name), variables)
  {}

  ThrowSignal::ThrowSignal(const web::json::value& name,
                           const Variables&        variables) noexcept
  {
    this->ThrowSignal::addName(name);
    this->ThrowSignal::addVariables(variables);
  }

  ThrowSignal::ThrowSignal(const std::string& name,
                           const std::string& executionId,
                           const Variables&   variables) noexcept
   : ThrowSignal(web::json::value(name), web::json::value(executionId), variables)
   {}

  ThrowSignal::ThrowSignal(const web::json::value& name,
                           const web::json::value& executionId,
                           const Variables&        variables) noexcept
  {
    this->ThrowSignal::addName(name);
    this->ThrowSignal::addExecutionId(executionId);
    this->ThrowSignal::addVariables(variables);
  }

  void ThrowSignal::addName(const std::string& name) noexcept
  {
    this->ThrowSignal::addName(web::json::value(name));
  }

  void ThrowSignal::addName(const web::json::value& name) noexcept
  {
    this->add("name", name);
  }

  void ThrowSignal::addExecutionId(const std::string& executionId) noexcept
  {
    this->ThrowSignal::addExecutionId(web::json::value(executionId));
  }

  void ThrowSignal::addExecutionId(const web::json::value& executionId) noexcept
  {
    this->add("executionId", executionId);
  }

  void ThrowSignal::addVariables(const Variables& variables) noexcept
  {
    this->ThrowSignal::addVariables(variables.cget());
  }

  void ThrowSignal::addVariables(const web::json::value& variables) noexcept
  {
    this->add("variables", variables);
  }

  void ThrowSignal::removeName() noexcept
  {
    this->remove("name");
  }

  void ThrowSignal::removeExecutionId() noexcept
  {
    this->remove("executionId");
  }

  void ThrowSignal::removeVariables() noexcept
  {
    this->remove("variables");
  }

  const web::json::value& ThrowSignal::getName() const
  {
    try
    {
      return this->at("name");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ThrowSignal::getName error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& ThrowSignal::getExecutionId() const
  {
    try
    {
      return this->at("executionId");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ThrowSignal::getExecutionId error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& ThrowSignal::getVariables() const
  {
    try
    {
      return this->at("variables");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ThrowSignal::getVariables error, " + static_cast<std::string>(e.what()));
    }
  }

  bool ThrowSignal::is_safe() const noexcept
  {
    if(this->Json::is_safe()      &&
       this->cget().has_string_field("name"))
    {
      return true;
    }
    return false;
  }

  const web::json::value& ThrowSignal::cgetSignal() const
  {
    if(this->is_safe())
    {
      return this->cget();
    }
    throw std::runtime_error("ThrowSignal::cgetSignal error, object is missing values");
  }

  web::json::value& ThrowSignal::getSignal()
  {
    if(this->is_safe())
    {
      return this->get();
    }
    throw std::runtime_error("ThrowSignal::getSignal error, object is missing values");
  }
}// camunda

/* throw_signal.cpp */

