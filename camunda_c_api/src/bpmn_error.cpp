/**
 * @File: bpmn_error.cpp
 * @Date: 26 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * A bass class to describe a handle BPMN error request body.
 **/

/* Local Headers */
#include"camunda_c_api/bpmn_error.hpp"
#include"camunda_c_api/bpmn_exception.hpp"
#include"camunda_c_api/json.hpp"
#include"camunda_c_api/variables.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>

namespace camunda
{
  BpmnError::BpmnError(const web::json::value& other) noexcept
   : Json::Json(other)
  {}

  BpmnError::BpmnError(const std::string& workerId,
                       const std::string& errorCode,
                       const std::string& errorMessage,
                       const Variables&   variables) noexcept
   : BpmnError(web::json::value(workerId),
               web::json::value(errorCode),
               web::json::value(errorMessage),
               variables)
  {}

  BpmnError::BpmnError(const std::string&   workerId,
                       const BpmnException& except) noexcept
   : BpmnError(web::json::value(workerId), except)
  {}

  BpmnError::BpmnError(const web::json::value& workerId,
                       const web::json::value& errorCode,
                       const web::json::value& errorMessage,
                       const Variables&        variables) noexcept
  {
    this->BpmnError::addWorkerId(workerId);
    this->BpmnError::addErrorCode(errorCode);
    this->BpmnError::addErrorMessage(errorMessage);
    this->BpmnError::addVariables(variables);
  }

  BpmnError::BpmnError(const web::json::value& workerId,
                       const BpmnException&    except) noexcept
   : BpmnError(workerId,
               except.getErrorCode(),
               except.getErrorMessage(),
               Variables(except.cgetVariables()))
  {}

  void BpmnError::addWorkerId(const std::string& workerId) noexcept
  {
    this->BpmnError::addWorkerId(web::json::value(workerId));
  }

  void BpmnError::addWorkerId(const web::json::value& workerId) noexcept
  {
    this->add("workerId", workerId);
  }

  void BpmnError::addErrorCode(const std::string& errorCode) noexcept
  {
    this->BpmnError::addErrorCode(web::json::value(errorCode));
  }

  void BpmnError::addErrorCode(const web::json::value& errorCode) noexcept
  {
    this->add("errorCode", errorCode);
  }

  void BpmnError::addErrorMessage(const std::string& errorMessage) noexcept
  {
    this->BpmnError::addErrorMessage(web::json::value(errorMessage));
  }

  void BpmnError::addErrorMessage(const web::json::value& errorMessage) noexcept
  {
    this->add("errorMessage", errorMessage);
  }

  void BpmnError::addVariables(const Variables& variables) noexcept
  {
    this->add("variables", variables.cget());
  }

  void BpmnError::removeWorkerId() noexcept
  {
    this->remove("workerId");
  }

  void BpmnError::removeErrorCode() noexcept
  {
    this->remove("errorCode");
  }

  void BpmnError::removeErrorMessage() noexcept
  {
    this->remove("errorMessage");
  }

  void BpmnError::removeVariables() noexcept
  {
    this->remove("variables");
  }

  void BpmnError::addBpmnException(const BpmnException& except) noexcept
  {
    this->BpmnError::addErrorCode(   except.getErrorCode());
    this->BpmnError::addErrorMessage(except.getErrorMessage());
    this->BpmnError::addVariables(   Variables(except.cgetVariables()));
  }

  const web::json::value& BpmnError::getWorkerId() const
  {
    try
    {
      return this->at("workerId");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("BpmnError::getWorkerId error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& BpmnError::getErrorCode() const
  {
    try
    {
      return this->at("errorCode");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("BpmnError::getErrorCode error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& BpmnError::getErrorMessage() const
  {
    try
    {
      return this->at("errorMessage");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("BpmnError::getErrorMessage error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& BpmnError::getVariables() const
  {
    try
    {
      return this->at("variables");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("BpmnError::getVariables error, " + static_cast<std::string>(e.what()));
    }
  }

  bool BpmnError::is_safe() const noexcept
  {
    if(this->Json::is_safe()                         &&
       this->cget().has_string_field("workerId")     &&
       this->cget().has_string_field("errorCode")    &&
       this->cget().has_string_field("errorMessage") &&
      !this->at("workerId")    .as_string().empty()  &&
      !this->at("errorCode")   .as_string().empty()  &&
      !this->at("errorMessage").as_string().empty())
    {
      return true;
    }
    return false;
  }

  const web::json::value& BpmnError::cgetError() const
  {
    if(this->is_safe())
    {
      return this->cget();
    }
    throw std::runtime_error("BpmnError::cgetError error, object missing needed data");
  }

  web::json::value& BpmnError::getError()
  {
    if(this->is_safe())
    {
      return this->get();
    }
    throw std::runtime_error("BpmnError::getError error, object missing needed data");
  }
}// camunda

/* bpmn_error.cpp */

