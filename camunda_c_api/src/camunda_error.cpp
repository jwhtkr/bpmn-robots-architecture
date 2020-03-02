/**
 * @File: camunda_error.cpp
 * @Date: 6 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Base class for describing a handle external task failure request.
 **/

/* Local Headers */
#include"camunda_c_api/camunda_error.hpp"
#include"camunda_c_api/camunda_exception.hpp"
#include"camunda_c_api/json.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>
#include<stdexcept>

namespace camunda
{
  CamundaError::CamundaError(const web::json::value& other) noexcept
   : Json::Json(other)
   {}

  CamundaError::CamundaError(const std::string& workerId,
                             const std::string& errorMessage,
                             const std::string& errorDetails,
                             const uint32_t     retries,
                             const uint32_t     retryTimeout) noexcept
   : CamundaError(web::json::value(workerId),
                  web::json::value(errorMessage),
                  web::json::value(errorDetails),
                  web::json::value(retries),
                  web::json::value(retryTimeout))
  {}

  CamundaError::CamundaError(const web::json::value& workerId,
                             const web::json::value& errorMessage,
                             const web::json::value& errorDetails,
                             const web::json::value& retries,
                             const web::json::value& retryTimeout) noexcept
  {
    this->CamundaError::addWorkerId(workerId);
    this->CamundaError::addErrorMessage(errorMessage);
    this->CamundaError::addErrorDetails(errorDetails);
    this->CamundaError::addRetries(retries);
    this->CamundaError::addRetryTimeout(retryTimeout);
  }

  CamundaError::CamundaError(const std::string&      workerId,
                             const CamundaException& exception,
                             const uint32_t          retries,
                             const uint32_t          retryTimeout) noexcept
   : CamundaError(web::json::value(workerId),
                  exception,
                  web::json::value(retries),
                  web::json::value(retryTimeout))
  {}

  CamundaError::CamundaError(const web::json::value& workerId,
                             const CamundaException& exception,
                             const web::json::value& retries,
                             const web::json::value& retryTimeout) noexcept
   : CamundaError(workerId,
                  exception.getErrorMessage(),
                  exception.getErrorDetails(),
                  retries,
                  retryTimeout)
  {}

  void CamundaError::addWorkerId(const std::string& workerId) noexcept
  {
    this->CamundaError::addWorkerId(web::json::value(workerId));
  }

  void CamundaError::addWorkerId(const web::json::value& workerId) noexcept
  {
    this->add("workerId", workerId);
  }

  void CamundaError::addErrorMessage(const std::string& errorMessage) noexcept
  {
    this->CamundaError::addErrorMessage(web::json::value(errorMessage));
  }

  void CamundaError::addErrorMessage(const web::json::value& errorMessage) noexcept
  {
    this->add("errorMessage", errorMessage);
  }

  void CamundaError::addErrorDetails(const std::string& errorDetails) noexcept
  {
    this->CamundaError::addErrorDetails(web::json::value(errorDetails));
  }

  void CamundaError::addErrorDetails(const web::json::value& errorDetails) noexcept
  {
    this->add("errorDetails", errorDetails);
  }

  void CamundaError::addRetries(const uint32_t retries) noexcept
  {
    this->CamundaError::addRetries(web::json::value(retries));
  }

  void CamundaError::addRetries(const web::json::value& retries) noexcept
  {
    this->add("retries", retries);
  }

  void CamundaError::addRetryTimeout(const uint32_t retryTimeout) noexcept
  {
    this->CamundaError::addRetryTimeout(web::json::value(retryTimeout));
  }

  void CamundaError::addRetryTimeout(const web::json::value& retryTimeout) noexcept
  {
    this->add("retryTimeout", retryTimeout);
  }

  void CamundaError::addException(const CamundaException& exception) noexcept
  {
    this->CamundaError::addErrorMessage(exception.getErrorMessage());
    this->CamundaError::addErrorDetails(exception.getErrorDetails());
  }

  void CamundaError::removeWorkerId() noexcept
  {
    this->remove("workerId");
  }

  void CamundaError::removeErrorMessage() noexcept
  {
    this->remove("errorMessage");
  }

  void CamundaError::removeErrorDetails() noexcept
  {
    this->remove("errorDetails");
  }

  void CamundaError::removeRetries() noexcept
  {
    this->remove("retries");
  }

  void CamundaError::removeRetryTimeout() noexcept
  {
    this->remove("retryTimeout");
  }

  bool CamundaError::is_safe() const noexcept
  {
    if(this->Json::is_safe()                         &&
       this->cget().has_string_field("workerId")     &&
       this->cget().has_string_field("errorMessage") &&
       this->cget().has_string_field("errorDetails") &&
       this->cget().has_integer_field("retries")     &&
       this->cget().has_integer_field("retryTimeout"))
    {
      return true;
    }
    return false;
  }

  const web::json::value& CamundaError::getWorkerId() const
  {
    try
    {
      return this->at("workerId");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CamundaError::getWorkerId error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CamundaError::getErrorMessage() const
  {
    try
    {
      return this->at("errorMessage");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CamundaError::getErrorMessage error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CamundaError::getErrorDetails() const
  {
    try
    {
      return this->at("errorDetails");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CamundaError::getErrorDetails error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CamundaError::getRetries() const
  {
    try
    {
      return this->at("retries");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CamundaError::getRetries error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CamundaError::getRetryTimeout() const
  {
    try
    {
      return this->at("retryTimeout");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CamundaError::getRetryTimeout error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CamundaError::cgetError() const
  {
    try
    {
      if(!this->is_safe())
      {
        throw std::runtime_error("object not ready for use");
      }
      return this->cget();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CamundaError::cgetError error, " + static_cast<std::string>(e.what()));
    }
  }

  web::json::value& CamundaError::getError()
  {
    try
    {
      if(!this->is_safe())
      {
        throw std::runtime_error("object not ready for use");
      }
      return this->get();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CamundaError::getError error, " + static_cast<std::string>(e.what()));
    }
  }
}// camunda

/* camunda_error.cpp */

