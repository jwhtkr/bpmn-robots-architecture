/**
 * @File: camunda_exception.cpp
 * @Date: 31 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * This exception class is made specifically so that when it is throw it will be
 * passed all the way up to Camunda and handle failure being thrown.
 **/

/* Local Headers */
#include"camunda_c_api/camunda_exception.hpp"

/* C++ Headers */
#include<string>

namespace camunda
{
  CamundaException::CamundaException(const std::string& errorMessage,
                                     const std::string& errorDetails) noexcept
   : CamundaException(web::json::value(errorMessage), web::json::value(errorDetails))
  {}

  CamundaException::CamundaException(const web::json::value& errorMessage,
                                     const web::json::value& errorDetails) noexcept
   : m_errorMessage(errorMessage),
     m_errorDetails(errorDetails)
  {}

  const char* CamundaException::what() const noexcept
  {
    if(this->getErrorMessage().is_string())
    {
      return this->getErrorMessage().as_string().c_str();
    }
    return NULL;
  }

  const web::json::value& CamundaException::getErrorMessage() const noexcept
  {
    return this->m_errorMessage;
  }

  const web::json::value& CamundaException::getErrorDetails() const noexcept
  {
    return this->m_errorDetails;
  }

  void CamundaException::addErrorMessage(const std::string& errorMessage) noexcept
  {
    this->addErrorMessage(web::json::value(errorMessage));
  }

  void CamundaException::addErrorMessage(const web::json::value& errorMessage) noexcept
  {
    this->m_errorMessage = errorMessage;
  }

  void CamundaException::addErrorDetails(const std::string& errorDetails) noexcept
  {
    this->addErrorDetails(web::json::value(errorDetails));
  }

  void CamundaException::addErrorDetails(const web::json::value& errorDetails) noexcept
  {
    this->m_errorDetails = errorDetails;
  }

}; // camunda

/* camunda_exception.cpp */

