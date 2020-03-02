/**
 * @File: bpmn_exception.cpp
 * @Date: 6 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * This exception class is made specifically so that when it is throw it will be
 * passed all the way up to Camunda and bpmn failure being thrown.
 **/

/* Local Headers */
#include"camunda_c_api/bpmn_exception.hpp"
#include"camunda_c_api/variables.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<exception>
#include<string>

namespace camunda
{
  BpmnException::BpmnException(const std::string& errorMessage,
                               const std::string& errorCode,
                               const Variables&   variables) noexcept
   : BpmnException(web::json::value(errorMessage),
                   web::json::value(errorCode),
                   variables)
  {}

  BpmnException::BpmnException(const web::json::value& errorMessage,
                               const web::json::value& errorCode,
                               const Variables&        variables) noexcept
   : m_errorMessage(errorMessage),
     m_errorCode(errorCode),
     m_variables(variables)
  {}

  const char* BpmnException::what() const noexcept
  {
    if(this->getErrorMessage().is_string())
    {
      return this->getErrorMessage().as_string().c_str();
    }
    return NULL;
  }

  const web::json::value& BpmnException::getErrorMessage() const noexcept
  {
    return this->m_errorMessage;
  }

  const web::json::value& BpmnException::getErrorCode() const noexcept
  {
    return this->m_errorCode;
  }

  const web::json::value& BpmnException::cgetVariables() const noexcept
  {
    return this->m_variables.cget();
  }

  web::json::value& BpmnException::getVariables() noexcept
  {
    return this->m_variables.get();
  }

  void BpmnException::addErrorMessage(const std::string& errorMessage) noexcept
  {
    this->addErrorMessage(web::json::value(errorMessage));
  }

  void BpmnException::addErrorMessage(const web::json::value& errorMessage) noexcept
  {
    this->m_errorMessage = errorMessage;
  }

  void BpmnException::addErrorCode(const std::string& errorCode) noexcept
  {
    this->addErrorCode(web::json::value(errorCode));
  }

  void BpmnException::addErrorCode(const web::json::value& errorCode) noexcept
  {
    this->m_errorCode = errorCode;
  }

  void BpmnException::addVariables(const Variables& variables) noexcept
  {
    this->m_variables = variables;
  }
}// camunda

/* bpmn_exception.cpp */

