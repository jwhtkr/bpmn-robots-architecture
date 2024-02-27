/**
 * @File: bpmn_exception.hpp
 * @Date: 6 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * This exception class is made specifically so that when it is throw it will be
 * passed all the way up to Camunda and bpmn failure being thrown.
 **/

#ifndef CAMUNDA_ERROR_HANDLING_BPMN_EXCEPTION_HPP
#define CAMUNDA_ERROR_HANDLING_BPMN_EXCEPTION_HPP

/* Local Headers */
#include"camunda_c_api/variables.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<exception>
#include<string>

namespace camunda
{
  class BpmnException : public std::exception
  {
  public:
    /**
     * @Default Constructor
     **/
    BpmnException() = default;
    /**
     * @Copy Constructor
     **/
    BpmnException(const BpmnException&)     = default;
    BpmnException(BpmnException&&) noexcept = default;
    /**
     * @Constructor
     *
     * @brief
     * After construction object can be filled with needed information for use.
     * @errorMessage: Message describing error
     * @errorCode: Code that relates to error
     * @variables: Object holding variables to be passed to Camunda
     **/
    explicit BpmnException(const std::string& errorMessage,
                           const std::string& errorCode = std::string(),
                           const Variables&   variables = Variables()) noexcept;

    explicit BpmnException(const web::json::value& errorMessage,
                           const web::json::value& errorCode = web::json::value(std::string()),
                           const Variables&        variables = Variables()) noexcept;
    /**
     * @Deconstructor
     **/
    ~BpmnException() noexcept override = default;
    /**
     * @Assignment Operator
     **/
    BpmnException& operator=(const BpmnException&)     = default;
    BpmnException& operator=(BpmnException&&) noexcept = default;
    /**
     * @what
     *
     * Returns a string of the underling error message.
     **/
    const char* what() const noexcept override;
    /**
     * @getErrorMessage
     *
     * @brief
     * Returns the error message.
     **/
    virtual const web::json::value& getErrorMessage() const noexcept;
    /**
     * @getErrorDetails
     *
     * @brief
     * Returns the error details.
     **/
    virtual const web::json::value& getErrorCode() const noexcept;
    /**
     * @getVariables
     *
     * @brief
     * Returns the error variables.
     **/
    virtual const web::json::value& cgetVariables() const noexcept;
    virtual       web::json::value&  getVariables()       noexcept;
    /**
     * @addErrorMessage
     *
     * @brief
     * Removes any message there was and adds the new one.
     * @errorMessage: A short error related message for Camunda
     **/
    virtual void addErrorMessage(const std::string&      errorMessage) noexcept;
    virtual void addErrorMessage(const web::json::value& errorMessage) noexcept;
    /**
     * @addErrorCode
     *
     * @brief
     * Removes any message there was and adds the new one.
     * @errorCode: Code related to error
     **/
    virtual void addErrorCode(const std::string&      errorCode) noexcept;
    virtual void addErrorCode(const web::json::value& errorCode) noexcept;
    /**
     * @addVariables
     *
     * @brief
     * Removes any variables present and adds the ones passed in.
     * @variables: Values to be sent to Camunda
     **/
    virtual void addVariables(const Variables& variables) noexcept;
  private:
    web::json::value m_errorMessage;
    web::json::value m_errorCode;
    Variables        m_variables;
  };
}// camunda
#endif
/* bpmn_exception.hpp */

