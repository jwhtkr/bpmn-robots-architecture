/**
 * @File: camunda_exception.hpp
 * @Date: 31 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * This exception class is made specifically so that when it is throw it will be
 * passed all the way up to Camunda and handle failure being thrown.
 **/

#ifndef CAMUNDA_ERROR_HANDLING_CAMUNDA_EXCEPTION_HPP
#define CAMUNDA_ERROR_HANDLING_CAMUNDA_EXCEPTION_HPP

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<exception>
#include<string>

namespace camunda
{
  class CamundaException : public std::exception
  {
  public:
    /**
     * @Default Constructor
     **/
    CamundaException() = default;
    /**
     * @Copy Constructor
     **/
    CamundaException(const CamundaException&)     = default;
    CamundaException(CamundaException&&) noexcept = default;
    /**
     * @Constructor
     *
     * @brief
     * Fills in the values
     * @errorMessage: A short error related message for Camunda
     * @errorDetails: As much information about the error as possible
     **/
    explicit CamundaException(const std::string& errorMessage,
                              const std::string& errorDetails = std::string()) noexcept;
    explicit CamundaException(const web::json::value& errorMessage,
                              const web::json::value& errorDetails = web::json::value()) noexcept;
    /**
     * @Deconstructor
     **/
    ~CamundaException() noexcept override = default;
    /**
     * @Assignment Operator
     **/
    CamundaException& operator=(const CamundaException&) = default;
    CamundaException& operator=(CamundaException&&) noexcept = default;
    /**
     * @what
     *
     * @brief
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
    virtual const web::json::value& getErrorDetails() const noexcept;
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
     * @addErrorDetails
     *
     * @brief
     * Removes any message there was and adds the new one.
     * @errorDetails: As much information about the error as possible
     **/
    virtual void addErrorDetails(const std::string&      errorDetails) noexcept;
    virtual void addErrorDetails(const web::json::value& errorDetails) noexcept;
  private:
    web::json::value m_errorMessage;
    web::json::value m_errorDetails;
  };
}; // camunda

#endif
/* camunda_exception.hpp */

