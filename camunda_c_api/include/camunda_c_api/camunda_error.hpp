/**
 * @File: camunda_error.hpp
 * @Date: 5 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Base class for describing a handle external task failure request.
 **/

#ifndef CAMUNDA_ERROR_HANDLING_CAMUNDA_ERROR_HPP
#define CAMUNDA_ERROR_HANDLING_CAMUNDA_ERROR_HPP

/* Local Headers */
#include"camunda_c_api/json.hpp"
#include"camunda_c_api/camunda_exception.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>

namespace camunda
{
  class CamundaError : public Json
  {
  public:
    /**
     * @Default Constructor
     **/
    CamundaError() = default;
    /**
     * @Copy Constructor
     **/
    CamundaError(const CamundaError&) = default;
    explicit CamundaError(const web::json::value& other) noexcept;
    /**
     * @Move Constructor
     **/
    CamundaError(CamundaError&&) noexcept = default;
    /**
     * @Constructor
     *
     * @brief
     * After construction object can have all needed information for use.
     * @workerId: The worker id of the process that locked the failed task
     * @errorMessage: Message about the failure
     * @errorDetails: Details about what failed
     * @retries: How many retries the Camunda task has left
     * @retryTimeout: How long Camunda will wait on the retry
     * @exception: A exception object holding information about the failure
     **/
    explicit CamundaError(const std::string& workerId,
                          const std::string& errorMessage = std::string(),
                          const std::string& errorDetails = std::string(),
                          const uint32_t     retries      = 0,
                          const uint32_t     retryTimeout = 0) noexcept;

    explicit CamundaError(const web::json::value& workerId,
                          const web::json::value& errorMessage = web::json::value(std::string()),
                          const web::json::value& errorDetails = web::json::value(std::string()),
                          const web::json::value& retries      = web::json::value(0),
                          const web::json::value& retryTimeout = web::json::value(0)) noexcept;

    CamundaError(const std::string&      workerId,
                 const CamundaException& exception,
                 const uint32_t          retries      = 0,
                 const uint32_t          retryTimeout = 0) noexcept;

    CamundaError(const web::json::value& workerId,
                 const CamundaException& exception,
                 const web::json::value& retries      = web::json::value(0),
                 const web::json::value& retryTimeout = web::json::value(0)) noexcept;
    /**
     * @Deconstructor
     **/
    ~CamundaError() noexcept override = default;
    /**
     * @Assignment Operator
     **/
    CamundaError& operator=(const CamundaError&)     = default;
    CamundaError& operator=(CamundaError&&) noexcept = default;
    /**
     * @add
     *
     * @brief
     * Replaces whatever value was there with the value passed in.
     * @workerId: The worker id of the process that locked the failed task
     * @errorMessage: Message about the failure
     * @errorDetails: Details about what failed
     * @retries: How many retries the Camunda task has left
     * @retryTimeout: How long Camunda will wait on the retry
     * @exception: A exception object holding information about the failure
     */
    virtual void addWorkerId    (const std::string&      workerId)     noexcept;
    virtual void addWorkerId    (const web::json::value& workerId)     noexcept;
    virtual void addErrorMessage(const std::string&      errorMessage) noexcept;
    virtual void addErrorMessage(const web::json::value& errorMessage) noexcept;
    virtual void addErrorDetails(const std::string&      errorDetails) noexcept;
    virtual void addErrorDetails(const web::json::value& errorDetails) noexcept;
    virtual void addRetries     (const uint32_t          retries)      noexcept;
    virtual void addRetries     (const web::json::value& retries)      noexcept;
    virtual void addRetryTimeout(const uint32_t          retryTimeout) noexcept;
    virtual void addRetryTimeout(const web::json::value& retryTimeout) noexcept;
    virtual void addException   (const CamundaException& exception)    noexcept;
    /**
     * @remove
     *
     * @brief
     * Removes the value from the underlining json object.
     **/
    virtual void removeWorkerId()     noexcept;
    virtual void removeErrorMessage() noexcept;
    virtual void removeErrorDetails() noexcept;
    virtual void removeRetries()      noexcept;
    virtual void removeRetryTimeout() noexcept;
    /**
     * @is_safe
     *
     * @brief
     * Returns true if this object has all the information needed to be used.
     **/
    bool is_safe() const noexcept override;
    /**
     * @get
     *
     * @brief
     * Returns the asked for value, throws a runtime error if it's
     * not present.
     **/
    virtual const web::json::value&  getWorkerId()     const;
    virtual const web::json::value&  getErrorMessage() const;
    virtual const web::json::value&  getErrorDetails() const;
    virtual const web::json::value&  getRetries()      const;
    virtual const web::json::value&  getRetryTimeout() const;
    /**
     * @getError
     *
     * @brief
     * Returns the inderlieing json object as long as it's ready to use,
     * throws a runtime error if it's not.
     **/
    virtual const web::json::value& cgetError()        const;
    virtual       web::json::value&  getError();
  };
}// camunda

#endif
/* camunda_error.hpp */

