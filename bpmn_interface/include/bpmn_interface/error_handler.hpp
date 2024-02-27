/**
 * @File: error_handler.hpp
 * @Date: 8 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Whenever a error needs to be throws to camunda this object can be used.
 **/

#ifndef ERROR_HANDLER_ERROR_HANDLER_HPP
#define ERROR_HANDLER_ERROR_HANDLER_HPP

/* Camunda API Headers */
#include<camunda_c_api/camunda_c_api.hpp>

/* Rest Headers */
#include<cpprest/json.h>
#include<cpprest/http_client.h>

/* C++ Headers */
#include<string>
#include<type_traits>
#include<stdexcept>

/**
 * @ERROR_HANDLER_DEBUG
 * If set to true additional debugging exceptions will be thrown.
 **/
#define ERROR_HANDLER_DEBUG false

namespace bpmn
{
  template<typename BPMNE_OBJ = camunda::BpmnError, typename FAIL_OBJ = camunda::CamundaError>
  class ErrorHandler
  {
  public:
    /**
     * @Default Constructor
     **/
    ErrorHandler() = delete;
    /**
     * @Copy Constructor
     **/
    ErrorHandler(const ErrorHandler&) = delete;
    ErrorHandler(ErrorHandler&&)      = delete;
    /**
     * @Constructor
     *
     * @brief
     * After construction object may be used to throw failure and bpmn error to camunda.
     * @base_uri: The URI of the camunda server
     * @external_task_id: The id of the external task that will be having errors
     * @bpmn_error: Object held internally until it's used
     * @failure_error: Object held internally until it's used
     * @workerId: The id of the process that is throwing errors
     **/
    ErrorHandler<BPMNE_OBJ, FAIL_OBJ>(const std::string& base_uri,
                                      const std::string& external_task_id,
                                      const BPMNE_OBJ&   bpmn_error,
                                      const FAIL_OBJ&    failure_error);

    ErrorHandler<BPMNE_OBJ, FAIL_OBJ>(const std::string&      base_uri,
                                      const web::json::value& external_task_id,
                                      const BPMNE_OBJ&        bpmn_error,
                                      const FAIL_OBJ&         failure_error);

    ErrorHandler<BPMNE_OBJ, FAIL_OBJ>(const std::string& base_uri,
                                      const std::string& external_task_id,
                                      const std::string& workerId);

    ErrorHandler<BPMNE_OBJ, FAIL_OBJ>(const std::string& base_uri,
                                      const web::json::value& external_task_id,
                                      const web::json::value& workerId);
    /**
     * @Deconstructor
     **/
    ~ErrorHandler() noexcept = default;
    /**
     * @Assignment Operator
     **/
    ErrorHandler& operator=(const ErrorHandler&) = delete;
    ErrorHandler& operator=(ErrorHandler&&)      = delete;
    /**
     * @throw
     *
     * @brief
     * Overrides the values held internally with the values passed in then throws an
     * error to Camunda.
     * @bpmn_error: Object held internally until it's used
     * @failure_error: Object held internally until it's used
     * @errorCode: Code that Camunda will connect to an error
     * @errorMessage: Description of error
     * @errorDetails: Detailed description of error
     * @variables: Variables that will be sent to Camunda with the error
     * @retries: How many more times Camunda will try to run the task
     * @retryTimeout: How long Camunda will wait for the retries
     **/
    void throwBpmnError(const BPMNE_OBJ& bpmn_error = BPMNE_OBJ());

    void throwBpmnError(const camunda::BpmnException& bpmn_exception);

    void throwBpmnError(const std::string&        errorCode,
                        const std::string&        errorMessage,
                        const camunda::Variables& variables = camunda::Variables());

    void throwBpmnError(const web::json::value&   errorCode,
                        const web::json::value&   errorMessage,
                        const camunda::Variables& variables = camunda::Variables());

    void throwFailure(const FAIL_OBJ& failure_error = FAIL_OBJ());

    void throwFailure(const camunda::CamundaException& failure_exception,
                      const uint32_t                   retries,
                      const uint32_t                   retryTimeout = 0);

    void throwFailure(const camunda::CamundaException& failure_exception,
                      const web::json::value&          retries      = web::json::value(0),
                      const web::json::value&          retryTimeout = web::json::value(0));

    void throwFailure(const std::string& errorMessage,
                      const std::string& errorDetails,
                      const uint32_t     retries      = 0,
                      const uint32_t     retryTimeout = 0);

    void throwFailure(const web::json::value& errorMessage,
                      const web::json::value& errorDetails,
                      const web::json::value& retries      = web::json::value(0),
                      const web::json::value& retryTimeout = web::json::value(0));
    /**
     * @add
     *
     * @brief
     * Adds or replaces the internal values with those passed in.
     * @external_task_id: The id of the external task that will be having errors
     * @bpmn_error: Object held internally until it's used
     * @failure_error: Object held internally until it's used
     * @workerId: The id of the process that is throwing errors
     **/
    void addBpmnError(     const BPMNE_OBJ&        bpmn_error)       noexcept;
    void addFailure(       const FAIL_OBJ&         failure_error)    noexcept;
    void addWorkerId(      const std::string&      workerId)         noexcept;
    void addWorkerId(      const web::json::value& workerId)         noexcept;
    void addExternalTaskId(const std::string&      external_task_id) noexcept;
    /**
     * @get
     *
     * @brief
     * Returns the asked for values.
     **/
    const std::string&      getExternalTaskId() const noexcept;
    const web::json::value& getWorkerId()       const;
    const BPMNE_OBJ&       cgetBpmnError()      const noexcept;
          BPMNE_OBJ&        getBpmnError()            noexcept;
    const FAIL_OBJ&        cgetFailure()        const noexcept;
          FAIL_OBJ&         getFailure()              noexcept;
  private:
    /* Talks to Camunda */
    web::http::client::http_client m_client;
    std::string m_external_task_id;
    /* Holds error information */
    BPMNE_OBJ m_bpmn_error;
    FAIL_OBJ  m_failure;
  };

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::ErrorHandler(const std::string& base_uri,
                                                  const std::string& external_task_id,
                                                  const BPMNE_OBJ&   bpmn_error,
                                                  const FAIL_OBJ&    failure_error)
   : m_client(base_uri + "engine-rest/external-task/"),
     m_external_task_id(external_task_id),
     m_bpmn_error(bpmn_error),
     m_failure(failure_error)
  {
    static_assert(std::is_base_of<camunda::BpmnError, BPMNE_OBJ>::value,
                  "BPMNE_OBJ has to be a child of the camunda::BpmnError base class");
    static_assert(std::is_base_of<camunda::CamundaError, FAIL_OBJ>::value,
                  "FAIL_OBJ has to be a child of the camunda::CamundaError base class");
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::ErrorHandler(const std::string&      base_uri,
                                                  const web::json::value& external_task_id,
                                                  const BPMNE_OBJ&        bpmn_error,
                                                  const FAIL_OBJ&         failure_error)
   : ErrorHandler<BPMNE_OBJ, FAIL_OBJ>(base_uri, external_task_id.as_string(), bpmn_error, failure_error)
  {}

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::ErrorHandler(const std::string& base_uri,
                                                  const std::string& external_task_id,
                                                  const std::string& workerId)
   : ErrorHandler<BPMNE_OBJ, FAIL_OBJ>(base_uri,
                                       external_task_id,
                                       camunda::BpmnError(workerId),
                                       camunda::CamundaError(workerId))
  {}

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::ErrorHandler(const std::string& base_uri,
                                                  const web::json::value& external_task_id,
                                                  const web::json::value& workerId)
   : ErrorHandler<BPMNE_OBJ, FAIL_OBJ>(base_uri, external_task_id.as_string(), workerId.as_string())
  {}

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::throwBpmnError(const BPMNE_OBJ& bpmn_error)
  {
    if(BPMNE_OBJ() != bpmn_error)
    {
      this->addBpmnError(bpmn_error);
    }

    try
    {
      this->m_client.request(web::http::methods::POST,
                             this->getExternalTaskId() + "/bpmnError",
                             this->cgetBpmnError().cgetError())
        #if ERROR_HANDLER_DEBUG
        .then([](const web::http::http_response& response)
        {
          web::json::value res = response.extract_json().get();
          if(web::json::value() != res)
          {
            throw std::runtime_error(res.serialize());
          }
        })
        #endif
         .wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ErrorHandler::throwBpmnError error, " + static_cast<std::string>(e.what()));
    }
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::throwBpmnError(const camunda::BpmnException& bpmn_exception)
  {
    this->getBpmnError().addBpmnException(bpmn_exception);
    this->throwBpmnError();
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::throwBpmnError(const std::string&        errorCode,
                                                         const std::string&        errorMessage,
                                                         const camunda::Variables& variables)
  {
    this->throwBpmnError(web::json::value(errorCode), web::json::value(errorMessage), variables);
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::throwBpmnError(const web::json::value&   errorCode,
                                                         const web::json::value&   errorMessage,
                                                         const camunda::Variables& variables)
  {
    this->getBpmnError().addErrorCode(errorCode);
    this->getBpmnError().addErrorMessage(errorMessage);
    if(camunda::Variables() != variables)
    {
      this->getBpmnError().addVariables(variables);
    }
    this->throwBpmnError();
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::throwFailure(const FAIL_OBJ& failure_error)
  {
    if(FAIL_OBJ() != failure_error)
    {
      this->addFailure(failure_error);
    }

    try
    {
      this->m_client.request(web::http::methods::POST,
                             this->getExternalTaskId() + "/failure",
                             this->cgetFailure().cgetError())
        #if ERROR_HANDLER_DEBUG
        .then([](const web::http::http_response& response)
        {
          web::json::value res = response.extract_json().get();
          if(web::json::value() != res)
          {
            throw std::runtime_error(res.serialize());
          }
        })
        #endif
          .wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ErrorHandler::throwFailure error, " + static_cast<std::string>(e.what()));
    }
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::throwFailure(const camunda::CamundaException& failure_exception,
                                                       const uint32_t                   retries,
                                                       const uint32_t                   retryTimeout)
  {
    this->throwFailure(failure_exception, web::json::value(retries), web::json::value(retryTimeout));
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::throwFailure(const camunda::CamundaException& failure_exception,
                                                       const web::json::value&          retries,
                                                       const web::json::value&          retryTimeout)
  {
    this->getFailure().addException(failure_exception);
    if(web::json::value(0) != retries)
    {
      this->getFailure().addRetries(retries);
    }
    if(web::json::value(0) != retryTimeout)
    {
      this->getFailure().addRetryTimeout(retryTimeout);
    }

    this->throwFailure();
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::throwFailure(const std::string& errorMessage,
                                                       const std::string& errorDetails,
                                                       const uint32_t     retries,
                                                       const uint32_t     retryTimeout)
  {
    this->throwFailure(web::json::value(errorMessage),
                       web::json::value(errorDetails),
                       web::json::value(retries),
                       web::json::value(retryTimeout));
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::throwFailure(const web::json::value& errorMessage,
                                                       const web::json::value& errorDetails,
                                                       const web::json::value& retries,
                                                       const web::json::value& retryTimeout)
  {
    this->getFailure().addErrorMessage(errorMessage);
    this->getFailure().addErrorDetails(errorDetails);
    if(web::json::value(0) != retries)
    {
      this->getFailure().addRetries(retries);
    }
    if(web::json::value(0) != retryTimeout)
    {
      this->getFailure().addRetryTimeout(retryTimeout);
    }

    this->throwFailure();
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::addBpmnError(const BPMNE_OBJ& bpmn_error) noexcept
  {
    this->getBpmnError() = bpmn_error;
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::addFailure(const FAIL_OBJ& failure_error) noexcept
  {
    this->getFailure() = failure_error;
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::addWorkerId(const std::string& workerId) noexcept
  {
    this->addWorkerId(web::json::value(workerId));
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::addWorkerId(const web::json::value& workerId) noexcept
  {
    this->getBpmnError().addWorkerId(workerId);
    this->getFailure()  .addWorkerId(workerId);
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  void ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::addExternalTaskId(const std::string& external_task_id) noexcept
  {
    this->m_external_task_id = external_task_id;
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  const std::string& ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::getExternalTaskId() const noexcept
  {
    return this->m_external_task_id;
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  const web::json::value& ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::getWorkerId() const
  {
    try
    {
      return this->cgetBpmnError().getWorkerId();
    }
    catch(const std::exception& ex)
    {
      try
      {
        return this->cgetFailure().getWorkerId();
      }
      catch(const std::exception& e)
      {
        throw std::runtime_error("ErrorHandler::getWorkerId error, " +
                                 static_cast<std::string>(e.what())  +
                                 " and "                             +
                                 static_cast<std::string>(ex.what()));
      }
    }
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  const BPMNE_OBJ& ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::cgetBpmnError() const noexcept
  {
    return this->m_bpmn_error;
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  BPMNE_OBJ& ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::getBpmnError() noexcept
  {
    return this->m_bpmn_error;
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  const FAIL_OBJ& ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::cgetFailure() const noexcept
  {
    return this->m_failure;
  }

  template<typename BPMNE_OBJ, typename FAIL_OBJ>
  FAIL_OBJ& ErrorHandler<BPMNE_OBJ, FAIL_OBJ>::getFailure() noexcept
  {
    return this->m_failure;
  }
}// bpmn

#endif
/* error_handler.hpp */

