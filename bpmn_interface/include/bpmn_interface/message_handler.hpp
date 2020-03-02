/**
 * @File: message_handler.hpp
 * @Date: 10 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * This class can be used to send messages and signals to and from bpmn.
 **/

#ifndef MESSAGE_HANDLER_MESSAGE_HANDLER_HPP
#define MESSAGE_HANDLER_MESSAGE_HANDLER_HPP

/* Local Headers */
#include"bpmn_interface/task_lock.hpp"

/* Camunda API Headers */
#include<camunda_c_api/camunda_c_api.hpp>

/* Rest Headers */
#include<cpprest/json.h>
#include<cpprest/http_client.h>

/* C++ Headers */
#include<string>
#include<stdexcept>
#include<memory>
#include<type_traits>

/**
 * @DEBUG
 * If debug is set to true additional debugging exceptions will be thrown.
 **/
#define MESSAGE_HANDLER_DEBUG false

namespace bpmn
{
  class MessageHandler
  {
  public:
    /**
     * @Default Constructor
     **/
    MessageHandler() = delete;
    /**
     * @Copy Constructor
     **/
    MessageHandler(const MessageHandler&) = delete;
    MessageHandler(MessageHandler&&)      = delete;
    /**
     * @Constructor
     **/
    MessageHandler(const std::string&      base_uri, const std::string&      workerId);
    MessageHandler(const web::json::value& base_uri, const web::json::value& workerId);
    /**
     * @Deconstructor
     **/
    ~MessageHandler() noexcept = default;
    /**
     * @Assignment Operator
     **/
    MessageHandler& operator=(const MessageHandler&) = delete;
    MessageHandler& operator=(MessageHandler&&)      = delete;
    /**
     * @sendMessage
     **/
    template<typename VAR_OBJ = camunda::Variables>
    void sendMessage(const std::string& messageName,
                     const VAR_OBJ&     processVariables,
                     const std::string& processInstanceId = std::string());
    /**
     * @throwSignal
     **/
    template<typename SIG_OBJ = camunda::ThrowSignal>
    void throwSignal(const SIG_OBJ& signal);
    /**
     * @getMessage
     **/
    template<typename VAR_OBJ = camunda::Variables, typename TOP_OBJ = camunda::Topics>
    std::shared_ptr<VAR_OBJ> getMessage(const TOP_OBJ& topics,
                                        const bool     usePriority          = false,
                                        const uint32_t asyncResponseTimeout = 180000,
                                        const bool     allowFailure         = true);
    /**
     * @get
     **/
    const std::string& getBaseUri()  const noexcept;
    const std::string& getWorkerId() const noexcept;
  private:
    /* Talks to Camunda */
    web::http::client::http_client m_client;
    /* Holds this objects info */
    std::string m_base_uri;
    std::string m_workerId;
  };

  template<typename VAR_OBJ>
  void MessageHandler::sendMessage(const std::string& messageName,
                                   const VAR_OBJ&     processVariables,
                                   const std::string& processInstanceId)
  {
    static_assert(std::is_base_of<camunda::Variables, VAR_OBJ>::value,
                  "VAR_OBJ has to be a base class of camunda::Variables");

    try
    {
      web::json::value message;

      message["messageName"] = web::json::value(messageName);
      if(std::string() != processInstanceId)
      {
        message["processInstanceId"] = web::json::value(processInstanceId);
      }
      if(web::json::value() != processVariables.cget())
      {
        message["processVariables"] = processVariables.cget();
      }

      this->m_client.request(web::http::methods::POST, "message", message).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("MessageHandler::sendMessage error, " + static_cast<std::string>(e.what()));
    }
  }

  template<typename SIG_OBJ>
  void MessageHandler::throwSignal(const SIG_OBJ& signal)
  {
    static_assert(std::is_base_of<camunda::ThrowSignal, SIG_OBJ>::value,
                  "SIG_OBJ has to be a base class of camunda::ThrowSignal");

    try
    {
      this->m_client.request(web::http::methods::POST, "signal", signal.cgetSignal())
        #if MESSAGE_HANDLER_DEBUG
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
      throw std::runtime_error("MessageHandler::throwSignal error, " + static_cast<std::string>(e.what()));
    }
  }

  template<typename VAR_OBJ, typename TOP_OBJ>
  std::shared_ptr<VAR_OBJ> MessageHandler::getMessage(const TOP_OBJ& topics,
                                                      const bool     usePriority,
                                                      const uint32_t asyncResponseTimeout,
                                                      const bool     allowFailure)
  {
    static_assert(std::is_base_of<camunda::Variables, VAR_OBJ>::value,
                  "VAR_OBJ has to be a base class of camunda::Variables");
    static_assert(std::is_base_of<camunda::Topics,    TOP_OBJ>::value,
                  "TOP_OBJ has to be a base class of camunda::Topics");

    try
    {
      std::unique_ptr<bpmn::TaskLock<VAR_OBJ, TOP_OBJ>> task_lock;
      bool failed;

      // Get the task
      do
      {
        failed = false;
        try
        {
          task_lock.reset(new bpmn::TaskLock<VAR_OBJ, TOP_OBJ, camunda::CompleteRequest>(this->getBaseUri(),
                                                                                         this->getWorkerId(),
                                                                                         topics,
                                                                                         camunda::CompleteRequest(),
                                                                                         1,
                                                                                         usePriority,
                                                                                         asyncResponseTimeout));
        }
        catch(const std::runtime_error&)
        {
          failed = true;
        }
      } while(allowFailure && failed);

      return std::shared_ptr<VAR_OBJ>(new VAR_OBJ(task_lock->getResponsVars()));
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("MessageHandler::getMessage error, " + static_cast<std::string>(e.what()));
    }
  }
}// bpmn

#endif
/* message_handler.hpp */

