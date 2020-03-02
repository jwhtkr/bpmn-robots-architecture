/**
 * @File: message_handler.cpp
 * @Date: 10 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * This class can be used to send messages and signals to and from bpmn.
 **/

/* Local Headers */
#include"bpmn_interface/message_handler.hpp"

/* Rest Headers */
#include<cpprest/json.h>
#include<cpprest/http_client.h>

/* C++ Headers */
#include<string>

namespace bpmn
{
  MessageHandler::MessageHandler(const std::string& base_uri, const std::string& workerId)
   : m_client(base_uri + "engine-rest/"),
     m_base_uri(base_uri),
     m_workerId(workerId)
  {}

  MessageHandler::MessageHandler(const web::json::value& base_uri, const web::json::value& workerId)
   : MessageHandler(base_uri.as_string(), workerId.as_string())
  {}

  const std::string& MessageHandler::getBaseUri() const noexcept
  {
    return this->m_base_uri;
  }

  const std::string& MessageHandler::getWorkerId() const noexcept
  {
    return this->m_workerId;
  }
}// bpmn

/* message_handler.cpp */

