/**
 * @File: lock_request.cpp
 * @Date: 16 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * A raper implementation around a json based on the request body
 * of a fetch and lock rest call.
 **/

/* Local Headers */
#include"camunda_c_api/lock_request.hpp"
#include"camunda_c_api/json.hpp"
#include"camunda_c_api/topics.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>
#include<stdexcept>

namespace camunda
{
  LockRequest::LockRequest(const web::json::value& other) noexcept
   : Json::Json(other)
  {}

  LockRequest::LockRequest(const std::string& workerId,
                           const uint32_t     maxTasks,
                           const Topics&      topics,
                           const bool         usePriority,
                           const uint32_t     asyncResponseTimeout) noexcept
   : LockRequest(web::json::value(workerId),
                 web::json::value(maxTasks),
                 topics,
                 web::json::value(usePriority),
                 web::json::value(asyncResponseTimeout))
  {}

  LockRequest::LockRequest(const web::json::value& workerId,
                           const web::json::value& maxTasks,
                           const Topics&           topics,
                           const web::json::value& usePriority,
                           const web::json::value& asyncResponseTimeout) noexcept
  {
    this->addWorkerId(workerId);
    this->addMaxTasks(maxTasks);
    this->addTopics(topics);
    this->addUsePriority(usePriority);
    this->addAsyncResponseTimeout(asyncResponseTimeout);
  }

  void LockRequest::addWorkerId(const std::string& workerId) noexcept
  {
    this->addWorkerId(web::json::value(workerId));
  }

  void LockRequest::addWorkerId(const web::json::value& workerId) noexcept
  {
    this->add("workerId", workerId);
  }

  void LockRequest::addMaxTasks(const uint32_t maxTasks) noexcept
  {
    this->addMaxTasks(web::json::value(maxTasks));
  }

  void LockRequest::addMaxTasks(const web::json::value& maxTasks) noexcept
  {
    this->add("maxTasks", maxTasks);
  }

  void LockRequest::addUsePriority(const bool usePriority) noexcept
  {
    this->addUsePriority(web::json::value(usePriority));
  }

  void LockRequest::addUsePriority(const web::json::value& usePriority) noexcept
  {
    this->add("usePriority", usePriority);
  }

  void LockRequest::addAsyncResponseTimeout(const uint32_t asyncResponseTimeout) noexcept
  {
    this->addAsyncResponseTimeout(web::json::value(asyncResponseTimeout));
  }

  void LockRequest::addAsyncResponseTimeout(const web::json::value& asyncResponseTimeout) noexcept
  {
    this->add("asyncResponseTimeout", asyncResponseTimeout);
  }

  void LockRequest::addTopics(const Topics& topics)
  {
    try
    {
      uint32_t topics_size = (*this)["topics"].size();
      for(uint32_t topic_it = 0; topic_it < topics.size(); topic_it++)
      {
        (*this)["topics"][topics_size] = topics.getTopic(topic_it);
        topics_size++;
      }
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockRequest::addTopics error: " + static_cast<std::string>(e.what()));
    }
  }

  web::json::value& LockRequest::getLockRequested()
  {
    try
    {
      if(!this->is_safe())
      {
        throw std::runtime_error("object is missing needed fields");
      }
      return this->get();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockRequest::getLockRequested error: " + static_cast<std::string>(e.what()));
    }
  }


  const web::json::value& LockRequest::cgetLockRequested() const
  {
    try
    {
      if(!this->is_safe())
      {
        throw std::runtime_error("object is missing needed fields");
      }
      return this->cget();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockRequest::cgetLockRequested error: " + static_cast<std::string>(e.what()));
    }
  }

  bool LockRequest::is_safe() const noexcept
  {
    if(this->Json::is_safe()            &&
       this->has_field("workerId")      &&
       this->has_field("maxTasks")      &&
       this->has_field("topics")        &&
       this->getWorkerId().is_string()  &&
       this->getMaxTasks().is_integer() &&
       Topics(this->getTopics()).is_safe())
    {
      return true;
    }
    return false;
  }

  const web::json::value& LockRequest::getWorkerId() const
  {
    try
    {
      return this->at("workerId");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockRequest::getWorkerId error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& LockRequest::getMaxTasks() const
  {
    try
    {
      return this->at("maxTasks");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockRequest::getMaxTasks error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& LockRequest::getUsePriority() const
  {
    try
    {
      return this->at("usePriority");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockRequest::getUsePriority error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& LockRequest::getAsyncResponseTimeout() const
  {
    try
    {
      return this->at("asyncResponseTimeout");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockRequest::asyncResponseTimeout error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& LockRequest::getTopics() const
  {
    try
    {
      return this->at("topics");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockRequest::getTopics error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& LockRequest::getTopic(const uint32_t array_index) const
  {
    try
    {
      return this->getTopics().at(array_index);
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("LockRequest::getTopic error, " + static_cast<std::string>(e.what()));
    }
  }
}; // camunda

/* lock_request.cpp */

