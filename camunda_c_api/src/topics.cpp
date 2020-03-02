/**
 * @File: topics.cpp
 * @Date: 10 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Base class for getting specific information from a fetch and lock's parameters.
 **/

/* Local Headers */
#include"camunda_c_api/topics.hpp"
#include"camunda_c_api/json.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>
#include<vector>
#include<stdexcept>

namespace camunda
{
  Topics::Topics(const web::json::value& other) noexcept
   : Json::Json(other)
  {}

  Topics::Topics(const std::string&              topicName,
                 const uint32_t                  lockDuration,
                 const std::vector<std::string>& variables) noexcept
  {
    this->Topics::addTopic(topicName, lockDuration, variables);
  }

  Topics::Topics(const web::json::value& topicName,
                 const web::json::value& lockDuration,
                 const web::json::value& variables) noexcept
  {
    this->Topics::addTopic(topicName, lockDuration, variables);
  }

  void Topics::addTopicName(const std::string& topicName, const uint32_t array_index)
  {
    this->Topics::addTopicName(web::json::value(topicName), array_index);
  }

  void Topics::addTopicName(const web::json::value& topicName, const uint32_t array_index)
  {
    try
    {
      (*this)[array_index]["topicName"] = topicName;
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::addTopicName error, " + static_cast<std::string>(e.what()));
    }
  }

  void Topics::addLockDuration(const uint32_t lockDuration, const uint32_t array_index)
  {
    this->Topics::addLockDuration(web::json::value(lockDuration), array_index);
  }

  void Topics::addLockDuration(const web::json::value& lockDuration, const uint32_t array_index)
  {
    try
    {
      (*this)[array_index]["lockDuration"] = lockDuration;
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::addLockDuration error, " + static_cast<std::string>(e.what()));
    }
  }

  void Topics::addVariables(const std::vector<std::string>& variables, const uint32_t array_index)
  {
    try
    {
      for(uint32_t var_it = 0; var_it < variables.size(); var_it++)
      {
        this->Topics::addVariable(variables.at(var_it), array_index);
      }
    }
    catch(std::exception& e)
    {
      throw std::runtime_error("Topics::addVariables error, " + static_cast<std::string>(e.what()));
    }
  }

  void Topics::addVariables(const web::json::value& variables, const uint32_t array_index)
  {
    try
    {
      for(uint32_t var_it = 0; var_it < variables.as_array().size(); var_it++)
      {
        this->Topics::addVariable(variables.as_array().at(var_it), array_index);
      }
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::addVariables error, " + static_cast<std::string>(e.what()));
    }
  }

  void Topics::addVariable(const std::string& variable, const uint32_t array_index)
  {
    this->Topics::addVariable(web::json::value(variable), array_index);
  }

  void Topics::addVariable(const web::json::value& variable, const uint32_t array_index)
  {
    try
    {
      auto vars = &(*this)[array_index]["variables"];
      if(!vars->is_null())
      {
        (*vars)[vars->as_array().size()] = variable;
      }
      else
      {
        (*vars)[0] = variable;
      }
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::addVariable error, " + static_cast<std::string>(e.what()));
    }
  }

  void Topics::addTopic(const std::string&              topicName,
                        const uint32_t                  lockDuration,
                        const std::vector<std::string>& variables) noexcept
  {
    uint32_t size = this->size();

    this->Topics::addTopicName(   topicName,    size);
    this->Topics::addLockDuration(lockDuration, size);
    this->Topics::addVariables(   variables,    size);
  }

  void Topics::addTopic(const web::json::value& topicName,
                        const web::json::value& lockDuration,
                        const web::json::value& variables)
  {
    try
    {
      uint32_t size = this->size();

      this->Topics::addTopicName(topicName,       size);
      this->Topics::addLockDuration(lockDuration, size);
      this->Topics::addVariables(variables,       size);
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::addTopic error, " + static_cast<std::string>(e.what()));
    }
  }

  void Topics::removeTopic(const uint32_t array_index)
  {
    try
    {
      this->remove(array_index);
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::removeTopic error, " + static_cast<std::string>(e.what()));
    }
  }

  void Topics::removeTopicName(const uint32_t array_index)
  {
    try
    {
      this->at(array_index).erase("topicName");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::removeTopicName error, " + static_cast<std::string>(e.what()));
    }
  }

  void Topics::removeLockDuration(const uint32_t array_index)
  {
    try
    {
      this->at(array_index).erase("lockDuration");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::removeLockDuration error, " + static_cast<std::string>(e.what()));
    }
  }

  void Topics::removeVariables(const uint32_t array_index)
  {
    try
    {
      this->at(array_index).erase("variables");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::removeVariables error, " + static_cast<std::string>(e.what()));
    }
  }

  bool Topics::is_safe() const noexcept
  {
    if(!this->Json::is_safe())
    {
      return false;
    }
    for(uint32_t var_it = 0; var_it < this->size(); var_it++)
    {
      if(!this->at(var_it).has_string_field ("topicName")  ||
         !this->at(var_it).has_integer_field("lockDuration"))
      {
        return false;
      }
    }
    return true;
  }

  const web::json::value& Topics::cgetTopics() const
  {
    try
    {
      if(!this->is_safe())
      {
        throw std::runtime_error("Object not safe");
      }
      return this->cget();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::cgetTopics error, " + static_cast<std::string>(e.what()));
    }
  }

  web::json::value& Topics::getTopics()
  {
    try
    {
      if(!this->is_safe())
      {
        throw std::runtime_error("Object not safe");
      }
      return this->get();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::getTopics error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& Topics::getTopicName(const uint32_t array_index) const
  {
    try
    {
      return this->getTopic(array_index).at("topicName");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::getTopicName error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& Topics::getLockDuration(const uint32_t array_index) const
  {
    try
    {
      return this->getTopic(array_index).at("lockDuration");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::getLockDuration error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& Topics::getVariables(const uint32_t array_index) const
  {
    try
    {
      return this->getTopic(array_index).at("variables");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::getVariables error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& Topics::getTopic(const uint32_t array_index) const
  {
    try
    {
      return this->at(array_index);
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Topics::getTopic error, " + static_cast<std::string>(e.what()));
    }
  }
}; // camunda

/* topics.cpp */

