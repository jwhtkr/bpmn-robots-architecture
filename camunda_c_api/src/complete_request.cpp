/**
 * @File: complete_request.cpp
 * @Date: 4 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * A class to make completing tasks easier.
 **/

/* Local Headers */
#include"camunda_c_api/complete_request.hpp"
#include"camunda_c_api/variables.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>
#include<stdexcept>

namespace camunda
{
  CompleteRequest::CompleteRequest(const web::json::value& other) noexcept
   : Json::Json(other)
  {}

  CompleteRequest::CompleteRequest(const std::string& workerId, const Variables& variables) noexcept
   : CompleteRequest(web::json::value(workerId), variables)
  {}

  CompleteRequest::CompleteRequest(const web::json::value& workerId, const Variables& variables) noexcept
  {
    this->addWorkerId(workerId);
    if(Variables() != variables)
    {
      this->addVariables(variables);
    }
  }

  void CompleteRequest::addWorkerId(const std::string& workerId) noexcept
  {
    this->addWorkerId(web::json::value(workerId));
  }

  void CompleteRequest::addWorkerId(const web::json::value& workerId) noexcept
  {
    if(web::json::value() != workerId)
    {
      this->add("workerId", workerId);
    }
  }

  void CompleteRequest::addVariables(const Variables& variables) noexcept
  {
    if(Variables() != variables)
    {
      for(auto var_it = variables.cbegin(); var_it != variables.cend(); var_it++)
      {
        this->addVariable(var_it->first,
                          variables.getValue(var_it->first),
                          variables.getType(var_it->first),
                          variables.getValueInfo(var_it->first));
      }
    }
  }

  void CompleteRequest::addVariable(const std::string& key,
                                    const web::json::value& value,
                                    const web::json::value& type,
                                    const web::json::value& valueInfo) noexcept
  {
    if(web::json::value() != value)
    {
      this->get()["variables"][key]["value"] = value;
    }
    if(web::json::value() != type)
    {
      this->get()["variables"][key]["type"] = type;
    }
    if(web::json::value() != valueInfo)
    {
      this->get()["variables"][key]["valueInfo"] = valueInfo;
    }
  }

  void CompleteRequest::removeWorkerId() noexcept
  {
    this->remove("workerId");
  }

  void CompleteRequest::removeVariables() noexcept
  {
    this->remove("variables");
  }

  void CompleteRequest::removeVariable(const std::string& key)
  {
    try
    {
      this->at("variables").erase(key);
      if(0 == this->at("variables").size())
      {
        this->removeVariables();
      }
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CompleteRequest::removeVariable error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CompleteRequest::getWorkerId() const
  {
    try
    {
      return this->at("workerId");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CompleteRequest::getWorkerId error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CompleteRequest::getVariables() const
  {
    try
    {
      return this->at("variables");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CompleteRequest::getVariables error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CompleteRequest::getVariable(const std::string& key) const
  {
    try
    {
      return this->getVariables().at(key);
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CompleteRequest::getWorkerId error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CompleteRequest::getValue(const std::string& key) const
  {
    try
    {
      return this->getVariable(key).at("value");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CompleteRequest::getValue error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CompleteRequest::getValueType(const std::string& key) const
  {
    try
    {
      return this->getVariable(key).at("type");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CompleteRequest::getValueType error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CompleteRequest::getValueInfo(const std::string& key) const
  {
    try
    {
      return this->getVariable(key).at("valueInfo");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CompleteRequest::getValueInfo error, " + static_cast<std::string>(e.what()));
    }
  }

  bool CompleteRequest::is_safe() const noexcept
  {
    if(!this->Json::is_safe()     ||
       !this->has_field("workerId"))
    {
      return false;
    }
    return true;
  }

  web::json::value& CompleteRequest::getCompleteRequest()
  {
    try
    {
      if(!this->is_safe())
      {
        throw std::runtime_error("object not ready to use");
      }
      return this->get();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CompleteRequest::getCompletRequest error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& CompleteRequest::cgetCompleteRequest() const
  {
    try
    {
      if(!this->is_safe())
      {
        throw std::runtime_error("object not ready to use");
      }
      return this->cget();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("CompleteRequest::cgetCompletRequest error, " + static_cast<std::string>(e.what()));
    }
  }
}; // camunda

/* complete_request.cpp */

