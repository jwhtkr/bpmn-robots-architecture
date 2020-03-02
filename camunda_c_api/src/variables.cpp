/**
 * @variables.cpp
 * @Date: 4 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Base class for getting case specific information from the LockResponse
 * class as well as filling case specific information into the CompleteTask class.
 **/

/* Local Headers */
#include"camunda_c_api/variables.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>
#include<stdexcept>

namespace camunda
{
  Variables::Variables(const web::json::value& other) noexcept
   : Json::Json(other)
  {}

  void Variables::addVariable(const std::string& key,
                              const web::json::value& value,
                              const web::json::value& type,
                              const web::json::value& valueInfo) noexcept
  {
    this->get()[key]["value"]     = value;
    this->get()[key]["type"]      = type;
    this->get()[key]["valueInfo"] = valueInfo;
  }

  void Variables::addVariable(const std::string&      key,
                              const std::string&      value,
                              const std::string&      type,
                              const web::json::value& valueInfo) noexcept
  {
    this->addVariable(key, web::json::value(value), web::json::value(type), valueInfo);
  }

  void Variables::removeVariable(const std::string& key)
  {
    try
    {
      this->remove(key);
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Variables::removeVariable error, " + static_cast<std::string>(e.what()));
    }
  }

  bool Variables::is_safe() const noexcept
  {
    if(!this->Json::is_safe() ||
       !this->cget().is_object())
    {
      return false;
    }
    return true;
  }

  web::json::value& Variables::getVariables()
  {
    try
    {
      if(!this->is_safe())
      {
        throw std::runtime_error("object not safe to use");
      }
      return this->get();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Variables::getVariables error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& Variables::cgetVariables() const
  {
    try
    {
      if(!this->is_safe())
      {
        throw std::runtime_error("object not safe to use");
      }
      return this->cget();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Variables::cgetVariables error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& Variables::getValue(const std::string& key) const
  {
    try
    {
      return this->getVariable(key).at("value");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Variables::getValue error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& Variables::getType(const std::string& key) const
  {
    try
    {
      return this->getVariable(key).at("type");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Variables::getType error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& Variables::getValueInfo(const std::string& key) const
  {
    try
    {
      return this->getVariable(key).at("valueInfo");
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Variables::getValueInfo error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& Variables::getVariable(const std::string& key) const
  {
    try
    {
      return this->at(key);
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Variables::getVariable error, " + static_cast<std::string>(e.what()));
    }
  }
}; // camunda

/* variables.cpp */

