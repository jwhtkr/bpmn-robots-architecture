/**
 * @File: json.cpp
 * @Date: 20 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * A base class for other camunda objects to inherit from.
 **/

/* Local Headers */
#include"camunda_c_api/json.hpp"
#include"camunda_c_api/json_print.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>
#include<stdexcept>

namespace camunda
{
  Json::Json(const web::json::value& other)
   : m_json(other)
  {}

  Json& Json::operator=(const web::json::value& other) noexcept
  {
    this->m_json = other;
    return *this;
  }

  bool Json::operator==(const Json& other) const noexcept
  {
    return (this->m_json == other.m_json);
  }

  bool Json::operator!=(const Json& other) const noexcept
  {
    return !(*this == other);
  }

  web::json::value& Json::operator[](const std::string& field_name) noexcept
  {
    return this->m_json[field_name];
  }

  web::json::value& Json::operator[](const uint32_t array_index) noexcept
  {
    return this->m_json[array_index];
  }

  web::json::value& Json::at(const std::string& field_name)
  {
    try
    {
      return this->m_json.at(field_name);
    }
    catch(std::exception& e)
    {
      throw std::runtime_error("Json::at error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& Json::at(const std::string& field_name) const
  {
    try
    {
      return this->m_json.at(field_name);
    }
    catch(std::exception& e)
    {
      throw std::runtime_error("Json::at error, " + static_cast<std::string>(e.what()));
    }
  }

  web::json::value& Json::at(const uint32_t array_index)
  {
    try
    {
      return this->m_json.at(array_index);
    }
    catch(std::exception& e)
    {
      throw std::runtime_error("Json::at error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::value& Json::at(const uint32_t array_index) const
  {
    try
    {
      return this->m_json.at(array_index);
    }
    catch(std::exception& e)
    {
      throw std::runtime_error("Json::at error, " + static_cast<std::string>(e.what()));
    }
  }

  void Json::add(const std::string& field_name, const web::json::value& value) noexcept
  {
    this->m_json[field_name] = value;
  }

  void Json::add(const uint32_t array_index, const web::json::value& value) noexcept
  {
    this->m_json[array_index] = value;
  }

  void Json::remove(const std::string& field_name) noexcept
  {
    if(this->has_field(field_name))
    {
      this->m_json.erase(field_name);
    }
  }

  void Json::remove(const uint32_t array_index) noexcept
  {
    if(this->size() >= array_index)
    {
      this->m_json.erase(array_index);
    }
  }

  web::json::value& Json::get()
  {
    return this->m_json;
  }

  const web::json::value& Json::cget() const
  {
    return this->m_json;
  }

  bool Json::has_field(const std::string& field_name) const noexcept
  {
    return this->m_json.has_field(field_name);
  }

  bool Json::is_safe() const noexcept
  {
    if(0 == this->m_json.size())
    {
      return false;
    }
    return true;
  }

  web::json::object::iterator Json::begin()
  {
    try
    {
      return this->get().as_object().begin();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Json::begin error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::object::const_iterator Json::cbegin() const
  {
    try
    {
      return this->cget().as_object().cbegin();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Variables::begin error, " + static_cast<std::string>(e.what()));
    }
  }

  web::json::object::iterator Json::end()
  {
    try
    {
      return this->get().as_object().end();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Json::end error, " + static_cast<std::string>(e.what()));
    }
  }

  const web::json::object::const_iterator Json::cend() const
  {
    try
    {
      return this->cget().as_object().cend();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Json::end error, " + static_cast<std::string>(e.what()));
    }
  }

  uint32_t Json::size() const
  {
    try
    {
      return this->cget().size();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("Json::size error, " + static_cast<std::string>(e.what()));
    }
  }

  void Json::print() const noexcept
  {
    printJson(this->cget());
  }
}; // camunda

/* json.cpp */

