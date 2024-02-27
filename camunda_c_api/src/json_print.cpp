/**
 * @File: json_print.cpp
 * @Date: 20 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * This file holds printing functions for json objects.
 **/

/* Local Headers */
#include"camunda_c_api/json_print.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<iostream>
#include<string>

namespace camunda
{
  /* How wide you want one indent to be */
  #define INDENT " "

  /**
   * Helper functions
   * It's not recommended to use these directly
   **/
  void printJsonArray (const web::json::array & json_in, const std::string& indents);
  void printJsonValue (const web::json::value & json_in, const std::string& indents);
  void printJsonObject(const web::json::object& json_in, const std::string& indents);

  void printJson(const web::json::value& json_in)
  {
    try
    {
      if(json_in.is_array())
      {
        std::cout << "{";
      }
      else
      {
        std::cout << "{\n";
      }
      printJsonValue(json_in, INDENT);
      std::cout << "}\n";
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("printJson error: " + static_cast<std::string>(e.what()));
    }
    std::cout.flush();
  }

  void printJsonValue(const web::json::value& json_in, const std::string& indents)
  {
    try
    {
      switch(json_in.type())
      {
        case web::json::value::value_type::Array:
          printJsonArray(json_in.as_array(), indents + INDENT);
          break;
        case web::json::value::value_type::Boolean:
          std::cout << json_in.as_bool() << "\n";
          break;
        case web::json::value::value_type::Null:
          if(INDENT == indents)
          {
            std::cout << INDENT << INDENT << json_in.serialize() << "\n";
          }
          else
          {
            std::cout << json_in.serialize() << "\n";
          }
          break;
        case web::json::value::value_type::Number:
          std::cout << json_in.serialize() << "\n";
          break;
        case web::json::value::value_type::Object:
          if(INDENT != indents)
          {
            std::cout << "\n";
          }
          printJsonObject(json_in.as_object(), indents + INDENT);
          break;
        case web::json::value::value_type::String:
          std::cout << json_in.as_string() << "\n";
          break;
        default:
          throw std::runtime_error("Default case was triggered.");
          break;
      }
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("printJsonValue error: " + static_cast<std::string>(e.what()));
    }
  }

  void printJsonObject(const web::json::object& json_in, const std::string& indents)
  {
    try
    {
      for(auto json_it = json_in.cbegin(); json_it != json_in.cend(); json_it++)
      {
        std::cout << indents << json_it->first << " : ";
        printJsonValue(json_it->second, indents + INDENT);
      }
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("printJsonObject error: " + static_cast<std::string>(e.what()));
    }
  }

  void printJsonArray(const web::json::array& json_in, const std::string& indents)
  {
    try
    {
      for(auto json_it = 0; json_it < static_cast<int32_t>(json_in.size()); json_it++)
      {
        if(0 == json_it)
        {
          std::cout << "\n";
        }
        std::cout << indents << json_it << ": ";
        printJsonValue(json_in.at(json_it), indents);
      }
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("printJsonArray error: " + static_cast<std::string>(e.what()));
    }
  }
}; // camunda

/* json_helpers.cpp */

