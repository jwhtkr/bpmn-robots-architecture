/**
 * @File: process_variable.hpp
 * @Date: 1 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * This class can be used to maintain and access Camunda process variables.
 **/

#ifndef PROCESS_VARIABLES_PROCESS_VARIABLE_HPP
#define PROCESS_VARIABLES_PROCESS_VARIABLE_HPP

/* Local Headers */
#include"camunda_c_api/camunda_c_api.hpp"

/* Rest Headers */
#include<cpprest/http_client.h>
#include<cpprest/json.h>

/* C++ Headers */
#include<string>
#include<stdexcept>
#include<type_traits>

namespace camunda
{
  /**
   * @T
   *
   * @brief
   * What type of variable this object is going to represent.
   * @Possible types: std::string, float, double, int, long, bool
   **/
  template<typename T>
  class ProcessVariable
  {
  public:
    /**
     * @Default Constructor
     **/
    ProcessVariable<T>() = delete;
    /**
     * @Copy Constructor
     **/
    ProcessVariable<T>(const ProcessVariable&) = delete;
    /**
     * @Constructor
     *
     * @brief
     * Links this object to a camunda variable.
     * @base_uri: URI of the camunda server
     * @process_id: The instance id of the process that holds the variable
     *              this object will link to
     * @variable_name: The name of the variable this object will link to
     **/
    ProcessVariable<T>(const std::string& base_uri,
                       const std::string& process_id,
                       const std::string& variable_name);
    /**
     * @Deconstructor
     **/
    ~ProcessVariable<T>() noexcept = default;
    /**
     * @get
     *
     * @brief
     * Gets the value of the variable from the Camunda process instance
     * then returns it.
     **/
    T get();
    /**
     * @update
     *
     * @brief
     * Updates the Camunda process instance variable or make it if needed.
     **/
    void update(const T var);
    /**
     * @remove
     *
     * @brief
     * Removes the variable from a Camunda process instance.
     **/
    void remove();
    /**
     * @get""
     *
     * @brief
     * Returns the requested value.
     **/
          std::string  getFullUri() const noexcept;
    const std::string& getName()    const noexcept;
  private:
    /* Identifies object */
    const std::string m_name;
    /* For communication with Camunda */
    web::http::client::http_client m_client;
  };

  template<typename T>
  ProcessVariable<T>::ProcessVariable(const std::string& base_uri,
                                      const std::string& process_id,
                                      const std::string& variable_name)
   : m_name(variable_name),
     m_client(base_uri + "engine-rest/process-instance/" + process_id + "/variables/" + variable_name)
  {
    static_assert(!std::is_base_of<T, int64_t>::value     ||
                  !std::is_base_of<T, double>::value      ||
                  !std::is_base_of<T, std::string>::value ||
                  !std::is_base_of<T, bool>::value,
                  "ProcessVariable object can only be templated to int64_t, int32_t, float, double, std::string, or bool");
  }

  template<>
  inline int64_t ProcessVariable<int64_t>::get()
  {
    int64_t val = 0;
    try
    {
      this->m_client.request(web::http::methods::GET)
        .then([&val](const web::http::http_response& response)
        {
          web::json::value from_request = response.extract_json().get();

          if(web::json::value("Integer") != from_request.at("type"))
          {
            throw std::runtime_error(from_request.serialize());
          }
          val = from_request.at("value").as_number().to_int64();
        }).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::get error, " + static_cast<std::string>(e.what()));
    }
    return val;
  }

  template<>
  inline int32_t ProcessVariable<int32_t>::get()
  {
    int32_t val = 0;
    try
    {
      this->m_client.request(web::http::methods::GET)
        .then([&val](const web::http::http_response& response)
        {
          web::json::value from_request = response.extract_json().get();

          if(web::json::value("Integer") != from_request.at("type"))
          {
            throw std::runtime_error(from_request.serialize());
          }
          val = from_request.at("value").as_number().to_int32();
        }).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::get error, " + static_cast<std::string>(e.what()));
    }
    return val;

  }

  template<>
  inline double ProcessVariable<double>::get()
  {
    double val = 0;
    try
    {
      this->m_client.request(web::http::methods::GET)
        .then([&val](const web::http::http_response& response)
        {
          web::json::value from_request = response.extract_json().get();

          if(web::json::value("Double") != from_request.at("type"))
          {
            throw std::runtime_error(from_request.serialize());
          }
          val = from_request.at("value").as_number().to_double();
        }).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::get error, " + static_cast<std::string>(e.what()));
    }
    return val;
  }

  template<>
  inline float ProcessVariable<float>::get()
  {
    float val = 0;
    try
    {
      this->m_client.request(web::http::methods::GET)
        .then([&val](const web::http::http_response& response)
        {
          web::json::value from_request = response.extract_json().get();

          if(web::json::value("Double") != from_request.at("type"))
          {
            throw std::runtime_error(from_request.serialize());
          }
          val = from_request.at("value").as_number().to_double();
        }).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::get error, " + static_cast<std::string>(e.what()));
    }
    return val;
  }


  template<>
  inline std::string ProcessVariable<std::string>::get()
  {
    std::string val = std::string();
    try
    {
      this->m_client.request(web::http::methods::GET)
        .then([&val](const web::http::http_response& response)
        {
          web::json::value from_request = response.extract_json().get();

          if(web::json::value("String") != from_request.at("type"))
          {
            throw std::runtime_error(from_request.serialize());
          }
          val = from_request.at("value").as_string();
        }).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::get error, " + static_cast<std::string>(e.what()));
    }
    return val;
  }

  template<>
  inline bool ProcessVariable<bool>::get()
  {
    bool val = false;
    try
    {
      this->m_client.request(web::http::methods::GET)
        .then([&val](const web::http::http_response& response)
        {
          web::json::value from_request = response.extract_json().get();

          if(web::json::value("Boolean") != from_request.at("type"))
          {
            throw std::runtime_error(from_request.serialize());
          }
          val = from_request.at("value").as_bool();
        }).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::get error, " + static_cast<std::string>(e.what()));
    }
    return val;
  }

  template<>
  inline void ProcessVariable<int64_t>::update(const int64_t var)
  {
    try
    {
      Json var_obj;

      var_obj.add("value", web::json::value(var));
      var_obj.add("type", web::json::value("Integer"));

      this->m_client.request(web::http::methods::PUT, std::string(), var_obj.get()).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::update error, " + static_cast<std::string>(e.what()));
    }
    return;
  }

  template<>
  inline void ProcessVariable<int32_t>::update(const int32_t var)
  {
    try
    {
      Json var_obj;

      var_obj.add("value", web::json::value(var));
      var_obj.add("type", web::json::value("Integer"));

      this->m_client.request(web::http::methods::PUT, std::string(), var_obj.get()).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::update error, " + static_cast<std::string>(e.what()));
    }
    return;
  }

  template<>
  inline void ProcessVariable<double>::update(const double var)
  {
    try
    {
      Json var_obj;

      var_obj.add("value", web::json::value(var));
      var_obj.add("type", web::json::value("Double"));

      this->m_client.request(web::http::methods::PUT, std::string(), var_obj.get()).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::update error, " + static_cast<std::string>(e.what()));
    }
    return;
  }

  template<>
  inline void ProcessVariable<float>::update(const float var)
  {
    try
    {
      Json var_obj;

      var_obj.add("value", web::json::value(var));
      var_obj.add("type", web::json::value("Double"));

      this->m_client.request(web::http::methods::PUT, std::string(), var_obj.get()).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::update error, " + static_cast<std::string>(e.what()));
    }
    return;
  }

  template<>
  inline void ProcessVariable<std::string>::update(const std::string var)
  {
    try
    {
      Json var_obj;

      var_obj.add("value", web::json::value(var));
      var_obj.add("type", web::json::value("String"));

      this->m_client.request(web::http::methods::PUT, std::string(), var_obj.get()).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::update error, " + static_cast<std::string>(e.what()));
    }
    return;
  }

  template<>
  inline void ProcessVariable<bool>::update(const bool var)
  {
    try
    {
      Json var_obj;

      var_obj.add("value", web::json::value(var));
      var_obj.add("type", web::json::value("Boolean"));

      this->m_client.request(web::http::methods::PUT, std::string(), var_obj.get()).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::update error, " + static_cast<std::string>(e.what()));
    }
    return;
  }

  template<typename T>
  void ProcessVariable<T>::remove()
  {
    try
    {
      this->m_client.request(web::http::methods::DEL).wait();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ProcessVariable::remove error, " + static_cast<std::string>(e.what()));
    }
    return;
  }

  template<typename T>
  std::string ProcessVariable<T>::getFullUri() const noexcept
  {
    return this->m_client.base_uri().to_string();
  }

  template<typename T>
  const std::string& ProcessVariable<T>::getName() const noexcept
  {
    return this->m_name;
  }
}// camunda

#endif
/* process_variable.hpp */

