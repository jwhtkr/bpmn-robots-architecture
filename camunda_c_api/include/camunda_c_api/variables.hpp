/**
 * @variables.hpp
 * @Date: 4 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Base class for getting case specific information from the LockResponse
 * class as well as filling case specific information into the CompleteTask class.
 **/

#ifndef CAMUNDA_OBJECTS_VARIABLES_HPP
#define CAMUNDA_OBJECTS_VARIABLES_HPP

/* Local Headers */
#include"camunda_c_api/json.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>

namespace camunda
{
  class Variables : public Json
  {
  public:
    /**
     * @Default Constructor
     **/
    Variables() = default;
    /**
     * @Copy Constructors
     **/
    Variables(const Variables&) = default;
    explicit Variables(const web::json::value& other) noexcept;
    /**
     * @Move Constructor
     **/
    Variables(Variables&&) noexcept = default;
    /**
     * @Deconstructor
     **/
    ~Variables() noexcept override = default;
    /**
     * @Assignment Operator
     **/
    Variables& operator=(const Variables&)     = default;
    Variables& operator=(Variables&&) noexcept = default;
    /**
     * @addVariable
     *
     * @brief
     * Adds the variable or updates its values.
     * @key: The variable's name
     * @value: The variable's value
     * @type: What type of object the value is. Can be 'String', 'Number',
     *        'Boolean', or 'Object'
     * @valueInfo: An object with information about the variable if the
     *             variables value is an object
     **/
    virtual void addVariable(const std::string&      key,
                             const web::json::value& value,
                             const web::json::value& type,
                             const web::json::value& valueInfo = web::json::value()) noexcept;

    virtual void addVariable(const std::string&      key,
                             const std::string&      value,
                             const std::string&      type,
                             const web::json::value& valueInfo = web::json::value()) noexcept;
    /**
     * @removeVariable
     *
     * @brief
     * Removes the variable with that key if its here.
     * @key: The variables name
     **/
    virtual void removeVariable(const std::string& key);
    /**
     * @is_safe
     *
     * @brief
     * Should be defined in each inheriting class to make sure the needed
     * values are present.
     **/
    bool is_safe() const noexcept override;
    /**
     * @getVariables
     *
     * @brief
     * Uses is_safe to make sure it's ready then returns the object
     * this class raps.
     **/
    virtual const web::json::value& cgetVariables() const;
    virtual       web::json::value&  getVariables();
    /**
     * @get
     *
     * @brief
     * Returns the value for the corresponding key if it's here.
     * @key: The variables name
     **/
    virtual const web::json::value& getValue    (const std::string& key) const;
    virtual const web::json::value& getType     (const std::string& key) const;
    virtual const web::json::value& getValueInfo(const std::string& key) const;
    virtual const web::json::value& getVariable (const std::string& key) const;
  };
}; // camunda

#endif
/* variables.hpp */

