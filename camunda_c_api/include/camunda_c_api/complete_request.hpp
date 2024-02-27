/**
 * @File: complete_request.hpp
 * @Date: 4 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * A class to make completing tasks easier.
 **/

#ifndef CAMUNDA_OBJECTS_COMPLETE_REQUEST_HPP
#define CAMUNDA_OBJECTS_COMPLETE_REQUEST_HPP

/* Local Headers */
#include"camunda_c_api/json.hpp"
#include"camunda_c_api/variables.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>

namespace camunda
{
  class CompleteRequest : public Json
  {
  public:
    /**
     * @Default Constructor
     **/
    CompleteRequest() = default;
    /**
     * @Copy Constructor
     **/
    CompleteRequest(const CompleteRequest&) = default;
    explicit CompleteRequest(const web::json::value& other) noexcept;
    /**
     * @Move Constructor
     **/
    CompleteRequest(CompleteRequest&&) noexcept = default;
    /**
     * @Constructor
     *
     * @brief
     * Fills the object with the given values.
     * @workerId: Camunda's worker id for this process
     * @variables: A Variables object with values that you want to pass up to Camunda
     **/
    explicit CompleteRequest(const std::string&      workerId, const Variables& variables = Variables()) noexcept;
             CompleteRequest(const web::json::value& workerId, const Variables& variables)               noexcept;
    /**
     * @Deconstructor
     **/
    ~CompleteRequest() noexcept override = default;
    /**
     * @Assignment Operator
     **/
    CompleteRequest& operator=(const CompleteRequest&) = default;
    CompleteRequest& operator=(CompleteRequest&&) noexcept = default;
    /**
     * @add
     *
     * @brief
     * Adds or updates value. For Variables it will step through
     * every item and add or update it.
     * @workerId: Camunda's worker id for this process
     * @variables: A Variables object with values that you want to pass up to Camunda
     * @key: The name of the variable your adding
     * @value: The value of the variable your adding
     * @type: What type of variable your adding. Can be 'String', 'Number',
     *        'Boolean', or 'Object'
     * @valueInfo: If the variable type is Object then this is an object that
     *             gives more information about the variable
     **/
    void addWorkerId (const std::string&      workerId)  noexcept;
    void addWorkerId (const web::json::value& workerId)  noexcept;
    void addVariables(const Variables&        variables) noexcept;
    void addVariable (const std::string&      key,
                      const web::json::value& value,
                      const web::json::value& type,
                      const web::json::value& valueInfo = web::json::value()) noexcept;
    /**
     * @remove
     *
     * @brief
     * Removes element if it is present.
     * @key: The variable name
     **/
    void removeWorkerId()  noexcept;
    void removeVariables() noexcept;
    void removeVariable(const std::string& key);
    /**
     * @get
     *
     * @brief
     * Returns the value, throws a runtime error if it's not present.
     * @key: The variable name
     **/
    const web::json::value& getWorkerId ()                       const;
    const web::json::value& getVariables()                       const;
    const web::json::value& getVariable (const std::string& key) const;
    const web::json::value& getValue    (const std::string& key) const;
    const web::json::value& getValueType(const std::string& key) const;
    const web::json::value& getValueInfo(const std::string& key) const;
    /**
     * @is_safe
     *
     * @brief
     * Makes sure all the needed parts are present.
     **/
    bool is_safe() const noexcept override;
    /**
     * @getCompletRequest
     *
     * @brief
     * Checks the object this one holds and returns it if it's safe to use and
     * throws a runtime error if it's not.
     **/
    virtual       web::json::value&  getCompleteRequest();
    virtual const web::json::value& cgetCompleteRequest() const;
  };
}; // camunda

#endif
/* complete_request.hpp */

