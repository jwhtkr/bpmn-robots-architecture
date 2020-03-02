/**
 * @File: bpmn_error.hpp
 * @Date: 26 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * A bass class to describe a handle BPMN error request body.
 **/

#ifndef CAMUNDA_ERROR_HANDLING_BPMN_ERROR_HPP
#define CAMUNDA_ERROR_HANDLING_BPMN_ERROR_HPP

/* Local Headers */
#include"camunda_c_api/bpmn_exception.hpp"
#include"camunda_c_api/json.hpp"
#include"camunda_c_api/variables.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>

namespace camunda
{
  class BpmnError : public Json
  {
  public:
    /**
     * @Default Constructor
     **/
    BpmnError() = default;
    /**
     * @Copy Constructor
     **/
    BpmnError(const BpmnError&) = default;
    explicit BpmnError(const web::json::value& other) noexcept;
    /**
     * @Move Constructor
     **/
    BpmnError(BpmnError&&) noexcept = default;
    /**
     * @Constructor
     *
     * @brief
     * At the end of construction object should be ready for use.
     * @workerId: This node's Camunda workerId
     * @errorCode: A code that is specific to the problem that accord
     * @errorMessage: Describes the problem
     * @variables: A list of keys and values to pass up to Camunda
     * @except: A BpmnException holding error data to be sent to Camunda
     **/
    explicit BpmnError(const std::string& workerId,
                       const std::string& errorCode    = std::string("Default bpmn error code"),
                       const std::string& errorMessage = std::string("Default bpmn error message"),
                       const Variables&   variables    = Variables()) noexcept;

    BpmnError(const std::string&   workerId,
              const BpmnException& except) noexcept;

    explicit BpmnError(const web::json::value& workerId,
                       const web::json::value& errorCode    = web::json::value("Default bpmn error code"),
                       const web::json::value& errorMessage = web::json::value("Default bpmn error message"),
                       const Variables&        variables    = Variables()) noexcept;

    BpmnError(const web::json::value& workerId,
              const BpmnException&    except) noexcept;
    /**
     * @Deconstructor
     **/
    ~BpmnError() noexcept override = default;
    /**
     * @Assignment Operator
     **/
    BpmnError& operator=(const BpmnError&)     = default;
    BpmnError& operator=(BpmnError&&) noexcept = default;
    /**
     * @add
     *
     * @brief
     * Adds or replaces the passed in values.
     * @workerId: This node's Camunda workerId
     * @errorCode: A code that is specific to the problem that accord
     * @errorMessage: Describes the problem
     * @variables: A list of keys and values to pass up to Camunda
     * @except: A BpmnException holding error data to be sent to Camunda
     **/
    virtual void addWorkerId     (const std::string&      workerId)     noexcept;
    virtual void addWorkerId     (const web::json::value& workerId)     noexcept;
    virtual void addErrorCode    (const std::string&      errorCode)    noexcept;
    virtual void addErrorCode    (const web::json::value& errorCode)    noexcept;
    virtual void addErrorMessage (const std::string&      errorMessage) noexcept;
    virtual void addErrorMessage (const web::json::value& errorMessage) noexcept;
    virtual void addVariables    (const Variables&        variables)    noexcept;
    virtual void addBpmnException(const BpmnException&    except)       noexcept;
    /**
     * @remove
     *
     * @brief
     * Removes the value if present.
     **/
    virtual void removeWorkerId()     noexcept;
    virtual void removeErrorCode()    noexcept;
    virtual void removeErrorMessage() noexcept;
    virtual void removeVariables()    noexcept;
    /**
     * @get
     *
     * @brief
     * Returns the requested value as long as its present. Throws
     * a runtime exception if its not.
     **/
    virtual const web::json::value& getWorkerId()     const;
    virtual const web::json::value& getErrorCode()    const;
    virtual const web::json::value& getErrorMessage() const;
    virtual const web::json::value& getVariables()    const;
    /**
     * @is_safe
     *
     * @brief
     * Returns true if this object is ready for use.
     **/
    bool is_safe() const noexcept override;
    /**
     * @getError
     *
     * @brief
     * Makes sure it's ready to use then returns the underlying json
     * object. Throws a runtime error if it isn't.
     **/
    virtual const web::json::value& cgetError() const;
    virtual       web::json::value&  getError();
  };
}// camunda

#endif
/* bpmn_error.hpp */

