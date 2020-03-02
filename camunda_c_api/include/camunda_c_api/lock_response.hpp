/**
 * @File: lock_response.hpp
 * @Date: 20 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * A raper implementation around a json based on the response body
 * of a fetch and lock rest call.
 **/

#ifndef CAMUNDA_OBJECTS_LOCK_RESPONSE_HPP
#define CAMUNDA_OBJECTS_LOCK_RESPONSE_HPP

/* Local Headers */
#include"camunda_c_api/json.hpp"

/* Rest Headers */
#include<cpprest/json.h>
#include<cpprest/http_client.h>

namespace camunda
{
  class LockResponse : public Json
  {
  public:
    /**
     * @Default Constructor
     **/
    LockResponse() = default;
    /**
     * @Copy Constructor
     **/
    LockResponse(const LockResponse&) = default;
    /**
     * @Move Constructor
     **/
    LockResponse(LockResponse&&) noexcept = default;
    /**
     * @Constructor
     *
     * @brief
     * This constructor is meant to be used with an object
     * strait out of a successful fetch and lock. Passed in
     * objects will not be usable afterward.
     **/
    explicit LockResponse(const web::http::http_response&     http_in);
    explicit LockResponse(const pplx::task<web::json::value>& task_in);
    explicit LockResponse(const web::json::value&             json_in) noexcept;
    /**
     * @Destructor
     **/
    ~LockResponse() noexcept override = default;
    /**
     * @Assignment Operator
     **/
    LockResponse& operator=(const LockResponse&)     = default;
    LockResponse& operator=(LockResponse&&) noexcept = default;
    /**
     * @get
     *
     * @brief
     * Returns the values of commonly used fields.
     * @array_index: The location for the value in this object
     **/
    virtual const web::json::value& getId(        const size_t array_index) const;
    virtual const web::json::value& getWorkerId(  const size_t array_index) const;
    virtual const web::json::value& getRetries(   const size_t array_index) const;
    virtual const web::json::value& getVariables( const size_t array_index) const;
    virtual const web::json::value& getPriority(  const size_t array_index) const;
    virtual const web::json::value& getActivityId(const size_t array_index) const;
  };
}; // camunda

#endif
/* lock_response.hpp */

