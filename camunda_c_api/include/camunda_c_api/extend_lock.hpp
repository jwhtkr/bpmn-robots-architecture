/**
 * @File: extend_lock.hpp
 * @Date: 26 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * A class to describe a extend lock request.
 **/

#ifndef CAMUNDA_OBJECTS_EXTEND_LOCK_HPP
#define CAMUNDA_OBJECTS_EXTEND_LOCK_HPP

/* Local Headers */
#include"camunda_c_api/json.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>

namespace camunda
{
  class ExtendLock : public Json
  {
  public:
    /**
     * @Default Constructor
     **/
    ExtendLock() = default;
    /**
     * @Copy Constructor
     **/
    ExtendLock(const ExtendLock&) = default;
    explicit ExtendLock(const web::json::value& other) noexcept;
    /**
     * @Move Constructor
     **/
    ExtendLock(ExtendLock&&) noexcept = default;
    /**
     * @Constructor
     *
     * @brief
     * After construction object should be ready to use.
     * @workerId: The Camunda workerId of this node
     * @newDuration: How long the lock will have after Camunda presses this request
     **/
    ExtendLock(const std::string&      workerId, const uint32_t          newDuration) noexcept;
    ExtendLock(const web::json::value& workerId, const web::json::value& newDuration) noexcept;
    /**
     * @Deconstructor
     **/
    ~ExtendLock() noexcept override = default;
    /**
     * @Assignment Operator
     **/
    ExtendLock& operator=(const ExtendLock&)     = default;
    ExtendLock& operator=(ExtendLock&&) noexcept = default;
    /**
     * @add
     *
     * @brief
     * Adds or replaces the passed in values.
     * @workerId: The Camunda workerId of this node
     * @newDuration: How long the lock will have after Camunda presses this request
     **/
    void addWorkerId(   const std::string&      workerId)    noexcept;
    void addWorkerId(   const web::json::value& workerId)    noexcept;
    void addNewDuration(const uint32_t          newDuration) noexcept;
    void addNewDuration(const web::json::value& newDuration) noexcept;
    /**
     * @get
     *
     * @brief
     * Returns the value if it's present, throws a runtime error if it's not.
     **/
    const web::json::value& getWorkerId()    const;
    const web::json::value& getNewDuration() const;
    /**
     * @is_safe
     *
     * @brief
     * Returns true if this object is ready to use.
     **/
    bool is_safe() const noexcept final;
    /**
     * @getExtendLockRequest
     *
     * @brief
     * Checks that this object is ready to use and returns its json if it is,
     * throws a runtime error if it isn't.
     **/
    const web::json::value& cgetExtendLockRequest() const;
          web::json::value&  getExtendLockRequest();
  };
}// camunda

#endif
/* extend_lock.hpp */

