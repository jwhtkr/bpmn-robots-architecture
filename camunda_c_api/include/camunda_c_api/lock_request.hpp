/**
 * @File: lock_request.hpp
 * @Date: 20 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * A raper implementation around a json based on the request body
 * of a fetch and lock rest call.
 **/

#ifndef CAMUNDA_OBJECTS_LOCK_REQUEST_HPP
#define CAMUNDA_OBJECTS_LOCK_REQUEST_HPP

/* Local Headers */
#include"camunda_c_api/json.hpp"
#include"camunda_c_api/topics.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>

namespace camunda
{
  class LockRequest : public Json
  {
  public:
    /**
     * @Default Constructor
     **/
    LockRequest() = default;
    /**
     * @Copy Constructor
     **/
    LockRequest(const LockRequest&) = default;
    explicit LockRequest(const web::json::value& other) noexcept;
    /**
     * @Move Constructor
     **/
    LockRequest(LockRequest&&) noexcept = default;
    /**
     * @Constructor
     *
     * @brief
     * Makes a json file with just the mandatory fields.
     * @workerId: Your worker ID
     * @maxTasks: The max number of tasks you want Cumunda to give you
     * @topics: This will become an array of topics in the json file. The must be at least one pair
     * @usePriority: Weather or not you want Camunda to use priority sorting when getting your tasks
     * @asyncResponseTimeout: Camunda's long pulling timeout
     **/
    LockRequest(const std::string&      workerId,
                const uint32_t          maxTasks,
                const Topics&           topics               = Topics(),
                const bool              usePriority          = false,
                const uint32_t          asyncResponseTimeout = 0) noexcept;
    LockRequest(const web::json::value& workerId,
                const web::json::value& maxTasks,
                const Topics&           topics               = Topics(),
                const web::json::value& usePriority          = web::json::value(false),
                const web::json::value& asyncResponseTimeout = web::json::value(0)) noexcept;
    /**
     * @Deconstructor
     **/
    ~LockRequest() noexcept override = default;
    /**
     * @Assignment Operator
     **/
    LockRequest& operator=(const LockRequest&)     = default;
    LockRequest& operator=(LockRequest&&) noexcept = default;
    /**
     * @add
     *
     * @brief
     * Adds or replaces whatever value and key is given.
     * @workerId: This processes worker id with Camunda
     * @maxTasks: The max number of tasks you want Camunda to fetch and lock for you
     * @usePriority: Whether or not you want Camunda to use priority sorting when fetching for you
     * @asyncResponseTimeout: Camunda's long pulling timeout
     **/
    void addWorkerId            (const std::string&      workerId)             noexcept;
    void addWorkerId            (const web::json::value& workerId)             noexcept;
    void addMaxTasks            (const uint32_t          maxTasks)             noexcept;
    void addMaxTasks            (const web::json::value& maxTasks)             noexcept;
    void addUsePriority         (const bool              usePriority)          noexcept;
    void addUsePriority         (const web::json::value& usePriority)          noexcept;
    void addAsyncResponseTimeout(const uint32_t          asyncResponseTimeout) noexcept;
    void addAsyncResponseTimeout(const web::json::value& asyncResponseTimeout) noexcept;
    /**
     * @addTopics
     *
     * @brief
     * Adds the given topic names and lock durations.
     * @topics: A fully initialized Topics object
     **/
    void addTopics(const Topics& topics);
    /**
     * @if_safe
     *
     * @brief
     * Checks that the json object has all the needed for a fetch and lock.
     * @return: True if it is safe
     **/
    bool is_safe() const noexcept override;
    /**
     * @getLockRequest
     *
     * @brief
     * Makes sure that this object has all the needed fields then
     * returns a reference to the json value.
     **/
    virtual const web::json::value& cgetLockRequested() const;
    virtual       web::json::value&  getLockRequested();
    /**
     * @get
     *
     * @brief
     * Returns the value if its present.
     * @array_index: The location in this object it is stored
     **/
    const web::json::value& getWorkerId()                            const;
    const web::json::value& getMaxTasks()                            const;
    const web::json::value& getUsePriority()                         const;
    const web::json::value& getAsyncResponseTimeout()                const;
    const web::json::value& getTopics()                              const;
    const web::json::value& getTopic(const uint32_t array_index = 0) const;
  };
}; // camunda

#endif
/* lock_request.hpp */

