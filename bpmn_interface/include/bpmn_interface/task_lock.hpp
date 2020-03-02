/**
 * @File: task_lock.hpp
 * @Date: 3 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Smart object used to fetch and lock, and complete a Camunda task.
 **/

#ifndef TASK_LOCK_TASK_LOCK_HPP
#define TASK_LOCK_TASK_LOCK_HPP

/* Camunda API Headers */
#include<camunda_c_api/camunda_c_api.hpp>

/* Rest API Headers */
#include<cpprest/json.h>
#include<cpprest/http_client.h>

/* C++ Headers */
#include<string>
#include<thread>
#include<type_traits>
#include<stdexcept>
#include<cmath>

/* ROS Headers */
#include<ros/ros.h>

/**
 * @DEBUG
 * If debug is set to true additional debugging exceptions will be thrown.
 **/
#define TASK_LOCK_DEBUG false

namespace bpmn
{
  /**
   * @VAR_OBJ
   * A child class of camunda::Variables. Used to to pass variables to this objects
   * complete request.
   *
   * @TOP_OBJ
   * A child class of camunda::Topics. Used to modify what topics this object fetches
   * and locks.
   *
   * @COMP_OBJ
   * A child class of camunda::CompleteRequest. Used to make the final complete task call.
   **/
  template<typename VAR_OBJ = camunda::Variables, typename TOP_OBJ = camunda::Topics, typename COMP_OBJ = camunda::CompleteRequest>
  class TaskLock
  {
  public:
    /**
     * @Default Constructor
     **/
    TaskLock() = delete;
    /**
     * @Copy Constructor
     **/
    TaskLock(const TaskLock&) = delete;
    TaskLock(TaskLock&&)      = delete;
    /**
     * @Constructor
     *
     * @brief
     * During construction this object will fetch and lock a bpmn task. Because of this construction
     * might be a slow operation.
     * @base_uri: The http location on the camunda server
     * @workerId: This programs workerId
     * @topics: An object describing what topics this object will fetch and lock from
     * @complete_request: An object describing this objects complete call. Used in deconstructor
     * @lockExtendFrequency: The frequency at which the task lock will be extended. Note, how long
     *                       the extension is for is also based on this number and is about double
     *                       the minimum for working code
     * @usePriority: Whether or not camunda will use a priority based system to fetch and lock
     * @asyncResponseTimeout: How long camunda has to respond before this object gives up
     **/
    TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>(const std::string& base_uri,
                                         const std::string& workerId,
                                         const TOP_OBJ&     topics,
                                         const COMP_OBJ&    complete_request     = COMP_OBJ(),
                                         const uint32_t     lockExtendFrequency  = 1,
                                         const bool         usePriority          = false,
                                         const uint32_t     asyncResponseTimeout = 180000);

    TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>(const std::string&      base_uri,
                                         const web::json::value& workerId,
                                         const TOP_OBJ&          topics,
                                         const COMP_OBJ&         complete_request     = COMP_OBJ(),
                                         const uint32_t          lockExtendFrequency  = 1,
                                         const web::json::value& usePriority          = web::json::value(false),
                                         const web::json::value& asyncResponseTimeout = web::json::value(180000));
    /**
     * @Deconstructor
     *
     * @brief
     * At deconstruction this object completes the task it locks, sending any variables it has stored.
     * Because of this deconstruction might be a slow operation.
     **/
    ~TaskLock();
    /**
     * @Assignment Operator
     **/
    TaskLock& operator=(const TaskLock&) = delete;
    TaskLock& operator=(TaskLock&&)      = delete;
    /**
     * @startExtending
     * @stopExtending
     *
     * @brief
     * Starts or stops this object from extending the task lock anymore.
     * @warning: This is no guaranty that if this object is told to stop extending then start again
     *           that it will be correctly extending the task lock.
     **/
    void startExtending() noexcept;
    void stopExtending()  noexcept;
    /**
     * @unlock
     *
     * @brief
     * Unlocks the Camunda task. If this method is called then complete will not be called at
     * deconstruction.
     **/
    void unlock();
    /**
     * @complete
     *
     * @brief
     * Completes the task. Note that this is done by default when this object is destructed,
     * but if this method is called it will not happen again at deconstruction.
     **/
    void complete(const COMP_OBJ& complete_request = COMP_OBJ());
    /**
     * @addCompleteRequest
     *
     * @brief
     * Remove the old internal CompleteRequest object and replace it with the
     * one passed in.
     * @complete_request: Object used to make a complete call
     **/
    void addCompleteRequest(const COMP_OBJ& complete_request) noexcept;
    /**
     * @addVariables
     *
     * @brief
     * Adds the passed in Variables object to the internal CompleteRequest object
     * overriding the old Variables object.
     * @variables: Values to be passed to camunda at task completion
     **/
    void addVariables(const VAR_OBJ& variables) noexcept;
    /**
     * @get
     *
     * @brief
     * Returns the asked for value.
     **/
          std::string            getBaseUri()         const noexcept;
    const web::json::value&      getWorkerId()        const noexcept;
    const web::json::value&      getTaskId()          const;
    const camunda::LockRequest&  getLockRequest()     const noexcept;
    const camunda::LockResponse& getLockResponse()    const noexcept;
    const web::json::value&      getResponsVars()     const;
          VAR_OBJ&               getCompleteVars()          noexcept;
    const VAR_OBJ&              cgetCompleteVars()    const noexcept;
    const COMP_OBJ&             cgetCompleteRequest() const noexcept;
          COMP_OBJ&              getCompleteRequest()       noexcept;
          uint8_t                getPriority()        const noexcept;
    const std::string&           getActivityId()      const noexcept;
  private:
    /* Objects for Camunda operations */
    camunda::LockRequest  m_lock_request;
    camunda::LockResponse m_lock_response;
    COMP_OBJ              m_complete_request;
    VAR_OBJ               m_complete_vars;
    /* Talks to Camunda */
    web::http::client::http_client m_client;
    /* Controls thread */
    bool update;
    bool run;
    bool locked;
    /* Refreshes task lock */
    std::thread m_thread;
    /**
     * @threadFunc
     *
     * @brief
     * Used to auto extend the camunda task lock.
     * @lockExtendFrequency: The frequency that this object will extend its task
     *                       lock.
     **/
    void threadFunc(const uint32_t lockExtendFrequency);
  };

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::TaskLock(const std::string& base_uri,
                                                 const std::string& workerId,
                                                 const TOP_OBJ&     topics,
                                                 const COMP_OBJ&    complete_request,
                                                 const uint32_t     lockExtendFrequency,
                                                 const bool         usePriority,
                                                 const uint32_t     asyncResponseTimeout)
   : TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>(base_uri,
                                          web::json::value(workerId),
                                          topics,
                                          complete_request,
                                          lockExtendFrequency,
                                          web::json::value(usePriority),
                                          web::json::value(asyncResponseTimeout))
  {}

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::TaskLock(const std::string&      base_uri,
                                                 const web::json::value& workerId,
                                                 const TOP_OBJ&          topics,
                                                 const COMP_OBJ&         complete_request,
                                                 const uint32_t          lockExtendFrequency,
                                                 const web::json::value& usePriority,
                                                 const web::json::value& asyncResponseTimeout)
   : m_lock_request(workerId, 1, topics, usePriority, asyncResponseTimeout),
     m_complete_request((COMP_OBJ() == complete_request) ? camunda::CompleteRequest(workerId, camunda::Variables()) : complete_request),
     m_complete_vars((COMP_OBJ() != complete_request) ? VAR_OBJ(m_complete_request.getVariables()) : VAR_OBJ()),
     m_client(base_uri + "engine-rest/external-task/"),
     update(false),
     run   (true),
     locked(true),
     m_thread(&TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::threadFunc, this, lockExtendFrequency)
  {
    static_assert(std::is_base_of<camunda::Variables,       VAR_OBJ>::value,
                  "VAR_OBJ has to inherit from the camunda::Variables class.");
    static_assert(std::is_base_of<camunda::Topics,          TOP_OBJ>::value,
                  "TOP_OBJ has to inherit from the camunda::Topics class.");
    static_assert(std::is_base_of<camunda::CompleteRequest, COMP_OBJ>::value,
                  "COMP_OBJ has to inherit from the camunda::CompleteRequest class.");

    try
    {
      this->m_client.request(web::http::methods::POST, "fetchAndLock", this->getLockRequest().cgetLockRequested())
        .then([this](const web::http::http_response& response)
        {
          this->m_lock_response = camunda::LockResponse(response);
        }).wait();
      this->startExtending();
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("TaskLock::Constructor error, " + static_cast<std::string>(e.what()));
    }
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::~TaskLock()
  {
    this->stopExtending();
    this->run = false;
    if(this->locked)
    {
      this->complete();
    }
    this->m_thread.join();
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  void TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::startExtending() noexcept
  {
    this->update = true;
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  void TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::stopExtending() noexcept
  {
    this->update = false;
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  void TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::unlock()
  {
    try
    {
      this->m_client.request(web::http::methods::POST, this->getTaskId().as_string() + "/unlock")
        #if TASK_LOCK_DEBUG
        .then([](const web::http::http_response& response)
        {
          web::json::value res = response.extract_json().get();
          if(web::json::value() != res)
          {
            throw std::runtime_error(res.serialize());
          }
        })
        #endif
          .wait();
      this->locked = false;
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("TaskLock::unlock error, " + static_cast<std::string>(e.what()));
    }
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  void TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::complete(const COMP_OBJ& complete_request)
  {
    try
    {
      if(COMP_OBJ() != complete_request)
      {
        this->addCompleteRequest(complete_request);
      }
      else
      {
        this->getCompleteRequest().addVariables(this->cgetCompleteVars());
      }

      this->m_client.request(web::http::methods::POST,
                             this->getTaskId().as_string() + "/complete",
                             this->cgetCompleteRequest().cgetCompleteRequest())
        #if TASK_LOCK_DEBUG
        .then([](const web::http::http_response& response)
        {
          web::json::value res = response.extract_json().get();
          if(web::json::value() != res)
          {
            throw std::runtime_error(res.serialize());
          }
        })
        #endif
          .wait();
      this->locked = false;
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("TaskLock::complete error, " + static_cast<std::string>(e.what()));
    }
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  void TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::addCompleteRequest(const COMP_OBJ& complete_request) noexcept
  {
    this->getCompleteRequest() = complete_request;
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  void TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::addVariables(const VAR_OBJ& variables) noexcept
  {
    this->getCompleteVars() = variables;
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  std::string TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::getBaseUri() const noexcept
  {
    return this->m_client.base_uri().to_string();
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  const web::json::value& TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::getWorkerId() const noexcept
  {
    return this->getLockRequest().getWorkerId();
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  const web::json::value& TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::getTaskId() const
  {
    return this->getLockResponse().getId(0);
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  const camunda::LockRequest& TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::getLockRequest() const noexcept
  {
    return this->m_lock_request;
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  const camunda::LockResponse& TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::getLockResponse() const noexcept
  {
    return this->m_lock_response;
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  const web::json::value& TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::getResponsVars() const
  {
    return this->getLockResponse().getVariables(0);
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  VAR_OBJ& TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::getCompleteVars() noexcept
  {
    return this->m_complete_vars;
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  const VAR_OBJ& TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::cgetCompleteVars() const noexcept
  {
    return this->m_complete_vars;
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  const COMP_OBJ& TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::cgetCompleteRequest() const noexcept
  {
    return this->m_complete_request;
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  COMP_OBJ& TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::getCompleteRequest() noexcept
  {
    return this->m_complete_request;
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  uint8_t TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::getPriority() const noexcept
  {
    return this->getLockResponse().getPriority(0).as_number().to_uint32();
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  const std::string& TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::getActivityId() const noexcept
  {
    return this->getLockResponse().getActivityId(0).as_string();
  }

  template<typename VAR_OBJ, typename TOP_OBJ, typename COMP_OBJ>
  void TaskLock<VAR_OBJ, TOP_OBJ, COMP_OBJ>::threadFunc(const uint32_t lockExtendFrequency)
  {
    try
    {
      ros::Time::init();
      ros::Rate loop_rate(lockExtendFrequency);
      camunda::ExtendLock lock_extender(this->getWorkerId(), std::ceil((double(1)/double(lockExtendFrequency)) * double(2000)));

      while(ros::ok() && this->run)
      {
        if(this->update)
        {
          this->m_client.request(web::http::methods::POST, this->getTaskId().as_string() + "/extendLock",
                                 lock_extender.getExtendLockRequest()).wait();
        }
        loop_rate.sleep();
      }
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("TaskLock::threadFunc error, " + static_cast<std::string>(e.what()));
    }
  }
}//bpmn

#endif
/* task_lock.hpp */

