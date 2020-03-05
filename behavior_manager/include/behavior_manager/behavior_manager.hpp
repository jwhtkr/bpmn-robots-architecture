/**
 * @File: behavior_manager.hpp
 * @Date: 15 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * This is a base class that should be inherited from to define each behavior manager instance.
 **/

#ifndef BEHAVIOR_MANAGER_BEHAVIOR_MANAGER_HPP
#define BEHAVIOR_MANAGER_BEHAVIOR_MANAGER_HPP

/* Local Headers */
#include"behavior_manager/resource_pool.hpp"
#include"behavior_manager/monitored_resource.hpp"

/* Camunda API Headers */
#include<camunda_c_api/camunda_c_api.hpp>

/* BPMN Interface Headers */
#include<bpmn_interface/bpmn_interface.hpp>

/* Architecture Messages */
#include<architecture_msgs/ModifyRobots.h>
#include<architecture_msgs/ResourceRequest.h>
#include<architecture_msgs/BehaviorStatus.h>

#include<architecture_msgs/Robot.h>
#include<architecture_msgs/Role.h>

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>

/* C++ Headers */
#include<string>
#include<thread>
#include<memory>
#include<vector>
#include<type_traits>
#include<stdexcept>
#include<functional>
#include<fstream>
#include<system_error>
#include<mutex>
#include<limits>

namespace behavior_manager
{
  template<typename TASK_LOCK = bpmn::TaskLock<>, typename ERROR = bpmn::ErrorHandler<>>
  class BehaviorManager
  {
  public:
    /**
     * @Default Constructor
     **/
    BehaviorManager() = delete;
    /**
     * @Copy Constructor
     **/
    BehaviorManager(const BehaviorManager&) = delete;
    /**
     * @Move Constructor
     **/
    BehaviorManager(BehaviorManager&&) = delete;
    /**
     * @Constructor
     *
     * @brief
     * Sets up all the services and topics to talk to the resource
     * manager, and makes a fetch and lock request to Camunda.
     * @base_uri: The URI of the camunda server
     * @name: The string that will used to denote this behavior
     * @camunda_topic: The camunda topic that this object will attempt to fetch and lock from
     * @status_topic: ROS topic of a service that provides status updates from this object
     * @get_resources_topic: ROS service topic that this object will use to ask the Resource
     *                       Manager for resources
     * @give_resources_topic: ROS service topic that this object will use to give the Resource
     *                        Manager resources it's not using anymore
     * @update_resources_topic: ROS service topic that the Resource Manager will use to give
     *                          and take resources from this object
     * @modify_robots_topic: ROS service topic that this object will use to tell the Thread
     *                       Pool Manager what resources to spin-up
     * @resources_template: A template of all the resources and roles this behavior wants
     * @variables: The variables you want to get in the fetch and lock. If left blank
     *             Camunda will return all of the global variables
     * @managing_rate: The frequency that this object will attempt to run all of its
     *                 duties at least once
     **/
    BehaviorManager(const std::string&              base_uri,
                    const std::string&              name,
                    const std::string&              camunda_topic,
                    const std::string&              status_topic,
                    const std::string&              get_resources_topic,
                    const std::string&              give_resources_topic,
                    const std::string&              update_resources_topic,
                    const std::string&              modify_robots_topic,
                    const ResourcePool&             resources_template,
                    const std::vector<std::string>& variables     = std::vector<std::string>(),
                    const uint32_t                  managing_rate = 30);
    /**
     * @Deconstructor
     *
     * @brief
     * Releases all resources and completes the Camunda task.
     **/
    virtual ~BehaviorManager();
    /**
     * @Assignment Operators
     **/
    BehaviorManager& operator=(const BehaviorManager&)  = delete;
    BehaviorManager& operator=(      BehaviorManager&&) = delete;
    /**
     * @unpause
     *
     * @brief
     * Tells the thread to unpause.
     **/
    void unpause() noexcept;
    /**
     * @pause
     *
     * @brief
     * Tells the thread to pause. When in pause state resources will still be ask for
     * and given but behavior specific code will not be ran.
     **/
    void pause() noexcept;
    /**
     * @get
     *
     * @brief
     * Returns the ask for values.
     * Note: Name consists of '<base behavior name>/<camunda task instance id>'.
     **/
                  uint8_t                                    getState()            const noexcept;
                  std::string                                getBaseUri()          const noexcept;
            const std::string&                               getName()             const noexcept;
            const std::string&                               getInstanceId()       const noexcept;
                  uint8_t                                    getBehaviorPriority() const noexcept;
    virtual architecture_msgs::BehaviorStatus::Response::Ptr getStatus()           const noexcept;
                  std::mutex&                                getLock()             const noexcept;
  protected:
    /* General class members */
    ros::NodeHandle c_nh;
    /* Holds information about this behaviors resources */
    ResourcePool resources;
    /* Locks the BPMN task */
    TASK_LOCK task_lock;
    /* Used to throw BPMN level errors */
    ERROR error_handler;
    /**
     * @runBehavior
     *
     * @brief
     * This is where all behavior specific implementation goes.
     **/
    virtual void runBehavior();
    /**
     * @waking
     *
     * @brief
     * A function that gets called when this behavior is woken up.
     **/
    virtual void waking();
    /**
     * @sleeping
     *
     * @brief
     * A function that gets called when this behavior is put to sleep.
     **/
    virtual void sleeping();
    /**
     * @resourcesUpdated
     *
     * @brief
     * A function that gets called when this behavior is given new resources, its resources are
     * taken, and the resources change in any way.
     *
     *
     *
     **/
    virtual void resourcesUpdated(const std::vector<MonitoredResource>& bots_getting_deallocated,
                                  const std::vector<MonitoredResource>& bots_getting_allocated);
  private:
    /* Uniquely identifies this object */
    const std::string name;
    const uint8_t     priority;
    /* Reflects the current behavior of the object */
    uint8_t            state;
    mutable std::mutex state_mux;
    /* Keeps this class contained */
    ros::CallbackQueue callback_queue;
    /* Provides status updates when asked for */
    ros::ServiceServer m_status_srv;
    /* Talks to the Resource Manager */
    ros::ServiceClient give_resources_srv;
    ros::ServiceServer update_resources_srv;
    /* Talks to the Master Threadpool */
    ros::ServiceClient modify_robots_srv;
    /* Manages the behavior */
    std::thread thread;
    /**
     * @manage
     *
     * @brief
     * This is the function that gets run in the thread and maintains the object.
     * @managing_rate: The frequency that this object will try to do all of its tasks
     *                 at least once
     **/
    void manage(const uint32_t managing_rate);
    /**
     * @give_resources
     *
     * @brief
     * This service is called by the resource manager when it needs to give or take resources.
     **/
    bool updateResources(architecture_msgs::ResourceRequest::Request&  req,
                         architecture_msgs::ResourceRequest::Response& res);
    /**
     * @get_status
     *
     * @brief
     * Service that other nodes call to get a status update on this behavior.
     **/
    bool getStatus(architecture_msgs::BehaviorStatus::Request&  req,
                   architecture_msgs::BehaviorStatus::Response& res);
    /**
     * @releaseAllRobots
     *
     * @brief
     * This function is called at deconstruction to release all of the robots this
     * behavior was using.
     **/
    void releaseAllRobots();
    /**
     * @callThreadPool
     *
     * @brief
     * Uses the passed in object to ask the ThreadPool Manager to spin up, end, or remap
     * robots.
     **/
    void callThreadPool(architecture_msgs::ModifyRobots& robots);
    /**
     * @askForResources
     *
     * @brief
     * Uses the passed in values to ask the Resource Manager for resources, then
     * returns a pointer to the response data.
     **/
    architecture_msgs::ResourceRequest::Response::Ptr askForResources(std::vector<architecture_msgs::Role>& call);
    /**
     * @giveResources
     *
     * @brief
     * Tells the Resource Manager that it can have the list of resources back.
     **/
    void giveResources(std::vector<architecture_msgs::Role>& call);
    /**
     * @dashToUnderscore
     *
     * @brief
     * Iterates threw string and replaces all '-' with '_'.
     * @str: The string to be transformed
     * Return: The resulting string
     **/
    std::shared_ptr<std::string> dashToUnderscore(const std::string& str);
    /**
     * @update_status
     *
     * @brief
     * Used to change the object status in a thread safe way.
     **/
    void updateState(const uint8_t new_state) noexcept;
  };

  template<typename TASK_LOCK, typename ERROR>
  BehaviorManager<TASK_LOCK, ERROR>::BehaviorManager(const std::string&              base_uri,
                                                     const std::string&              name,
                                                     const std::string&              camunda_topic,
                                                     const std::string&              status_topic,
                                                     const std::string&              get_resources_topic,
                                                     const std::string&              give_resources_topic,
                                                     const std::string&              update_resources_topic,
                                                     const std::string&              modify_robots_topic,
                                                     const ResourcePool&             resources_template,
                                                     const std::vector<std::string>& variables,
                                                     const uint32_t                  managing_rate)
   : resources(resources_template),
     task_lock(base_uri, name, camunda::Topics(camunda_topic, 180000, variables)),
     error_handler(base_uri, task_lock.getTaskId(), task_lock.getWorkerId()),
     name(name + "/" + *dashToUnderscore(getInstanceId())),
     priority(task_lock.getPriority()),
     state(architecture_msgs::BehaviorStatus::Request::STARTING),
     thread(&BehaviorManager::manage, this, managing_rate)
  {
    static_assert(std::is_base_of<bpmn::TaskLock<>, TASK_LOCK>::value,
                  "TASK_LOCK has to be a base class of bpmn::TaskLock");
    static_assert(std::is_base_of<bpmn::ErrorHandler<>, ERROR>::value,
                  "ERROR has to be a base class of bpmn::ErrorHandler");

    architecture_msgs::ResourceRequest resource_manager_call;
    ros::ServiceClient                 get_resources_srv(c_nh.template serviceClient<architecture_msgs::ResourceRequest>(get_resources_topic));

    // Set up subscriptions and publications
    this->c_nh.setCallbackQueue(&this->callback_queue);

    this->m_status_srv         = this->c_nh.advertiseService(getName() + "/" + status_topic,           &BehaviorManager::getStatus,       this);
    this->update_resources_srv = this->c_nh.advertiseService(getName() + "/" + update_resources_topic, &BehaviorManager::updateResources, this);
    this->give_resources_srv   = this->c_nh.template serviceClient<architecture_msgs::ResourceRequest>(give_resources_topic);
    this->modify_robots_srv    = this->c_nh.template serviceClient<architecture_msgs::ModifyRobots>(   modify_robots_topic);

    // Ask for resources
    resource_manager_call.request.behavior_id = this->getName();
    resource_manager_call.request.priority    = this->getBehaviorPriority();
    this->resources.getLock().lock();
    resource_manager_call.request.roles = this->resources.getAll();
    this->resources.getLock().unlock();

    if(!get_resources_srv.waitForExistence(ros::Duration(5)))
    {
      throw std::runtime_error("failed to contact to Resource Manager");
    }

    if(!get_resources_srv.call(resource_manager_call))
    {
      throw std::runtime_error("call to Resource Manager failed");
    }

    if(!resource_manager_call.response.success)
    {
      this->error_handler.throwFailure(web::json::value("Failed to get all of the needed resources"),
                                       web::json::value("This happened because the Resource Manager returned failure when this object asked for it's resources."),
                                       this->task_lock.getLockResponse().getRetries(0),
                                       web::json::value(std::numeric_limits<int32_t>::max()));
    }

    // Setup those robots
    resource_manager_call.request.roles    = resource_manager_call.response.roles;
    resource_manager_call.response.success = false;
    this->updateResources(resource_manager_call.request, resource_manager_call.response);
  }

  template<typename TASK_LOCK, typename ERROR>
  BehaviorManager<TASK_LOCK, ERROR>::~BehaviorManager()
  {
    this->updateState(architecture_msgs::BehaviorStatus::Request::SHUTTING_DOWN);
    this->update_resources_srv.shutdown();
    this->thread.join();
    this->releaseAllRobots();
  }

  template<typename TASK_LOCK, typename ERROR>
  void BehaviorManager<TASK_LOCK, ERROR>::unpause() noexcept
  {
    this->waking();
    this->updateState(architecture_msgs::BehaviorStatus::Request::RUNNING);
  }

  template<typename TASK_LOCK, typename ERROR>
  void BehaviorManager<TASK_LOCK, ERROR>::pause() noexcept
  {
    this->sleeping();
    this->updateState(architecture_msgs::BehaviorStatus::Request::PAUSED);
  }

  template<typename TASK_LOCK, typename ERROR>
  inline uint8_t BehaviorManager<TASK_LOCK, ERROR>::getState() const noexcept
  {
    return this->state;
  }

  template<typename TASK_LOCK, typename ERROR>
  inline std::string BehaviorManager<TASK_LOCK, ERROR>::getBaseUri() const noexcept
  {
    return this->task_lock.getBaseUri();
  }

  template<typename TASK_LOCK, typename ERROR>
  inline const std::string& BehaviorManager<TASK_LOCK, ERROR>::getName() const noexcept
  {
    return this->name;
  }

  template<typename TASK_LOCK, typename ERROR>
  inline const std::string& BehaviorManager<TASK_LOCK, ERROR>::getInstanceId() const noexcept
  {
    return this->task_lock.getTaskId().as_string();
  }

  template<typename TASK_LOCK, typename ERROR>
  inline uint8_t BehaviorManager<TASK_LOCK, ERROR>::getBehaviorPriority() const noexcept
  {
    return this->priority;
  }

  template<typename TASK_LOCK, typename ERROR>
  architecture_msgs::BehaviorStatus::Response::Ptr BehaviorManager<TASK_LOCK, ERROR>::getStatus() const noexcept
  {
    architecture_msgs::BehaviorStatus::Response::Ptr output(new architecture_msgs::BehaviorStatus::Response());

    output->name          = this->getName();
    output->managerStatus = this->getState();

    return output;
  }

  template<typename TASK_LOCK, typename ERROR>
  std::mutex& BehaviorManager<TASK_LOCK, ERROR>::getLock() const noexcept
  {
    return this->state_mux;
  }

  template<typename TASK_LOCK, typename ERROR>
  void BehaviorManager<TASK_LOCK, ERROR>::runBehavior()
  {}

  template<typename TASK_LOCK, typename ERROR>
  void BehaviorManager<TASK_LOCK, ERROR>::waking()
  {}

  template<typename TASK_LOCK, typename ERROR>
  void BehaviorManager<TASK_LOCK, ERROR>::sleeping()
  {}

  template<typename TASK_LOCK, typename ERROR>
  void BehaviorManager<TASK_LOCK, ERROR>::resourcesUpdated(const std::vector<MonitoredResource>& bots_getting_deallocated,
                                                           const std::vector<MonitoredResource>& bots_getting_allocated)
  {}

  template<typename TASK_LOCK, typename ERROR>
  void BehaviorManager<TASK_LOCK, ERROR>::manage(const uint32_t managing_rate)
  {
    try
    {
      ros::Rate loop_rate(managing_rate);

      // Maintain object
      while(this->c_nh.ok() && (architecture_msgs::BehaviorStatusRequest::SHUTTING_DOWN != this->getState()))
      {
        if(this->getLock().try_lock())
        {
          // Interact with other nodes
          this->callback_queue.callAvailable();

          // If this object is in a operating state
          if(architecture_msgs::BehaviorStatusRequest::RUNNING == this->getState())
          {
            try
            {
              this->runBehavior();
            }
            catch(const camunda::BpmnException& bpmn_ex)
            {
              this->error_handler.throwBpmnError(bpmn_ex);
              this->updateState(architecture_msgs::BehaviorStatusRequest::PAUSED);
            }
            catch(const camunda::CamundaException& camunda_ex)
            {
              this->error_handler.throwFailure(camunda_ex);
              this->updateState(architecture_msgs::BehaviorStatusRequest::PAUSED);
            }
          }
          this->getLock().unlock();
        }
        loop_rate.sleep();
      }
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("BehaviorManager::manage error, " + static_cast<std::string>(e.what()));
    }
  }

  template<typename TASK_LOCK, typename ERROR>
  bool BehaviorManager<TASK_LOCK,ERROR>::updateResources(architecture_msgs::ResourceRequest::Request&  req,
                                                         architecture_msgs::ResourceRequest::Response& res)
  {
    try
    {
      std::lock_guard<std::mutex>     resources_lock(this->resources.getLock());
      std::vector<MonitoredResource>  bots_to_allocate;
      std::vector<MonitoredResource>  bots_to_deallocate;
      architecture_msgs::ModifyRobots thread_pool_call;

      this->resources.update(req.roles, bots_to_allocate, bots_to_deallocate);

      thread_pool_call.request.robots.reserve(bots_to_allocate.size() + bots_to_deallocate.size());

      for(auto bot_it = bots_to_allocate.cbegin(); bot_it != bots_to_allocate.cend(); bot_it++)
      {
        thread_pool_call.request.robots.emplace_back();

        thread_pool_call.request.robots.back().name  = bot_it->cgetGroupName();
        thread_pool_call.request.robots.back().start = bot_it->nodes;
      }
      for(auto bot_it = bots_to_deallocate.begin(); bot_it != bots_to_deallocate.end(); bot_it++)
      {
        thread_pool_call.request.robots.emplace_back();

        thread_pool_call.request.robots.back().name = bot_it->getGroupName();
        thread_pool_call.request.robots.back().end  = bot_it->nodes;
      }

      this->callThreadPool(thread_pool_call);

      // Let child class respond
      this->resourcesUpdated(bots_to_allocate, bots_to_deallocate);

      res.success = true;
    }
    catch(const std::exception& ex)
    {
      res.success = false;
      throw std::runtime_error("BehaviorManager::updateResources error, " + static_cast<std::string>(ex.what()));
    }
    return true;
  }

  template<typename TASK_LOCK, typename ERROR>
  bool BehaviorManager<TASK_LOCK, ERROR>::getStatus(architecture_msgs::BehaviorStatus::Request&,
                                                    architecture_msgs::BehaviorStatus::Response& res)
  {
    res = std::move(*this->getStatus());
    return true;
  }

  template<typename TASK_LOCK, typename ERROR>
  void BehaviorManager<TASK_LOCK, ERROR>::releaseAllRobots()
  {
    std::lock_guard<std::mutex> res_lock(this->resources.getLock());
    // Get a list of the robots this object is using
    std::vector<std::reference_wrapper<MonitoredRole>> in_use(this->resources.getAllInUse());

    // Tell Master Thread Pool to shut them down
    architecture_msgs::ModifyRobots      thread_pool_call;
    std::vector<architecture_msgs::Role> roles;

    thread_pool_call.request.robots.reserve(in_use.size()*5);
    roles.reserve(in_use.size());

    for(auto role_it = in_use.cbegin(); role_it != in_use.cend(); role_it++)
    {
      roles.emplace_back(role_it->get().m_data);

      for(auto res_it = role_it->get().resources.cbegin(); res_it != role_it->get().resources.cend(); res_it++)
      {
        if(res_it->second.cinUse())
        {
          thread_pool_call.request.robots.emplace_back();
          thread_pool_call.request.robots.back().name = res_it->second.cgetGroupName();
          thread_pool_call.request.robots.back().end  = res_it->second.nodes;
        }
      }
    }

    this->callThreadPool(thread_pool_call);
    // Tell everyone resources were released
    this->giveResources(roles);
  }

  template<typename TASK_LOCK, typename ERROR>
  void BehaviorManager<TASK_LOCK, ERROR>::callThreadPool(architecture_msgs::ModifyRobots& call)
  {
    if(!this->modify_robots_srv.waitForExistence(ros::Duration(5)))
    {
      throw std::runtime_error("failed to connect to Master Thread Pool");
    }

    if(!this->modify_robots_srv.call(call))
    {
      throw std::runtime_error("call to thread pool master failed");
    }
  }

  template<typename TASK_LOCK, typename ERROR>
  void BehaviorManager<TASK_LOCK, ERROR>::giveResources(std::vector<architecture_msgs::Role>& call)
  {
    architecture_msgs::ResourceRequest roles_to_give;

    roles_to_give.request.behavior_id = this->getName();
    roles_to_give.request.priority    = this->getBehaviorPriority();
    roles_to_give.request.roles       = std::move(call);

    if(!this->give_resources_srv.waitForExistence(ros::Duration(5)))
    {
      throw std::runtime_error("failed to connect to Resource Manager");
    }

    if(!this->give_resources_srv.call(roles_to_give) ||
       !roles_to_give.response.success)
    {
      throw std::runtime_error("call to Resource Manager failed");
    }
  }

  template<typename TASK_LOCK, typename ERROR>
  std::shared_ptr<std::string> BehaviorManager<TASK_LOCK, ERROR>::dashToUnderscore(const std::string& str)
  {
    std::shared_ptr<std::string> output(new std::string(str));

    for(auto char_it = output->begin(); char_it != output->end(); char_it++)
    {
      if('-' == *char_it)
      {
        *char_it = '_';
      }
    }

    return output;
  }

  template<typename TASK_LOCK, typename ERROR>
  void BehaviorManager<TASK_LOCK, ERROR>::updateState(const uint8_t new_state) noexcept
  {
    this->state = new_state;
  }

}// behavior_manager

#endif
/* behavior_manager.hpp */
