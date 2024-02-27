/**
 * @File: resource_pool.hpp
 * @Date: 16 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Holds and organizes information about this behaviors resources.
 **/

#ifndef BEHAVIOR_MANAGER_RESOURCE_POOL_HPP
#define BEHAVIOR_MANAGER_RESOURCE_POOL_HPP

/* Local Headers */
#include"behavior_manager/monitored_role.hpp"
#include"behavior_manager/monitored_resource.hpp"

/* Architecture messages */
#include<architecture_msgs/Role.h>

/* C++ Headers */
#include<string>
#include<mutex>
#include<vector>
#include<memory>
#include<functional>

namespace behavior_manager
{
  class ResourcePool
  {
  public:
    /**
     * @Default Constructor
     **/
    ResourcePool() noexcept = default;
    /**
     * @Copy Constructors
     **/
    ResourcePool(const ResourcePool&  other) noexcept;
    ResourcePool(      ResourcePool&& other) noexcept;
    /**
     * @Deconstructor
     **/
    ~ResourcePool() noexcept;
    /**
     * @Assignment Operators
     **/
    ResourcePool& operator=(const ResourcePool&  other) noexcept;
    ResourcePool& operator=(      ResourcePool&& other) noexcept;
    /**
     * @addRole
     *
     * @brief
     * Adds the passed in Role by copy.
     * @role: Role to be added
     **/
    void addRole(const MonitoredRole&  role) noexcept;
    void addRole(      MonitoredRole&& role) noexcept;
    /**
     * @update
     *
     * @brief
     *
     **/
    void update(const std::vector<architecture_msgs::Role>& new_vals,
                      std::vector<MonitoredResource>&       added_res,
                      std::vector<MonitoredResource>&       removed_res);
    /**
     * @hasAll
     *
     * @brief
     * Returns true if all the resources are set to in_use = true.
     **/
    bool hasAll() const noexcept;
    /**
     * @hasRequired
     *
     * @brief
     * Returns true if all of the required resources are set to in_use = true.
     **/
    bool hasRequired() const noexcept;
    /**
     * @getLock
     *
     * @brief
     * Returns a reference to the mutex that is used to keep this object thread safe.
     * Note, only the get Role functions need to be protected.
     **/
    std::mutex& getLock() const noexcept;
    /**
     * @get
     *
     * @brief
     * Returns a pointer to memory populated with the requested information.
     **/
    std::vector<architecture_msgs::Role>               getAll()      const noexcept;
    std::vector<std::reference_wrapper<MonitoredRole>> getAllInUse()       noexcept;
    /**
     * @getRole
     *
     * @brief
     * Returns a reference to the MonitoredRole object that is ask for. Throws if it's not present.
     **/
    const MonitoredRole& cgetRole(const std::string& name)        const;
          MonitoredRole&  getRole(const std::string& name);
  protected:
    std::vector<MonitoredRole> m_data;
    mutable std::mutex         m_data_mutex;
  };
}// behavior_manager

#endif
/* resource_pool.hpp */
