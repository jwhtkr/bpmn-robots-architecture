/**
 * @File: resource_pool.cpp
 * @Date: 16 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Holds and organizes information about this behaviors resources.
 **/

/* Local Headers */
#include"behavior_manager/resource_pool.hpp"
#include"behavior_manager/monitored_role.hpp"
#include"behavior_manager/monitored_resource.hpp"

/* Architecture messages */
#include<architecture_msgs/Role.h>

/* C++ Headers */
#include<string>
#include<vector>
#include<mutex>
#include<memory>
#include<stdexcept>
#include<functional>

namespace behavior_manager
{
  ResourcePool::ResourcePool(const ResourcePool& other) noexcept
   : m_data(other.m_data)
  {}

  ResourcePool::ResourcePool(ResourcePool&& other) noexcept
   : m_data(std::move(other.m_data))
  {}

  ResourcePool::~ResourcePool() noexcept
  {
    this->m_data_mutex.lock();
  }

  ResourcePool& ResourcePool::operator=(const ResourcePool& other) noexcept
  {
    this->m_data = std::move(other.m_data);
    return *this;
  }

  ResourcePool& ResourcePool::operator=(ResourcePool&& other) noexcept
  {
    this->m_data = std::move(other.m_data);
    return *this;
  }

  void ResourcePool::addRole(const MonitoredRole& role) noexcept
  {
    this->m_data.emplace_back(role);
  }

  void ResourcePool::addRole(MonitoredRole&& role) noexcept
  {
    this->m_data.emplace_back(std::move(role));
  }

  void ResourcePool::update(const std::vector<architecture_msgs::Role>& new_vals,
                                  std::vector<MonitoredResource>&       added_res,
                                  std::vector<MonitoredResource>&       removed_res)
  {
    added_res.  reserve(10);
    removed_res.reserve(10);

    for(auto m_roles_it = this->m_data.begin(); m_roles_it != this->m_data.end(); m_roles_it++)
    {
      auto new_role = std::find_if(new_vals.cbegin(), new_vals.cend(),
                                   [&m_roles_it](const architecture_msgs::Role& ittr) -> bool
                                   { return ittr.name == m_roles_it->cgetName(); });

      if(new_vals.cend() == new_role) // Not present in list so shut off all resources
      {
        for(auto resource_it = m_roles_it->resources.begin(); resource_it != m_roles_it->resources.end(); resource_it++)
        {
          if(resource_it->second.cinUse())
          {
            removed_res.emplace_back(resource_it->second);
            resource_it->second.getGroupName().clear();
            resource_it->second.in_use = false;
          }
        }
      }
      else // Present in list
      {
        // Find anything that is different in each resource
        for(auto resource_it = new_role->resources.cbegin(); resource_it != new_role->resources.cend(); resource_it++)
        {
          MonitoredResource& m_res = m_roles_it->getResource(resource_it->name);

          if(!m_res.cinUse())
          {
            m_res.in_use = true;
            m_res.getGroupName() = resource_it->group_name;
            added_res.emplace_back(m_res);
          }
          else // Resource already being used
          {
            if(m_res.cgetGroupName() != resource_it->group_name) // Robot changed
            {
              removed_res.emplace_back(m_res);
              m_res.getGroupName() = resource_it->group_name;
              added_res.emplace_back(m_res);
            }
          }
        }
      }
    }
  }

  bool ResourcePool::hasAll() const noexcept
  {
    for(auto data_it = this->m_data.cbegin(); data_it != this->m_data.cend(); data_it++)
    {
      if(!data_it->hasAll())
      {
        return false;
      }
    }
    return true;
  }

  bool ResourcePool::hasRequired() const noexcept
  {
    for(auto data_it = this->m_data.cbegin(); data_it != this->m_data.cend(); data_it++)
    {
      if(!data_it->hasRequired())
      {
        return false;
      }
    }
    return true;
  }

  std::mutex& ResourcePool::getLock() const noexcept
  {
    return this->m_data_mutex;
  }

  std::vector<architecture_msgs::Role> ResourcePool::getAll() const noexcept
  {
    std::vector<architecture_msgs::Role> output;
    output.reserve(this->m_data.size());

    for(auto role_it = this->m_data.cbegin(); role_it != this->m_data.cend(); role_it++)
    {
      output.emplace_back(role_it->m_data);
      output.back().resources.reserve(role_it->resources.size());

      for(auto resource_it = role_it->resources.cbegin(); resource_it != role_it->resources.cend(); resource_it++)
      {
        output.back().resources.emplace_back(resource_it->second.m_data);
      }
    }
    return output;
  }

  std::vector<std::reference_wrapper<MonitoredRole>> ResourcePool::getAllInUse() noexcept
  {
    std::vector<std::reference_wrapper<MonitoredRole>> output;
    output.reserve(this->m_data.size());

    for(auto role_it = this->m_data.begin(); role_it != this->m_data.end(); role_it++)
    {
      if(role_it->inUse())
      {
        output.emplace_back(*role_it);
      }
    }
    return output;
  }

  const MonitoredRole& ResourcePool::cgetRole(const std::string& name) const
  {
    try
    {
      for(auto data_it = this->m_data.cbegin(); data_it != this->m_data.cend(); data_it++)
      {
        if(data_it->cgetName() == name)
        {
          return *data_it;
        }
      }
      throw std::runtime_error("role not found with name: " + name);
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ResourcePool::cgetRole error, " + static_cast<std::string>(e.what()));
    }
  }

  MonitoredRole& ResourcePool::getRole(const std::string& name)
  {
    try
    {
      for(auto data_it = this->m_data.begin(); data_it != this->m_data.end(); data_it++)
      {
        if(name == data_it->getName())
        {
          return *data_it;
        }
      }
      throw std::runtime_error("role not found with name: " + name);
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("ResourcePool::getRole error, " + static_cast<std::string>(e.what()));
    }
  }

}// behavior_manager

/* resource_pool.cpp */
