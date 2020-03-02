/**
 * @File: monitored_role.cpp
 * @Date: 16 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * A rapper class to hold roles.
 **/

/* Local Headers */
#include"behavior_manager/monitored_role.hpp"
#include"behavior_manager/monitored_resource.hpp"

/* Architecture Messages */
#include<architecture_msgs/Role.h>

/* C++ Headers */
#include<string>
#include<mutex>
#include<vector>
#include<memory>
#include<stdexcept>
#include<utility>

namespace behavior_manager
{
  MonitoredRole::MonitoredRole(const MonitoredRole& other) noexcept
   : resources(other.resources),
     m_data(other.m_data)
  {}

  MonitoredRole::MonitoredRole(MonitoredRole&& other) noexcept
   : resources(std::move(other.resources)),
     m_data(std::move(other.m_data))
  {}

  MonitoredRole::MonitoredRole(const std::string&                    name,
                               const bool                            required,
                               const uint8_t                         priority,
                               const std::vector<MonitoredResource>& resources) noexcept
   : MonitoredRole()
  {
    this->m_data.name     = name;
    this->m_data.required = required;
    this->m_data.priority = priority;
    std::for_each(resources.cbegin(), resources.cend(),
                  [&](const MonitoredResource& ittr)
                  {
                    this->resources.emplace(ittr.cgetName(), ittr);
                  });
  }

  MonitoredRole::MonitoredRole(      std::string&&                    name,
                               const bool                             required,
                               const uint8_t                          priority,
                                     std::vector<MonitoredResource>&& resources) noexcept
   : MonitoredRole()
  {
    this->m_data.name     = std::move(name);
    this->m_data.required = required;
    this->m_data.priority = priority;
    std::for_each(resources.begin(), resources.end(),
                  [&](MonitoredResource& ittr)
                  {
                    this->resources.emplace(ittr.cgetName(), std::move(ittr));
                  });
  }

  MonitoredRole& MonitoredRole::operator=(const MonitoredRole& other) noexcept
  {
    this->resources = other.resources;
    this->m_data = other.m_data;

    return *this;
  }

  MonitoredRole& MonitoredRole::operator=(MonitoredRole&& other) noexcept
  {
    this->resources = std::move(other.resources);
    this->m_data    = std::move(other.m_data);

    return *this;
  }

  bool MonitoredRole::operator==(const MonitoredRole& other) const noexcept
  {
    if(this->cgetName()       != other.cgetName() ||
       this->resources.size() != other.resources.size())
    {
      return false;
    }

    for(auto resource_it = this->resources.cbegin(); resource_it != this->resources.cend(); resource_it++)
    {
      if(other.resources.cend() == other.resources.find(resource_it->first))
      {
        return false;
      }
    }

    return true;
  }

  bool MonitoredRole::operator!=(const MonitoredRole& other) const noexcept
  {
    return !(*this == other);
  }

  bool MonitoredRole::operator==(const architecture_msgs::Role& other) const noexcept
  {
    if(this->cgetName()       != other.name ||
       this->resources.size() != other.resources.size())
    {
      return false;
    }

    for(size_t resource_it = 0; resource_it < other.resources.size(); resource_it++)
    {
      if(this->resources.cend() == this->resources.find(other.resources.at(resource_it).name))
      {
        return false;
      }
    }

    return true;
  }

  bool MonitoredRole::operator!=(const architecture_msgs::Role& other) const noexcept
  {
    return (this->cgetName() != other.name);
  }

  bool MonitoredRole::hasAll() const noexcept
  {
    for(auto resource_it = this->resources.cbegin(); resource_it != this->resources.cend(); resource_it++)
    {
      if(!resource_it->second.cinUse())
      {
        return false;
      }
    }
    return true;
  }

  bool MonitoredRole::hasRequired() const noexcept
  {
    if(!this->cgetRequired())
    {
      return true;
    }

    for(auto resource_it = this->resources.cbegin(); resource_it != this->resources.cend(); resource_it++)
    {
      if(resource_it->second.cgetRequired() && !resource_it->second.cinUse())
      {
        return false;
      }
    }
    return true;
  }

  bool MonitoredRole::inUse() const noexcept
  {
    for(auto resource_it = this->resources.cbegin(); resource_it != this->resources.cend(); resource_it++)
    {
      if(resource_it->second.cinUse())
      {
        return true;
      }
    }
    return false;
  }

  const MonitoredResource& MonitoredRole::cgetResource(const std::string& name) const
  {
    auto output = this->resources.find(name);
    if(output == this->resources.cend())
    {
      throw std::runtime_error("MonitoredRole::cgetResource error, Resource " + name + " not found");
    }
    return output->second;
  }

  MonitoredResource& MonitoredRole::getResource(const std::string& name)
  {
    auto output = this->resources.find(name);
    if(output == this->resources.end())
    {
      throw std::runtime_error("MonitoredRole::cgetResource error, Resource " + name + " not found");
    }
    return output->second;
  }

  const std::vector<std::string>& MonitoredRole::cgetNodes(const std::string& name) const
  {
    try
    {
      return this->cgetResource(name).nodes;
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("MonitoredRole::cgetNodes error, " + static_cast<std::string>(e.what()));
    }
  }

  std::vector<std::string>& MonitoredRole::getNodes(const std::string& name)
  {
    try
    {
      return this->getResource(name).nodes;
    }
    catch(const std::exception& e)
    {
      throw std::runtime_error("MonitoredRole::getNodes error, " + static_cast<std::string>(e.what()));
    }
  }

  std::string& MonitoredRole::getName() noexcept
  {
    return this->m_data.name;
  }

  const std::string& MonitoredRole::cgetName() const noexcept
  {
    return this->m_data.name;
  }

  uint8_t& MonitoredRole::getRequired() noexcept
  {
    return this->m_data.required;
  }

  const uint8_t& MonitoredRole::cgetRequired() const noexcept
  {
    return this->m_data.required;
  }

  uint8_t& MonitoredRole::getPriority() noexcept
  {
    return this->m_data.priority;
  }

  const uint8_t& MonitoredRole::cgetPriority() const noexcept
  {
    return this->m_data.priority;
  }

}// behavior_manager

/* monitored_role.cpp */
