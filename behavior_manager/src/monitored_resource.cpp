/**
 * @File: monitored_resource.cpp
 * @Date: 16 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * This object raps a architecture_msgs::Resource object and adds more information
 * holding capabilities.
 **/

/* Local Headers */
#include"behavior_manager/monitored_resource.hpp"

/* Architecture Headers */
#include<architecture_msgs/Resource.h>

/* C++ Headers */
#include<string>
#include<vector>
#include<utility>

namespace behavior_manager
{
  MonitoredResource::MonitoredResource() noexcept
   : in_use(false)
  {}

  MonitoredResource::MonitoredResource(const MonitoredResource& other) noexcept
   : in_use(other.in_use),
     nodes(other.nodes),
     m_data(other.m_data)
  {}

  MonitoredResource::MonitoredResource(const architecture_msgs::Resource& resource,
                                       const std::vector<std::string>&    nodes) noexcept
   : in_use(false),
     nodes(nodes),
     m_data(resource)
  {}

  MonitoredResource::MonitoredResource(architecture_msgs::Resource&& resource,
                                       std::vector<std::string>&&    nodes) noexcept
   : in_use(false),
     nodes(std::move(nodes)),
     m_data(std::move(resource))
  {}

  MonitoredResource::MonitoredResource(const std::string&              name,
                                       const std::string&              category,
                                       const std::string&              type,
                                       const bool                      required,
                                       const uint8_t                   priority,
                                       const std::vector<std::string>& nodes) noexcept
   : in_use(false),
     nodes(nodes)
  {
    this->m_data.name     = name;
    this->m_data.category = category;
    this->m_data.type     = type;
    this->m_data.required = required;
    this->m_data.priority = priority;
  }

  MonitoredResource::MonitoredResource(      std::string&&              name,
                                             std::string&&              category,
                                             std::string&&              type,
                                       const bool                       required,
                                       const uint8_t                    priority,
                                             std::vector<std::string>&& nodes) noexcept
   : in_use(false),
     nodes(std::move(nodes))
  {
    this->m_data.name     = std::move(name);
    this->m_data.category = std::move(category);
    this->m_data.type     = std::move(type);
    this->m_data.required = required;
    this->m_data.priority = priority;
  }

  MonitoredResource& MonitoredResource::operator=(const MonitoredResource& other) noexcept
  {
    this->in_use = other.in_use;
    this->nodes  = other.nodes;
    this->m_data = other.m_data;

    return *this;
  }

  bool MonitoredResource::operator==(const MonitoredResource& other) const noexcept
  {
    return (this->cgetName() == other.cgetName()) && (this->cgetGroupName() == other.cgetGroupName());
  }

  bool MonitoredResource::operator!=(const MonitoredResource& other) const noexcept
  {
    return !(*this == other);
  }

  bool MonitoredResource::operator==(const architecture_msgs::Resource& other) const noexcept
  {
    return (this->cgetGroupName() == other.name) && (this->cgetGroupName() == other.group_name);
  }

  bool MonitoredResource::operator!=(const architecture_msgs::Resource& other) const noexcept
  {
    return !(*this == other);
  }

  bool MonitoredResource::cinUse() const noexcept
  {
    return this->in_use;
  }

  std::string& MonitoredResource::getGroupName() noexcept
  {
    return this->m_data.group_name;
  }

  const std::string& MonitoredResource::cgetGroupName() const noexcept
  {
    return this->m_data.group_name;
  }

  std::string& MonitoredResource::getName() noexcept
  {
    return this->m_data.name;
  }

  const std::string& MonitoredResource::cgetName() const noexcept
  {
    return this->m_data.name;
  }

  std::string& MonitoredResource::getCategory() noexcept
  {
    return this->m_data.category;
  }

  const std::string& MonitoredResource::cgetCategory() const noexcept
  {
    return this->m_data.category;
  }

  std::string& MonitoredResource::getType() noexcept
  {
    return this->m_data.type;
  }

  const std::string& MonitoredResource::cgetType() const noexcept
  {
    return this->m_data.type;
  }

  uint8_t& MonitoredResource::getRequired() noexcept
  {
    return this->m_data.required;
  }

  const uint8_t& MonitoredResource::cgetRequired() const noexcept
  {
    return this->m_data.required;
  }

  uint8_t& MonitoredResource::getPriority() noexcept
  {
    return this->m_data.priority;
  }

  const uint8_t& MonitoredResource::cgetPriority() const noexcept
  {
    return this->m_data.priority;
  }

}// behavior_manager

/* monitored_resource.cpp */
