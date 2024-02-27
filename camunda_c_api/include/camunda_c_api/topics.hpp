/**
 * @File: topics.hpp
 * @Date: 10 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Base class for getting specific information from a fetch and lock parameters.
 **/

#ifndef CAMUNDA_OBJECTS_TOPICS_HPP
#define CAMUNDA_OBJECTS_TOPICS_HPP

/* Local Headers */
#include"camunda_c_api/json.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>
#include<vector>

namespace camunda
{
  class Topics : public Json
  {
    public:
    /**
     * @Default Constructor
     **/
    Topics() = default;
    /**
     * @Copy Constructors
     **/
    Topics(const Topics&) = default;
    explicit Topics(const web::json::value& other) noexcept;
    /**
     * @Move Constructor
     **/
    Topics(Topics&&) noexcept = default;
    /**
     * @Constructor
     *
     * @brief
     * The result of this constructor has to be ready for use.
     * @topicName: The Camunda topic you want to add
     * @lockDuration: How long the lock will last in milliseconds
     * @variables: The Camunda variables you want to get from your fetch and lock
     *             in the form of a list of the variable names.
     **/
    Topics(const std::string&              topicName,
           const uint32_t                  lockDuration,
           const std::vector<std::string>& variables = std::vector<std::string>()) noexcept;
    Topics(const web::json::value&         topicName,
           const web::json::value&         lockDuration,
           const web::json::value&         variables = web::json::value()) noexcept;
    /**
     * @Deconstructor
     **/
    ~Topics() noexcept override = default;
    /**
     * @Assignment Operator
     **/
    Topics& operator=(const Topics&)     = default;
    Topics& operator=(Topics&&) noexcept = default;
    /**
     * @add
     *
     * @brief
     * Adds or updates the passed in values.
     * @topicName: The Camunda topic you want to add
     * @lockDuration: How long the lock will last in milliseconds
     * @variables: The Camunda variables you want to get from your fetch and lock
     *             in the form of a list of the variable names.
     * @array_index: Where in this object the value is or will be stored
     **/
    virtual void addTopicName   (const std::string&              topicName,    const uint32_t array_index);
    virtual void addTopicName   (const web::json::value&         topicName,    const uint32_t array_index);
    virtual void addLockDuration(const uint32_t                  lockDuration, const uint32_t array_index);
    virtual void addLockDuration(const web::json::value&         lockDuration, const uint32_t array_index);
    virtual void addVariables   (const std::vector<std::string>& variables,    const uint32_t array_index);
    virtual void addVariables   (const web::json::value&         variables,    const uint32_t array_index);
    virtual void addVariable    (const std::string&              variable,     const uint32_t array_index);
    virtual void addVariable    (const web::json::value&         variable,     const uint32_t array_index);

    virtual void addTopic(const std::string&              topicName,
                          const uint32_t                  lockDuration,
                          const std::vector<std::string>& variables = std::vector<std::string>()) noexcept;

    virtual void addTopic(const web::json::value& topicName,
                          const web::json::value& lockDuration,
                          const web::json::value& variables = web::json::value());
    /**
     * @remove
     *
     * @brief
     * Removes value.
     * @array_index: Where in this object the value is stored
     **/
    virtual void removeTopic(       const uint32_t array_index);
    virtual void removeTopicName(   const uint32_t array_index);
    virtual void removeLockDuration(const uint32_t array_index);
    virtual void removeVariables(   const uint32_t array_index);
    /**
     * @is_safe
     *
     * @brief
     * Makes sure its ready to be used.
     **/
    bool is_safe() const noexcept override;
    /**
     * @getTopics
     *
     * @brief
     * Makes sure its ready for use with the is_safe function
     * the returns the topics object.
     **/
    virtual const web::json::value& cgetTopics() const;
    virtual       web::json::value&  getTopics();
    /**
     * @get
     *
     * @brief
     * Returns the value.
     * @array_index: Where in this object the value is stored
     **/
    virtual const web::json::value& getTopicName(   const uint32_t array_index) const;
    virtual const web::json::value& getLockDuration(const uint32_t array_index) const;
    virtual const web::json::value& getVariables(   const uint32_t array_index) const;
    virtual const web::json::value& getTopic(       const uint32_t array_index) const;
  };
} // camunda

#endif
/* topics.hpp */

