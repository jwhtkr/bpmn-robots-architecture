/**
 * @File: throw_signal.hpp
 * @Date: 27 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * A base class to describe a Throw Signal request.
 **/

#ifndef CAMUNDA_OBJECTS_THROW_SIGNAL_HPP
#define CAMUNDA_OBJECTS_THROW_SIGNAL_HPP

/* Local Headers */
#include"camunda_c_api/json.hpp"
#include"camunda_c_api/variables.hpp"

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>

namespace camunda
{
  class ThrowSignal : public Json
  {
  public:
    /**
     * @Default Constructor
     **/
    ThrowSignal() = default;
    /**
     * @Copy Constructor
     **/
    ThrowSignal(const ThrowSignal&) = default;
    explicit ThrowSignal(const web::json::value& other) noexcept;
    /**
     * @Move Constructor
     **/
    ThrowSignal(ThrowSignal&&) noexcept = default;
    /**
     * @Constructor
     *
     * @brief
     * After construction object should be ready for use.
     * @name: The name of the signal as it appears in Camunda
     * @variables: A list of values that are sent with the signal
     * @executionId: The specific Camunda task that should receive the signal
     *               Note: If no executionId is specified then signal goes to
     *                     global Camunda namespace
     **/
    explicit ThrowSignal(const std::string& name,
                         const Variables&   variables = Variables()) noexcept;

    explicit ThrowSignal(const web::json::value& name,
                         const Variables&        variables = Variables()) noexcept;

    ThrowSignal(const std::string& name,
                const std::string& executionId,
                const Variables&   variables = Variables()) noexcept;

    ThrowSignal(const web::json::value& name,
                const web::json::value& executionId,
                const Variables&        variables = Variables()) noexcept;
    /**
     * @Deconstructor
     **/
    ~ThrowSignal() noexcept override = default;
    /**
     * @Assignment Operator
     **/
    ThrowSignal& operator=(const ThrowSignal&)     = default;
    ThrowSignal& operator=(ThrowSignal&&) noexcept = default;
    /**
     * @add
     *
     * @brief
     * Adds or updates the value given.
     * @name: The name of the signal as it appears in Camunda
     * @variables: A list of values that are sent with the signal
     * @executionId: The specific Camunda task that should receive the signal
     *               Note: If no executionId is specified then signal goes to
     *                     global Camunda namespace
     **/
    virtual void addName(       const std::string&      name)        noexcept;
    virtual void addName(       const web::json::value& name)        noexcept;
    virtual void addExecutionId(const std::string&      executionId) noexcept;
    virtual void addExecutionId(const web::json::value& executionId) noexcept;
    virtual void addVariables(  const Variables&        variables)   noexcept;
    virtual void addVariables(  const web::json::value& variables)   noexcept;
    /**
     * @remove
     *
     * @brief
     * Removes the value.
     **/
    virtual void removeName()        noexcept;
    virtual void removeExecutionId() noexcept;
    virtual void removeVariables()   noexcept;
    /**
     * @get
     *
     * @brief
     * Returns the value if it's present.
     **/
    virtual const web::json::value& getName()        const;
    virtual const web::json::value& getExecutionId() const;
    virtual const web::json::value& getVariables()   const;
    /**
     * @is_safe
     *
     * @brief
     * Makes sure its ready to use.
     **/
    bool is_safe() const noexcept override;
    /**
     * @getSignal
     *
     * @brief
     * Makes sure the object it ready for use then returns
     * the json it holds.
     **/
    virtual const web::json::value& cgetSignal() const;
    virtual       web::json::value&  getSignal();
  };
}// camunda

#endif
/* throw_signal.hpp */

