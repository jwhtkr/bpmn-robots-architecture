/**
 * @File: json.hpp
 * @Date: 20 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * A base class for other camunda objects to inherit from.
 **/

#ifndef CAMUNDA_OBJECTS_JSON_HPP
#define CAMUNDA_OBJECTS_JSON_HPP

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>

namespace camunda
{
  class Json
  {
  public:
    /**
     * @Default Constructor
     **/
    Json() = default;
    /**
     * @Copy Constructors
     **/
    Json(const Json&) = default;
    explicit Json(const web::json::value& other);
    /**
     * @Move Constructor
     **/
    Json(Json&&) noexcept = default;
    /**
     * @Deconstructor
     **/
    virtual ~Json() noexcept = default;
    /**
     * @Assignment Operator
     *
     * @brief
     * Empties this object and fills it with information from the other
     * Json object.
     **/
    Json& operator=(const Json&)     = default;
    Json& operator=(Json&&) noexcept = default;
    Json& operator=(const web::json::value& other) noexcept;
    /**
     * @Comparison Operators
     **/
    bool operator==(const Json& other) const noexcept;
    bool operator!=(const Json& other) const noexcept;
    /**
     * @Subscript Operator
     *
     * @brief
     * Works the same way a web::json::value does. As in if the object has
     * the field it simply returns a referents to the field and if it doesn't it dynamically
     * makes it and then returns the reference.
     **/
    virtual web::json::value& operator[](const std::string& field_name)  noexcept;
    virtual web::json::value& operator[](const uint32_t     array_index) noexcept;
    /**
     * @at
     *
     * @brief
     * Returns a reference to the field only if that field exists. Throws a
     * runtime exception if it doesn't.
     * @field_name: The web::json::value name
     * @array_index: The location of the value in this object
     **/
    virtual const web::json::value& at(const std::string& field_name)  const;
    virtual const web::json::value& at(const uint32_t     array_index) const;
    virtual       web::json::value& at(const std::string& field_name);
    virtual       web::json::value& at(const uint32_t     array_index);
    /**
     * @add
     *
     * @brief
     * If the field doesn't already exist it will make it and update it's value and if it
     * does already exist it will just update the value.
     * @field_name: The Value's name
     * @array_index: Where in this object you want the value to be stored
     * @value: The value to be stored
     **/
    virtual void add(const std::string& field_name, const web::json::value& value) noexcept;
    virtual void add(const uint32_t array_index,    const web::json::value& value) noexcept;
    /**
     * @remove
     *
     * @brief
     * Removes field or array index.
     * @field_name: The web::json::value name
     * @array_index: The location of the value in this object
     **/
    virtual void remove(const std::string& field_name)  noexcept;
    virtual void remove(const uint32_t     array_index) noexcept;
    /**
     * @get
     *
     * @brief
     * Returns a referents to the web::json::value this object holds.
     **/
    const web::json::value& cget() const;
          web::json::value&  get();
    /**
     * @has_field
     *
     * @brief
     * Checks to see of the web::json::value object this object holds has the
     * passed in field.
     * @field_name: The web::json::value name
     * @return: True if is does and false otherwise
     **/
    virtual bool has_field(const std::string& field_name) const noexcept;
    /**
     * @is_safe
     *
     * @brief
     * Checks whether or not the json is null.
     * @return: True if it isn't and false if it is null
     **/
    virtual bool is_safe() const noexcept;
    /**
     * @begin
     * @end
     *
     * @brief
     * Returns iterators to the underlying web::json::object.
     **/
    virtual       web::json::object::iterator        begin();
    virtual       web::json::object::iterator        end();
    virtual const web::json::object::const_iterator cbegin() const;
    virtual const web::json::object::const_iterator cend()   const;
    /**
     * @size
     *
     * @brief
     * Returns how big the array this object holds is if it is an array.
     **/
    virtual uint32_t size() const;
    /**
     * @print
     *
     * @brief
     * Prints the contents of this object using the printJson
     * function.
     **/
    virtual void print() const noexcept;
  protected:
    /* Object this class raps */
    web::json::value m_json;
  };
}; // camunda

#endif
/* json.hpp */

