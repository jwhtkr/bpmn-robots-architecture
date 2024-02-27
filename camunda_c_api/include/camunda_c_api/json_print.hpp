/**
 * @File: json_print.hpp
 * @Date: 20 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * This file holds printing functions for json objects.
 **/

#ifndef CAMUNDA_HELPERS_JSON_PRINT_HPP
#define CAMUNDA_HELPERS_JSON_PRINT_HPP

/* Rest Headers */
#include<cpprest/json.h>

/* C++ Headers */
#include<string>

namespace camunda
{
  /**
   * @printJson
   *
   * @brief
   * Prints the json in a nicer format then cout does by default.
   **/
  void printJson(const web::json::value& json_in);
}; // camunda

#endif
/* json_print.hpp */

