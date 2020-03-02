/**
 * @File: test_print_json.cpp
 * @Date: 29 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests functions defined in json_print.hpp
 **/

/* Google Headers */
#include<gtest/gtest.h>
#define _TURN_OFF_PLATFORM_STRING

/* Local Headers */
#include"camunda_c_api/json_print.hpp"

TEST(Number, one)
{
  web::json::value test;
  test["positiveInt"] = 42;

  camunda::printJson(test);

  test = web::json::value();
  test["negativeInt"] = -42;

  camunda::printJson(test);

  test = web::json::value();
  test["positiveFloat"] = 0.42;

  camunda::printJson(test);

  test = web::json::value();
  test["negativeFloat"] = -0.42;

  camunda::printJson(test);
}

TEST(Number, many)
{
  web::json::value test;

  test["positiveInt"]   = 42;
  test["negativeInt"]   = -42;
  test["positiveFloat"] = 0.42;
  test["negativeFloat"] = -0.42;

  camunda::printJson(test);
}

TEST(Boolean, one)
{
  web::json::value test;
  test["boolTrue"] = true;

  camunda::printJson(test);

  test = web::json::value();
  test["boolFalse"] = false;

  camunda::printJson(test);
}

TEST(Boolean, many)
{
  web::json::value test;
  test["boolTrue"]  = true;
  test["boolFalse"] = false;

  camunda::printJson(test);
}

TEST(String, one)
{
  web::json::value test;
  test["string"] = web::json::value("this_is_a_string");

  camunda::printJson(test);
}

TEST(String, many)
{
  web::json::value test;
  test["string1"] = web::json::value("this_is_a_string");
  test["string2"] = web::json::value("this is a string");
  test["string3"] = web::json::value("this is sorta a string");
  test["string4"] = web::json::value("this is most of a string");

  camunda::printJson(test);
}

TEST(Null, one)
{
  web::json::value test;

  camunda::printJson(test);
}

TEST(Null, many)
{
  web::json::value test;
  test["null1"];
  test["null2"];
  test["null3"];

  camunda::printJson(test);
}

TEST(Array, oneVal)
{
  web::json::value test;
  test[0] = 1;

  camunda::printJson(test);
}

TEST(Array, manyVal)
{
  web::json::value test;

  for(auto it = 0; it < 26; it++)
  {
    test[it] = it;
  }

  camunda::printJson(test);
}

TEST(Object, complicated)
{
  web::json::value test, holder1, holder2;

  holder1["string"] = web::json::value("a string");
  holder1["bool"]   = false;
  holder1["number"] = 42;
  for(auto it = 0; it < 10; it++)
  {
    holder1["array"][it] = it;
  }

  holder2["number"] = 24;
  holder2["null"];
  holder2["holder1"];
  for(auto it = 0; it < 4; it++)
  {
    holder2["holder1"][it] = holder1;
  }

  test["holder1"] = holder1;
  test["number"]  = 0.78;
  test["holder2"] = holder2;

  camunda::printJson(test);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_print_json.cpp */

