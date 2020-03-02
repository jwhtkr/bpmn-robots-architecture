/**
 * @File: test_complete_request.cpp
 * @Date: 5 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the CompleteRequest object.
 **/

/* Google Headers */
#include<gtest/gtest.h>
#define _TURN_OFF_PLATFORM_STRING

/* Local Headers */
#include"camunda_c_api/complete_request.hpp"

TEST(Constructor, stack)
{
  camunda::CompleteRequest test;
}

TEST(Constructor, heap)
{
  camunda::CompleteRequest* test = new camunda::CompleteRequest();

  delete(test);
}

TEST(Constructor, normal)
{
  camunda::CompleteRequest test1(std::string     ("workerId"));
  camunda::CompleteRequest test2(web::json::value("workerId"), camunda::Variables());

  EXPECT_EQ(test1, test2);
}

TEST(Add, workerId)
{
  camunda::CompleteRequest test1, test2;

  EXPECT_ANY_THROW(test1.getWorkerId());
  EXPECT_ANY_THROW(test2.getWorkerId());

  test1.addWorkerId                 ("worker1");
  test2.addWorkerId(web::json::value("worker2"));

  EXPECT_EQ(web::json::value("worker1"), test1.getWorkerId());
  EXPECT_EQ(web::json::value("worker2"), test2.getWorkerId());
}

TEST(Add, variables)
{
  web::json::value var;
  var["key1"]["value"] = web::json::value(1);
  var["key1"]["type"]  = web::json::value("number");
  var["key1"]["valueInfo"] = web::json::value("info");

  camunda::CompleteRequest test;

  EXPECT_ANY_THROW(test.getVariables());

  test.addVariables(camunda::Variables(var));

  EXPECT_EQ(var, test.getVariables());
}

TEST(Add, variable)
{
  web::json::value var;
  var["value"] = web::json::value(1);
  var["type"]  = web::json::value("number");

  camunda::CompleteRequest test;

  test.addVariable("key1", web::json::value(1), web::json::value("number"));

  EXPECT_EQ(var, test.getVariable("key1"));
}

TEST(Remove, workerId)
{
  camunda::CompleteRequest test("worker1");

  EXPECT_EQ(web::json::value("worker1"), test.getWorkerId());

  test.removeWorkerId();

  EXPECT_ANY_THROW(test.getWorkerId());

  test.removeWorkerId();
}

TEST(Remove, variables)
{
  web::json::value var;
  var["key1"]["value"] = web::json::value(1);
  var["key1"]["type"]  = web::json::value("number");
  var["key1"]["valueInfo"] = web::json::value("info");

  camunda::CompleteRequest test;

  test.removeVariables();
  EXPECT_ANY_THROW(test.getVariables());

  test.addVariables(camunda::Variables(var));

  EXPECT_EQ(var, test.getVariables());

  test.removeVariables();

  EXPECT_ANY_THROW(test.getVariables());
}

TEST(Remove, variable)
{
  web::json::value var;
  var["key1"]["value"] = web::json::value(1);
  var["key1"]["type"]  = web::json::value("number");
  var["key1"]["valueInfo"] = web::json::value("info");

  camunda::CompleteRequest test;

  EXPECT_ANY_THROW(test.removeVariable("key1"));
  EXPECT_ANY_THROW(test.getVariables());

  test.addVariables(camunda::Variables(var));

  EXPECT_EQ(var, test.getVariables());

  test.removeVariable("key1");

  EXPECT_ANY_THROW(test.getVariable("key1"));
}

TEST(Get, workerId)
{
  camunda::CompleteRequest test("worker1");

  EXPECT_EQ(web::json::value("worker1"), test.getWorkerId());

  test.removeWorkerId();

  EXPECT_ANY_THROW(test.getWorkerId());
}

TEST(Get, variables)
{
  web::json::value var;
  var["key1"]["value"] = web::json::value(1);
  var["key1"]["type"]  = web::json::value("number");
  var["key1"]["valueInfo"] = web::json::value("info");

  camunda::CompleteRequest test;

  EXPECT_ANY_THROW(test.getVariables());

  test.addVariables(camunda::Variables(var));

  EXPECT_EQ(var, test.getVariables());
}

TEST(Get, variable)
{
  web::json::value var;
  var["key1"]["value"] = web::json::value(1);
  var["key1"]["type"]  = web::json::value("number");
  var["key1"]["valueInfo"] = web::json::value("info");

  web::json::value var2;
  var2["value"] = web::json::value(1);
  var2["type"]  = web::json::value("number");
  var2["valueInfo"] = web::json::value("info");

  camunda::CompleteRequest test;

  EXPECT_ANY_THROW(test.getVariable("key1"));

  test.addVariables(camunda::Variables(var));

  EXPECT_EQ(var2, test.getVariable("key1"));
}

TEST(Get, value)
{
  web::json::value var;
  var["key1"]["value"] = web::json::value(1);
  var["key1"]["type"]  = web::json::value("number");
  var["key1"]["valueInfo"] = web::json::value("info");

  camunda::CompleteRequest test("worker1", camunda::Variables(var));

  EXPECT_EQ(web::json::value(1), test.getValue("key1"));
}

TEST(Get, valueType)
{
  web::json::value var;
  var["key1"]["value"] = web::json::value(1);
  var["key1"]["type"]  = web::json::value("number");
  var["key1"]["valueInfo"] = web::json::value("info");

  camunda::CompleteRequest test("worker1", camunda::Variables(var));

  EXPECT_EQ(web::json::value("number"), test.getValueType("key1"));
}

TEST(Get, valueInfo)
{
  web::json::value var;
  var["key1"]["value"] = web::json::value(1);
  var["key1"]["type"]  = web::json::value("number");
  var["key1"]["valueInfo"] = web::json::value("info");

  camunda::CompleteRequest test("worker1", camunda::Variables(var));

  EXPECT_EQ(web::json::value("info"), test.getValueInfo("key1"));
}

TEST(Is_safe, full)
{
  camunda::CompleteRequest test;

  EXPECT_FALSE(test.is_safe());

  test.addWorkerId("workerId");

  EXPECT_TRUE(test.is_safe());
}

TEST(Get, completeRequest)
{
  camunda::CompleteRequest test;

  EXPECT_ANY_THROW(test.getCompleteRequest());

  test.addWorkerId("workerId");

  EXPECT_NO_THROW(test.getCompleteRequest());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_complete_request.cpp */

