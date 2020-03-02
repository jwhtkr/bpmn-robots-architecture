/**
 * @File: test_throw_signal.cpp
 * @Date: 27 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the base ThrowSignal object.
 **/

/* Google Headers */
#include<gtest/gtest.h>
#define _TURN_OFF_PLATFORM_STRING

/* Local Headers */
#include"camunda_c_api/throw_signal.hpp"

TEST(Constructor, stack)
{
  camunda::ThrowSignal();
}

TEST(Constructor, heap)
{
  camunda::ThrowSignal* test = new camunda::ThrowSignal();

  delete(test);
}

TEST(Constructor, full)
{
  camunda::Variables var;
  var.addVariable("varKey", web::json::value("varValue"), web::json::value("String"));

  camunda::ThrowSignal test("name", "executionId", var);

  EXPECT_EQ(web::json::value("name"),        test.getName());
  EXPECT_EQ(web::json::value("executionId"), test.getExecutionId());
  EXPECT_EQ(var.get().at("varKey"),          test.getVariables().at("varKey"));
}

TEST(Add, name)
{
  camunda::ThrowSignal test;

  EXPECT_ANY_THROW(test.getName());

  test.addName("name");

  EXPECT_EQ(web::json::value("name"), test.getName());
}

TEST(Add, executionId)
{
  camunda::ThrowSignal test;

  EXPECT_ANY_THROW(test.getExecutionId());

  test.addExecutionId("executionId");

  EXPECT_EQ(web::json::value("executionId"), test.getExecutionId());
}

TEST(Add, variables)
{
  camunda::Variables var;
  var.addVariable("varKey", web::json::value("varValue"), web::json::value("String"));

  camunda::ThrowSignal test;

  EXPECT_ANY_THROW(test.getVariables());

  test.addVariables(var);

  EXPECT_EQ(var.get().at("varKey"), test.getVariables().at("varKey"));
}

TEST(Remove, name)
{
  camunda::ThrowSignal test("name");

  EXPECT_EQ(web::json::value("name"), test.getName());

  test.removeName();

  EXPECT_ANY_THROW(test.getName());
}

TEST(Remove, executionId)
{
  camunda::ThrowSignal test("name", "executionId");

  EXPECT_EQ(web::json::value("executionId"), test.getExecutionId());

  test.removeExecutionId();

  EXPECT_ANY_THROW(test.getExecutionId());
}

TEST(Remove, variables)
{
  camunda::Variables var;
  var.addVariable("varKey", web::json::value("varValue"), web::json::value("String"));

  camunda::ThrowSignal test("name", "executionId", var);

  EXPECT_EQ(var.get().at("varKey"), test.getVariables().at("varKey"));

  test.removeVariables();

  EXPECT_ANY_THROW(test.getVariables());
}

TEST(Is_safe, full)
{
  camunda::ThrowSignal test;

  EXPECT_FALSE(test.is_safe());

  test.addName("name");

  EXPECT_TRUE(test.is_safe());
}

TEST(GetSignal, full)
{
  camunda::ThrowSignal test;

  EXPECT_ANY_THROW(test.getSignal());

  test.addName("name");

  test.getSignal();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_throw_signal.cpp */

