/**
 * @File: test_variables.cpp
 * @Date: 3 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the base variables object.
 **/

/* Google Headers */
#include<gtest/gtest.h>
#define _TURN_OFF_PLATFORM_STRING

/* Local Headers */
#include"camunda_c_api/variables.hpp"

web::json::value makeVal()
{
  web::json::value test;

  test["val1"]["value"]     = web::json::value(1);
  test["val1"]["type"]      = web::json::value("number");
  test["val1"]["valueInfo"] = web::json::value();
  test["val2"]["value"]     = web::json::value("Author Dent");
  test["val2"]["type"]      = web::json::value("string");
  test["val2"]["valueInfo"] = web::json::value();
  test["val3"]["value"]     = web::json::value(true);
  test["val3"]["type"]      = web::json::value("bool");
  test["val3"]["valueInfo"] = web::json::value();

  return test;
}

TEST(Constructor, stack)
{
  camunda::Variables test;
}

TEST(Constructor, heap)
{
  camunda::Variables* test = new camunda::Variables();

  delete(test);
}

TEST(Constructor, normal)
{
  camunda::Variables test(makeVal());
}

TEST(Is_safe, shouldBe)
{
  camunda::Variables test(makeVal());

  EXPECT_TRUE(test.is_safe());
}

TEST(Is_safe, shouldntBe)
{
  camunda::Variables test;

  EXPECT_FALSE(test.is_safe());
}

TEST(GetVariables, shouldWork)
{
  camunda::Variables test(makeVal());

  EXPECT_NO_THROW(test.getVariables());
}

TEST(GetVariables, shouldntWork)
{
  camunda::Variables test;

  EXPECT_ANY_THROW(test.getVariables());
}

TEST(GetValue, shouldWork)
{
  camunda::Variables test(makeVal());

  EXPECT_EQ(web::json::value(1),             test.getValue("val1"));
  EXPECT_EQ(web::json::value("Author Dent"), test.getValue("val2"));
  EXPECT_EQ(web::json::value(true),          test.getValue("val3"));
}

TEST(GetValue, shouldntWork)
{
  camunda::Variables test;

  EXPECT_ANY_THROW(test.getValue("val1"));
  EXPECT_ANY_THROW(test.getValue("val2"));
  EXPECT_ANY_THROW(test.getValue("val3"));
}

TEST(GetType, shouldWork)
{
  camunda::Variables test(makeVal());

  EXPECT_EQ(web::json::value("number"), test.getType("val1"));
  EXPECT_EQ(web::json::value("string"), test.getType("val2"));
  EXPECT_EQ(web::json::value("bool"),   test.getType("val3"));
}

TEST(GetType, shouldntWork)
{
  camunda::Variables test;

  EXPECT_ANY_THROW(test.getType("val1"));
  EXPECT_ANY_THROW(test.getType("val2"));
  EXPECT_ANY_THROW(test.getType("val3"));
}

TEST(GetValueInfo, shouldWork)
{
  camunda::Variables test(makeVal());

  EXPECT_EQ(web::json::value(), test.getValueInfo("val1"));
  EXPECT_EQ(web::json::value(), test.getValueInfo("val2"));
  EXPECT_EQ(web::json::value(), test.getValueInfo("val3"));
}

TEST(GetValueInfo, shouldntWork)
{
  camunda::Variables test;

  EXPECT_ANY_THROW(test.getValueInfo("val1"));
  EXPECT_ANY_THROW(test.getValueInfo("val2"));
  EXPECT_ANY_THROW(test.getValueInfo("val3"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* variables.hpp */

