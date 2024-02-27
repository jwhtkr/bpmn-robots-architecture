/**
 * @File: test_json.cpp
 * @Date: 30 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the base Json object.
 **/

/* Google Headers */
#include<gtest/gtest.h>
#define _TURN_OFF_PLATFORM_STRING

/* Local Headers */
#include"camunda_c_api/json.hpp"

TEST(Constructor, stack)
{
  camunda::Json test;
}

TEST(Constructor, heap)
{
  camunda::Json* test = new camunda::Json();

  delete(test);
}

TEST(Constructor, copy)
{
  camunda::Json test;
  test["one"] = 1;

  camunda::Json test1(test);

  EXPECT_EQ(web::json::value(1), test1.at("one"));
  EXPECT_EQ(web::json::value(1), test. at("one"));
}

TEST(Constructor, normal)
{
  camunda::Json test(web::json::value("hi"));

  EXPECT_EQ(web::json::value("hi"), test.get());
}

TEST(AssignmentOperator, atInit)
{
  camunda::Json test;
  test["one"] = 1;

  camunda::Json test2 = test;

  EXPECT_EQ(web::json::value(1), test2.at("one"));
}

TEST(AssignmentOperator, afterUse)
{
  web::json::value test;
  test["one"] = 1;

  camunda::Json test2;
  test2["two"] = 2;

  test2 = test;

  EXPECT_EQ(web::json::value(1), test2.at("one"));
  EXPECT_ANY_THROW(test2.at("two"));
}

TEST(ComarisonOperator, is)
{
  camunda::Json test1, test2;

  EXPECT_TRUE (test1 == test2);
  EXPECT_FALSE(test1 != test2);
}

TEST(ComarisonOperator, isnot)
{
  camunda::Json test1, test2;
  test1["one"] = 1;

  EXPECT_TRUE (test1 != test2);
  EXPECT_FALSE(test1 == test2);
}

TEST(SubscriptOperator, fillWithString)
{
  camunda::Json test;
  test["one"] = 1;

  EXPECT_EQ(web::json::value(1), test.at("one"));
  EXPECT_EQ(web::json::value(1), test["one"]);

  test["null"] = 0;

  EXPECT_TRUE(web::json::value(0) == test.at("null"));
}

TEST(SubscriptOperator, fillWithNumber)
{
  camunda::Json test;

  test[0] = 0;
  test[1] = 1;
  test[2] = 2;

  EXPECT_EQ(test.at(1), test[1]);
}

TEST(At, present)
{
  camunda::Json test, test2;
  test["one"] = 1;
  test2[1] = 1;

  EXPECT_EQ(test.at("one"), test2.at(1));
}

TEST(At, notPresent)
{
  camunda::Json test, test2;
  test["one"] = 1;
  test2[1] = 1;

  EXPECT_ANY_THROW(test.at("notHere"));
  EXPECT_ANY_THROW(test2.at(42));
}

TEST(Add, notPresent)
{
  camunda::Json test;

  test.add("one", 1);

  EXPECT_EQ(web::json::value(1), test.at("one"));
}

TEST(Add, present)
{
  camunda::Json test;

  test.add("one", 1);
  test.add("one", 2);

  EXPECT_EQ(web::json::value(2), test.at("one"));
}

TEST(Remove, present)
{
  camunda::Json test1, test2;
  test1["one"] = 1;
  test2[2] = 2;

  test1.remove("one");
  test2.remove(2);

  EXPECT_FALSE(test1.has_field("one"));
  EXPECT_ANY_THROW(test2.at(2));
}

TEST(Remove, notPresent)
{
  camunda::Json test;

  test.remove("notHere");
}

TEST(Get, present)
{
  camunda::Json test;
  test.get()["one"] = 1;

  EXPECT_EQ(web::json::value(1), test.at("one"));
}

TEST(Has_field, present)
{
  camunda::Json test;
  test["one"];

  EXPECT_TRUE(test.has_field("one"));
}

TEST(Has_field, notPresent)
{
  camunda::Json test;

  EXPECT_FALSE(test.has_field("notHere"));
}

TEST(Is_safe, full)
{
  camunda::Json test;

  EXPECT_FALSE(test.is_safe());

  test["one"];

  EXPECT_TRUE(test.is_safe());
}

TEST(BeginAndEnd, full)
{
  camunda::Json test;
  test["val1"] = web::json::value(1);
  test["val2"] = web::json::value(2);
  test["val3"] = web::json::value(3);

  int count = 0;

  for(auto it = test.begin(); it < test.end(); it++)
  {
    count++;
    EXPECT_EQ("val" + std::to_string(count), it->first);
    EXPECT_EQ(web::json::value(count), it->second);
  }
}

TEST(Size, full)
{
  camunda::Json test;

  EXPECT_EQ(0, test.size());

  test[0] = web::json::value(12);

  EXPECT_EQ(1, test.size());
}

TEST(Print, full)
{
  camunda::Json test;

  test.print();

  test["val1"] = web::json::value(1);
  test["val2"] = web::json::value(2);
  test["val3"] = web::json::value(3);

  test.print();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_json.cpp */

