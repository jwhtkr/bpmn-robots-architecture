/**
 * @File: test_error_handler.cpp
 * @Date: 31 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the ErrorHandler object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"camunda_error_handling/error_handler.hpp"

TEST(Constructor, stack)
{
  camunda::ErrorHandler test("http://localhost:8080/", "workerId");

  EXPECT_EQ("http://localhost:8080/",     test.getUri());
  EXPECT_EQ(web::json::value("workerId"), test.getWorkerId());
}

TEST(Constructor, onHeap)
{
  camunda::ErrorHandler* test = new camunda::ErrorHandler("http://localhost:8080/", "workerId");

  delete(test);
}

TEST(Constructor, copy)
{
  camunda::ErrorHandler test1("http://localhost:8080/", "workerId");

  camunda::ErrorHandler test(test1);

  EXPECT_EQ("http://localhost:8080/",     test.getUri());
  EXPECT_EQ(web::json::value("workerId"), test.getWorkerId());
  EXPECT_EQ("http://localhost:8080/",     test1.getUri());
  EXPECT_EQ(web::json::value("workerId"), test1.getWorkerId());
}

TEST(SetForTest, Full)
{
  camunda::ErrorHandler test("http://localhost:8080/", "workerId");

  test.setForTask("Id", 10, 42);

  EXPECT_EQ("Id",                 test.getId());
  EXPECT_EQ(web::json::value(10), test.getRetries());
  EXPECT_EQ(web::json::value(42), test.getRetryTimeout());
}

TEST(ThrowError, Full)
{
  camunda::ErrorHandler test("http://localhost:8080/", "workerId");

  test.setForTask("Id");

  test.throwError("errorMessage");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_error_handler.cpp */

