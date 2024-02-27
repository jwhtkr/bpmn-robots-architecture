/**
 * @File: test_camunda_error.cpp
 * @Date: 6 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the CamundaError object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"camunda_c_api/camunda_error.hpp"

TEST(Constructor, stack)
{
  camunda::CamundaError();
}

TEST(Constructor, heap)
{
  camunda::CamundaError* test = new camunda::CamundaError();

  delete(test);
}

TEST(Constructor, full)
{
  camunda::CamundaError test("workerId",
                             "errorMessage",
                             "errorDetails",
                             42,
                             10);
  EXPECT_EQ(web::json::value("workerId"),     test.getWorkerId());
  EXPECT_EQ(web::json::value("errorMessage"), test.getErrorMessage());
  EXPECT_EQ(web::json::value("errorDetails"), test.getErrorDetails());
  EXPECT_EQ(web::json::value(42),             test.getRetries());
  EXPECT_EQ(web::json::value(10),             test.getRetryTimeout());
}

TEST(AddRemove, workerId)
{
  camunda::CamundaError test;

  EXPECT_ANY_THROW(test.getWorkerId());

  test.addWorkerId("workerId");

  EXPECT_EQ(web::json::value("workerId"), test.getWorkerId());

  test.removeWorkerId();

  EXPECT_ANY_THROW(test.getWorkerId());
}

TEST(AddRemove, errorMessage)
{
  camunda::CamundaError test;

  EXPECT_ANY_THROW(test.getErrorMessage());

  test.addErrorMessage("errorMessage");

  EXPECT_EQ(web::json::value("errorMessage"), test.getErrorMessage());

  test.removeErrorMessage();

  EXPECT_ANY_THROW(test.getErrorMessage());
}

TEST(AddRemove, errorDetails)
{
  camunda::CamundaError test;

  EXPECT_ANY_THROW(test.getErrorDetails());

  test.addErrorDetails("errorDetails");

  EXPECT_EQ(web::json::value("errorDetails"), test.getErrorDetails());

  test.removeErrorDetails();

  EXPECT_ANY_THROW(test.getErrorDetails());
}

TEST(AddRemove, retries)
{
  camunda::CamundaError test;

  EXPECT_ANY_THROW(test.getRetries());

  test.addRetries(42);

  EXPECT_EQ(web::json::value(42), test.getRetries());

  test.removeRetries();

  EXPECT_ANY_THROW(test.getRetries());
}

TEST(AddRemove, retryTimeout)
{
  camunda::CamundaError test;

  EXPECT_ANY_THROW(test.getRetryTimeout());

  test.addRetryTimeout(42);

  EXPECT_EQ(web::json::value(42), test.getRetryTimeout());

  test.removeRetryTimeout();

  EXPECT_ANY_THROW(test.getRetryTimeout());
}

TEST(Add, exception)
{
  camunda::CamundaError test;

  EXPECT_ANY_THROW(test.getErrorMessage());
  EXPECT_ANY_THROW(test.getErrorDetails());

  test.addException(camunda::CamundaException("errorMessage", "errorDetails"));

  EXPECT_EQ(web::json::value("errorMessage"), test.getErrorMessage());
  EXPECT_EQ(web::json::value("errorDetails"), test.getErrorDetails());
}

TEST(Get, error)
{
  camunda::CamundaError test;

  EXPECT_ANY_THROW(test.getError());

  test.addWorkerId("workerId");
  test.addErrorMessage("errorMessage");
  test.addErrorDetails("errorDetails");
  test.addRetries(42);
  test.addRetryTimeout(10);

  test.getError();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_camunda_error.cpp */

