/**
 * @File: test_lock_response.cpp
 * @Date: 30 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the lock response object.
 **/

/* Google Headers */
#include<gtest/gtest.h>
#define _TURN_OFF_PLATFORM_STRING

/* Local Headers */
#include"camunda_c_api/lock_response.hpp"

web::json::value makeTest(const uint32_t array_size, const uint32_t num_variables)
{
  web::json::value test;

  for(uint32_t it = 0; it < array_size; it++)
  {
    test[it]["activityId"]           = web::json::value(std::string("activityId"           + std::to_string(it)));
    test[it]["activityInstanceId"]   = web::json::value(std::string("activityInstanceId"   + std::to_string(it)));
    test[it]["errorMessage"]         = web::json::value(std::string("errorMessage"         + std::to_string(it)));
    test[it]["errorDetails"]         = web::json::value(std::string("errorDetails"         + std::to_string(it)));
    test[it]["executionId"]          = web::json::value(std::string("executionId"          + std::to_string(it)));
    test[it]["id"]                   = web::json::value(std::string("id"                   + std::to_string(it)));
    test[it]["lockExpirationTime"]   = web::json::value(std::string("lockExpirationTime"   + std::to_string(it)));
    test[it]["processDefinitionId"]  = web::json::value(std::string("processDefinitionId"  + std::to_string(it)));
    test[it]["processDefinitionKey"] = web::json::value(std::string("processDefinitionKey" + std::to_string(it)));
    test[it]["processInstanceId"]    = web::json::value(std::string("processInstanceId"    + std::to_string(it)));
    test[it]["tenantId"]             = web::json::value(std::string("tenantId"             + std::to_string(it)));
    (0 == (it % 2)) ? test[it]["retries"] = web::json::value(0) : test[it]["retries"];
    test[it]["workerId"]             = web::json::value(std::string("workerId"             + std::to_string(it)));
    test[it]["priority"]             = web::json::value(1);
    test[it]["topicName"]            = web::json::value(std::string("topicName"            + std::to_string(it)));
    test[it]["businessKey"]          = web::json::value(std::string("businessKey"          + std::to_string(it)));

    for(uint32_t var_it = 0; var_it < num_variables; var_it++)
    {
      test[it]["variables"]["variable" + std::to_string(var_it)]["value"] = web::json::value("val");
      test[it]["variables"]["variable" + std::to_string(var_it)]["type"]  = web::json::value("String");
    }
  }

  return test;
}

TEST(Constructor, default)
{
  camunda::LockResponse test;
}

TEST(Constructor, value)
{
  camunda::LockResponse test(makeTest(1,1));
}

TEST(getId, nun)
{
  camunda::LockResponse test;

  EXPECT_ANY_THROW(test.getId(0));
  EXPECT_ANY_THROW(test.getId(42));
}

TEST(getId, one)
{
  camunda::LockResponse test(makeTest(1,1));

  EXPECT_EQ(web::json::value("id0"), test.getId(0));
}

TEST(getId, meny)
{
  camunda::LockResponse test(makeTest(10, 1));

  for(auto it = 0; it < 10; it++)
  {
    EXPECT_EQ(web::json::value("id" + std::to_string(it)), test.getId(it));
  }
}

TEST(getWorkerId, nun)
{
  camunda::LockResponse test;

  EXPECT_ANY_THROW(test.getWorkerId(0));
  EXPECT_ANY_THROW(test.getWorkerId(42));
}

TEST(getWorkerId, one)
{
  camunda::LockResponse test(makeTest(1,1));

  EXPECT_EQ(web::json::value("workerId0"), test.getWorkerId(0));
}

TEST(getWorkerId, meny)
{
  camunda::LockResponse test(makeTest(10, 1));

  for(auto it = 0; it < 10; it++)
  {
    EXPECT_EQ(web::json::value("workerId" + std::to_string(it)), test.getWorkerId(it));
  }
}

TEST(getRetries, nun)
{
  camunda::LockResponse test;

  EXPECT_ANY_THROW(test.getRetries(0));
  EXPECT_ANY_THROW(test.getRetries(42));
}

TEST(getRetries, one)
{
  camunda::LockResponse test(makeTest(1,1));

  EXPECT_EQ(web::json::value(0), test.getRetries(0));
}

TEST(getRetries, meny)
{
  camunda::LockResponse test(makeTest(10, 1));

  for(auto it = 0; it < 10; it++)
  {
    EXPECT_EQ(web::json::value(0), test.getRetries(it));
  }
}

TEST(getVariables, one)
{
  camunda::LockResponse test(makeTest(1,1));

  EXPECT_NO_THROW(test.getVariables(0));
}

TEST(getVariables, meny)
{
  camunda::LockResponse test(makeTest(10, 1));

  for(auto it = 0; it < 10; it++)
  {
    EXPECT_NO_THROW(test.getVariables(it));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_lock_response.cpp */

