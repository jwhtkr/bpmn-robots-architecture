/**
 * @File: test_lock_request.cpp
 * @Date: 30 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the LockRequest object.
 **/

/* Google Headers */
#include<gtest/gtest.h>
#define _TURN_OFF_PLATFORM_STRING

/* Local Headers */
#include"camunda_c_api/lock_request.hpp"

TEST(Constructor, stack)
{
  camunda::LockRequest test;
}

TEST(Constructor, heap)
{
  camunda::LockRequest* test = new camunda::LockRequest();

  delete(test);
}

TEST(Constructor, full)
{
  camunda::LockRequest test("workerId", 42, camunda::Topics("topicName", 42));

  EXPECT_EQ(web::json::value("workerId"), test.getWorkerId());
  EXPECT_EQ(web::json::value(42), test.getMaxTasks());
  EXPECT_EQ(camunda::Topics("topicName", 42).getTopic(0), test.getTopic(0));
}

TEST(AddWorkerId, full)
{
  camunda::LockRequest test;

  test.addWorkerId("theWorkerID");

  EXPECT_EQ(web::json::value("theWorkerID"), test.getWorkerId());

  test.addWorkerId("newWorkerID");

  EXPECT_EQ(web::json::value("newWorkerID"), test.getWorkerId());
}

TEST(AddMaxTasks, full)
{
  camunda::LockRequest test;

  test.addMaxTasks(42);

  EXPECT_EQ(web::json::value(42), test.getMaxTasks());

  test.addMaxTasks(1);

  EXPECT_EQ(web::json::value(1), test.getMaxTasks());
}

TEST(AddUsePriority, full)
{
  camunda::LockRequest test;

  test.addUsePriority(true);

  EXPECT_EQ(web::json::value(true), test.getUsePriority());

  test.addUsePriority(false);

  EXPECT_EQ(web::json::value(false), test.getUsePriority());
}

TEST(AddAysncResponseTimeout, full)
{
  camunda::LockRequest test;

  test.addAsyncResponseTimeout(42);

  EXPECT_EQ(web::json::value(42), test.getAsyncResponseTimeout());

  test.addAsyncResponseTimeout(1);

  EXPECT_EQ(web::json::value(1), test.getAsyncResponseTimeout());
}

TEST(AddTopic, full)
{
  camunda::LockRequest test;

  camunda::Topics top;
  top.addLockDuration(1, 0);
  top.addLockDuration(1, 1);

  test.addTopics(top);

  EXPECT_EQ(top.get(), test.getTopics());
}

TEST(Is_safe, full)
{
  camunda::LockRequest test;

  EXPECT_FALSE(test.is_safe());

  test.addWorkerId("theWorkerID");

  EXPECT_FALSE(test.is_safe());

  test.addMaxTasks(1);

  EXPECT_FALSE(test.is_safe());

  camunda::Topics top;
  top.addLockDuration(1, 0);
  top.addTopicName("hi", 0);

  test.addTopics(top);

  EXPECT_TRUE(test.is_safe());

  camunda::Topics top2;
  top.addLockDuration(2, 0);
  top.addTopicName("kjh", 0);

  test.addTopics(top2);

  test.addUsePriority(true);
  test.addAsyncResponseTimeout(1000);

  EXPECT_TRUE(test.is_safe());
}

TEST(GetLockRequested, full)
{
  camunda::LockRequest test;

  EXPECT_ANY_THROW(test.getLockRequested());

  test.addWorkerId("theWorkerID");

  EXPECT_ANY_THROW(test.getLockRequested());

  test.addMaxTasks(1);

  EXPECT_ANY_THROW(test.getLockRequested());

  camunda::Topics top;
  top.addLockDuration(1, 0);
  top.addTopicName("kjh", 0);

  test.addTopics(top);

  EXPECT_NE(test.getLockRequested(), web::json::value("test"));

  test.addTopics(top);

  test.addUsePriority(true);
  test.addAsyncResponseTimeout(1000);

  EXPECT_NE(test.getLockRequested(), web::json::value("test"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_lock_request.cpp */

