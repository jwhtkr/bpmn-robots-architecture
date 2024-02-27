/**
 * @File: test_extend_lock.cpp
 * @Date: 26 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the ExtendLock object.
 **/

/* Google Headers */
#include<gtest/gtest.h>
#define _TURN_OFF_PLATFORM_STRING

/* Local Headers */
#include"camunda_c_api/extend_lock.hpp"

TEST(Constructor, stack)
{
  camunda::ExtendLock test;
}

TEST(Constructor, heap)
{
  camunda::ExtendLock* test = new camunda::ExtendLock();

  delete(test);
}

TEST(Constructor, normal)
{
  camunda::ExtendLock test("workerId", 42);

  EXPECT_EQ(web::json::value("workerId"), test.getWorkerId());
  EXPECT_EQ(web::json::value(42),         test.getNewDuration());
}

TEST(Add, workerId)
{
  camunda::ExtendLock test;

  EXPECT_ANY_THROW(test.getWorkerId());

  test.addWorkerId("workerId");

  EXPECT_EQ(web::json::value("workerId"), test.getWorkerId());
}

TEST(Add, newDuration)
{
  camunda::ExtendLock test;

  EXPECT_ANY_THROW(test.getNewDuration());

  test.addNewDuration(42);

  EXPECT_EQ(web::json::value(42), test.getNewDuration());
}

TEST(Is_safe, full)
{
  camunda::ExtendLock test;

  EXPECT_FALSE(test.is_safe());

  test.addWorkerId("workerId");
  test.addNewDuration(42);

  EXPECT_TRUE(test.is_safe());
}

TEST(GetExtendLockRequest, full)
{
  camunda::ExtendLock test;

  EXPECT_ANY_THROW(test.getExtendLockRequest());

  test.addWorkerId("workerId");
  test.addNewDuration(42);

  test.getExtendLockRequest();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_extend_lock.cpp */

