/**
 * @File: test_message_handler.cpp
 * @Date: 10 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the MessageHandler object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"bpmn_interface/message_handler.hpp"

#define BASE_URI "http://localhost:8080/"

TEST(Constructor, stack)
{
  bpmn::MessageHandler(BASE_URI, "workerId");
}

TEST(Constructor, heap)
{
  bpmn::MessageHandler* test = new bpmn::MessageHandler(BASE_URI, "workerId");

  delete(test);
}

TEST(Constructor, full)
{
  bpmn::MessageHandler test(BASE_URI, "workerId");

  EXPECT_EQ(BASE_URI,   test.getBaseUri());
  EXPECT_EQ("workerId", test.getWorkerId());
}

TEST(SendMessage, full)
{
  bpmn::MessageHandler test(BASE_URI, "workerId");

  test.sendMessage("message_name",
                   camunda::Variables());
}

TEST(ThrowSignal, full)
{
  bpmn::MessageHandler test(BASE_URI, "workerId");

  test.throwSignal(camunda::ThrowSignal("signal_name"));
}

TEST(GetMessage, full)
{
  bpmn::MessageHandler test(BASE_URI, "workerId");

  test.getMessage(camunda::Topics("get_message", 18000))->print();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_message_handler.cpp */

