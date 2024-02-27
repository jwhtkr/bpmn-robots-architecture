/**
 * @File: test_topics.cpp
 * @Date: 11 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the base Topics object.
 **/

/* Google Headers */
#include<gtest/gtest.h>
#define _TURN_OFF_PLATFORM_STRING

/* Local Headers */
#include"camunda_c_api/topics.hpp"

TEST(Constructor, strack)
{
  camunda::Topics();
}

TEST(Constructor, heap)
{
  camunda::Topics* test = new camunda::Topics();

  delete(test);
}

TEST(Constructor, normal)
{
  camunda::Topics test("topicName", 42, std::vector<std::string>({"one", "two", "three"}));

  EXPECT_EQ(web::json::value("topicName"), test.getTopicName(0));
  EXPECT_EQ(web::json::value(42),          test.getLockDuration(0));
  EXPECT_EQ(web::json::value("one"),       test.getVariables(0).at(0));
}

TEST(Add, topicName)
{
  camunda::Topics test;

  EXPECT_ANY_THROW(test.getTopicName(0));

  test.addTopicName("one", 0);
  test.addTopicName("two", 1);

  EXPECT_EQ(web::json::value("one"), test.getTopicName(0));
  EXPECT_EQ(web::json::value("two"), test.getTopicName(1));
}

TEST(Add, lockDuration)
{
  camunda::Topics test;

  EXPECT_ANY_THROW(test.getLockDuration(0));

  test.addLockDuration(0, 0);
  test.addLockDuration(1, 1);

  EXPECT_EQ(web::json::value(0), test.getLockDuration(0));
  EXPECT_EQ(web::json::value(1), test.getLockDuration(1));
}

TEST(Add, variables)
{
  camunda::Topics test;

  EXPECT_ANY_THROW(test.getVariables(0));

  test.addVariables(std::vector<std::string>({"var1.1", "var2.1"}), 0);
  test.addVariables(std::vector<std::string>({"var1.2", "var2.2"}), 1);

  EXPECT_EQ(web::json::value("var2.1"), test.getVariables(0).at(1));
  EXPECT_EQ(web::json::value("var1.2"), test.getVariables(1).at(0));
}

TEST(Add, variable)
{
  camunda::Topics test;

  EXPECT_ANY_THROW(test.getVariables(0));

  test.addVariable("var1.1", 0);
  test.addVariable("var2.1", 0);
  test.addVariable("var1.2", 1);
  test.addVariable("var2.2", 1);

  EXPECT_EQ(web::json::value("var2.1"), test.getVariables(0).at(1));
  EXPECT_EQ(web::json::value("var1.2"), test.getVariables(1).at(0));
}

TEST(Add, topic)
{
  camunda::Topics test;

  EXPECT_ANY_THROW(test.getTopics());

  test.addTopic("topic", 42);

  EXPECT_EQ(web::json::value("topic"), test.getTopicName(0));
  EXPECT_EQ(web::json::value(42),      test.getLockDuration(0));
}

TEST(Remove, topic)
{
  camunda::Topics test;

  EXPECT_ANY_THROW(test.getTopics());

  test.addTopic("topic", 42);

  EXPECT_EQ(web::json::value("topic"), test.getTopicName(0));
  EXPECT_EQ(web::json::value(42),      test.getLockDuration(0));

  test.removeTopic(0);

  EXPECT_ANY_THROW(test.getTopic(0));
}

TEST(Remove, topicName)
{
  camunda::Topics test;

  EXPECT_ANY_THROW(test.getTopics());

  test.addTopic("topic", 42);

  EXPECT_EQ(web::json::value("topic"), test.getTopicName(0));
  EXPECT_EQ(web::json::value(42),      test.getLockDuration(0));

  test.removeTopicName(0);

  EXPECT_ANY_THROW(test.getTopicName(0));
}

TEST(Remove, lockDuration)
{
  camunda::Topics test;

  EXPECT_ANY_THROW(test.getTopics());

  test.addTopic("topic", 42);

  EXPECT_EQ(web::json::value("topic"), test.getTopicName(0));
  EXPECT_EQ(web::json::value(42),      test.getLockDuration(0));

  test.removeLockDuration(0);

  EXPECT_ANY_THROW(test.getLockDuration(0));
}

TEST(Remove, variables)
{
  camunda::Topics test;

  EXPECT_ANY_THROW(test.getVariables(0));

  test.addVariables(std::vector<std::string>({"var1.1", "var2.1"}), 0);
  test.addVariables(std::vector<std::string>({"var1.2", "var2.2"}), 1);

  EXPECT_EQ(web::json::value("var2.1"), test.getVariables(0).at(1));
  EXPECT_EQ(web::json::value("var1.2"), test.getVariables(1).at(0));

  test.removeVariables(0);

  EXPECT_ANY_THROW(test.getVariables(0));
}

TEST(Is_safe, full)
{
  camunda::Topics test;

  EXPECT_FALSE(test.is_safe());

  test.addTopic("topicName", 42);

  EXPECT_TRUE(test.is_safe());
}

TEST(GetTopics, full)
{
  camunda::Topics test;

  EXPECT_ANY_THROW(test.getTopics());

  test.addTopic("topicName", 42);

  test.getTopics();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_topics.cpp */

