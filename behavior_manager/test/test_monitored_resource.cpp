/**
 * @File: test_monitored_resource.cpp
 * @Date: 17 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the MonitoredResource object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"behavior_manager/monitored_resource.hpp"

MonitoredResource mkResource()
{
  MonitoredResource output;

  output.name.get()     = "name";
  output.category.get() = "category";
  output.type.get()     = "type";
  output.required.get() = true;
  output.priority.get() = 42;

  return output;
}

MonitoredResource resource(mkResource());

TEST(Constructor, stack)
{
  MonitoredResource test(resource);
}

TEST(Constructor, heap)
{
  MonitoredResource* test = new MonitoredResource(resource);

  delete(test);
}

TEST(Constructor, full)
{
  MonitoredResource test(resource);

  EXPECT_EQ("name",     test.name.get());
  EXPECT_EQ("category", test.category.get());
  EXPECT_EQ("type",     test.type.get());
  EXPECT_TRUE(          test.required.get());
  EXPECT_EQ(42,         test.priority.get());
}

TEST(Constructor, copy)
{
  MonitoredResource one(resource);
  MonitoredResource test(one);

  EXPECT_EQ("name",     test.name.get());
  EXPECT_EQ("category", test.category.get());
  EXPECT_EQ("type",     test.type.get());
  EXPECT_TRUE(          test.required.get());
  EXPECT_EQ(42,         test.priority.get());
}

TEST(Constructor, move)
{
  MonitoredResource one(resource);
  MonitoredResource test(std::move(one));

  EXPECT_EQ("name",     test.name.get());
  EXPECT_EQ("category", test.category.get());
  EXPECT_EQ("type",     test.type.get());
  EXPECT_TRUE(          test.required.get());
  EXPECT_EQ(42,         test.priority.get());
}

TEST(Operator, assign)
{
  architecture_msgs::Resource res;

  MonitoredResource one(resource);
  MonitoredResource test;

  EXPECT_EQ(test, res);

  test = std::move(one);

  EXPECT_EQ("name",     test.name.get());
  EXPECT_EQ("category", test.category.get());
  EXPECT_EQ("type",     test.type.get());
  EXPECT_TRUE(          test.required.get());
  EXPECT_EQ(42,         test.priority.get());
}

TEST(Operator, compare)
{
  MonitoredResource one(resource);
  MonitoredResource test(resource);

  EXPECT_TRUE( one == test);
  EXPECT_FALSE(one != test);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_monitored_resource.cpp */
