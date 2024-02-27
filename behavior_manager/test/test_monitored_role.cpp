/**
 * @File: test_monitored_role.cpp
 * @Date: 17 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the MonitoredRole object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"behavior_manager/monitored_role.hpp"

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

MonitoredRole mkRole()
{
  MonitoredRole output;

  output.name.get()     = "roleName";
  output.required.get() = true;
  output.priority.get() = 23;
  output.resources.push_back(mkResource());

  return output;
}

TEST(Constructor, stack)
{
  MonitoredRole test;
}

TEST(Constructor, heap)
{
  MonitoredRole* test = new MonitoredRole();

  delete(test);
}

TEST(Cosntructor, full)
{
  MonitoredRole test(mkRole());

  EXPECT_EQ("roleName", test.name.get());
  EXPECT_TRUE(          test.required.get());
  EXPECT_EQ(23,         test.priority.get());
  EXPECT_EQ("name",     test.resources.at(0).name.get());
}

TEST(Constructor, copy)
{
  MonitoredRole one(mkRole());
  MonitoredRole test(one);

  EXPECT_EQ("roleName", test.name.get());
  EXPECT_TRUE(          test.required.get());
  EXPECT_EQ(23,         test.priority.get());
  EXPECT_EQ("name",     test.resources.at(0).name.get());
}

TEST(Constructor, move)
{
  MonitoredRole one(mkRole());
  MonitoredRole test(std::move(one));

  EXPECT_EQ("roleName", test.name.get());
  EXPECT_TRUE(          test.required.get());
  EXPECT_EQ(23,         test.priority.get());
  EXPECT_EQ("name",     test.resources.at(0).name.get());
}

TEST(Operator, CopyAssign)
{
  MonitoredRole one(mkRole());
  MonitoredRole test;

  test = one;

  EXPECT_EQ("roleName", test.name.get());
  EXPECT_TRUE(          test.required.get());
  EXPECT_EQ(23,         test.priority.get());
  EXPECT_EQ("name",     test.resources.at(0).name.get());
}

TEST(Operator, MoveAssign)
{
  MonitoredRole one(mkRole());
  MonitoredRole test;

  test = std::move(one);

  EXPECT_EQ("roleName", test.name.get());
  EXPECT_TRUE(          test.required.get());
  EXPECT_EQ(23,         test.priority.get());
  EXPECT_EQ("name",     test.resources.at(0).name.get());
}

TEST(Operator, compare)
{
  MonitoredRole one( mkRole());
  MonitoredRole test(mkRole());

  EXPECT_TRUE( one == test);
  EXPECT_FALSE(one != test);
}

/*

TEST(HasAll, full)
{
  MonitoredRole test;

  EXPECT_TRUE(test.hasAll());

  test.resources.push_back(mkResource());

  EXPECT_FALSE(test.hasAll());

  std::vector<architecture_msgs::Resource> resources;
  resources.push_back(mkResource());

  test.resourcesReceived(resources);

  EXPECT_TRUE(test.hasAll());
}

TEST(Get, Needed)
{
  architecture_msgs::Resource res(mkResource());
  res.required = false;
  res.name = "qwe";

  std::vector<architecture_msgs::Resource> resources;
  resources.push_back(mkResource());

  MonitoredRole test;

  EXPECT_EQ(architecture_msgs::Role().name, test.getNeeded()->name);

  test.addResource(res);
  test.resources.at(0).in_use = true;
  test.addResource(mkResource());

  EXPECT_EQ(mkRole().resources.size(), test.getNeeded()->resources.size());
}

*/

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_monitored_role.cpp */
