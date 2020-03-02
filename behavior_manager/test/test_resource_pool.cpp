/**
 * @File: test_resource_pool.cpp
 * @Date: 17 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the ResourcePool object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"behavior_manager/resource_pool.hpp"

architecture_msgs::Resource mkResource()
{
  architecture_msgs::Resource output;

  output.name     = "name";
  output.category = "category";
  output.type     = "type";
  output.required = true;
  output.priority = 42;

  return output;
}

architecture_msgs::Role mkRole()
{
  architecture_msgs::Role output;

  output.name     = "roleName";
  output.required = true;
  output.priority = 23;
  output.resources.push_back(mkResource());

  return output;
}

std::vector<architecture_msgs::Role> mkRoleList()
{
  std::vector<architecture_msgs::Role> output;

  architecture_msgs::Role role = mkRole();

  role.name = "name1";
  output.push_back(role);

  role.name = "name2";
  output.push_back(role);

  role.name = "name3";
  output.push_back(role);

  return output;
}

TEST(Constructor, stack)
{
  ResourcePool test;
}

TEST(Constructor, heap)
{
  ResourcePool* test = new ResourcePool();

  delete(test);
}

/*

TEST(Constructor, full)
{
  ResourcePool test(mkRoleList());

  EXPECT_EQ("name1", test.cgetRole(0)      .name.get());
  EXPECT_EQ("name1", test.cgetRole("name1").name.get());
  EXPECT_EQ("name1", test.getRole(0)       .name.get());
  EXPECT_EQ("name1", test.getRole("name1") .name.get());
}

TEST(Role, add)
{
  ResourcePool test;

  test.addRole(mkRole());

  EXPECT_EQ("roleName", test.getRole(0).name.get());
}

TEST(Role, remove)
{
  ResourcePool test(mkRoleList());

  test.removeRole("name1");
  test.removeRole(0);

  EXPECT_EQ("name3", test.getRole(0).name.get());
}

TEST(Roles, received)
{
  ResourcePool test(mkRoleList());

  EXPECT_FALSE(test.hasAll());

  test.rolesReceived(mkRoleList());

  EXPECT_TRUE(test.hasAll());
}

TEST(Roles, giveup)
{
  ResourcePool test(mkRoleList());

  test.rolesReceived(mkRoleList());

  EXPECT_TRUE(test.hasAll());

  test.giveupRoles(mkRoleList());

  EXPECT_FALSE(test.hasAll());
}

TEST(Get, needed)
{
  ResourcePool test(mkRoleList());

  EXPECT_EQ(mkRoleList().size(), test.getNeeded()->size());

  test.rolesReceived(mkRoleList());

  EXPECT_EQ(0, test.getNeeded()->size());
}

*/

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_resource_pool.cpp */
