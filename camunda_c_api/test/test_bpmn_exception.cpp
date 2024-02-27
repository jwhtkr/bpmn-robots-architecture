/**
 * @File: test_bpmn_exception.cpp
 * @Date: 6 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the BpnmException object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"camunda_c_api/bpmn_exception.hpp"

TEST(Constructor, stack)
{
  camunda::BpmnException();
}

TEST(Cosntructor, heap)
{
  camunda::BpmnException* test = new camunda::BpmnException();

  delete(test);
}

TEST(Cosntructor, full)
{
  camunda::Variables var;
  var.addVariable("key", web::json::value("val"), web::json::value("String"));

  camunda::BpmnException test("errorMessage", "errorCode", var);

  EXPECT_EQ(web::json::value("errorMessage"), test.getErrorMessage());
  EXPECT_EQ(web::json::value("errorCode"),    test.getErrorCode());
  EXPECT_EQ(var.get(),                        test.getVariables());
}

TEST(What, full)
{
  camunda::BpmnException test;

  EXPECT_EQ(NULL, test.what());

  test.addErrorMessage("errorMessage");

  EXPECT_EQ(std::string("errorMessage"), std::string(test.what()));
}

TEST(Add, errorMessage)
{
  camunda::BpmnException test;

  EXPECT_EQ(web::json::value(), test.getErrorMessage());

  test.addErrorMessage("errorMessage");

  EXPECT_EQ(web::json::value("errorMessage"), test.getErrorMessage());
}

TEST(Add, errorCode)
{
  camunda::BpmnException test;

  EXPECT_EQ(web::json::value(), test.getErrorCode());

  test.addErrorCode("errorCode");

  EXPECT_EQ(web::json::value("errorCode"), test.getErrorCode());
}

TEST(Add, variables)
{
  camunda::Variables var;
  var.addVariable("key", web::json::value("val"), web::json::value("String"));

  camunda::BpmnException test;

  EXPECT_EQ(camunda::Variables().get(), test.getVariables());

  test.addVariables(var);

  EXPECT_EQ(var.get(), test.getVariables());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_bpmn_exception.cpp */

