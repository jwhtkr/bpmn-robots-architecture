/**
 * @File: test_bpmn_error.cpp
 * @Date: 31 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the BpnmError object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"camunda_c_api/bpmn_error.hpp"

TEST(Constructor, stack)
{
  camunda::BpmnError();
}

TEST(Constructor, heap)
{
  camunda::BpmnError* test = new camunda::BpmnError();

  delete(test);
}

TEST(Constructor, full)
{
  camunda::Variables var;
  var.addVariable("varKey", web::json::value("varValue"), web::json::value("String"));

  camunda::BpmnError test("workerId", "errorCode", "errorMessage", var);

  EXPECT_EQ(web::json::value("workerId"),     test.getWorkerId());
  EXPECT_EQ(web::json::value("errorCode"),    test.getErrorCode());
  EXPECT_EQ(web::json::value("errorMessage"), test.getErrorMessage());
  EXPECT_EQ(var.get(),                        test.getVariables());
}

TEST(Add, workerId)
{
  camunda::BpmnError test;

  EXPECT_ANY_THROW(test.getWorkerId());

  test.addWorkerId("workerId");

  EXPECT_EQ(web::json::value("workerId"), test.getWorkerId());
}

TEST(Add, errorCode)
{
  camunda::BpmnError test;

  EXPECT_ANY_THROW(test.getErrorCode());

  test.addErrorCode("errorCode");

  EXPECT_EQ(web::json::value("errorCode"), test.getErrorCode());
}

TEST(Add, errorMessage)
{
  camunda::BpmnError test;

  EXPECT_ANY_THROW(test.getErrorCode());

  test.addErrorCode("errorCode");

  EXPECT_EQ(web::json::value("errorCode"), test.getErrorCode());
}

TEST(Add, variables)
{
  camunda::BpmnError test;
  camunda::Variables var;

  var.addVariable("varKey", web::json::value("varValue"), web::json::value("String"));

  EXPECT_ANY_THROW(test.getVariables());

  test.addVariables(var);

  EXPECT_EQ(var.get(), test.getVariables());
}

TEST(Add, bpmnException)
{
  camunda::BpmnError test;

  EXPECT_ANY_THROW(test.getErrorCode());
  EXPECT_ANY_THROW(test.getErrorMessage());

  test.addBpmnException(camunda::BpmnException("errorMessage", "errorCode"));

  EXPECT_EQ(web::json::value("errorMessage"), test.getErrorMessage());
  EXPECT_EQ(web::json::value("errorCode"),    test.getErrorCode());
}

TEST(Remove, workerId)
{
  camunda::BpmnError test("workerId");

  EXPECT_EQ(web::json::value("workerId"), test.getWorkerId());

  test.removeWorkerId();

  EXPECT_ANY_THROW(test.getWorkerId());
}

TEST(Remove, errorCode)
{
  camunda::BpmnError test("workerId", "errorCode");

  EXPECT_EQ(web::json::value("errorCode"), test.getErrorCode());

  test.removeErrorCode();

  EXPECT_ANY_THROW(test.getErrorCode());
}

TEST(Remove, errorMessage)
{
  camunda::BpmnError test("workerId", "errorCode", "errorMessage");

  EXPECT_EQ(web::json::value("errorMessage"), test.getErrorMessage());

  test.removeErrorMessage();

  EXPECT_ANY_THROW(test.getErrorMessage());
}

TEST(Remove, variables)
{
  camunda::Variables var;
  var.addVariable("varKey", web::json::value("varValue"), web::json::value("String"));

  camunda::BpmnError test("workerId", "errorCode", "errorMessage", var);

  EXPECT_EQ(var.get(), test.getVariables());

  test.removeVariables();

  EXPECT_ANY_THROW(test.getVariables());
}

TEST(Is_safe, full)
{
  camunda::BpmnError test;

  EXPECT_FALSE(test.is_safe());

  test.addBpmnException(camunda::BpmnException("errorMessage", "errorCode"));
  test.addWorkerId("workerId");

  EXPECT_TRUE(test.is_safe());
}

TEST(GetError, full)
{
  camunda::BpmnError test;

  EXPECT_ANY_THROW(test.getError());

  test.addBpmnException(camunda::BpmnException("errorMessage", "errorCode"));
  test.addWorkerId("workerId");

  test.getError();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_bpmn_error.cpp */

