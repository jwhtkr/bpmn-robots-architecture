/**
 * @File: test_camunda_exception.cpp
 * @Date: 31 May 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the CamundaException object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"camunda_c_api/camunda_exception.hpp"

TEST(Constructor, default)
{
  camunda::CamundaException test;
}

TEST(Constructor, onHeap)
{
  camunda::CamundaException* test = new camunda::CamundaException;

  delete(test);
}

TEST(Constructor, normal)
{
  camunda::CamundaException test("errorMessage", "errorDetails");

  EXPECT_EQ(web::json::value("errorMessage"), test.getErrorMessage());
  EXPECT_EQ(web::json::value("errorDetails"), test.getErrorDetails());
}

TEST(What, full)
{
  camunda::CamundaException test;

  test.addErrorMessage("error message");

  std::cout << test.what() << std::endl;
}

TEST(GetErrorMessage, full)
{
  camunda::CamundaException test;

  EXPECT_EQ(web::json::value(), test.getErrorMessage());

  test.addErrorMessage("error");

  EXPECT_EQ(web::json::value("error"), test.getErrorMessage());
}

TEST(GetErrorDetails, full)
{
  camunda::CamundaException test;

  EXPECT_EQ(web::json::value(), test.getErrorDetails());

  test.addErrorDetails("error");

  EXPECT_EQ(web::json::value("error"), test.getErrorDetails());
}

TEST(AddErrorMessage, full)
{
  camunda::CamundaException test;

  EXPECT_EQ(web::json::value(), test.getErrorMessage());

  test.addErrorMessage("error");

  EXPECT_EQ(web::json::value("error"), test.getErrorMessage());

  test.addErrorMessage("error2");

  EXPECT_EQ(web::json::value("error2"), test.getErrorMessage());
}

TEST(AddErrorDetails, full)
{
  camunda::CamundaException test;

  EXPECT_EQ(web::json::value(), test.getErrorDetails());

  test.addErrorDetails("error");

  EXPECT_EQ(web::json::value("error"), test.getErrorDetails());

  test.addErrorDetails("error2");

  EXPECT_EQ(web::json::value("error2"), test.getErrorDetails());
}

TEST(CanBeThrownAndCaught, full)
{
  try
  {
    throw camunda::CamundaException("error");
  }
  catch(const camunda::CamundaException& e)
  {
    EXPECT_EQ(web::json::value("error"), e.getErrorMessage());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_camunda_exception.cpp */

