/**
 * @File: test_task_lock.cpp
 * @Date: 3 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the TaskLock object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Camunda API Headers */
#include<camunda_c_api/camunda_c_api.hpp>

/* Local Headers */
#include"bpmn_interface/error_handler.hpp"

#define BASE_URI "http://localhost:8080/"

TEST(Constructor, stack)
{
  bpmn::ErrorHandler<>(BASE_URI, "task_id", "workerId");
  bpmn::ErrorHandler<>(BASE_URI, web::json::value("task_id"), web::json::value("workerId"));
}

TEST(Constructor, heap)
{
  bpmn::ErrorHandler<>* test = new bpmn::ErrorHandler<>(BASE_URI, "task_id", "workerId");

  delete(test);
}

TEST(Constructor, full)
{
  bpmn::ErrorHandler<> test(BASE_URI, "task_id", "workerId");

  EXPECT_EQ("task_id",                         test.getExternalTaskId());
  EXPECT_EQ(web::json::value("workerId"),      test.getWorkerId());
  EXPECT_EQ(camunda::BpmnError("workerId"),    test.cgetBpmnError());
  EXPECT_EQ(camunda::CamundaError("workerId"), test.cgetFailure());
}

TEST(Add, bpmnError)
{
  bpmn::ErrorHandler<> test(BASE_URI, "task_id", "workerId");

  EXPECT_EQ(camunda::BpmnError(), test.cgetBpmnError());

  test.addBpmnError(camunda::BpmnError("workerId"));

  EXPECT_EQ(camunda::BpmnError("workerId"), test.cgetBpmnError());
}

TEST(Add, failure)
{
  bpmn::ErrorHandler<> test(BASE_URI, "task_id", "workerId");

  EXPECT_EQ(camunda::CamundaError(), test.cgetFailure());

  test.addFailure(camunda::CamundaError("workerId"));

  EXPECT_EQ(camunda::CamundaError("workerId"), test.cgetFailure());
}

TEST(Add, workerId)
{
  bpmn::ErrorHandler<> test(BASE_URI, "task_id", "workerId");

  EXPECT_ANY_THROW(test.getWorkerId());

  test.addWorkerId("one");

  EXPECT_EQ("one", test.getWorkerId().as_string());

  test.addWorkerId(web::json::value("two"));

  EXPECT_EQ(web::json::value("two"), test.cgetBpmnError().getWorkerId());
  EXPECT_EQ(web::json::value("two"), test.cgetFailure()  .getWorkerId());
}

TEST(Add, externalTaskId)
{
  bpmn::ErrorHandler<> test(BASE_URI, "task_id", "workerId");

  EXPECT_EQ("task_id", test.getExternalTaskId());

  test.addExternalTaskId("one");

  EXPECT_EQ("one", test.getExternalTaskId());
}

TEST(Throw, bpmnError)
{
  std::cout << "Ready? Need at least 4 tasks ready." << std::endl;
  std::cin.get();

  camunda::LockRequest req("workerId", 4, camunda::Topics("Topic", 10));
  camunda::LockResponse res;

  web::http::client::http_client client(std::string(BASE_URI) + std::string("engine-rest/"));

  client.request(web::http::methods::POST, "external-task/fetchAndLock", req.cgetLockRequested())
    .then([&](const web::http::http_response& response)
    {
      res = camunda::LockResponse(response);
    }).wait();

  bpmn::ErrorHandler<> test(BASE_URI, res.getId(0).as_string(), "workerId");

  test.throwBpmnError();

  test.addExternalTaskId(res.getId(1).as_string());

  test.throwBpmnError(camunda::BpmnException("errorMessageThrown", "errorCodeThrown"));

  test.addExternalTaskId(res.getId(2).as_string());

  test.throwBpmnError("errorCodeOne", "errorMessageOne");

  test.addExternalTaskId(res.getId(3).as_string());

  test.throwBpmnError(web::json::value("errorCodeOne"), web::json::value("errorMessageOne"));
}

TEST(Throw, failure)
{
  std::cout << "Ready? Need at least 5 tasks ready." << std::endl;
  std::cin.get();

  camunda::LockRequest req("workerId", 5, camunda::Topics("Topic", 1000));
  camunda::LockResponse res;

  web::http::client::http_client client(std::string(BASE_URI) + std::string("engine-rest/"));

  client.request(web::http::methods::POST, "external-task/fetchAndLock", req.cgetLockRequested())
    .then([&](const web::http::http_response& response)
    {
      res = camunda::LockResponse(response);
    }).wait();

  bpmn::ErrorHandler<> test(BASE_URI, res.getId(0).as_string(), "workerId");

  test.throwFailure();

  test.addExternalTaskId(res.getId(1).as_string());

  test.throwFailure(camunda::CamundaException("errorMessageThrown"), 0);

  test.addExternalTaskId(res.getId(2).as_string());

  test.throwFailure(camunda::CamundaException("errorMessageThrownTwo"), web::json::value(0));

  test.addExternalTaskId(res.getId(3).as_string());

  test.throwFailure("errorCodeOne", "errorMessageOne");

  test.addExternalTaskId(res.getId(4).as_string());

  test.throwFailure(web::json::value("errorCodeOne"), web::json::value("errorMessageOne"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_error_handler.cpp */

