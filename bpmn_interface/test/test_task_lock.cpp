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

/* Local Headers */
#include"bpmn_interface/task_lock.hpp"

/* C++ Headers */
#include<thread>
#include<chrono>

#define BASE_URI "http://localhost:8080/"
#define CAMUNDA_TOPIC "Topic"

TEST(Constructor, stack)
{
  bpmn::TaskLock<> test(BASE_URI, "workerId", camunda::Topics(CAMUNDA_TOPIC, 10000));
}

TEST(Constructor, heap)
{
  bpmn::TaskLock<>* test = new bpmn::TaskLock<>(BASE_URI, "workerId", camunda::Topics(CAMUNDA_TOPIC, 10000));

  delete(test);
}

TEST(Constructor, full)
{
  bpmn::TaskLock<> test(BASE_URI, "workerId", camunda::Topics(CAMUNDA_TOPIC, 10000));

  EXPECT_EQ("workerId", test.getWorkerId().as_string());
  test.getTaskId();
  EXPECT_EQ(camunda::LockRequest("workerId", 1, camunda::Topics(CAMUNDA_TOPIC, 10000), false, 180000), test.getLockRequest());
  test.getLockResponse();
  test.getResponsVars();
  EXPECT_EQ(camunda::Variables(), test.getCompleteVars());
  EXPECT_EQ(camunda::CompleteRequest("workerId"), test.getCompleteRequest());
}

TEST(Constructor, locks)
{
  std::cout << "Ready?" << std::endl;
  std::cin.get();

  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task/count?notLocked=true")
    .then([](const web::http::http_response& response)
    {
      EXPECT_EQ(web::json::value(1), response.extract_json().get().at("count"));
    }).wait();

  bpmn::TaskLock<> test(BASE_URI, "workerId", camunda::Topics(CAMUNDA_TOPIC, 500), camunda::CompleteRequest(), 20);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  client.request(web::http::methods::GET, "/engine-rest/external-task/count?notLocked=true")
    .then([](const web::http::http_response& response)
    {
      EXPECT_EQ(web::json::value(0), response.extract_json().get().at("count"));
    }).wait();
}

TEST(Stop, extending)
{
  std::cout << "Ready?" << std::endl;
  std::cin.get();

  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task/count?notLocked=true")
    .then([](const web::http::http_response& response)
    {
      EXPECT_EQ(web::json::value(1), response.extract_json().get().at("count"));
    }).wait();

  bpmn::TaskLock<> test(BASE_URI, "workerId", camunda::Topics(CAMUNDA_TOPIC, 100000), camunda::CompleteRequest(), 50);

  client.request(web::http::methods::GET, "/engine-rest/external-task/count?notLocked=true")
    .then([](const web::http::http_response& response)
    {
      EXPECT_EQ(web::json::value(0), response.extract_json().get().at("count"));
    }).wait();

  test.stopExtending();

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  client.request(web::http::methods::GET, "/engine-rest/external-task/count?notLocked=true")
    .then([](const web::http::http_response& response)
    {
      EXPECT_EQ(web::json::value(1), response.extract_json().get().at("count"));
    }).wait();
}

TEST(Add, full)
{
  std::cout << "Ready?" << std::endl;
  std::cin.get();

  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task/count?notLocked=true")
    .then([](const web::http::http_response& response)
    {
      EXPECT_EQ(web::json::value(1), response.extract_json().get().at("count"));
    }).wait();

  bpmn::TaskLock<> test(BASE_URI, "workerId", camunda::Topics(CAMUNDA_TOPIC, 10000));

  EXPECT_EQ(camunda::CompleteRequest("workerId"), test.getCompleteRequest());

  test.addCompleteRequest(camunda::CompleteRequest("one"));

  EXPECT_EQ(camunda::CompleteRequest("one"), test.getCompleteRequest());
  EXPECT_EQ(camunda::Variables(),            test.getCompleteVars());

  camunda::Variables var_obj;
  var_obj.addVariable("key", web::json::value("val"), web::json::value("String"), web::json::value("val_info"));
  test.addVariables(var_obj);

  EXPECT_EQ(var_obj, test.getCompleteVars());

  test.addCompleteRequest(camunda::CompleteRequest("workerId"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_task_lock_node");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}

/* test_task_lock.cpp */

