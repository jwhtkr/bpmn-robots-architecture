/**
 * @File: test_process_variable.cpp
 * @Date: 2 July 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the base ProcessVariable object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"bpmn_interface/process_variable.hpp"

#define BASE_URI "http://localhost:8080/"

TEST(Constructor, stack)
{
  camunda::ProcessVariable<int64_t>(BASE_URI, "p_instance_id", "var_name");
}

TEST(Constructor, heap)
{
  camunda::ProcessVariable<long>* test = new camunda::ProcessVariable<long>(BASE_URI, "p_instance_id", "var_name");

  delete(test);
}

TEST(Constructor, full)
{
  camunda::ProcessVariable<long> test(BASE_URI, "p_instance_id", "var_name");

  EXPECT_EQ(std::string(BASE_URI) + std::string("engine-rest/process-instance/p_instance_id/variables/var_name"), test.getFullUri());
  EXPECT_EQ(std::string("var_name"), test.getName());
}

TEST(Float, get)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<float> test(BASE_URI, id, "float_var");

  EXPECT_EQ(std::floor(0.42), std::floor(test.get()));
}

TEST(Float, update)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<float> test(BASE_URI, id, "float_var2");

  EXPECT_ANY_THROW(test.get());

  test.update(42.76);

  EXPECT_EQ(std::floor(42.76), std::floor(test.get()));
}

TEST(Float, remove)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<float> test(BASE_URI, id, "float_var2");

  EXPECT_EQ(std::floor(42.76), std::floor(test.get()));

  test.remove();

  EXPECT_ANY_THROW(test.get());
}

TEST(Int, get)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<int> test(BASE_URI, id, "int_var");

  EXPECT_EQ(42, test.get());
}

TEST(Int, update)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<int> test(BASE_URI, id, "int_var2");

  EXPECT_ANY_THROW(test.get());

  test.update(42);

  EXPECT_EQ(42, test.get());

  test.update(54);

  EXPECT_EQ(54, test.get());
}

TEST(Int, remove)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<int> test(BASE_URI, id, "int_var2");

  EXPECT_EQ(54, test.get());

  test.remove();

  EXPECT_ANY_THROW(test.get());
}

TEST(String, get)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<std::string> test(BASE_URI, id, "string_var");

  EXPECT_EQ(std::string("string_var_val"), test.get());
}

TEST(String, update)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<std::string> test(BASE_URI, id, "string_var2");

  EXPECT_ANY_THROW(test.get());

  test.update("boo");

  EXPECT_EQ(std::string("boo"), test.get());

  test.update("string_var2_val");

  EXPECT_EQ(std::string("string_var2_val"), test.get());
}

TEST(String, remove)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<std::string> test(BASE_URI, id, "string_var2");

  EXPECT_EQ(std::string("string_var2_val"), test.get());

  test.remove();

  EXPECT_ANY_THROW(test.get());
}

TEST(Bool, get)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<bool> test(BASE_URI, id, "bool_var");

  EXPECT_EQ(false, test.get());
}

TEST(Bool, update)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<bool> test(BASE_URI, id, "bool_var2");

  EXPECT_ANY_THROW(test.get());

  test.update(false);

  EXPECT_EQ(false, test.get());

  test.update(true);

  EXPECT_EQ(true, test.get());
}

TEST(Bool, remove)
{
  std::string id = std::string();
  web::http::client::http_client client(BASE_URI);
  client.request(web::http::methods::GET, "/engine-rest/external-task")
    .then([&id](const web::http::http_response& response)
    {
      id = response.extract_json().get().at(0).at("processInstanceId").as_string();
    }).wait();

  camunda::ProcessVariable<bool> test(BASE_URI, id, "bool_var2");

  EXPECT_EQ(true, test.get());

  test.remove();

  EXPECT_ANY_THROW(test.get());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/* test_process_variable.cpp */

