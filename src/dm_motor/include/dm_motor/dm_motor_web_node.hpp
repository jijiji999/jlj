#ifndef DM_MOTOR__DM_MOTOR_WEB_NODE_HPP_
#define DM_MOTOR__DM_MOTOR_WEB_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "dm_motor/dm_motor_manager.hpp"

namespace dm_motor
{

class DmMotorWebNode : public rclcpp::Node
{
public:
  DmMotorWebNode();
  ~DmMotorWebNode() override;

private:
  struct HttpRequest
  {
    std::string method;
    std::string path;
    std::string body;
  };

  struct HttpResponse
  {
    int status_code {200};
    std::string content_type {"application/json; charset=utf-8"};
    std::string body;
  };

  std::string resolve_config_path(const std::string & configured_path) const;
  std::string resolve_web_root(const std::string & configured_path) const;
  void start_http_server();
  void stop_http_server();
  void http_server_loop();
  void process_client(int client_fd);
  HttpRequest parse_request(const std::string & raw_request) const;
  HttpResponse route_request(const HttpRequest & request);
  HttpResponse serve_static_file(const std::string & path) const;
  HttpResponse handle_api_motors();
  HttpResponse handle_api_states();
  HttpResponse handle_api_enable(const std::string & body);
  HttpResponse handle_api_disable(const std::string & body);
  HttpResponse handle_api_clear_error(const std::string & body);
  HttpResponse handle_api_save_zero(const std::string & body);
  HttpResponse handle_api_mit(const std::string & body);
  HttpResponse make_json_response(const std::string & body, int status_code = 200) const;
  HttpResponse make_text_response(
    const std::string & body,
    int status_code,
    const std::string & content_type) const;
  std::string read_file(const std::string & path) const;
  std::string json_escape(const std::string & value) const;
  std::string extract_string_field(const std::string & body, const std::string & key) const;
  float extract_float_field(const std::string & body, const std::string & key) const;
  void write_response(int client_fd, const HttpResponse & response) const;
  std::string status_text(int status_code) const;

  std::unique_ptr<DmMotorManager> manager_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  std::string web_root_;
  std::string http_bind_host_ {"127.0.0.1"};
  int http_port_ {8080};
  int command_timeout_ms_ {20};
  int server_fd_ {-1};
  std::atomic<bool> stop_server_ {false};
  std::thread server_thread_;
};

}  // namespace dm_motor

#endif  // DM_MOTOR__DM_MOTOR_WEB_NODE_HPP_
