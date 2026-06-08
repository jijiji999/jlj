#ifndef DM_HUMANOID__ANKLE_PARALLEL_WEB_NODE_HPP_
#define DM_HUMANOID__ANKLE_PARALLEL_WEB_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "dm_humanoid/parallel_kinematics.hpp"
#include "dm_motor/dm_motor_manager.hpp"

namespace dm_humanoid
{

class AnkleParallelWebNode : public rclcpp::Node
{
public:
  AnkleParallelWebNode();
  ~AnkleParallelWebNode() override;

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

  struct LogicalCommand
  {
    float pitch {0.0F};
    float roll {0.0F};
    float pitch_velocity {0.0F};
    float roll_velocity {0.0F};
    float kp {0.0F};
    float kd {0.0F};
    bool continuous_enabled {false};
  };

  struct MotorCommand
  {
    float motor_1_position {0.0F};
    float motor_2_position {0.0F};
    float kp {0.0F};
    float kd {0.0F};
    bool continuous_enabled {false};
  };

  struct AnkleRig
  {
    std::string side;
    std::string display_name;
    std::string motor_1_name;
    std::string motor_2_name;
    ParallelKinematics kinematics;
    LogicalCommand latest_command {};
    MotorCommand latest_motor_command {};
  };

  std::string resolve_motor_config_path(const std::string & configured_path) const;
  std::string resolve_solver_config_path(
    const std::string & configured_path,
    const std::string & default_filename) const;
  std::string resolve_web_root(const std::string & configured_path) const;

  void start_http_server();
  void stop_http_server();
  void http_server_loop();
  void process_client(int client_fd);
  HttpRequest parse_request(const std::string & raw_request) const;
  HttpResponse route_request(const HttpRequest & request);
  HttpResponse serve_static_file(const std::string & path) const;
  HttpResponse handle_api_state();
  HttpResponse handle_api_enable(const std::string & body);
  HttpResponse handle_api_disable(const std::string & body);
  HttpResponse handle_api_solve(const std::string & body);
  HttpResponse handle_api_send(const std::string & body);
  HttpResponse handle_api_command(const std::string & body);
  HttpResponse handle_api_stop(const std::string & body);
  HttpResponse make_json_response(const std::string & body, int status_code = 200) const;
  HttpResponse make_text_response(
    const std::string & body,
    int status_code,
    const std::string & content_type) const;
  std::string read_file(const std::string & path) const;
  std::string json_escape(const std::string & value) const;
  std::string extract_string_field(const std::string & body, const std::string & key) const;
  float extract_float_field(const std::string & body, const std::string & key) const;
  bool extract_bool_field(const std::string & body, const std::string & key) const;
  void write_response(int client_fd, const HttpResponse & response) const;
  std::string status_text(int status_code) const;

  bool send_parallel_command(
    AnkleRig & rig,
    const LogicalCommand & command,
    std::string & status_message,
    bool & success);
  bool send_motor_space_command(
    AnkleRig & rig,
    const MotorCommand & command,
    std::string & status_message,
    bool & success);
  bool snapshot_parallel_state(
    const AnkleRig & rig,
    double & pitch,
    double & roll,
    double & motor_1_position,
    double & motor_2_position,
    double & motor_1_velocity,
    double & motor_2_velocity,
    double & motor_1_torque,
    double & motor_2_torque,
    std::string & status_message,
    bool & forward_success);
  bool update_feedback_cache();
  AnkleRig * find_rig(const std::string & side);
  const AnkleRig * find_rig(const std::string & side) const;

  std::unique_ptr<dm_motor::DmMotorManager> manager_;
  AnkleRig waist_rig_ {};
  AnkleRig left_rig_ {};
  AnkleRig right_rig_ {};
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr continuous_command_timer_;
  std::unordered_map<std::string, dm_motor::MotorState> motor_state_cache_;
  std::string web_root_;
  std::string http_bind_host_ {"127.0.0.1"};
  int http_port_ {8091};
  int command_timeout_ms_ {20};
  int poll_interval_ms_ {20};
  int continuous_rate_hz_ {20};
  int server_fd_ {-1};
  std::atomic<bool> stop_server_ {false};
  std::thread server_thread_;
};

}  // namespace dm_humanoid

#endif  // DM_HUMANOID__ANKLE_PARALLEL_WEB_NODE_HPP_
