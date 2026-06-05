#include "dm_humanoid/ankle_parallel_web_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace dm_humanoid
{

namespace
{

std::string trim_leading_slash(const std::string & path)
{
  if (!path.empty() && path.front() == '/') {
    return path.substr(1);
  }
  return path;
}

std::string content_type_for_path(const std::string & path)
{
  if (path.size() >= 5U && path.substr(path.size() - 5U) == ".html") {
    return "text/html; charset=utf-8";
  }
  if (path.size() >= 3U && path.substr(path.size() - 3U) == ".js") {
    return "application/javascript; charset=utf-8";
  }
  if (path.size() >= 4U && path.substr(path.size() - 4U) == ".css") {
    return "text/css; charset=utf-8";
  }
  if (path.size() >= 5U && path.substr(path.size() - 5U) == ".json") {
    return "application/json; charset=utf-8";
  }
  return "text/plain; charset=utf-8";
}

}  // namespace

AnkleParallelWebNode::AnkleParallelWebNode()
: Node("ankle_parallel_web_node")
{
  this->declare_parameter<std::string>("motor_config_path", "");
  this->declare_parameter<std::string>("left_solver_config_path", "");
  this->declare_parameter<std::string>("right_solver_config_path", "");
  this->declare_parameter<std::string>("web_root", "");
  this->declare_parameter<std::string>("http_bind_host", "127.0.0.1");
  this->declare_parameter<int>("http_port", 8091);
  this->declare_parameter<int>("command_timeout_ms", 20);
  this->declare_parameter<int>("poll_interval_ms", 20);
  this->declare_parameter<int>("continuous_rate_hz", 20);

  const auto motor_config_path = resolve_motor_config_path(
    this->get_parameter("motor_config_path").as_string());
  const auto left_solver_config_path = resolve_solver_config_path(
    this->get_parameter("left_solver_config_path").as_string(),
    "left_ankle.yaml");
  const auto right_solver_config_path = resolve_solver_config_path(
    this->get_parameter("right_solver_config_path").as_string(),
    "right_ankle.yaml");
  web_root_ = resolve_web_root(this->get_parameter("web_root").as_string());
  http_bind_host_ = this->get_parameter("http_bind_host").as_string();
  http_port_ = this->get_parameter("http_port").as_int();
  command_timeout_ms_ = this->get_parameter("command_timeout_ms").as_int();
  poll_interval_ms_ = this->get_parameter("poll_interval_ms").as_int();
  continuous_rate_hz_ = this->get_parameter("continuous_rate_hz").as_int();

  left_rig_.side = "left";
  left_rig_.display_name = "Left Ankle";
  left_rig_.motor_1_name = "left_ankle_parallel_1";
  left_rig_.motor_2_name = "left_ankle_parallel_2";

  right_rig_.side = "right";
  right_rig_.display_name = "Right Ankle";
  right_rig_.motor_1_name = "right_ankle_parallel_1";
  right_rig_.motor_2_name = "right_ankle_parallel_2";

  manager_ = std::make_unique<dm_motor::DmMotorManager>(motor_config_path);

  std::string solver_error;
  if (!left_rig_.kinematics.load_from_yaml(left_solver_config_path, solver_error)) {
    throw std::runtime_error("Failed to load left ankle solver config: " + solver_error);
  }
  if (!right_rig_.kinematics.load_from_yaml(right_solver_config_path, solver_error)) {
    throw std::runtime_error("Failed to load right ankle solver config: " + solver_error);
  }

  poll_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_interval_ms_),
    [this]() {
      (void)update_feedback_cache();
    });

  continuous_command_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(std::max(1, 1000 / std::max(1, continuous_rate_hz_))),
    [this]() {
      std::string status_message;
      bool success = false;
      if (left_rig_.latest_command.continuous_enabled) {
        (void)send_parallel_command(left_rig_, left_rig_.latest_command, status_message, success);
      }
      if (left_rig_.latest_motor_command.continuous_enabled) {
        (void)send_motor_space_command(
          left_rig_, left_rig_.latest_motor_command, status_message, success);
      }
      if (right_rig_.latest_command.continuous_enabled) {
        (void)send_parallel_command(right_rig_, right_rig_.latest_command, status_message, success);
      }
      if (right_rig_.latest_motor_command.continuous_enabled) {
        (void)send_motor_space_command(
          right_rig_, right_rig_.latest_motor_command, status_message, success);
      }
    });
  continuous_command_timer_->cancel();

  start_http_server();

  RCLCPP_INFO(
    this->get_logger(),
    "ankle_parallel_web_node serving %s on http://%s:%d",
    web_root_.c_str(),
    http_bind_host_.c_str(),
    http_port_);
}

AnkleParallelWebNode::~AnkleParallelWebNode()
{
  stop_http_server();
}

std::string AnkleParallelWebNode::resolve_motor_config_path(const std::string & configured_path) const
{
  if (!configured_path.empty()) {
    return configured_path;
  }
  const auto share_dir = ament_index_cpp::get_package_share_directory("dm_motor");
  return share_dir + "/config/dm_motor_29.yaml";
}

std::string AnkleParallelWebNode::resolve_solver_config_path(
  const std::string & configured_path,
  const std::string & default_filename) const
{
  if (!configured_path.empty()) {
    return configured_path;
  }
  const auto share_dir = ament_index_cpp::get_package_share_directory("dm_humanoid");
  return share_dir + "/config/parallel/" + default_filename;
}

std::string AnkleParallelWebNode::resolve_web_root(const std::string & configured_path) const
{
  if (!configured_path.empty()) {
    return configured_path;
  }
  const auto share_dir = ament_index_cpp::get_package_share_directory("dm_humanoid");
  return share_dir + "/web/ankle_parallel";
}

void AnkleParallelWebNode::start_http_server()
{
  server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    throw std::runtime_error("Failed to create HTTP socket");
  }

  int opt = 1;
  if (::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    throw std::runtime_error("Failed to set SO_REUSEADDR");
  }

  sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(http_port_));
  if (::inet_pton(AF_INET, http_bind_host_.c_str(), &addr.sin_addr) != 1) {
    ::close(server_fd_);
    server_fd_ = -1;
    throw std::runtime_error("Invalid http_bind_host: " + http_bind_host_);
  }

  if (::bind(server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    const auto error = std::string("Failed to bind HTTP socket: ") + std::strerror(errno);
    ::close(server_fd_);
    server_fd_ = -1;
    throw std::runtime_error(error);
  }

  if (::listen(server_fd_, 16) < 0) {
    const auto error = std::string("Failed to listen on HTTP socket: ") + std::strerror(errno);
    ::close(server_fd_);
    server_fd_ = -1;
    throw std::runtime_error(error);
  }

  stop_server_ = false;
  server_thread_ = std::thread(&AnkleParallelWebNode::http_server_loop, this);
}

void AnkleParallelWebNode::stop_http_server()
{
  stop_server_ = true;

  if (server_fd_ >= 0) {
    ::shutdown(server_fd_, SHUT_RDWR);
    ::close(server_fd_);
    server_fd_ = -1;
  }

  if (server_thread_.joinable()) {
    server_thread_.join();
  }
}

void AnkleParallelWebNode::http_server_loop()
{
  while (!stop_server_) {
    const int client_fd = ::accept(server_fd_, nullptr, nullptr);
    if (client_fd < 0) {
      if (stop_server_) {
        break;
      }
      continue;
    }

    process_client(client_fd);
    ::close(client_fd);
  }
}

void AnkleParallelWebNode::process_client(const int client_fd)
{
  std::array<char, 32768> buffer {};
  const ssize_t bytes_read = ::read(client_fd, buffer.data(), buffer.size() - 1U);
  if (bytes_read <= 0) {
    return;
  }

  const std::string raw_request(buffer.data(), static_cast<size_t>(bytes_read));
  HttpResponse response;
  try {
    const auto request = parse_request(raw_request);
    response = route_request(request);
  } catch (const std::exception & error) {
    response = make_json_response(
      "{\"success\":false,\"message\":\"" + json_escape(error.what()) + "\"}",
      400);
  }

  write_response(client_fd, response);
}

AnkleParallelWebNode::HttpRequest AnkleParallelWebNode::parse_request(
  const std::string & raw_request) const
{
  HttpRequest request;

  const auto header_end = raw_request.find("\r\n\r\n");
  const auto header_block = raw_request.substr(0, header_end);
  const auto body = header_end == std::string::npos ? std::string() : raw_request.substr(header_end + 4U);

  std::istringstream stream(header_block);
  std::string request_line;
  std::getline(stream, request_line);
  if (!request_line.empty() && request_line.back() == '\r') {
    request_line.pop_back();
  }

  std::istringstream request_line_stream(request_line);
  request_line_stream >> request.method >> request.path;
  request.body = body;

  if (request.method.empty() || request.path.empty()) {
    throw std::runtime_error("Malformed HTTP request");
  }

  return request;
}

AnkleParallelWebNode::HttpResponse AnkleParallelWebNode::route_request(const HttpRequest & request)
{
  if (request.method == "GET" && request.path == "/api/state") {
    return handle_api_state();
  }
  if (request.method == "POST" && request.path == "/api/enable") {
    return handle_api_enable(request.body);
  }
  if (request.method == "POST" && request.path == "/api/disable") {
    return handle_api_disable(request.body);
  }
  if (request.method == "POST" && request.path == "/api/solve") {
    return handle_api_solve(request.body);
  }
  if (request.method == "POST" && request.path == "/api/send") {
    return handle_api_send(request.body);
  }
  if (request.method == "POST" && request.path == "/api/command") {
    return handle_api_command(request.body);
  }
  if (request.method == "POST" && request.path == "/api/stop") {
    return handle_api_stop(request.body);
  }

  if (request.method == "GET") {
    return serve_static_file(request.path);
  }

  return make_json_response("{\"success\":false,\"message\":\"Not found\"}", 404);
}

AnkleParallelWebNode::HttpResponse AnkleParallelWebNode::serve_static_file(
  const std::string & path) const
{
  std::string relative_path = trim_leading_slash(path);
  if (relative_path.empty()) {
    relative_path = "index.html";
  }

  if (relative_path.find("..") != std::string::npos) {
    return make_text_response("Forbidden", 403, "text/plain; charset=utf-8");
  }

  const auto full_path = web_root_ + "/" + relative_path;
  std::ifstream file(full_path, std::ios::binary);
  if (!file.good()) {
    return make_text_response("Not found", 404, "text/plain; charset=utf-8");
  }

  std::ostringstream oss;
  oss << file.rdbuf();
  return make_text_response(oss.str(), 200, content_type_for_path(relative_path));
}

AnkleParallelWebNode::HttpResponse AnkleParallelWebNode::handle_api_state()
{
  auto serialize_rig = [this](const AnkleRig & rig) {
    double pitch = 0.0;
    double roll = 0.0;
    double motor_1_position = 0.0;
    double motor_2_position = 0.0;
    double motor_1_velocity = 0.0;
    double motor_2_velocity = 0.0;
    double motor_1_torque = 0.0;
    double motor_2_torque = 0.0;
    std::string status_message;
    bool forward_success = false;
    const bool success = snapshot_parallel_state(
      rig,
      pitch, roll,
      motor_1_position, motor_2_position,
      motor_1_velocity, motor_2_velocity,
      motor_1_torque, motor_2_torque,
      status_message,
      forward_success);

    std::ostringstream oss;
    oss << "{"
        << "\"success\":" << (success ? "true" : "false") << ","
        << "\"display_name\":\"" << json_escape(rig.display_name) << "\","
        << "\"message\":\"" << json_escape(status_message) << "\","
        << "\"forward_success\":" << (forward_success ? "true" : "false") << ","
        << "\"logical\":{"
        << "\"pitch\":" << pitch << ","
        << "\"roll\":" << roll
        << "},"
        << "\"motors\":{"
        << "\"motor_1\":{"
        << "\"name\":\"" << json_escape(rig.motor_1_name) << "\","
        << "\"position\":" << motor_1_position << ","
        << "\"velocity\":" << motor_1_velocity << ","
        << "\"torque\":" << motor_1_torque
        << "},"
        << "\"motor_2\":{"
        << "\"name\":\"" << json_escape(rig.motor_2_name) << "\","
        << "\"position\":" << motor_2_position << ","
        << "\"velocity\":" << motor_2_velocity << ","
        << "\"torque\":" << motor_2_torque
        << "}"
        << "},"
        << "\"command\":{"
        << "\"pitch\":" << rig.latest_command.pitch << ","
        << "\"roll\":" << rig.latest_command.roll << ","
        << "\"kp\":" << rig.latest_command.kp << ","
        << "\"kd\":" << rig.latest_command.kd << ","
        << "\"continuous_enabled\":" << (rig.latest_command.continuous_enabled ? "true" : "false")
        << "},"
        << "\"motor_command\":{"
        << "\"motor_1_position\":" << rig.latest_motor_command.motor_1_position << ","
        << "\"motor_2_position\":" << rig.latest_motor_command.motor_2_position << ","
        << "\"kp\":" << rig.latest_motor_command.kp << ","
        << "\"kd\":" << rig.latest_motor_command.kd << ","
        << "\"continuous_enabled\":" << (rig.latest_motor_command.continuous_enabled ? "true" : "false")
        << "}"
        << "}";
    return oss.str();
  };

  const std::string body =
    "{\"success\":true,\"left\":" + serialize_rig(left_rig_) +
    ",\"right\":" + serialize_rig(right_rig_) + "}";
  return make_json_response(body, 200);
}

AnkleParallelWebNode::HttpResponse AnkleParallelWebNode::handle_api_enable(const std::string & body)
{
  const auto side = extract_string_field(body, "side");
  auto * rig = find_rig(side);
  if (rig == nullptr) {
    return make_json_response("{\"success\":false,\"message\":\"Unknown side\"}", 400);
  }

  const auto results = manager_->enable_motors(
    {rig->motor_1_name, rig->motor_2_name},
    std::chrono::milliseconds(command_timeout_ms_));

  bool success = true;
  std::ostringstream message;
  for (const auto & result : results) {
    success = success && result.success;
    if (message.tellp() > 0) {
      message << "; ";
    }
    message << result.name << ": " << result.message;
  }
  (void)update_feedback_cache();

  return make_json_response(
    "{\"success\":" + std::string(success ? "true" : "false") +
    ",\"message\":\"" + json_escape(message.str()) + "\"}",
    success ? 200 : 400);
}

AnkleParallelWebNode::HttpResponse AnkleParallelWebNode::handle_api_disable(const std::string & body)
{
  const auto side = extract_string_field(body, "side");
  auto * rig = find_rig(side);
  if (rig == nullptr) {
    return make_json_response("{\"success\":false,\"message\":\"Unknown side\"}", 400);
  }

  rig->latest_command.continuous_enabled = false;
  rig->latest_motor_command.continuous_enabled = false;
  if (
    !left_rig_.latest_command.continuous_enabled &&
    !left_rig_.latest_motor_command.continuous_enabled &&
    !right_rig_.latest_command.continuous_enabled &&
    !right_rig_.latest_motor_command.continuous_enabled)
  {
    continuous_command_timer_->cancel();
  }

  const auto results = manager_->disable_motors(
    {rig->motor_1_name, rig->motor_2_name},
    std::chrono::milliseconds(command_timeout_ms_));

  bool success = true;
  std::ostringstream message;
  for (const auto & result : results) {
    success = success && result.success;
    if (message.tellp() > 0) {
      message << "; ";
    }
    message << result.name << ": " << result.message;
  }
  (void)update_feedback_cache();

  return make_json_response(
    "{\"success\":" + std::string(success ? "true" : "false") +
    ",\"message\":\"" + json_escape(message.str()) + "\"}",
    success ? 200 : 400);
}

AnkleParallelWebNode::HttpResponse AnkleParallelWebNode::handle_api_solve(const std::string & body)
{
  const auto side = extract_string_field(body, "side");
  auto * rig = find_rig(side);
  if (rig == nullptr) {
    return make_json_response("{\"success\":false,\"message\":\"Unknown side\"}", 400);
  }

  const auto command_mode = extract_string_field(body, "command_mode");
  double pitch = 0.0;
  double roll = 0.0;
  double motor_1_position = 0.0;
  double motor_2_position = 0.0;
  std::string message;
  bool success = false;

  if (command_mode == "joint") {
    pitch = extract_float_field(body, "pitch");
    roll = extract_float_field(body, "roll");
    success = rig->kinematics.inverse(
      pitch, roll, motor_1_position, motor_2_position, message);
    if (!success) {
      message = rig->display_name + " inverse solve failed: " + message;
    } else {
      message = rig->display_name + " inverse solve success";
    }
  } else if (command_mode == "motor") {
    motor_1_position = extract_float_field(body, "motor_1_position");
    motor_2_position = extract_float_field(body, "motor_2_position");
    success = rig->kinematics.forward(
      motor_1_position,
      motor_2_position,
      pitch,
      roll,
      message,
      rig->latest_command.pitch,
      rig->latest_command.roll);
    if (!success) {
      message = rig->display_name + " forward solve failed: " + message;
    } else {
      message = rig->display_name + " forward solve success";
    }
  } else {
    return make_json_response(
      "{\"success\":false,\"message\":\"Unsupported command_mode\"}",
      400);
  }

  std::ostringstream oss;
  oss << "{"
      << "\"success\":" << (success ? "true" : "false") << ","
      << "\"message\":\"" << json_escape(message) << "\","
      << "\"side\":\"" << json_escape(side) << "\","
      << "\"command_mode\":\"" << json_escape(command_mode) << "\","
      << "\"logical\":{"
      << "\"pitch\":" << pitch << ","
      << "\"roll\":" << roll
      << "},"
      << "\"motors\":{"
      << "\"motor_1_position\":" << motor_1_position << ","
      << "\"motor_2_position\":" << motor_2_position
      << "}"
      << "}";
  return make_json_response(oss.str(), success ? 200 : 400);
}

AnkleParallelWebNode::HttpResponse AnkleParallelWebNode::handle_api_send(const std::string & body)
{
  const auto side = extract_string_field(body, "side");
  auto * rig = find_rig(side);
  if (rig == nullptr) {
    return make_json_response("{\"success\":false,\"message\":\"Unknown side\"}", 400);
  }

  MotorCommand command;
  command.motor_1_position = extract_float_field(body, "motor_1_position");
  command.motor_2_position = extract_float_field(body, "motor_2_position");
  command.kp = extract_float_field(body, "kp");
  command.kd = extract_float_field(body, "kd");
  command.continuous_enabled = false;
  rig->latest_motor_command = command;
  rig->latest_command.continuous_enabled = false;

  std::string status_message;
  bool success = false;
  success = send_motor_space_command(*rig, command, status_message, success);

  return make_json_response(
    "{\"success\":" + std::string(success ? "true" : "false") +
    ",\"message\":\"" + json_escape(status_message) + "\"}",
    success ? 200 : 400);
}

AnkleParallelWebNode::HttpResponse AnkleParallelWebNode::handle_api_command(const std::string & body)
{
  const auto side = extract_string_field(body, "side");
  auto * rig = find_rig(side);
  if (rig == nullptr) {
    return make_json_response("{\"success\":false,\"message\":\"Unknown side\"}", 400);
  }

  LogicalCommand command;
  const auto command_mode = extract_string_field(body, "command_mode");
  const float kp = extract_float_field(body, "kp");
  const float kd = extract_float_field(body, "kd");
  const bool continuous_enabled = extract_bool_field(body, "continuous");

  const int requested_rate = static_cast<int>(extract_float_field(body, "continuous_rate_hz"));
  if (requested_rate > 0 && requested_rate != continuous_rate_hz_) {
    continuous_rate_hz_ = requested_rate;
    continuous_command_timer_->cancel();
    continuous_command_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max(1, 1000 / std::max(1, continuous_rate_hz_))),
      [this]() {
        std::string status_message;
        bool success = false;
        if (left_rig_.latest_command.continuous_enabled) {
          (void)send_parallel_command(left_rig_, left_rig_.latest_command, status_message, success);
        }
        if (left_rig_.latest_motor_command.continuous_enabled) {
          (void)send_motor_space_command(
            left_rig_, left_rig_.latest_motor_command, status_message, success);
        }
        if (right_rig_.latest_command.continuous_enabled) {
          (void)send_parallel_command(right_rig_, right_rig_.latest_command, status_message, success);
        }
        if (right_rig_.latest_motor_command.continuous_enabled) {
          (void)send_motor_space_command(
            right_rig_, right_rig_.latest_motor_command, status_message, success);
        }
      });
  }

  std::string status_message;
  bool success = false;
  if (command_mode == "joint") {
    command.pitch = extract_float_field(body, "pitch");
    command.roll = extract_float_field(body, "roll");
    command.pitch_velocity = extract_float_field(body, "pitch_velocity");
    command.roll_velocity = extract_float_field(body, "roll_velocity");
    command.kp = kp;
    command.kd = kd;
    command.continuous_enabled = continuous_enabled;
    rig->latest_motor_command.continuous_enabled = false;
    rig->latest_command = command;
    success = send_parallel_command(*rig, rig->latest_command, status_message, success);
  } else if (command_mode == "motor") {
    MotorCommand motor_command;
    motor_command.motor_1_position = extract_float_field(body, "motor_1_position");
    motor_command.motor_2_position = extract_float_field(body, "motor_2_position");
    motor_command.kp = kp;
    motor_command.kd = kd;
    motor_command.continuous_enabled = continuous_enabled;
    rig->latest_command.continuous_enabled = false;
    rig->latest_motor_command = motor_command;
    success = send_motor_space_command(*rig, rig->latest_motor_command, status_message, success);
  } else {
    return make_json_response(
      "{\"success\":false,\"message\":\"Unsupported command_mode\"}",
      400);
  }

  if (
    success &&
    (rig->latest_command.continuous_enabled || rig->latest_motor_command.continuous_enabled))
  {
    continuous_command_timer_->reset();
  } else {
    rig->latest_command.continuous_enabled = false;
    rig->latest_motor_command.continuous_enabled = false;
    if (
      !left_rig_.latest_command.continuous_enabled &&
      !left_rig_.latest_motor_command.continuous_enabled &&
      !right_rig_.latest_command.continuous_enabled &&
      !right_rig_.latest_motor_command.continuous_enabled)
    {
      continuous_command_timer_->cancel();
    }
  }

  return make_json_response(
    "{\"success\":" + std::string(success ? "true" : "false") +
    ",\"message\":\"" + json_escape(status_message) + "\","
    "\"continuous_enabled\":" + std::string(rig->latest_command.continuous_enabled ? "true" : "false") + "}",
    success ? 200 : 400);
}

AnkleParallelWebNode::HttpResponse AnkleParallelWebNode::handle_api_stop(const std::string & body)
{
  const auto side = extract_string_field(body, "side");
  auto * rig = find_rig(side);
  if (rig == nullptr) {
    return make_json_response("{\"success\":false,\"message\":\"Unknown side\"}", 400);
  }

  rig->latest_command.continuous_enabled = false;
  rig->latest_motor_command.continuous_enabled = false;
  if (
    !left_rig_.latest_command.continuous_enabled &&
    !left_rig_.latest_motor_command.continuous_enabled &&
    !right_rig_.latest_command.continuous_enabled &&
    !right_rig_.latest_motor_command.continuous_enabled)
  {
    continuous_command_timer_->cancel();
  }

  return make_json_response(
    "{\"success\":true,\"message\":\"Continuous command stopped\",\"continuous_enabled\":false}",
    200);
}

AnkleParallelWebNode::HttpResponse AnkleParallelWebNode::make_json_response(
  const std::string & body,
  const int status_code) const
{
  return HttpResponse {status_code, "application/json; charset=utf-8", body};
}

AnkleParallelWebNode::HttpResponse AnkleParallelWebNode::make_text_response(
  const std::string & body,
  const int status_code,
  const std::string & content_type) const
{
  return HttpResponse {status_code, content_type, body};
}

std::string AnkleParallelWebNode::read_file(const std::string & path) const
{
  std::ifstream file(path, std::ios::binary);
  if (!file.good()) {
    throw std::runtime_error("Failed to read file: " + path);
  }
  std::ostringstream oss;
  oss << file.rdbuf();
  return oss.str();
}

std::string AnkleParallelWebNode::json_escape(const std::string & value) const
{
  std::ostringstream oss;
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        oss << "\\\\";
        break;
      case '"':
        oss << "\\\"";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        oss << ch;
        break;
    }
  }
  return oss.str();
}

std::string AnkleParallelWebNode::extract_string_field(
  const std::string & body,
  const std::string & key) const
{
  const auto pattern = "\"" + key + "\"";
  const auto key_pos = body.find(pattern);
  if (key_pos == std::string::npos) {
    throw std::runtime_error("Missing field: " + key);
  }

  const auto colon_pos = body.find(':', key_pos + pattern.size());
  const auto first_quote = body.find('"', colon_pos + 1U);
  const auto second_quote = body.find('"', first_quote + 1U);
  if (colon_pos == std::string::npos || first_quote == std::string::npos || second_quote == std::string::npos) {
    throw std::runtime_error("Invalid string field: " + key);
  }

  return body.substr(first_quote + 1U, second_quote - first_quote - 1U);
}

float AnkleParallelWebNode::extract_float_field(
  const std::string & body,
  const std::string & key) const
{
  const auto pattern = "\"" + key + "\"";
  const auto key_pos = body.find(pattern);
  if (key_pos == std::string::npos) {
    throw std::runtime_error("Missing field: " + key);
  }

  const auto colon_pos = body.find(':', key_pos + pattern.size());
  if (colon_pos == std::string::npos) {
    throw std::runtime_error("Invalid numeric field: " + key);
  }

  auto value_start = colon_pos + 1U;
  while (value_start < body.size() && std::isspace(static_cast<unsigned char>(body[value_start])) != 0) {
    ++value_start;
  }

  size_t value_end = value_start;
  while (value_end < body.size()) {
    const char ch = body[value_end];
    if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E') {
      ++value_end;
    } else {
      break;
    }
  }

  return std::stof(body.substr(value_start, value_end - value_start));
}

bool AnkleParallelWebNode::extract_bool_field(
  const std::string & body,
  const std::string & key) const
{
  const auto pattern = "\"" + key + "\"";
  const auto key_pos = body.find(pattern);
  if (key_pos == std::string::npos) {
    throw std::runtime_error("Missing field: " + key);
  }

  const auto colon_pos = body.find(':', key_pos + pattern.size());
  if (colon_pos == std::string::npos) {
    throw std::runtime_error("Invalid bool field: " + key);
  }

  auto value_start = colon_pos + 1U;
  while (value_start < body.size() && std::isspace(static_cast<unsigned char>(body[value_start])) != 0) {
    ++value_start;
  }

  if (body.compare(value_start, 4U, "true") == 0) {
    return true;
  }
  if (body.compare(value_start, 5U, "false") == 0) {
    return false;
  }

  throw std::runtime_error("Invalid bool field: " + key);
}

void AnkleParallelWebNode::write_response(
  const int client_fd,
  const HttpResponse & response) const
{
  std::ostringstream oss;
  oss << "HTTP/1.1 " << response.status_code << " " << status_text(response.status_code) << "\r\n"
      << "Content-Type: " << response.content_type << "\r\n"
      << "Content-Length: " << response.body.size() << "\r\n"
      << "Connection: close\r\n"
      << "Cache-Control: no-store\r\n"
      << "\r\n"
      << response.body;

  const auto text = oss.str();
  (void)::write(client_fd, text.data(), text.size());
}

std::string AnkleParallelWebNode::status_text(const int status_code) const
{
  switch (status_code) {
    case 200:
      return "OK";
    case 400:
      return "Bad Request";
    case 403:
      return "Forbidden";
    case 404:
      return "Not Found";
    default:
      return "Error";
  }
}

bool AnkleParallelWebNode::send_parallel_command(
  AnkleRig & rig,
  const LogicalCommand & command,
  std::string & status_message,
  bool & success)
{
  double motor_1_position = 0.0;
  double motor_2_position = 0.0;
  std::string solve_error;
  if (!rig.kinematics.inverse(
      command.pitch,
      command.roll,
      motor_1_position,
      motor_2_position,
      solve_error))
  {
    success = false;
    status_message = rig.display_name + " inverse solve failed: " + solve_error;
    return false;
  }

  double motor_1_velocity = 0.0;
  double motor_2_velocity = 0.0;
  if (!rig.kinematics.joint_to_motor_velocities(
      command.pitch,
      command.roll,
      motor_1_position,
      motor_2_position,
      command.pitch_velocity,
      command.roll_velocity,
      motor_1_velocity,
      motor_2_velocity,
      solve_error))
  {
    success = false;
    status_message = rig.display_name + " velocity mapping failed: " + solve_error;
    return false;
  }

  std::vector<dm_motor::NamedMitCommand> commands;
  dm_motor::MitCommand motor_1_command;
  motor_1_command.position = static_cast<float>(motor_1_position);
  motor_1_command.velocity = static_cast<float>(motor_1_velocity);
  motor_1_command.kp = command.kp;
  motor_1_command.kd = command.kd;
  motor_1_command.torque = 0.0F;
  commands.push_back(dm_motor::NamedMitCommand {rig.motor_1_name, motor_1_command});

  dm_motor::MitCommand motor_2_command;
  motor_2_command.position = static_cast<float>(motor_2_position);
  motor_2_command.velocity = static_cast<float>(motor_2_velocity);
  motor_2_command.kp = command.kp;
  motor_2_command.kd = command.kd;
  motor_2_command.torque = 0.0F;
  commands.push_back(dm_motor::NamedMitCommand {rig.motor_2_name, motor_2_command});

  const auto results = manager_->command_group_mit(
    commands,
    std::chrono::milliseconds(command_timeout_ms_));

  success = true;
  std::ostringstream message;
  message << rig.display_name << " pitch=" << command.pitch << ", roll=" << command.roll
          << " => motor1=" << motor_1_position << ", motor2=" << motor_2_position;
  for (const auto & result : results) {
    success = success && result.success;
    message << " | " << result.name << ": " << result.message;
  }
  status_message = message.str();
  (void)update_feedback_cache();
  return success;
}

bool AnkleParallelWebNode::send_motor_space_command(
  AnkleRig & rig,
  const MotorCommand & command,
  std::string & status_message,
  bool & success)
{
  double pitch = 0.0;
  double roll = 0.0;
  std::string solve_error;
  if (!rig.kinematics.forward(
      command.motor_1_position,
      command.motor_2_position,
      pitch,
      roll,
      solve_error,
      rig.latest_command.pitch,
      rig.latest_command.roll))
  {
    success = false;
    status_message = rig.display_name + " forward solve failed: " + solve_error;
    return false;
  }

  rig.latest_command.pitch = static_cast<float>(pitch);
  rig.latest_command.roll = static_cast<float>(roll);
  rig.latest_command.kp = command.kp;
  rig.latest_command.kd = command.kd;

  std::vector<dm_motor::NamedMitCommand> commands;
  dm_motor::MitCommand motor_1_command;
  motor_1_command.position = command.motor_1_position;
  motor_1_command.velocity = 0.0F;
  motor_1_command.kp = command.kp;
  motor_1_command.kd = command.kd;
  motor_1_command.torque = 0.0F;
  commands.push_back(dm_motor::NamedMitCommand {rig.motor_1_name, motor_1_command});

  dm_motor::MitCommand motor_2_command;
  motor_2_command.position = command.motor_2_position;
  motor_2_command.velocity = 0.0F;
  motor_2_command.kp = command.kp;
  motor_2_command.kd = command.kd;
  motor_2_command.torque = 0.0F;
  commands.push_back(dm_motor::NamedMitCommand {rig.motor_2_name, motor_2_command});

  const auto results = manager_->command_group_mit(
    commands,
    std::chrono::milliseconds(command_timeout_ms_));

  success = true;
  std::ostringstream message;
  message << rig.display_name
          << " motor1=" << command.motor_1_position
          << ", motor2=" << command.motor_2_position
          << " => pitch=" << pitch << ", roll=" << roll;
  for (const auto & result : results) {
    success = success && result.success;
    message << " | " << result.name << ": " << result.message;
  }
  status_message = message.str();
  (void)update_feedback_cache();
  return success;
}

bool AnkleParallelWebNode::snapshot_parallel_state(
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
  bool & forward_success)
{
  (void)update_feedback_cache();

  const auto motor_1_it = motor_state_cache_.find(rig.motor_1_name);
  const auto motor_2_it = motor_state_cache_.find(rig.motor_2_name);
  if (motor_1_it == motor_state_cache_.end() || motor_2_it == motor_state_cache_.end()) {
    status_message = rig.display_name + ": waiting for both motor feedback frames";
    return false;
  }

  motor_1_position = motor_1_it->second.position;
  motor_2_position = motor_2_it->second.position;
  motor_1_velocity = motor_1_it->second.velocity;
  motor_2_velocity = motor_2_it->second.velocity;
  motor_1_torque = motor_1_it->second.torque;
  motor_2_torque = motor_2_it->second.torque;

  std::string solve_error;
  if (!rig.kinematics.forward(
      motor_1_position,
      motor_2_position,
      pitch,
      roll,
      solve_error,
      rig.latest_command.pitch,
      rig.latest_command.roll))
  {
    forward_success = false;
    status_message = rig.display_name + ": forward solve failed: " + solve_error;
    return false;
  }

  forward_success = true;
  status_message = rig.display_name + ": feedback OK";
  return true;
}

bool AnkleParallelWebNode::update_feedback_cache()
{
  (void)manager_->poll_once(std::chrono::milliseconds(0));
  const auto states = manager_->snapshot_states();
  for (const auto & state : states) {
    motor_state_cache_[state.name] = state;
  }
  return true;
}

AnkleParallelWebNode::AnkleRig * AnkleParallelWebNode::find_rig(const std::string & side)
{
  if (side == "left") {
    return &left_rig_;
  }
  if (side == "right") {
    return &right_rig_;
  }
  return nullptr;
}

const AnkleParallelWebNode::AnkleRig * AnkleParallelWebNode::find_rig(const std::string & side) const
{
  if (side == "left") {
    return &left_rig_;
  }
  if (side == "right") {
    return &right_rig_;
  }
  return nullptr;
}

}  // namespace dm_humanoid
