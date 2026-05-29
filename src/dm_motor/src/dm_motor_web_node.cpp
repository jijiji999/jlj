#include "dm_motor/dm_motor_web_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace dm_motor
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

DmMotorWebNode::DmMotorWebNode()
: Node("dm_motor_web_node")
{
  this->declare_parameter<std::string>("config_path", "");
  this->declare_parameter<std::string>("web_root", "");
  this->declare_parameter<std::string>("http_bind_host", "127.0.0.1");
  this->declare_parameter<int>("http_port", 8080);
  this->declare_parameter<int>("command_timeout_ms", 20);
  this->declare_parameter<int>("poll_interval_ms", 20);

  const auto config_path = resolve_config_path(this->get_parameter("config_path").as_string());
  web_root_ = resolve_web_root(this->get_parameter("web_root").as_string());
  http_bind_host_ = this->get_parameter("http_bind_host").as_string();
  http_port_ = this->get_parameter("http_port").as_int();
  command_timeout_ms_ = this->get_parameter("command_timeout_ms").as_int();
  const int poll_interval_ms = this->get_parameter("poll_interval_ms").as_int();

  manager_ = std::make_unique<DmMotorManager>(config_path);
  poll_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_interval_ms),
    [this]() {
      (void)manager_->poll_once(std::chrono::milliseconds(0));
    });

  start_http_server();

  RCLCPP_INFO(
    this->get_logger(),
    "dm_motor_web_node serving %s on http://%s:%d",
    web_root_.c_str(),
    http_bind_host_.c_str(),
    http_port_);
}

DmMotorWebNode::~DmMotorWebNode()
{
  stop_http_server();
}

std::string DmMotorWebNode::resolve_config_path(const std::string & configured_path) const
{
  if (!configured_path.empty()) {
    return configured_path;
  }

  const auto share_dir = ament_index_cpp::get_package_share_directory("dm_motor");
  return share_dir + "/config/dm_motor_29.yaml";
}

std::string DmMotorWebNode::resolve_web_root(const std::string & configured_path) const
{
  if (!configured_path.empty()) {
    return configured_path;
  }

  const auto share_dir = ament_index_cpp::get_package_share_directory("dm_motor");
  return share_dir + "/web";
}

void DmMotorWebNode::start_http_server()
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
  server_thread_ = std::thread(&DmMotorWebNode::http_server_loop, this);
}

void DmMotorWebNode::stop_http_server()
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

void DmMotorWebNode::http_server_loop()
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

void DmMotorWebNode::process_client(const int client_fd)
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

DmMotorWebNode::HttpRequest DmMotorWebNode::parse_request(const std::string & raw_request) const
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

DmMotorWebNode::HttpResponse DmMotorWebNode::route_request(const HttpRequest & request)
{
  if (request.method == "GET" && request.path == "/api/motors") {
    return handle_api_motors();
  }
  if (request.method == "GET" && request.path == "/api/states") {
    return handle_api_states();
  }
  if (request.method == "POST" && request.path == "/api/enable") {
    return handle_api_enable(request.body);
  }
  if (request.method == "POST" && request.path == "/api/disable") {
    return handle_api_disable(request.body);
  }
  if (request.method == "POST" && request.path == "/api/clear_error") {
    return handle_api_clear_error(request.body);
  }
  if (request.method == "POST" && request.path == "/api/save_zero") {
    return handle_api_save_zero(request.body);
  }
  if (request.method == "POST" && request.path == "/api/mit") {
    return handle_api_mit(request.body);
  }

  if (request.method == "GET") {
    return serve_static_file(request.path);
  }

  return make_json_response("{\"success\":false,\"message\":\"Not found\"}", 404);
}

DmMotorWebNode::HttpResponse DmMotorWebNode::serve_static_file(const std::string & path) const
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

DmMotorWebNode::HttpResponse DmMotorWebNode::handle_api_motors()
{
  std::ostringstream oss;
  const auto configs = manager_->motor_configs();
  const auto states = manager_->snapshot_states();

  std::unordered_map<std::string, MotorState> state_map;
  for (const auto & state : states) {
    state_map[state.name] = state;
  }

  oss << "{\"success\":true,\"motors\":[";
  for (size_t i = 0; i < configs.size(); ++i) {
    const auto & config = configs[i];
    const auto state_it = state_map.find(config.name);

    oss << "{"
        << "\"name\":\"" << json_escape(config.name) << "\","
        << "\"interface\":\"" << json_escape(config.interface_name) << "\","
        << "\"can_id\":" << config.can_id << ","
        << "\"master_id\":" << config.master_id << ","
        << "\"model\":\"" << json_escape(config.model) << "\","
        << "\"notes\":\"" << json_escape(config.notes) << "\","
        << "\"limits\":{"
        << "\"position_min\":" << config.limits.position_min << ","
        << "\"position_max\":" << config.limits.position_max << ","
        << "\"velocity_min\":" << config.limits.velocity_min << ","
        << "\"velocity_max\":" << config.limits.velocity_max << ","
        << "\"torque_min\":" << config.limits.torque_min << ","
        << "\"torque_max\":" << config.limits.torque_max << ","
        << "\"kp_min\":" << config.limits.kp_min << ","
        << "\"kp_max\":" << config.limits.kp_max << ","
        << "\"kd_min\":" << config.limits.kd_min << ","
        << "\"kd_max\":" << config.limits.kd_max
        << "},"
        << "\"safety\":{"
        << "\"has_position_limit\":" << (config.has_position_limit ? "true" : "false") << ","
        << "\"upper_position_limit\":" << config.upper_position_limit << ","
        << "\"lower_position_limit\":" << config.lower_position_limit << ","
        << "\"max_output_torque\":" << config.max_output_torque << ","
        << "\"max_output_speed\":" << config.max_output_speed
        << "},"
        << "\"defaults\":{"
        << "\"kp\":" << config.default_kp << ","
        << "\"kd\":" << config.default_kd
        << "},";

    if (state_it != state_map.end()) {
      const auto & state = state_it->second;
      oss << "\"state\":{"
          << "\"valid\":true,"
          << "\"status\":\"" << json_escape(to_string(state.status)) << "\","
          << "\"position\":" << state.position << ","
          << "\"velocity\":" << state.velocity << ","
          << "\"torque\":" << state.torque << ","
          << "\"mos_temperature_c\":" << state.mos_temperature_c << ","
          << "\"rotor_temperature_c\":" << state.rotor_temperature_c
          << "}";
    } else {
      oss << "\"state\":{"
          << "\"valid\":false,"
          << "\"status\":\"unknown\","
          << "\"position\":0,"
          << "\"velocity\":0,"
          << "\"torque\":0,"
          << "\"mos_temperature_c\":0,"
          << "\"rotor_temperature_c\":0"
          << "}";
    }

    oss << "}";
    if (i + 1U < configs.size()) {
      oss << ",";
    }
  }
  oss << "]}";

  return make_json_response(oss.str(), 200);
}

DmMotorWebNode::HttpResponse DmMotorWebNode::handle_api_states()
{
  std::ostringstream oss;
  const auto configs = manager_->motor_configs();
  const auto states = manager_->snapshot_states();

  std::unordered_map<std::string, MotorState> state_map;
  for (const auto & state : states) {
    state_map[state.name] = state;
  }

  oss << "{\"success\":true,\"states\":[";
  for (size_t i = 0; i < configs.size(); ++i) {
    const auto & config = configs[i];
    const auto state_it = state_map.find(config.name);

    oss << "{"
        << "\"name\":\"" << json_escape(config.name) << "\",";

    if (state_it != state_map.end()) {
      const auto & state = state_it->second;
      oss << "\"state\":{"
          << "\"valid\":true,"
          << "\"status\":\"" << json_escape(to_string(state.status)) << "\","
          << "\"position\":" << state.position << ","
          << "\"velocity\":" << state.velocity << ","
          << "\"torque\":" << state.torque << ","
          << "\"mos_temperature_c\":" << state.mos_temperature_c << ","
          << "\"rotor_temperature_c\":" << state.rotor_temperature_c
          << "}";
    } else {
      oss << "\"state\":{"
          << "\"valid\":false,"
          << "\"status\":\"unknown\","
          << "\"position\":0,"
          << "\"velocity\":0,"
          << "\"torque\":0,"
          << "\"mos_temperature_c\":0,"
          << "\"rotor_temperature_c\":0"
          << "}";
    }

    oss << "}";
    if (i + 1U < configs.size()) {
      oss << ",";
    }
  }
  oss << "]}";

  return make_json_response(oss.str(), 200);
}

DmMotorWebNode::HttpResponse DmMotorWebNode::handle_api_enable(const std::string & body)
{
  const auto name = extract_string_field(body, "name");
  const auto result = manager_->enable_motor(name, std::chrono::milliseconds(command_timeout_ms_));
  return make_json_response(
    "{\"success\":" + std::string(result.success ? "true" : "false") +
    ",\"name\":\"" + json_escape(result.name) +
    "\",\"message\":\"" + json_escape(result.message) + "\"}",
    result.success ? 200 : 400);
}

DmMotorWebNode::HttpResponse DmMotorWebNode::handle_api_disable(const std::string & body)
{
  const auto name = extract_string_field(body, "name");
  const auto result = manager_->disable_motor(name, std::chrono::milliseconds(command_timeout_ms_));
  return make_json_response(
    "{\"success\":" + std::string(result.success ? "true" : "false") +
    ",\"name\":\"" + json_escape(result.name) +
    "\",\"message\":\"" + json_escape(result.message) + "\"}",
    result.success ? 200 : 400);
}

DmMotorWebNode::HttpResponse DmMotorWebNode::handle_api_clear_error(const std::string & body)
{
  const auto name = extract_string_field(body, "name");
  const auto result = manager_->clear_motor_error(name, std::chrono::milliseconds(command_timeout_ms_));
  return make_json_response(
    "{\"success\":" + std::string(result.success ? "true" : "false") +
    ",\"name\":\"" + json_escape(result.name) +
    "\",\"message\":\"" + json_escape(result.message) + "\"}",
    result.success ? 200 : 400);
}

DmMotorWebNode::HttpResponse DmMotorWebNode::handle_api_save_zero(const std::string & body)
{
  const auto name = extract_string_field(body, "name");
  const auto result = manager_->save_motor_zero(name, std::chrono::milliseconds(command_timeout_ms_));
  return make_json_response(
    "{\"success\":" + std::string(result.success ? "true" : "false") +
    ",\"name\":\"" + json_escape(result.name) +
    "\",\"message\":\"" + json_escape(result.message) + "\"}",
    result.success ? 200 : 400);
}

DmMotorWebNode::HttpResponse DmMotorWebNode::handle_api_mit(const std::string & body)
{
  const auto name = extract_string_field(body, "name");

  MitCommand command;
  command.position = extract_float_field(body, "position");
  command.velocity = extract_float_field(body, "velocity");
  command.kp = extract_float_field(body, "kp");
  command.kd = extract_float_field(body, "kd");
  command.torque = extract_float_field(body, "torque");

  const auto result = manager_->command_motor_mit(
    name,
    command,
    std::chrono::milliseconds(command_timeout_ms_));

  return make_json_response(
    "{\"success\":" + std::string(result.success ? "true" : "false") +
    ",\"name\":\"" + json_escape(result.name) +
    "\",\"message\":\"" + json_escape(result.message) + "\"}",
    result.success ? 200 : 400);
}

DmMotorWebNode::HttpResponse DmMotorWebNode::make_json_response(
  const std::string & body,
  const int status_code) const
{
  return HttpResponse {status_code, "application/json; charset=utf-8", body};
}

DmMotorWebNode::HttpResponse DmMotorWebNode::make_text_response(
  const std::string & body,
  const int status_code,
  const std::string & content_type) const
{
  return HttpResponse {status_code, content_type, body};
}

std::string DmMotorWebNode::read_file(const std::string & path) const
{
  std::ifstream file(path, std::ios::binary);
  if (!file.good()) {
    throw std::runtime_error("Failed to read file: " + path);
  }
  std::ostringstream oss;
  oss << file.rdbuf();
  return oss.str();
}

std::string DmMotorWebNode::json_escape(const std::string & value) const
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

std::string DmMotorWebNode::extract_string_field(
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

float DmMotorWebNode::extract_float_field(
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

void DmMotorWebNode::write_response(const int client_fd, const HttpResponse & response) const
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

std::string DmMotorWebNode::status_text(const int status_code) const
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

}  // namespace dm_motor
