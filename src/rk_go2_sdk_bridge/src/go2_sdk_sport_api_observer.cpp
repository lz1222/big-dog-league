// Go2 Sport API DDS 的被动观察器。
//
// 该程序仅订阅 rt/api/<service>/request 与 response；不创建 Client、Publisher
// 或运动控制对象。api_id=0 表示观察全部 API，用于人工切换步态时还原真实
// 控制入口；service 只允许服务名字符，避免诊断参数变成任意 DDS topic。
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/internal/internal_idl_decl/Request_.hpp>
#include <unitree/robot/internal/internal_idl_decl/Response_.hpp>

namespace
{

constexpr const char* kRequestTopic = "rt/api/sport/request";
constexpr const char* kResponseTopic = "rt/api/sport/response";
volatile std::sig_atomic_t g_running = 1;

int64_t MonotonicNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

void SignalHandler(int)
{
  g_running = 0;
}

std::string Summary(const std::string& data)
{
  // 仅保留有限可打印摘要，防止诊断日志泄漏或膨胀为任意大 payload。
  constexpr size_t kLimit = 160;
  std::ostringstream output;
  const size_t count = std::min(data.size(), kLimit);
  for (size_t index = 0; index < count; ++index) {
    const unsigned char value = static_cast<unsigned char>(data[index]);
    output << (value >= 32 && value <= 126 ? static_cast<char>(value) : '.');
  }
  if (data.size() > kLimit) {
    output << "...";
  }
  return output.str();
}

bool IsSafeServiceName(const std::string& service_name)
{
  // Unitree service 名使用字母、数字和下划线；诊断工具只接受该最小集合。
  return !service_name.empty() && std::all_of(
      service_name.begin(), service_name.end(), [](unsigned char character) {
        return std::isalnum(character) != 0 || character == '_';
      });
}

std::string ApiTopic(const std::string& service_name, const char* direction)
{
  return "rt/api/" + service_name + "/" + direction;
}

class SportApiObserver
{
public:
  SportApiObserver(int64_t observed_api_id, const std::string& service_name)
  : observed_api_id_(observed_api_id), service_name_(service_name)
  {
    request_subscriber_.reset(new unitree::robot::ChannelSubscriber<
        unitree_api::msg::dds_::Request_>(ApiTopic(service_name_, "request")));
    response_subscriber_.reset(new unitree::robot::ChannelSubscriber<
        unitree_api::msg::dds_::Response_>(ApiTopic(service_name_, "response")));
    request_subscriber_->InitChannel(
        std::bind(&SportApiObserver::OnRequest, this, std::placeholders::_1), 1);
    response_subscriber_->InitChannel(
        std::bind(&SportApiObserver::OnResponse, this, std::placeholders::_1), 1);
  }

  void PrintSummary() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout << "SPORT_API_OBSERVER event=SUMMARY"
              << " service=" << service_name_
              << " api_filter=";
    if (observed_api_id_ == 0) {
      std::cout << "ALL";
    } else {
      std::cout << observed_api_id_;
    }
    std::cout
              << " request_count=" << request_count_
              << " response_count=" << response_count_;
    if (first_response_latency_ns_ >= 0) {
      std::cout << " first_request_to_response_ms=" << std::fixed
                << std::setprecision(3)
                << first_response_latency_ns_ / 1000000.0;
    }
    std::cout << std::endl;
  }

private:
  void OnRequest(const void* raw_message)
  {
    if (raw_message == nullptr) {
      return;
    }
    const auto& message = *static_cast<const unitree_api::msg::dds_::Request_*>(
        raw_message);
    const auto& header = message.header();
    const auto& identity = header.identity();
    // 0 是只读观察器的通配过滤值，不是 SDK 调用的 API ID。
    if (observed_api_id_ != 0 && identity.api_id() != observed_api_id_) {
      return;
    }
    const int64_t timestamp_ns = MonotonicNanoseconds();
    std::lock_guard<std::mutex> lock(mutex_);
    ++request_count_;
    if (first_request_identity_ == 0) {
      first_request_identity_ = identity.id();
      first_request_timestamp_ns_ = timestamp_ns;
    }
    std::cout << "SPORT_API_OBSERVER event=REQUEST"
              << " monotonic_ns=" << timestamp_ns
              << " api_id=" << identity.api_id()
              << " identity_id=" << identity.id()
              << " lease_id=" << header.lease().id()
              << " priority=" << header.policy().priority()
              << " noreply=" << (header.policy().noreply() ? "true" : "false")
              << " parameter_length=" << message.parameter().size()
              << " parameter_summary=" << std::quoted(Summary(message.parameter()))
              << " binary_length=" << message.binary().size() << std::endl;
  }

  void OnResponse(const void* raw_message)
  {
    if (raw_message == nullptr) {
      return;
    }
    const auto& message = *static_cast<const unitree_api::msg::dds_::Response_*>(
        raw_message);
    const auto& header = message.header();
    const auto& identity = header.identity();
    if (observed_api_id_ != 0 && identity.api_id() != observed_api_id_) {
      return;
    }
    const int64_t timestamp_ns = MonotonicNanoseconds();
    std::lock_guard<std::mutex> lock(mutex_);
    ++response_count_;
    if (first_response_latency_ns_ < 0 &&
        identity.id() == first_request_identity_ &&
        first_request_timestamp_ns_ > 0) {
      first_response_latency_ns_ = timestamp_ns - first_request_timestamp_ns_;
    }
    std::cout << "SPORT_API_OBSERVER event=RESPONSE"
              << " monotonic_ns=" << timestamp_ns
              << " api_id=" << identity.api_id()
              << " identity_id=" << identity.id()
              << " status_code=" << header.status().code()
              << " data_length=" << message.data().size()
              << " binary_length=" << message.binary().size()
              << " data_summary=" << std::quoted(Summary(message.data()))
              << std::endl;
  }

  unitree::robot::ChannelSubscriberPtr<unitree_api::msg::dds_::Request_>
      request_subscriber_;
  unitree::robot::ChannelSubscriberPtr<unitree_api::msg::dds_::Response_>
      response_subscriber_;
  const int64_t observed_api_id_;
  const std::string service_name_;
  mutable std::mutex mutex_;
  int request_count_{0};
  int response_count_{0};
  int64_t first_request_identity_{0};
  int64_t first_request_timestamp_ns_{0};
  int64_t first_response_latency_ns_{-1};
};

int RunObserver(
    const std::string& network_interface, double duration_sec,
    int64_t observed_api_id, const std::string& service_name)
{
  if (network_interface.empty() || duration_sec <= 0.0 || observed_api_id < 0 ||
      !IsSafeServiceName(service_name)) {
    throw std::runtime_error(
        "interface/duration_sec/api_id/service_name arguments are invalid");
  }
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  std::cout << "SPORT_API_OBSERVER event=START"
            << " interface=" << network_interface
            << " domain=0 service=" << service_name
            << " request_topic=" << ApiTopic(service_name, "request")
            << " response_topic=" << ApiTopic(service_name, "response")
            << " api_filter="
            << (observed_api_id == 0 ? "ALL" : std::to_string(observed_api_id))
            << " duration_sec=" << duration_sec << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  SportApiObserver observer(observed_api_id, service_name);
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration_sec);
  while (g_running && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  observer.PrintSummary();
  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    if (argc < 3 || argc > 5) {
      throw std::runtime_error(
        "Usage: go2_sdk_sport_api_observer <network_interface> <duration_sec> "
          "[api_id; 0=all] [service_name]");
    }
    // 默认全量观察，避免人工入口使用非预期 API 时被静默漏记。
    const int64_t api_id = argc >= 4 ? std::stoll(argv[3]) : 0;
    const std::string service_name = argc == 5 ? argv[4] : "sport";
    return RunObserver(argv[1], std::stod(argv[2]), api_id, service_name);
  } catch (const std::exception& error) {
    std::cerr << "SPORT_API_OBSERVER event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
