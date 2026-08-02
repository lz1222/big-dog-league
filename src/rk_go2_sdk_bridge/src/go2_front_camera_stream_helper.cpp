#include <arpa/inet.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

namespace
{

volatile sig_atomic_t g_shutdown_requested = 0;

void SignalHandler(int)
{
  g_shutdown_requested = 1;
}

struct StreamConfig
{
  std::string network_interface;
  double sdk_timeout_sec{2.0};
  double retry_sleep_sec{0.20};
  int max_consecutive_retries{10};
  int frame_limit{0};
};

StreamConfig ParseArgs(int argc, char** argv)
{
  StreamConfig config;
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <network_interface> [sdk_timeout_sec] "
                 "[retry_sleep_sec] [max_retries] [frame_limit]"
              << std::endl;
    std::exit(2);
  }
  config.network_interface = argv[1];
  if (argc >= 3) config.sdk_timeout_sec = std::stod(argv[2]);
  if (argc >= 4) config.retry_sleep_sec = std::stod(argv[3]);
  if (argc >= 5) config.max_consecutive_retries = std::stoi(argv[4]);
  if (argc >= 6) config.frame_limit = std::stoi(argv[5]);

  if (config.sdk_timeout_sec <= 0.0) {
    throw std::runtime_error("sdk_timeout_sec must be positive");
  }
  if (config.retry_sleep_sec < 0.0) {
    throw std::runtime_error("retry_sleep_sec must be nonnegative");
  }
  if (config.max_consecutive_retries <= 0) {
    throw std::runtime_error("max_retries must be positive");
  }
  return config;
}

bool WriteFrame(const std::vector<uint8_t>& jpeg)
{
  if (jpeg.empty()) {
    return false;
  }
  const uint32_t length_be = htonl(static_cast<uint32_t>(jpeg.size()));
  if (write(STDOUT_FILENO, &length_be, sizeof(length_be))
      != static_cast<ssize_t>(sizeof(length_be))) {
    return false;
  }
  const uint8_t* data = jpeg.data();
  uint32_t remaining = static_cast<uint32_t>(jpeg.size());
  while (remaining > 0) {
    const ssize_t written =
        write(STDOUT_FILENO, data, remaining);
    if (written <= 0) {
      return false;
    }
    data += static_cast<size_t>(written);
    remaining -= static_cast<uint32_t>(written);
  }
  return true;
}

int Run(const StreamConfig& config)
{
  std::signal(SIGTERM, SignalHandler);
  std::signal(SIGINT, SignalHandler);

  std::cerr << "[stream_helper] Init ChannelFactory on "
            << config.network_interface << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(
      0, config.network_interface);

  unitree::robot::go2::VideoClient video_client;
  video_client.SetTimeout(static_cast<float>(config.sdk_timeout_sec));
  video_client.Init();

  std::cerr << "[stream_helper] VideoClient ready, streaming to stdout"
            << std::endl;

  int frame_count = 0;
  int consecutive_failures = 0;
  int total_failures = 0;
  std::vector<uint8_t> previous_frame;

  while (!g_shutdown_requested) {
    if (config.frame_limit > 0 && frame_count >= config.frame_limit) {
      std::cerr << "[stream_helper] Frame limit " << config.frame_limit
                << " reached, exiting" << std::endl;
      break;
    }

    std::vector<uint8_t> jpeg_buffer;
    video_client.SetTimeout(static_cast<float>(config.sdk_timeout_sec));
    const int32_t sdk_ret = video_client.GetImageSample(jpeg_buffer);

    if (sdk_ret != 0 || jpeg_buffer.empty()) {
      consecutive_failures++;
      total_failures++;
      std::cerr << "[stream_helper] GetImageSample failed: ret="
                << sdk_ret << " bytes=" << jpeg_buffer.size()
                << " consecutive=" << consecutive_failures << std::endl;

      if (consecutive_failures > config.max_consecutive_retries) {
        std::cerr << "[stream_helper] Too many consecutive failures, "
                     "exiting" << std::endl;
        return 1;
      }
      if (config.retry_sleep_sec > 0.0) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(config.retry_sleep_sec));
      }
      continue;
    }

    if (jpeg_buffer == previous_frame) {
      std::cerr << "[stream_helper] Duplicate frame dropped" << std::endl;
      continue;
    }

    if (!WriteFrame(jpeg_buffer)) {
      std::cerr << "[stream_helper] stdout write failed, exiting"
                << std::endl;
      return 1;
    }

    previous_frame = std::move(jpeg_buffer);
    consecutive_failures = 0;
    frame_count++;

    if (frame_count % 30 == 0) {
      std::cerr << "[stream_helper] frames=" << frame_count
                << " failures=" << total_failures << std::endl;
    }
  }

  std::cerr << "[stream_helper] Shutdown: frames=" << frame_count
            << " failures=" << total_failures << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    const StreamConfig config = ParseArgs(argc, argv);
    return Run(config);
  } catch (const std::exception& error) {
    std::cerr << "[stream_helper] Fatal: " << error.what() << std::endl;
    return 1;
  }
}
