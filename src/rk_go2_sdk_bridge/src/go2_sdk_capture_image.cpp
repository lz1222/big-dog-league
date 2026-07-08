#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

namespace
{

int ParsePositiveInt(const char* raw, const std::string& name)
{
  const int value = std::stoi(raw);
  if (value <= 0) {
    throw std::runtime_error(name + " must be positive");
  }
  return value;
}

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " <network_interface> <output.jpg> [attempts] [retry_sleep_ms]\n\n"
      << "Example:\n"
      << "  " << program << " eth0 /tmp/go2_front.jpg 5 150\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 3) {
    PrintUsage(argv[0]);
    return 2;
  }

  try {
    const std::string network_interface = argv[1];
    const std::string output_path = argv[2];
    const int attempts = argc >= 4 ? ParsePositiveInt(argv[3], "attempts") : 5;
    const int retry_sleep_ms =
        argc >= 5 ? ParsePositiveInt(argv[4], "retry_sleep_ms") : 150;

    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    unitree::robot::go2::VideoClient video_client;
    video_client.SetTimeout(1.0F);
    video_client.Init();

    std::vector<uint8_t> image_sample;
    int ret = -1;
    for (int index = 0; index < attempts; ++index) {
      image_sample.clear();
      ret = video_client.GetImageSample(image_sample);
      if (ret == 0 && !image_sample.empty()) {
        break;
      }
      std::this_thread::sleep_for(
          std::chrono::milliseconds(retry_sleep_ms));
    }

    if (ret != 0 || image_sample.empty()) {
      std::cerr
          << "Failed to capture Go2 front camera image: ret=" << ret
          << ", bytes=" << image_sample.size() << std::endl;
      return 1;
    }

    std::ofstream image_file(output_path, std::ios::binary);
    if (!image_file.is_open()) {
      std::cerr << "Failed to open output file: " << output_path << std::endl;
      return 1;
    }
    image_file.write(
        reinterpret_cast<const char*>(image_sample.data()),
        static_cast<std::streamsize>(image_sample.size()));
    image_file.close();

    std::cout
        << "Saved Go2 front camera image: " << output_path
        << " bytes=" << image_sample.size() << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }
}
