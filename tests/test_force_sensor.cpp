#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>

#include "linkerhand/linkerhand.hpp"

namespace {
volatile std::sig_atomic_t g_running = 1;

void signal_handler(int signal) {
  (void)signal;
  g_running = 0;
  std::cout << "\n正在停止..." << std::endl;
}

void print_finger_data(const std::string& name, const linkerhand::hand::l6::ForceSensorData& data) {
  std::cout << "  " << name << ": [";
  for (std::size_t i = 0; i < 12; ++i) {
    if (i > 0) std::cout << ", ";
    std::cout << std::setw(3) << static_cast<int>(data.values[i]);
  }
  std::cout << " ...]" << std::endl;
}

void print_usage(const char* program_name) {
  std::cout << "用法: " << program_name << " [选项]" << std::endl;
  std::cout << std::endl;
  std::cout << "选项:" << std::endl;
  std::cout << "  -i, --interface <name>   CAN 接口名称 (默认: can0)" << std::endl;
  std::cout << "  -s, --side <side>        手的方向: left 或 right (默认: left)" << std::endl;
  std::cout << "  -m, --mode <mode>        模式: once, loop, stream (默认: once)" << std::endl;
  std::cout << "  -t, --timeout <ms>       超时时间毫秒 (默认: 1000)" << std::endl;
  std::cout << "  -d, --delay <ms>         循环模式下的延迟毫秒 (默认: 500)" << std::endl;
  std::cout << "  -h, --help               显示帮助信息" << std::endl;
  std::cout << std::endl;
  std::cout << "示例:" << std::endl;
  std::cout << "  " << program_name << " -i can0 -s left -m once" << std::endl;
  std::cout << "  " << program_name << " -m loop -d 200" << std::endl;
  std::cout << "  " << program_name << " -m stream" << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string interface_name = "can0";
  std::string side = "left";
  std::string mode = "once";
  double timeout_ms = 1000;
  int delay_ms = 500;

  // 解析命令行参数
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      return 0;
    } else if ((arg == "-i" || arg == "--interface") && i + 1 < argc) {
      interface_name = argv[++i];
    } else if ((arg == "-s" || arg == "--side") && i + 1 < argc) {
      side = argv[++i];
    } else if ((arg == "-m" || arg == "--mode") && i + 1 < argc) {
      mode = argv[++i];
    } else if ((arg == "-t" || arg == "--timeout") && i + 1 < argc) {
      timeout_ms = std::stod(argv[++i]);
    } else if ((arg == "-d" || arg == "--delay") && i + 1 < argc) {
      delay_ms = std::stoi(argv[++i]);
    }
  }

  // 设置信号处理
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::cout << "========================================" << std::endl;
  std::cout << "LinkerHand L6 压力传感器测试" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "接口: " << interface_name << std::endl;
  std::cout << "方向: " << side << std::endl;
  std::cout << "模式: " << mode << std::endl;
  std::cout << "超时: " << timeout_ms << " ms" << std::endl;
  std::cout << "----------------------------------------" << std::endl;

  try {
    std::cout << "正在连接到 " << interface_name << "..." << std::endl;
    linkerhand::L6 hand(side, interface_name);
    std::cout << "连接成功!" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    if (mode == "once") {
      // 单次读取模式
      std::cout << "读取所有手指压力传感器数据..." << std::endl;
      auto data = hand.force_sensor.get_data_blocking(timeout_ms);

      std::cout << std::endl;
      std::cout << "压力传感器数据 (前12字节):" << std::endl;
      print_finger_data("拇指 (thumb) ", data.thumb);
      print_finger_data("食指 (index) ", data.index);
      print_finger_data("中指 (middle)", data.middle);
      print_finger_data("无名指 (ring)", data.ring);
      print_finger_data("小指 (pinky) ", data.pinky);

    } else if (mode == "loop") {
      // 循环读取模式
      std::cout << "循环读取模式 (按 Ctrl+C 停止)..." << std::endl;
      std::cout << std::endl;

      int count = 0;
      while (g_running) {
        try {
          auto data = hand.force_sensor.get_data_blocking(timeout_ms);
          ++count;

          std::cout << "\r读取 #" << count << " - ";
          std::cout << "拇指: " << static_cast<int>(data.thumb.values[0]) << " ";
          std::cout << "食指: " << static_cast<int>(data.index.values[0]) << " ";
          std::cout << "中指: " << static_cast<int>(data.middle.values[0]) << " ";
          std::cout << "无名指: " << static_cast<int>(data.ring.values[0]) << " ";
          std::cout << "小指: " << static_cast<int>(data.pinky.values[0]) << "   ";
          std::cout << std::flush;

          std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        } catch (const linkerhand::TimeoutError& e) {
          std::cout << "\r超时，重试中...                              " << std::flush;
        }
      }
      std::cout << std::endl;

    } else if (mode == "stream") {
      // 流式读取模式
      std::cout << "流式读取模式 (按 Ctrl+C 停止)..." << std::endl;
      std::cout << std::endl;

      auto queue = hand.force_sensor.stream(100, 10);

      int count = 0;
      for (const auto& data : queue) {
        if (!g_running) break;
        ++count;

        std::cout << "\r流 #" << count << " - ";
        std::cout << "拇指: " << static_cast<int>(data.thumb.values[0]) << " ";
        std::cout << "食指: " << static_cast<int>(data.index.values[0]) << " ";
        std::cout << "中指: " << static_cast<int>(data.middle.values[0]) << " ";
        std::cout << "无名指: " << static_cast<int>(data.ring.values[0]) << " ";
        std::cout << "小指: " << static_cast<int>(data.pinky.values[0]) << "   ";
        std::cout << std::flush;
      }
      std::cout << std::endl;
      hand.force_sensor.stop_streaming();

    } else {
      std::cerr << "未知模式: " << mode << std::endl;
      print_usage(argv[0]);
      return 1;
    }

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "测试完成!" << std::endl;

  } catch (const linkerhand::CANError& e) {
    std::cerr << "CAN 错误: " << e.what() << std::endl;
    std::cerr << std::endl;
    std::cerr << "请确保:" << std::endl;
    std::cerr << "  1. CAN 接口已启用:" << std::endl;
    std::cerr << "     sudo ip link set " << interface_name << " type can bitrate 1000000" << std::endl;
    std::cerr << "     sudo ip link set " << interface_name << " up" << std::endl;
    std::cerr << "  2. 机械手已正确连接" << std::endl;
    return 1;
  } catch (const linkerhand::TimeoutError& e) {
    std::cerr << "超时错误: " << e.what() << std::endl;
    std::cerr << "请检查机械手是否已连接并开机" << std::endl;
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "错误: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
