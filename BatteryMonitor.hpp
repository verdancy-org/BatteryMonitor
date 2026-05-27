#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Battery voltage monitor module
constructor_args:
  - data_topic_name: "battery_state"
  - sample_period_ms: 50
  - divider_ratio: 11.0
  - task_stack_depth: 1024
template_args: []
required_hardware:
  - battery_adc
  - ramfs
depends:
  - verdancy-org/LPFilter@main
=== END MANIFEST === */
// clang-format on

#include "adc.hpp"
#include "BatteryMonitorDeps.hpp"
#include "app_framework.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

class BatteryMonitor : public LibXR::Application {
public:
  struct Data {
    float voltage_v = 0.0f;
    float cell_voltage_v = 0.0f;
    uint8_t cell_count = 0;
    uint8_t power_state = 0;
    bool low = false;
    bool critical = false;
  };

  BatteryMonitor(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
                 const char *data_topic_name, uint32_t sample_period_ms,
                 float divider_ratio, size_t task_stack_depth)
      : sample_period_ms_(sample_period_ms), divider_ratio_(divider_ratio),
        topic_(data_topic_name, sizeof(state_), nullptr, true, true, true),
        adc_(hw.template FindOrExit<LibXR::ADC>({"battery_adc"})),
        cmd_file_(LibXR::RamFS::CreateFile("battery", CommandFunc, this)) {
    ASSERT(sample_period_ms_ > 0);
    ASSERT(divider_ratio_ > 0.0f);
    voltage_filter_.SetAlpha(0.05f);

    app.Register(*this);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    thread_.Create(this, ThreadFunc, "battery_thread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  void OnMonitor() override {}

private:
  void Update() {
    const float pin_voltage = adc_->Read();
    const float voltage = pin_voltage * divider_ratio_;

    if (!voltage_filter_initialized_) {
      voltage_filter_.Reset(voltage);
      voltage_filter_initialized_ = true;
    } else {
      voltage_filter_.Update(voltage);
    }

    state_.voltage_v = voltage_filter_.State();

    if (state_.voltage_v < 1.0f) {
      state_.cell_count = 0;
      state_.cell_voltage_v = 0.0f;
      state_.power_state = 0;
      state_.low = false;
      state_.critical = false;
      return;
    }

    state_.cell_count = static_cast<uint8_t>(std::clamp(
        static_cast<int>(std::lround(state_.voltage_v / 4.1f)), 1, 6));
    state_.cell_voltage_v =
        state_.voltage_v / static_cast<float>(state_.cell_count);
    state_.low = state_.cell_voltage_v < 3.60f;
    state_.critical = state_.cell_voltage_v < 3.45f;

    if (state_.critical) {
      state_.power_state = 3;
    } else if (state_.low) {
      state_.power_state = 2;
    } else {
      state_.power_state = 1;
    }
  }

  static void ThreadFunc(BatteryMonitor *battery_monitor) {
    while (true) {
      battery_monitor->Update();
      battery_monitor->topic_.Publish(battery_monitor->state_);
      LibXR::Thread::Sleep(battery_monitor->sample_period_ms_);
    }
  }

  static int CommandFunc(BatteryMonitor *battery_monitor, int argc,
                         char **argv) {
    if (argc == 1) {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  show [time_ms] [interval_ms] - Print battery status periodically.\r\n">();
      return 0;
    }

    if (argc == 4 && std::strcmp(argv[1], "show") == 0) {
      int time_ms = std::atoi(argv[2]);
      int interval_ms = std::atoi(argv[3]);
      interval_ms = std::clamp(interval_ms, 10, 1000);

      while (time_ms > 0) {
        LibXR::STDIO::Printf<
            "Battery: %.2fV | cells=%d | per_cell=%.2fV | low=%d | critical=%d\r\n">(
            battery_monitor->state_.voltage_v,
            static_cast<int>(battery_monitor->state_.cell_count),
            battery_monitor->state_.cell_voltage_v,
            static_cast<int>(battery_monitor->state_.low),
            static_cast<int>(battery_monitor->state_.critical));
        LibXR::Thread::Sleep(interval_ms);
        time_ms -= interval_ms;
      }
      return 0;
    }

    LibXR::STDIO::Printf<"Error: Invalid arguments.\r\n">();
    return -1;
  }

  uint32_t sample_period_ms_ = 50;
  float divider_ratio_ = 11.0f;
  bool voltage_filter_initialized_ = false;
  LPFilter<float> voltage_filter_;
  Data state_;
  LibXR::Topic topic_;
  LibXR::ADC *adc_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Thread thread_;
};
