#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include "esphome/components/time/real_time_clock.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/output/float_output.h"

namespace esphome {
namespace idf_dcf77_emitter {

class IDFDCF77Emitter : public Component {
 public:
  void set_time_id(time::RealTimeClock *time_id) {
    this->time_id_ = time_id;
  }

  void set_carrier_output(output::FloatOutput *carrier_output) {
    this->carrier_output_ = carrier_output;
  }

  void set_led_pin(InternalGPIOPin *led_pin) {
    this->led_pin_ = led_pin;
  }

  void set_sync_switch(switch_::Switch *sync_switch) {
    this->sync_switch_ = sync_switch;
  }

  void setup() override;
  void loop() override;
  void dump_config() override;

  float get_setup_priority() const override {
    return setup_priority::HARDWARE;
  }

 protected:
  // Carrier control
  void setup_carrier_();
  void stop_carrier_();

  // DCF77 timing
  void schedule_next_tick_();
  void dcf_out_tick();

  // DCF77 signal generation
  void generate_signal_(int current_sec);
  void code_time_();

  int bin2bcd_(int value);

  // ESPHome components
  time::RealTimeClock *time_id_{nullptr};
  output::FloatOutput *carrier_output_{nullptr};
  InternalGPIOPin *led_pin_{nullptr};
  switch_::Switch *sync_switch_{nullptr};

  // State
  bool is_initialized_{false};
  bool carrier_enabled_{false};

  // DCF77 frame
  //
  // 0 = minute marker / no carrier reduction
  // 1 = logical 0 / 100 ms carrier reduction
  // 2 = logical 1 / 200 ms carrier reduction
  int impulse_array_[60]{};

  int impulse_count_{0};
  int last_second_{-1};

  // Encoded time
  int actual_second_{0};
  int actual_minutes_{0};
  int actual_hours_{0};

  int actual_day_{0};
  int actual_month_{0};
  int actual_year_{0};

  int day_of_week_{0};

  // Synchronization / status
  uint32_t sync_start_millis_{0};
  uint32_t last_status_log_{0};
  uint32_t last_valid_time_{0};

};  // class IDFDCF77Emitter

}  // namespace idf_dcf77_emitter
}  // namespace esphome
