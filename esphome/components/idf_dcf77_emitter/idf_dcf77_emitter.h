#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include "esphome/components/switch/switch.h"
#include "esphome/components/time/real_time_clock.h"

#ifdef USE_ESP32
#include "driver/ledc.h"
#include "esp_err.h"
#endif

namespace esphome {
namespace idf_dcf77_emitter {

class IDFDCF77Emitter : public Component {
 public:
  void set_time_id(time::RealTimeClock *time_id) {
    this->time_id_ = time_id;
  }

  void set_antenna_pin(InternalGPIOPin *antenna_pin) {
    this->antenna_pin_ = antenna_pin;
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
  void setup_carrier_();
  void stop_carrier_();

  bool configure_ledc_();

  void schedule_next_tick_();
  void dcf_out_tick();

  void generate_signal_(int current_sec);
  void code_time_();

  int bin2bcd_(int value);

  time::RealTimeClock *time_id_{nullptr};

  InternalGPIOPin *antenna_pin_{nullptr};
  InternalGPIOPin *led_pin_{nullptr};

  switch_::Switch *sync_switch_{nullptr};

  bool is_initialized_{false};
  bool carrier_enabled_{false};
  bool ledc_initialized_{false};

  int impulse_array_[60]{};
  int impulse_count_{0};

  int last_second_{-1};

  int actual_second_{0};
  int actual_minutes_{0};
  int actual_hours_{0};

  int actual_day_{0};
  int actual_month_{0};
  int actual_year_{0};
  int day_of_week_{0};

  uint32_t sync_start_millis_{0};
  uint32_t last_status_log_{0};
  uint32_t last_valid_time_{0};

#ifdef USE_ESP32
  static constexpr ledc_mode_t LEDC_SPEED_MODE =
      LEDC_LOW_SPEED_MODE;

  static constexpr ledc_timer_t LEDC_TIMER =
      LEDC_TIMER_0;

  static constexpr ledc_channel_t LEDC_CHANNEL =
      LEDC_CHANNEL_0;

  static constexpr ledc_timer_bit_t LEDC_RESOLUTION =
      LEDC_TIMER_8_BIT;

  static constexpr uint32_t CARRIER_FREQUENCY = 77500;

  static constexpr uint32_t CARRIER_DUTY = 127;
#endif
};

}  // namespace idf_dcf77_emitter
}  // namespace esphome
