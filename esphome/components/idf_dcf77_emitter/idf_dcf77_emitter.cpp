#include "idf_dcf77_emitter.h"

#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace idf_dcf77_emitter {

static const char *const TAG = "idf_dcf77_emitter";

void IDFDCF77Emitter::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ESP-IDF DCF77 Emitter...");

  if (this->time_id_ == nullptr) {
    ESP_LOGE(TAG, "Time component is not configured");
    this->mark_failed();
    return;
  }

  if (this->antenna_pin_ == nullptr) {
    ESP_LOGE(TAG, "Antenna pin is not configured");
    this->mark_failed();
    return;
  }

  if (this->led_pin_ == nullptr) {
    ESP_LOGE(TAG, "LED pin is not configured");
    this->mark_failed();
    return;
  }

  if (this->sync_switch_ == nullptr) {
    ESP_LOGE(TAG, "Sync switch is not configured");
    this->mark_failed();
    return;
  }

  // Configure status LED.
  this->led_pin_->setup();
  this->led_pin_->digital_write(false);

  // Configure the antenna GPIO before assigning it to LEDC.
  this->antenna_pin_->setup();
  this->antenna_pin_->digital_write(false);

  if (!this->configure_ledc_()) {
    ESP_LOGE(TAG, "Failed to configure ESP-IDF LEDC");
    this->mark_failed();
    return;
  }

  this->stop_carrier_();

  this->code_time_();

  this->sync_start_millis_ = millis();
  this->last_valid_time_ = millis();

  auto current_time = this->time_id_->now();

  if (current_time.is_valid()) {
    this->last_second_ = current_time.second;
  } else {
    this->last_second_ = -1;
  }

  this->impulse_count_ = 0;
  this->is_initialized_ = false;
  this->last_status_log_ = 0;

  this->schedule_next_tick_();

  ESP_LOGI(
      TAG,
      "DCF77 emitter ready: frequency=%u Hz, resolution=8-bit",
      static_cast<unsigned int>(CARRIER_FREQUENCY)
  );

  ESP_LOGI(TAG, "Waiting for valid time and second boundary");
}

bool IDFDCF77Emitter::configure_ledc_() {
#ifdef USE_ESP32
  // Timer configuration.
  ledc_timer_config_t timer_config = {};

  timer_config.speed_mode = LEDC_SPEED_MODE;
  timer_config.duty_resolution = LEDC_RESOLUTION;
  timer_config.timer_num = LEDC_TIMER;
  timer_config.freq_hz = CARRIER_FREQUENCY;
  timer_config.clk_cfg = LEDC_AUTO_CLK;

  esp_err_t result = ledc_timer_config(&timer_config);

  if (result != ESP_OK) {
    ESP_LOGE(
        TAG,
        "ledc_timer_config failed: %s",
        esp_err_to_name(result)
    );

    return false;
  }

  // PWM channel configuration.
  ledc_channel_config_t channel_config = {};

  channel_config.gpio_num =
      static_cast<int>(this->antenna_pin_->get_pin());

  channel_config.speed_mode = LEDC_SPEED_MODE;
  channel_config.channel = LEDC_CHANNEL;
  channel_config.intr_type = LEDC_INTR_DISABLE;
  channel_config.timer_sel = LEDC_TIMER;
  channel_config.duty = 0;
  channel_config.hpoint = 0;

  result = ledc_channel_config(&channel_config);

  if (result != ESP_OK) {
    ESP_LOGE(
        TAG,
        "ledc_channel_config failed: %s",
        esp_err_to_name(result)
    );

    return false;
  }

  this->ledc_initialized_ = true;
  this->carrier_enabled_ = false;

  ESP_LOGI(
      TAG,
      "LEDC configured successfully on GPIO%u",
      static_cast<unsigned int>(
          this->antenna_pin_->get_pin()
      )
  );

  return true;
#else
  ESP_LOGE(TAG, "This component requires an ESP32");
  return false;
#endif
}

void IDFDCF77Emitter::loop() {
  if (this->sync_switch_ == nullptr)
    return;

  if (!this->sync_switch_->state) {
    if (this->is_initialized_ || this->carrier_enabled_) {
      ESP_LOGW(TAG, "DCF77 synchronization disabled");

      this->is_initialized_ = false;
      this->impulse_count_ = 0;
      this->sync_start_millis_ = millis();

      this->stop_carrier_();
      this->led_pin_->digital_write(false);
    }

    return;
  }

  if (!this->is_initialized_) {
    auto current_time = this->time_id_->now();

    if (!current_time.is_valid())
      return;

    this->last_valid_time_ = millis();

    if (this->last_second_ < 0) {
      this->last_second_ = current_time.second;
      this->sync_start_millis_ = millis();
      return;
    }

    // Begin transmission on a detected second boundary.
    if (current_time.second != this->last_second_) {
      this->last_second_ = current_time.second;
      this->impulse_count_ = 0;
      this->is_initialized_ = true;

      this->code_time_();

      ESP_LOGI(
          TAG,
          "Second boundary detected; DCF77 transmission started"
      );

      return;
    }

    // Safety fallback if a boundary was not detected.
    if (millis() - this->sync_start_millis_ > 5000) {
      ESP_LOGW(
          TAG,
          "Second-boundary timeout; starting transmission"
      );

      this->last_second_ = current_time.second;
      this->impulse_count_ = 0;
      this->is_initialized_ = true;

      this->code_time_();

      return;
    }

    return;
  }

  auto current_time = this->time_id_->now();

  if (current_time.is_valid()) {
    this->last_valid_time_ = millis();
  } else if (
      millis() - this->last_valid_time_ > 30000
  ) {
    ESP_LOGE(
        TAG,
        "No valid time for 30 seconds; stopping transmission"
    );

    this->is_initialized_ = false;
    this->last_second_ = -1;
    this->impulse_count_ = 0;
    this->sync_start_millis_ = millis();

    this->stop_carrier_();
    this->led_pin_->digital_write(false);

    return;
  }

  const uint32_t now = millis();

  if (now - this->last_status_log_ >= 10000) {
    this->last_status_log_ = now;

    if (current_time.is_valid()) {
      ESP_LOGD(
          TAG,
          "Status: transmitting; time=%02d:%02d:%02d; DST=%s",
          current_time.hour,
          current_time.minute,
          current_time.second,
          current_time.is_dst ? "ON" : "OFF"
      );
    }
  }
}

void IDFDCF77Emitter::schedule_next_tick_() {
  this->set_interval(
      "dcf77_tick",
      100,
      [this]() {
        this->dcf_out_tick();
      }
  );
}

void IDFDCF77Emitter::dcf_out_tick() {
  if (!this->is_initialized_)
    return;

  if (this->sync_switch_ == nullptr ||
      !this->sync_switch_->state) {
    return;
  }

  auto current_time = this->time_id_->now();

  if (!current_time.is_valid())
    return;

  const int current_second = current_time.second;

  if (current_second < 0 || current_second > 59)
    return;

  if (current_second != this->last_second_) {
    if (
        this->last_second_ >= 0 &&
        current_second != ((this->last_second_ + 1) % 60)
    ) {
      ESP_LOGW(
          TAG,
          "Irregular second transition: %d -> %d",
          this->last_second_,
          current_second
      );
    }

    this->last_second_ = current_second;
    this->impulse_count_ = 0;

    // Rebuild the complete frame at the beginning of each second.
    this->code_time_();
  }

  this->generate_signal_(current_second);
}

void IDFDCF77Emitter::generate_signal_(int current_sec) {
  const int symbol = this->impulse_array_[current_sec];

  switch (this->impulse_count_++) {
    case 0:
      // Start of second.
      //
      // Symbol 1 = logical 0:
      // carrier off for 100 ms.
      //
      // Symbol 2 = logical 1:
      // carrier off for 200 ms.
      //
      // Symbol 0 = second 59 minute marker:
      // no carrier reduction.

      if (symbol == 0) {
        this->led_pin_->digital_write(true);
        this->setup_carrier_();
      } else {
        this->led_pin_->digital_write(false);
        this->stop_carrier_();
      }

      break;

    case 1:
      // After 100 ms, restore carrier for logical 0.
      if (symbol == 1) {
        this->led_pin_->digital_write(true);
        this->setup_carrier_();
      }

      break;

    case 2:
      // After 200 ms, restore carrier for logical 1.
      if (symbol == 2) {
        this->led_pin_->digital_write(true);
        this->setup_carrier_();
      }

      break;

    case 9:
      // End of one-second symbol.
      this->impulse_count_ = 0;

      if (current_sec == 59) {
        ESP_LOGD(
            TAG,
            "DCF77 minute complete: %02d:%02d",
            this->actual_hours_,
            this->actual_minutes_
        );
      }

      break;

    default:
      break;
  }
}

void IDFDCF77Emitter::setup_carrier_() {
#ifdef USE_ESP32
  if (!this->ledc_initialized_ || this->carrier_enabled_)
    return;

  esp_err_t result = ledc_set_duty(
      LEDC_SPEED_MODE,
      LEDC_CHANNEL,
      CARRIER_DUTY
  );

  if (result != ESP_OK) {
    ESP_LOGE(
        TAG,
        "ledc_set_duty failed: %s",
        esp_err_to_name(result)
    );

    return;
  }

  result = ledc_update_duty(
      LEDC_SPEED_MODE,
      LEDC_CHANNEL
  );

  if (result != ESP_OK) {
    ESP_LOGE(
        TAG,
        "ledc_update_duty failed: %s",
        esp_err_to_name(result)
    );

    return;
  }

  this->carrier_enabled_ = true;
#endif
}

void IDFDCF77Emitter::stop_carrier_() {
#ifdef USE_ESP32
  if (!this->ledc_initialized_)
    return;

  if (!this->carrier_enabled_) {
    ledc_set_duty(
        LEDC_SPEED_MODE,
        LEDC_CHANNEL,
        0
    );

    ledc_update_duty(
        LEDC_SPEED_MODE,
        LEDC_CHANNEL
    );

    return;
  }

  esp_err_t result = ledc_set_duty(
      LEDC_SPEED_MODE,
      LEDC_CHANNEL,
      0
  );

  if (result != ESP_OK) {
    ESP_LOGE(
        TAG,
        "ledc_set_duty(0) failed: %s",
        esp_err_to_name(result)
    );

    return;
  }

  result = ledc_update_duty(
      LEDC_SPEED_MODE,
      LEDC_CHANNEL
  );

  if (result != ESP_OK) {
    ESP_LOGE(
        TAG,
        "ledc_update_duty failed: %s",
        esp_err_to_name(result)
    );

    return;
  }

  this->carrier_enabled_ = false;
#endif
}

int IDFDCF77Emitter::bin2bcd_(int value) {
  if (value < 10)
    return value;

  const int tens = (value / 10) << 4;
  const int units = value % 10;

  return tens + units;
}

void IDFDCF77Emitter::code_time_() {
  auto time = this->time_id_->now();

  if (!time.is_valid())
    return;

  // DCF77 transmits the date and time of the next minute.
  auto next_time =
      ESPTime::from_epoch_local(time.timestamp + 60);

  this->actual_day_ = next_time.day_of_month;
  this->actual_month_ = next_time.month;
  this->actual_year_ = next_time.year % 100;

  this->actual_hours_ = next_time.hour;
  this->actual_minutes_ = next_time.minute;

  // ESPHome:
  // Sunday=1, Monday=2 ... Saturday=7
  //
  // DCF77:
  // Monday=1 ... Saturday=6, Sunday=7
  this->day_of_week_ =
      ((next_time.day_of_week + 5) % 7) + 1;

  this->actual_second_ = time.second;

  if (this->actual_second_ >= 60)
    this->actual_second_ = 0;

  int temporary = 0;
  int temporary_input = 0;
  int parity_count = 0;

  // Bits 0..19 default to logical zero.
  for (int bit = 0; bit < 20; bit++) {
    this->impulse_array_[bit] = 1;
  }

  // Timezone bits:
  //
  // Summer time:
  // bit 17 = 1
  // bit 18 = 0
  //
  // Standard time:
  // bit 17 = 0
  // bit 18 = 1
  if (time.is_dst) {
    this->impulse_array_[17] = 2;
    this->impulse_array_[18] = 1;
  } else {
    this->impulse_array_[17] = 1;
    this->impulse_array_[18] = 2;
  }

  // Bit 20: start of encoded time.
  this->impulse_array_[20] = 2;

  // Minutes: bits 21..27.
  parity_count = 0;
  temporary_input =
      this->bin2bcd_(this->actual_minutes_);

  for (int bit = 21; bit < 28; bit++) {
    temporary = temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;
    temporary_input >>= 1;
  }

  // Minute parity: bit 28.
  this->impulse_array_[28] =
      ((parity_count & 1) == 0) ? 1 : 2;

  // Hours: bits 29..34.
  parity_count = 0;
  temporary_input =
      this->bin2bcd_(this->actual_hours_);

  for (int bit = 29; bit < 35; bit++) {
    temporary = temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;
    temporary_input >>= 1;
  }

  // Hour parity: bit 35.
  this->impulse_array_[35] =
      ((parity_count & 1) == 0) ? 1 : 2;

  // Date parity covers bits 36..57.
  parity_count = 0;

  // Day of month: bits 36..41.
  temporary_input =
      this->bin2bcd_(this->actual_day_);

  for (int bit = 36; bit < 42; bit++) {
    temporary = temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;
    temporary_input >>= 1;
  }

  // Day of week: bits 42..44.
  temporary_input =
      this->bin2bcd_(this->day_of_week_);

  for (int bit = 42; bit < 45; bit++) {
    temporary = temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;
    temporary_input >>= 1;
  }

  // Month: bits 45..49.
  temporary_input =
      this->bin2bcd_(this->actual_month_);

  for (int bit = 45; bit < 50; bit++) {
    temporary = temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;
    temporary_input >>= 1;
  }

  // Year: bits 50..57.
  temporary_input =
      this->bin2bcd_(this->actual_year_);

  for (int bit = 50; bit < 58; bit++) {
    temporary = temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;
    temporary_input >>= 1;
  }

  // Date parity: bit 58.
  this->impulse_array_[58] =
      ((parity_count & 1) == 0) ? 1 : 2;

  // Second 59: no carrier reduction.
  this->impulse_array_[59] = 0;
}

void IDFDCF77Emitter::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP-IDF DCF77 Emitter:");

  LOG_PIN(
      "  Antenna pin: ",
      this->antenna_pin_
  );

  LOG_PIN(
      "  LED pin: ",
      this->led_pin_
  );

  ESP_LOGCONFIG(
      TAG,
      "  Carrier frequency: %u Hz",
      static_cast<unsigned int>(CARRIER_FREQUENCY)
  );

  ESP_LOGCONFIG(TAG, "  LEDC timer: 0");
  ESP_LOGCONFIG(TAG, "  LEDC channel: 0");
  ESP_LOGCONFIG(TAG, "  LEDC resolution: 8-bit");

  ESP_LOGCONFIG(
      TAG,
      "  LEDC configured: %s",
      YESNO(this->ledc_initialized_)
  );
}

}  // namespace idf_dcf77_emitter
}  // namespace esphome
