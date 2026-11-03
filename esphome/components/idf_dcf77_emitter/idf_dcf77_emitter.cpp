#include "idf_dcf77_emitter.h"

#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace idf_dcf77_emitter {

static const char *const TAG = "idf_dcf77_emitter";


// ============================================================
// SETUP
// ============================================================

void IDFDCF77Emitter::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DCF77 Emitter...");

  if (this->time_id_ == nullptr) {
    ESP_LOGE(TAG, "Time component is not configured");
    this->mark_failed();
    return;
  }

  if (this->carrier_output_ == nullptr) {
    ESP_LOGE(TAG, "Carrier output is not configured");
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

  // Setup status LED.
  this->led_pin_->setup();
  this->led_pin_->digital_write(false);

  // Start with 77.5 kHz carrier disabled.
  // Frequency itself is configured by ESPHome's LEDC output.
  this->carrier_output_->set_level(0.0f);
  this->carrier_enabled_ = false;

  // Build initial DCF77 frame.
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

  // Start 100 ms DCF77 timing tick.
  this->schedule_next_tick_();

  ESP_LOGI(TAG, "DCF77 emitter ready");
  ESP_LOGI(TAG, "Carrier controlled by ESPHome LEDC output");
  ESP_LOGI(TAG, "Waiting for valid time and second boundary");
}


// ============================================================
// MAIN LOOP
// ============================================================

void IDFDCF77Emitter::loop() {
  if (this->sync_switch_ == nullptr)
    return;

  // ----------------------------------------------------------
  // Synchronization disabled
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // Initial second synchronization
  // ----------------------------------------------------------

  if (!this->is_initialized_) {
    auto current_time = this->time_id_->now();

    if (!current_time.is_valid())
      return;

    this->last_valid_time_ = millis();

    // First valid time reading.
    if (this->last_second_ < 0) {
      this->last_second_ = current_time.second;
      this->sync_start_millis_ = millis();
      return;
    }

    // Second changed -> second boundary detected.
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

    // Safety fallback.
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


  // ----------------------------------------------------------
  // Check time validity
  // ----------------------------------------------------------

  auto current_time = this->time_id_->now();

  if (current_time.is_valid()) {

    this->last_valid_time_ = millis();

  } else if (millis() - this->last_valid_time_ > 30000) {

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


  // ----------------------------------------------------------
  // Periodic status
  // ----------------------------------------------------------

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


// ============================================================
// 100 ms TIMER
// ============================================================

void IDFDCF77Emitter::schedule_next_tick_() {

  this->set_interval(
      "dcf77_tick",
      100,
      [this]() {
        this->dcf_out_tick();
      }
  );
}


// ============================================================
// DCF77 OUTPUT TICK
// ============================================================

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


  // ----------------------------------------------------------
  // New second
  // ----------------------------------------------------------

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

    // Rebuild frame using current time.
    this->code_time_();
  }


  // ----------------------------------------------------------
  // Generate current DCF77 symbol
  // ----------------------------------------------------------

  this->generate_signal_(current_second);
}


// ============================================================
// SIGNAL GENERATION
// ============================================================

void IDFDCF77Emitter::generate_signal_(int current_sec) {

  const int symbol = this->impulse_array_[current_sec];

  switch (this->impulse_count_++) {

    case 0:

      // Start of second.
      //
      // symbol 1:
      // logical 0 -> carrier OFF for 100 ms
      //
      // symbol 2:
      // logical 1 -> carrier OFF for 200 ms
      //
      // symbol 0:
      // second 59 -> no carrier reduction

      if (symbol == 0) {

        this->led_pin_->digital_write(true);
        this->setup_carrier_();

      } else {

        this->led_pin_->digital_write(false);
        this->stop_carrier_();
      }

      break;


    case 1:

      // 100 ms elapsed.
      // Restore carrier for logical 0.

      if (symbol == 1) {

        this->led_pin_->digital_write(true);
        this->setup_carrier_();
      }

      break;


    case 2:

      // 200 ms elapsed.
      // Restore carrier for logical 1.

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


// ============================================================
// CARRIER ON
// ============================================================

void IDFDCF77Emitter::setup_carrier_() {

  if (this->carrier_enabled_)
    return;

  if (this->carrier_output_ == nullptr)
    return;

  // ESPHome LEDC frequency is configured in YAML as 77500 Hz.
  //
  // 0.5 = 50% duty cycle.
  this->carrier_output_->set_level(0.5f);

  this->carrier_enabled_ = true;
}


// ============================================================
// CARRIER OFF
// ============================================================

void IDFDCF77Emitter::stop_carrier_() {

  if (this->carrier_output_ == nullptr)
    return;

  // Duty cycle 0% -> carrier disabled.
  this->carrier_output_->set_level(0.0f);

  this->carrier_enabled_ = false;
}


// ============================================================
// INTEGER TO BCD
// ============================================================

int IDFDCF77Emitter::bin2bcd_(int value) {

  if (value < 10)
    return value;

  const int tens = (value / 10) << 4;
  const int units = value % 10;

  return tens + units;
}


// ============================================================
// BUILD DCF77 FRAME
// ============================================================

void IDFDCF77Emitter::code_time_() {

  auto time = this->time_id_->now();

  if (!time.is_valid())
    return;


  // ----------------------------------------------------------
  // DCF77 transmits the time/date of the NEXT minute.
  //
  // timestamp + 60 correctly handles:
  // - hour rollover
  // - midnight
  // - month rollover
  // - year rollover
  // - leap days
  // ----------------------------------------------------------

  auto next_time =
      ESPTime::from_epoch_local(time.timestamp + 60);


  this->actual_day_ =
      next_time.day_of_month;

  this->actual_month_ =
      next_time.month;

  this->actual_year_ =
      next_time.year % 100;

  this->actual_hours_ =
      next_time.hour;

  this->actual_minutes_ =
      next_time.minute;


  // ----------------------------------------------------------
  // Weekday conversion
  //
  // ESPHome:
  // Sunday = 1
  // Monday = 2
  // ...
  // Saturday = 7
  //
  // DCF77:
  // Monday = 1
  // ...
  // Saturday = 6
  // Sunday = 7
  // ----------------------------------------------------------

  this->day_of_week_ =
      ((next_time.day_of_week + 5) % 7) + 1;


  this->actual_second_ = time.second;

  if (this->actual_second_ >= 60)
    this->actual_second_ = 0;


  int temporary = 0;
  int temporary_input = 0;
  int parity_count = 0;


  // ----------------------------------------------------------
  // Bits 0..19
  //
  // Default logical zero.
  // ----------------------------------------------------------

  for (int bit = 0; bit < 20; bit++) {

    this->impulse_array_[bit] = 1;
  }


  // ----------------------------------------------------------
  // DST / timezone bits
  //
  // CEST:
  // bit 17 = 1
  // bit 18 = 0
  //
  // CET:
  // bit 17 = 0
  // bit 18 = 1
  // ----------------------------------------------------------

  if (time.is_dst) {

    this->impulse_array_[17] = 2;
    this->impulse_array_[18] = 1;

  } else {

    this->impulse_array_[17] = 1;
    this->impulse_array_[18] = 2;
  }


  // ----------------------------------------------------------
  // Bit 20
  //
  // Start of encoded time information.
  // ----------------------------------------------------------

  this->impulse_array_[20] = 2;


  // ----------------------------------------------------------
  // MINUTES
  // bits 21..27
  // parity bit 28
  // ----------------------------------------------------------

  parity_count = 0;

  temporary_input =
      this->bin2bcd_(this->actual_minutes_);


  for (int bit = 21; bit < 28; bit++) {

    temporary =
        temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;

    temporary_input >>= 1;
  }


  this->impulse_array_[28] =
      ((parity_count & 1) == 0) ? 1 : 2;


  // ----------------------------------------------------------
  // HOURS
  // bits 29..34
  // parity bit 35
  // ----------------------------------------------------------

  parity_count = 0;

  temporary_input =
      this->bin2bcd_(this->actual_hours_);


  for (int bit = 29; bit < 35; bit++) {

    temporary =
        temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;

    temporary_input >>= 1;
  }


  this->impulse_array_[35] =
      ((parity_count & 1) == 0) ? 1 : 2;


  // ----------------------------------------------------------
  // DATE
  //
  // Parity covers bits 36..57.
  // ----------------------------------------------------------

  parity_count = 0;


  // ----------------------------------------------------------
  // Day of month
  // bits 36..41
  // ----------------------------------------------------------

  temporary_input =
      this->bin2bcd_(this->actual_day_);


  for (int bit = 36; bit < 42; bit++) {

    temporary =
        temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;

    temporary_input >>= 1;
  }


  // ----------------------------------------------------------
  // Day of week
  // bits 42..44
  // ----------------------------------------------------------

  temporary_input =
      this->bin2bcd_(this->day_of_week_);


  for (int bit = 42; bit < 45; bit++) {

    temporary =
        temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;

    temporary_input >>= 1;
  }


  // ----------------------------------------------------------
  // Month
  // bits 45..49
  // ----------------------------------------------------------

  temporary_input =
      this->bin2bcd_(this->actual_month_);


  for (int bit = 45; bit < 50; bit++) {

    temporary =
        temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;

    temporary_input >>= 1;
  }


  // ----------------------------------------------------------
  // Year
  // bits 50..57
  // ----------------------------------------------------------

  temporary_input =
      this->bin2bcd_(this->actual_year_);


  for (int bit = 50; bit < 58; bit++) {

    temporary =
        temporary_input & 1;

    this->impulse_array_[bit] =
        temporary + 1;

    parity_count += temporary;

    temporary_input >>= 1;
  }


  // ----------------------------------------------------------
  // Date parity bit 58
  // ----------------------------------------------------------

  this->impulse_array_[58] =
      ((parity_count & 1) == 0) ? 1 : 2;


  // ----------------------------------------------------------
  // Second 59
  //
  // No carrier reduction = DCF77 minute marker.
  // ----------------------------------------------------------

  this->impulse_array_[59] = 0;
}


// ============================================================
// CONFIG LOG
// ============================================================

void IDFDCF77Emitter::dump_config() {

  ESP_LOGCONFIG(TAG, "DCF77 Emitter:");

  LOG_PIN(
      "  LED Pin: ",
      this->led_pin_
  );

  ESP_LOGCONFIG(
      TAG,
      "  Carrier: ESPHome LEDC output"
  );

  ESP_LOGCONFIG(
      TAG,
      "  Carrier duty when enabled: 50%%"
  );

  ESP_LOGCONFIG(
      TAG,
      "  Initialized: %s",
      YESNO(this->is_initialized_)
  );
}

}  // namespace idf_dcf77_emitter
}  // namespace esphome
