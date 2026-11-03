#include "dcf77_emitter.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"

namespace esphome {
namespace dcf77_emitter {

static const char *TAG = "dcf77_emitter";

void DCF77Emitter::setup_timer_() {
  ESP_LOGD(TAG, "Using ESPHome scheduler");
}

void DCF77Emitter::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DCF77 Emitter...");
  
  // Setup LED and antenna pins
  this->led_pin_->setup();
  this->led_pin_->digital_write(false);
  this->antenna_pin_->setup();
  
  // Configure PWM for the DCF77 signal (77.5 kHz)
#ifdef USE_ESP32
  auto pin = this->antenna_pin_->get_pin();

    ESP_LOGE(TAG, "BEFORE LEDC ATTACH");
    
    bool result = ledcAttach(pin, 77500, 8);
    
    ESP_LOGE(TAG, "LEDC RESULT = %d", result);
    
    if (result) {
        ESP_LOGE(TAG, "LEDC OK");
    } else {
        ESP_LOGE(TAG, "LEDC FAILED");
    }
  ledcWrite(pin, 0);
#elif defined(USE_ESP8266)
  analogWriteFreq(77500);  // ESP8266 can't precisely do 77.5kHz but will get close
  analogWrite(this->antenna_pin_->get_pin(), 0);
#endif
  
  // Initialize impulse array
  code_time_();
  
  // Initialize state tracking for second synchronization
  this->sync_start_millis_ = millis();
  auto time = this->time_id_->now();
  if (time.is_valid()) {
    this->last_second_ = time.second;
  }

#ifdef USE_ESP32
  // For more precise initial synchronization
  int sync_count = 0;
  int start_second = time.second;
  
  // More aggressive sync for startup only (safe because it's just once)
  while (sync_count < 1000) { // Safety limit
    sync_count++;
    time = this->time_id_->now();
    if (!time.is_valid()) continue;
    if (time.second != start_second) {
      // Exact second boundary detected!
      ESP_LOGI(TAG, "Precise second boundary detected after %d checks", sync_count);
      break;
    }
    delayMicroseconds(500); // Much shorter delay for more precise detection
  }
#endif
  
  // Initialize timing resilience variables
  this->last_tick_time_ = 0;
  this->timing_drift_ms_ = 0;
  this->last_sync_millis_ = millis();
  
  ESP_LOGI(TAG, "DCF77 Emitter setup complete. Waiting for sync.");
}

void DCF77Emitter::loop() {
  // Handle sync switch state changes
  if (!this->sync_switch_->state) {
    if (this->is_initialized_) {
      ESP_LOGW(TAG, "DCF77 synchronization disabled by switch");
      this->is_initialized_ = false;
      
      // Disable the carrier and LED
      stop_carrier_();
      this->led_pin_->digital_write(false);
    }
    return;
  }
  
  // Non-blocking second synchronization for startup
  if (!this->is_initialized_) {
    auto current_time = this->time_id_->now();
    if (!current_time.is_valid()) {
      // Invalid time, just wait
      return;
    }
    
    // Check if the second has changed - this indicates we're at a second boundary
    if (current_time.second != this->last_second_) {
      ESP_LOGI(TAG, "Second transition detected after %u ms", millis() - this->sync_start_millis_);
      
      // Initialize impulse array and start scheduling ticks
      code_time_();
      this->impulse_count_ = 0;
      this->is_initialized_ = true;
      
      // Schedule first tick immediately
      schedule_next_tick_();
      
      ESP_LOGI(TAG, "DCF77 synchronization enabled. Starting signal generation");
    } else {
      // Check for timeout (5 seconds)
      if (millis() - this->sync_start_millis_ > 5000) {
        ESP_LOGW(TAG, "Second sync timeout - continuing anyway");
        code_time_();
        this->impulse_count_ = 0;
        this->is_initialized_ = true;
        
        // Schedule first tick immediately
        schedule_next_tick_();
      }
    }
    
    // Update the last second value for next comparison
    this->last_second_ = current_time.second;
  }

  // Periodically log status
  const uint32_t now = millis();
  if (now - this->last_status_log_ >= 10000) {
    this->last_status_log_ = now;
    auto time = this->time_id_->now();
    if (time.is_valid()) {
      ESP_LOGD(TAG, "DCF77 Status: %s, Time: %02d:%02d:%02d, DST: %s",
               this->is_initialized_ ? "Transmitting" : "Initializing",
               time.hour, time.minute, time.second,
               time.is_dst ? "ON" : "OFF");
    } else {
      ESP_LOGE(TAG, "DCF77 Status: Waiting for valid time source");
    }
  }

  if (this->is_initialized_) {
    static uint32_t last_valid_time = millis();
    auto time = this->time_id_->now();
    
    if (time.is_valid()) {
      last_valid_time = millis();
    } else if (millis() - last_valid_time > 30000) {
      // No valid time for 30 seconds - force resync
      ESP_LOGE(TAG, "No valid time for 30 seconds - forcing resynchronization");
      this->is_initialized_ = false;
      this->sync_start_millis_ = millis();
    }
  }
}

// Update the schedule_next_tick_ method to include timing resilience
void DCF77Emitter::schedule_next_tick_() {
  
  // Calculate next tick interval, compensating for accumulated drift
    App.scheduler.set_interval(
        this,
        "dcf77_tick",
        100,
        [this]() {
          this->dcf_out_tick();
        });
}

void DCF77Emitter::dcf_out_tick() {
  auto current_time = this->time_id_->now();
  if (!current_time.is_valid() || !this->is_initialized_)
    return;

  // Always recalculate time data on each tick (like Arduino implementation)
  code_time_();
  
  // Get current second
  int current_sec = current_time.second;
  
  // Only update last_second_ and reset impulse_count_ when second changes
  if (current_sec != this->last_second_) {
    // If we skipped a second, that indicates timing problems
    if ((this->last_second_ != -1) && 
        (current_sec != ((this->last_second_ + 1) % 60))) {
      ESP_LOGW(TAG, "Second transition irregular: %d → %d", 
               this->last_second_, current_sec);
      
      // Reset drift tracking since we just had a discontinuity
      this->timing_drift_ms_ = 0;
    }
    
    this->last_second_ = current_sec;
    this->impulse_count_ = 0;
  }
  
  // Generate signal based on current time
  generate_signal_(current_sec);
}

void DCF77Emitter::generate_signal_(int current_sec) {
    
//    ESP_LOGD(TAG,
//             "SEC=%d BIT=%d",
//             current_sec,
//             impulse_array_[current_sec]);
    
  switch (this->impulse_count_++) {
    case 0:
      // First 100ms: turn off carrier for pulse (except for second 59)
      if (this->impulse_array_[current_sec] != 0) {
        this->led_pin_->digital_write(false);
        stop_carrier_();
      } else {
        // For second 59: carrier stays on (no pulse)
        this->led_pin_->digital_write(true);
        setup_carrier_();
      }
      break;
      
    case 1:
      // At 100ms: turn on carrier for a logical "0"
      if (this->impulse_array_[current_sec] == 1) {
        this->led_pin_->digital_write(true);
        setup_carrier_();
      }
      break;
      
    case 2:
      // For logical 1, carrier on after 200ms
      this->led_pin_->digital_write(true);
      setup_carrier_();
      break;
      
    case 9:
      // Reset counter at end of 900ms cycle
      this->impulse_count_ = 0;
      
      // Log completion of minute
      if (current_sec == 59) {
        ESP_LOGD(TAG, "DCF77 minute complete. Time: %02d:%02d:%02d", 
                 actual_hours_, actual_minutes_, actual_second_);
      }
      break;
      
    default:
      // Nothing to do for other ticks
      break;
  }
}

void DCF77Emitter::setup_carrier_() {
  if (this->carrier_enabled_)
    return;
//ESP_LOGD(TAG, "Carrier ON");    
#ifdef USE_ESP32
  // Use exactly 50% duty cycle like in Arduino implementation
    ledcWrite(this->antenna_pin_->get_pin(), 127);
#elif defined(USE_ESP8266)
  analogWrite(this->antenna_pin_->get_pin(), 127);
#endif
  this->carrier_enabled_ = true;
}

void DCF77Emitter::stop_carrier_() {
  if (!this->carrier_enabled_)
    return;
//ESP_LOGD(TAG, "Carrier OFF");
#ifdef USE_ESP32
    ledcWrite(this->antenna_pin_->get_pin(), 0);
#elif defined(USE_ESP8266)
  analogWrite(this->antenna_pin_->get_pin(), 0);
#endif
  this->carrier_enabled_ = false;
}

void DCF77Emitter::dump_config() {
  ESP_LOGCONFIG(TAG, "DCF77 Emitter:");
  LOG_PIN("  Antenna Pin: ", this->antenna_pin_);
  LOG_PIN("  LED Pin: ", this->led_pin_);
}

int DCF77Emitter::bin2bcd_(int dato) {
  int msb, lsb;
  if (dato < 10)
    return dato;
  msb = (dato / 10) << 4;
  lsb = dato % 10;
  return msb + lsb;
}

void DCF77Emitter::code_time_() {
  auto time = this->time_id_->now();

  if (!time.is_valid())
    return;

  // DCF77 transmits the date/time of the NEXT minute.
  // timestamp + 60 correctly handles:
  // - minute/hour rollover
  // - midnight
  // - month rollover
  // - year rollover
  // - leap days
  auto next_time =
      esphome::ESPTime::from_epoch_local(time.timestamp + 60);

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
  this->day_of_week_ = ((next_time.day_of_week + 5) % 7) + 1;

  this->actual_second_ = time.second;
  if (this->actual_second_ >= 60)
    this->actual_second_ = 0;

  // Generate DCF77 signal bits
  int n, Tmp, TmpIn;
  int ParityCount = 0;

  // ---------------------------------------------------------
  // Bits 0..19
  // ---------------------------------------------------------

  // Default to logical 0 (100 ms carrier reduction)
  for (n = 0; n < 20; n++) {
    this->impulse_array_[n] = 1;
  }

  // ---------------------------------------------------------
  // DST / timezone bits
  //
  // Bit 17 = Z1
  // Bit 18 = Z2
  //
  // Summer time (CEST):
  // Z1 = 1, Z2 = 0
  //
  // Standard time (CET):
  // Z1 = 0, Z2 = 1
  // ---------------------------------------------------------

  if (time.is_dst) {
    // CEST
    this->impulse_array_[17] = 2;
    this->impulse_array_[18] = 1;
  } else {
    // CET
    this->impulse_array_[17] = 1;
    this->impulse_array_[18] = 2;
  }

  // ---------------------------------------------------------
  // Bit 20 - Start of encoded time information
  // ---------------------------------------------------------

  this->impulse_array_[20] = 2;

  // ---------------------------------------------------------
  // Minutes - bits 21..27
  // Parity - bit 28
  // ---------------------------------------------------------

  ParityCount = 0;

  TmpIn = bin2bcd_(this->actual_minutes_);

  for (n = 21; n < 28; n++) {
    Tmp = TmpIn & 1;

    this->impulse_array_[n] = Tmp + 1;

    ParityCount += Tmp;

    TmpIn >>= 1;
  }

  // Even parity
  this->impulse_array_[28] =
      ((ParityCount & 1) == 0) ? 1 : 2;

  // ---------------------------------------------------------
  // Hours - bits 29..34
  // Parity - bit 35
  // ---------------------------------------------------------

  ParityCount = 0;

  TmpIn = bin2bcd_(this->actual_hours_);

  for (n = 29; n < 35; n++) {
    Tmp = TmpIn & 1;

    this->impulse_array_[n] = Tmp + 1;

    ParityCount += Tmp;

    TmpIn >>= 1;
  }

  // Even parity
  this->impulse_array_[35] =
      ((ParityCount & 1) == 0) ? 1 : 2;

  // ---------------------------------------------------------
  // Date
  // ---------------------------------------------------------

  ParityCount = 0;

  // ---------------------------------------------------------
  // Day of month - bits 36..41
  // ---------------------------------------------------------

  TmpIn = bin2bcd_(this->actual_day_);

  for (n = 36; n < 42; n++) {
    Tmp = TmpIn & 1;

    this->impulse_array_[n] = Tmp + 1;

    ParityCount += Tmp;

    TmpIn >>= 1;
  }

  // ---------------------------------------------------------
  // Day of week - bits 42..44
  // ---------------------------------------------------------

  TmpIn = bin2bcd_(this->day_of_week_);

  for (n = 42; n < 45; n++) {
    Tmp = TmpIn & 1;

    this->impulse_array_[n] = Tmp + 1;

    ParityCount += Tmp;

    TmpIn >>= 1;
  }

  // ---------------------------------------------------------
  // Month - bits 45..49
  // ---------------------------------------------------------

  TmpIn = bin2bcd_(this->actual_month_);

  for (n = 45; n < 50; n++) {
    Tmp = TmpIn & 1;

    this->impulse_array_[n] = Tmp + 1;

    ParityCount += Tmp;

    TmpIn >>= 1;
  }

  // ---------------------------------------------------------
  // Year - bits 50..57
  // ---------------------------------------------------------

  TmpIn = bin2bcd_(this->actual_year_);

  for (n = 50; n < 58; n++) {
    Tmp = TmpIn & 1;

    this->impulse_array_[n] = Tmp + 1;

    ParityCount += Tmp;

    TmpIn >>= 1;
  }

  // ---------------------------------------------------------
  // Date parity - bit 58
  // ---------------------------------------------------------

  this->impulse_array_[58] =
      ((ParityCount & 1) == 0) ? 1 : 2;

  // ---------------------------------------------------------
  // Second 59
  //
  // No carrier reduction = DCF77 minute marker
  // ---------------------------------------------------------

  this->impulse_array_[59] = 0;
}

}  // namespace dcf77_emitter
}  // namespace esphome
