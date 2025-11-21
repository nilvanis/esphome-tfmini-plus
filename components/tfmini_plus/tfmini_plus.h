#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/defines.h"

#ifdef USE_API
#include "esphome/components/api/custom_api_device.h"
#endif

namespace esphome {
namespace tfmini_plus {

static const uint8_t TFMP_FRAME_SIZE = 9;
static const uint8_t TFMP_REPLY_SIZE = 8;
static const uint8_t TFMP_COMMAND_MAX = 8;

// Command IDs copied from the reference Arduino library
static const uint32_t GET_FIRMWARE_VERSION = 0x00010407;
static const uint32_t TRIGGER_DETECTION = 0x00040400;
static const uint32_t SOFT_RESET = 0x00020405;
static const uint32_t HARD_RESET = 0x00100405;
static const uint32_t SAVE_SETTINGS = 0x00110405;
static const uint32_t SET_FRAME_RATE = 0x00030606;
static const uint32_t SET_BAUD_RATE = 0x00060808;
static const uint32_t ENABLE_OUTPUT = 0x01070505;
static const uint32_t DISABLE_OUTPUT = 0x00070505;

static const uint16_t FRAME_0 = 0x0000;

enum class MeasurementStatus {
  OK = 0,
  HEADER,
  CHECKSUM,
  WEAK_SIGNAL,
  STRONG_SIGNAL,
  FLOOD_LIGHT,
};

enum class StatusCode {
  READY,
  SERIAL_ERR,
  HEADER,
  CHECKSUM,
  TIMEOUT,
  PASS,
  FAIL,
  I2C_READ,
  I2C_WRITE,
  I2C_LENGTH,
  WEAK,
  STRONG,
  FLOOD,
  MEASURE,
  OTHER,
  OFFLINE,
  SLEEPING
};

struct FrameData {
  int16_t distance_cm{0};
  int16_t strength{0};
  int16_t temperature_c{0};
  MeasurementStatus status{MeasurementStatus::OK};
};

enum class DeviceState { INIT, ONLINE, OFFLINE, SLEEPING };

class TFMiniPlusComponent : public PollingComponent, public uart::UARTDevice
#ifdef USE_API
    ,
                            public api::CustomAPIDevice
#endif
{
 public:
  void set_distance_sensor(sensor::Sensor *sensor) { this->distance_sensor_ = sensor; }
  void set_signal_strength_sensor(sensor::Sensor *sensor) { this->signal_strength_sensor_ = sensor; }
  void set_temperature_sensor(sensor::Sensor *sensor) { this->temperature_sensor_ = sensor; }
#ifdef USE_TFMINI_PLUS_STATUS_SENSOR
  void set_status_sensor(text_sensor::TextSensor *sensor) { this->status_sensor_ = sensor; }
#endif

  void set_frame_rate(uint16_t frame_rate) { this->frame_rate_ = frame_rate; }
  void set_soft_reset(bool soft_reset) { this->soft_reset_ = soft_reset; }
  void set_save_settings(bool save_settings) { this->save_settings_ = save_settings; }

  float get_setup_priority() const override { return setup_priority::DATA; }

  void setup() override;
  void dump_config() override;
  void update() override;

#ifdef USE_API
  void sleep_service();
  void wake_service();
#endif

 protected:
  bool read_frame_(FrameData &data);
  bool send_command_(uint32_t command, uint32_t param = 0);
  bool apply_frame_rate_(uint16_t frame_rate);
  void record_error_(StatusCode status, uint32_t now);

  void publish_online_(const FrameData &data);
  void publish_unavailable_();
  void set_status_(StatusCode status);
  void mark_offline_(const char *reason, StatusCode status = StatusCode::OFFLINE);

  void flush_input_();

  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *signal_strength_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
#ifdef USE_TFMINI_PLUS_STATUS_SENSOR
  text_sensor::TextSensor *status_sensor_{nullptr};
#endif

  float last_distance_{NAN};
  float last_signal_{NAN};
  float last_temperature_{NAN};
  bool have_distance_{false};
  bool have_signal_{false};
  bool have_temperature_{false};

  uint16_t frame_rate_{100};
  bool soft_reset_{false};
  bool save_settings_{false};

  DeviceState state_{DeviceState::INIT};
  uint32_t last_retry_ms_{0};
  bool published_unavailable_{false};
  StatusCode last_status_{StatusCode::OFFLINE};
  StatusCode last_published_status_{StatusCode::OTHER};
  bool has_published_status_{false};
  uint32_t wake_grace_until_{0};
  uint32_t last_sleep_unavailable_ms_{0};
  uint32_t last_good_frame_ms_{0};
  uint32_t error_window_start_ms_{0};
  uint32_t error_count_window_{0};
  uint32_t last_error_log_ms_{0};
};

}  // namespace tfmini_plus
}  // namespace esphome
