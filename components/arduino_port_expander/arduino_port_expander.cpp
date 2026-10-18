#include "arduino_port_expander.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace arduino_port_expander {

static const char *const TAG = "arduino_port_expander";

static const uint32_t CONFIGURE_TIMEOUT_MS = 5000;

static const uint8_t DIRECT_COMMAND = 0x96;
static const uint8_t ANTFREQ = 3;

static const uint8_t CMD_DIGITAL_READ = 0;
static const uint8_t CMD_ANALOG_READ = 1;  // + 1 byte: indice pin analogico 0..15 (A0..A15)
static const uint8_t CMD_WRITE_ANALOG = 2;
static const uint8_t CMD_WRITE_DIGITAL_HIGH = 3;
static const uint8_t CMD_WRITE_DIGITAL_LOW = 4;
static const uint8_t CMD_SETUP_PIN_OUTPUT = 5;
static const uint8_t CMD_SETUP_PIN_INPUT_PULLUP = 6;
static const uint8_t CMD_SETUP_PIN_INPUT = 7;

static const uint8_t CMD_SETUP_ANALOG_INTERNAL = 0x10;
static const uint8_t CMD_SETUP_ANALOG_DEFAULT = 0x11;

float ArduinoPortExpanderComponent::get_setup_priority() const { return setup_priority::IO; }

void ArduinoPortExpanderComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ArduinoPortExpander at %#02x ...", this->address_);
  uint8_t data[DIGITAL_BUFFER_LEN];
  uint32_t timeout = millis() + CONFIGURE_TIMEOUT_MS;

  while (this->read_register(CMD_DIGITAL_READ, data, DIGITAL_BUFFER_LEN) != i2c::ERROR_OK) {
    App.feed_wdt();
    if (millis() > timeout) {
      ESP_LOGE(TAG, "ArduinoPortExpander not available at 0x%02X", this->address_);
      this->mark_failed();
      return;
    }
  };
  this->write_register(this->analog_reference_, nullptr, 0);
  ESP_LOGCONFIG(TAG, "Successfully configured.");
}
void ArduinoPortExpanderComponent::loop() {
  uint32_t now = millis();
  if (now - this->last_read_ < 100)  // max ~10 letture/sec
    return;
  this->last_read_ = now;
  this->read_valid_ = this->read_gpio_();
}
void ArduinoPortExpanderComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "ArduinoPortExpander:");
  LOG_I2C_DEVICE(this)
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with ArduinoPortExpander failed!");
  }
}
bool ArduinoPortExpanderComponent::digital_read(uint8_t pin) {
  if (!this->read_valid_)
    this->read_valid_ = read_gpio_();
  uint8_t byte_index = pin / 8;
  uint8_t bit = pin % 8;
  uint8_t value = this->read_buffer_[byte_index];
  return value & (1 << bit);
}
void ArduinoPortExpanderComponent::digital_write(uint8_t pin, bool value) {
  if (this->is_failed())
    return;
  this->write_register(value ? CMD_WRITE_DIGITAL_HIGH : CMD_WRITE_DIGITAL_LOW, &pin, 1);
}
void ArduinoPortExpanderComponent::pin_mode(uint8_t pin, gpio::Flags flags) {
  ESP_LOGD(TAG, "Setting pin %d mode %d", pin, flags);
  if (flags == gpio::FLAG_INPUT) {
    this->write_register(CMD_SETUP_PIN_INPUT, &pin, 1);
  } else if (flags == (gpio::FLAG_INPUT | gpio::FLAG_PULLUP)) {
    this->write_register(CMD_SETUP_PIN_INPUT_PULLUP, &pin, 1);
  } else if (flags == gpio::FLAG_OUTPUT) {
    this->write_register(CMD_SETUP_PIN_OUTPUT, &pin, 1);
  } else {
    ESP_LOGE(TAG, "Invalid pin mode for pin %d", pin);
  }
  this->read_valid_ = false;
}
bool ArduinoPortExpanderComponent::read_gpio_() {
  bool success;

  success = (this->read_register(CMD_DIGITAL_READ, this->read_buffer_, DIGITAL_BUFFER_LEN) == i2c::ERROR_OK);

  if (!success) {
    if (!this->status_has_warning())
      ESP_LOGW(TAG, "Lost communication with ArduinoPortExpander - is the Mega down?");
    this->status_set_warning();
    return false;
  }

  if (this->status_has_warning()) {
    // Era offline ed e' appena tornato disponibile: dopo un reset del Mega tutti
    // i pin ripartono in INPUT, quindi risincronizziamo il pinMode di ogni pin
    // registrato prima che arrivino nuovi comandi digital_write().
    ESP_LOGW(TAG, "ArduinoPortExpander reconnected, resyncing %d pin(s)", (int) this->pins_.size());
        ESP_LOGW(TAG, "ArduinoPortExpander reconnected, resyncing %d pin(s)", (int) this->pins_.size());
    this->write_register(this->analog_reference_, nullptr, 0);
    for (auto *p : this->pins_) {
      this->pin_mode(p->get_pin(), p->get_flags());
    }
  }
  this->status_clear_warning();
  return true;
}

void ArduinoPortExpanderComponent::register_pin(ArduinoPortExpanderGPIOPin *pin) { this->pins_.push_back(pin); }

void ArduinoPortExpanderGPIOPin::setup() {
  this->parent_->register_pin(this);
  pin_mode(flags_);
  this->setup_ = true;
}
void ArduinoPortExpanderGPIOPin::pin_mode(gpio::Flags flags) { this->parent_->pin_mode(this->pin_, flags); }
bool ArduinoPortExpanderGPIOPin::digital_read() { return this->parent_->digital_read(this->pin_) != this->inverted_; }
void ArduinoPortExpanderGPIOPin::digital_write(bool value) {
  if (this->setup_)
    this->parent_->digital_write(this->pin_, value != this->inverted_);
}
size_t ArduinoPortExpanderGPIOPin::dump_summary(char *buffer, size_t len) const {
  return snprintf(buffer, len, "%u via ArduinoPortExpander", pin_);
}

float ArduinoPortExpanderComponent::analog_read(uint8_t pin) {
  uint16_t value = 0;
  uint8_t cmd[2] = {CMD_ANALOG_READ, pin};
  bool ok = (this->write_read(cmd, 2, (uint8_t *) &value, 2) == i2c::ERROR_OK);
  ESP_LOGV(TAG, "analog read pin: %d ok: %d value %d ", pin, ok, value);
  return value;
}

}  // namespace arduino_port_expander
}  // namespace esphome
