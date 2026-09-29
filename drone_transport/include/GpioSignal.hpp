#pragma once

#include <gpiod.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>
#include <unordered_map>

class GpioSignal {
  struct GpioChipWrapper {
    void operator()(gpiod_chip* c) const
    {
      if (c)
        gpiod_chip_close(c);
    }
  };

  std::unique_ptr<gpiod_chip, GpioChipWrapper> chip_;
  std::unordered_map<unsigned int, gpiod_line*> requested_lines_;

  gpiod_line* get_requested_line(unsigned int line) const
  {
    auto it = requested_lines_.find(line);
    if (it == requested_lines_.end()) {
      throw std::runtime_error("gpio line not requested before use");
    }
    return it->second;
  }

public:
  GpioSignal(const char* gpio_chip_name)
    : chip_([&gpio_chip_name]() {
      gpiod_chip* c = gpiod_chip_open_by_name(gpio_chip_name);

      if (!c) {
        throw std::runtime_error("cannot occupy gpio chip");
      }

      return std::unique_ptr<gpiod_chip, GpioChipWrapper>(c, GpioChipWrapper{});
    }())
  {
  }

  void request_output(unsigned int line, int default_value)
  {
    gpiod_line* l = gpiod_chip_get_line(chip_.get(), line);
    if (!l) {
      throw std::runtime_error("cannot get gpio line");
    }
    if (gpiod_line_request_output(l, "drone", default_value) < 0) {
      throw std::runtime_error("cannot request gpio line as output");
    }
    requested_lines_[line] = l;
  }

  void set_high(unsigned int line)
  {
    if (gpiod_line_set_value(get_requested_line(line), 1) < 0) {
      throw std::runtime_error("cannot set gpio line high");
    }
  }

  void set_low(unsigned int line)
  {
    if (gpiod_line_set_value(get_requested_line(line), 0) < 0) {
      throw std::runtime_error("cannot set gpio line low");
    }
  }

  void pulse_high(unsigned int line, std::chrono::milliseconds hold)
  {
    set_high(line);
    std::this_thread::sleep_for(hold);
    set_low(line);
  }

  ~GpioSignal()
  {
    for (auto& [line, ptr] : requested_lines_) {
      gpiod_line_release(ptr);
    }
  }
};