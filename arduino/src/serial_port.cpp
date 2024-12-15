// Copyright (c) 2024 Carologistics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "arduino/serial_port.hpp"

using serial_port_base = boost::asio::serial_port_base;

SerialPort::SerialPort(
    const std::string &port,
    boost::function<void(const std::string &)> receive_callback,
    boost::function<void()> disconnect_callback, unsigned int baud_rate,
    const std::string &start_of_command, const std::string &end_of_command) {
  receive_callback_ = receive_callback;
  disconnect_callback_ = disconnect_callback;
  start_of_command_ = start_of_command;
  end_of_command_ = end_of_command;

  try {
    port_ = std::make_shared<boost::asio::serial_port>(io_service_);
    boost::system::error_code ec;
    port_->open(port, ec);
    if (ec) {
      throw SerialPortError(ec);
    }

    port_->set_option(serial_port_base::baud_rate(baud_rate));
    port_->set_option(serial_port_base::character_size(8));
    port_->set_option(
        serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    port_->set_option(serial_port_base::parity(serial_port_base::parity::none));
    port_->set_option(
        serial_port_base::flow_control(serial_port_base::flow_control::none));

    serial_thread_ = boost::thread([this]() {
      while (!terminate_thread_) {
        io_service_.run_one_for(std::chrono::seconds(2));
        if (!port_ || !port_->is_open()) {
          terminate_thread_ = true;
          disconnect_callback_();
        }
      }
    });

    async_read_some_();
  } catch (const boost::system::system_error &e) {
    throw SerialPortError(e.what());
  }
}

SerialPort::~SerialPort() {
  terminate_thread_ = true;
  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }
  if (port_ && port_->is_open()) {
    port_->close();
  }
}

bool SerialPort::write(const std::string &buf) {
  return write(buf.c_str(), buf.size());
}

bool SerialPort::write(const char *buf, const int &size) {
  if (!port_ || !port_->is_open())
    return false;
  boost::mutex::scoped_lock lock(port_mutex_);
  try {
    boost::asio::write(*port_, boost::asio::buffer(buf, size));
    return true;
  } catch (std::exception &) {
    return false;
  }
}

void SerialPort::async_read_some_() {
  if (!port_ || !port_->is_open())
    return;
  port_->async_read_some(
      boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
      boost::bind(&SerialPort::on_receive_, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void SerialPort::on_receive_(const boost::system::error_code &ec,
                             size_t bytes_transferred) {
  if (terminate_thread_)
    return;

  if (ec || !port_ || !port_->is_open()) {
    terminate_thread_ = true;
    disconnect_callback_();
    return;
  }

  static bool has_started = false;
  for (size_t i = 0; i < bytes_transferred; ++i) {
    bool is_starting = (i + start_of_command_.length() <= bytes_transferred) &&
                       (memcmp(read_buf_raw_ + i, start_of_command_.c_str(),
                               start_of_command_.length()) == 0);
    bool is_ending = (i + end_of_command_.length() <= bytes_transferred) &&
                     (memcmp(read_buf_raw_ + i, end_of_command_.c_str(),
                             end_of_command_.length()) == 0);

    if (has_started || is_starting) {
      has_started = true;
      read_buf_str_ += read_buf_raw_[i];
    }

    if (has_started && is_ending) {
      has_started = false;
      receive_callback_(read_buf_str_);
      read_buf_str_.clear();
    }
  }

  async_read_some_();
}
