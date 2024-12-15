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

#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <string>

#define SERIAL_PORT_READ_BUF_SIZE 256

class SerialPortError : public std::runtime_error {
public:
  explicit SerialPortError(const std::string &msg) : std::runtime_error(msg) {}
};

class SerialPort {
public:
  SerialPort(const std::string &port,
             boost::function<void(const std::string &)> receive_callback,
             boost::function<void()> disconnect_callback,
             unsigned int baud_rate = 115200,
             const std::string &start_of_command = "AT ",
             const std::string &end_of_command = "+");
  ~SerialPort();

  bool write(const std::string &buf);
  bool write(const char *buf, const int &size);

private:
  void async_read_some_();
  void on_receive_(const boost::system::error_code &ec,
                   size_t bytes_transferred);

  boost::asio::io_service io_service_;
  std::shared_ptr<boost::asio::serial_port> port_;
  boost::thread serial_thread_;
  volatile bool terminate_thread_ = false;

  char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE];
  std::string read_buf_str_;
  std::string start_of_command_;
  std::string end_of_command_;

  boost::function<void(const std::string &)> receive_callback_;
  boost::function<void()> disconnect_callback_;
  boost::mutex port_mutex_;
};

#endif // SERIAL_PORT_HPP
