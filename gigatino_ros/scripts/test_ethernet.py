#!/bin/env python
# Copyright (c) 2025 Carologistics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import json
import socket
import sys

import msgpack


def send_udp_message(ip: str, port: int, message: str):
    try:
        # Create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        message_dict = json.loads(message)  # Convert string to dictionary
        for key, value in message_dict.items():
            if isinstance(value, (float, int)):  # Check if the value is a number
                message_dict[key] = float(value)  # Cast to 32-bit float

        # Send message
        print(msgpack.packb(message_dict))
        sock.sendto(msgpack.packb(message_dict), (ip, port))
        print(f"Message sent to {ip}:{port}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <IP> <Port> <Message>")
        sys.exit(1)

    ip = sys.argv[1]
    port = int(sys.argv[2])
    message = sys.argv[3]

    send_udp_message(ip, port, message)
