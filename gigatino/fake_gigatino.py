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
import socket
import threading
import time

import msgpack

# Global socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 8888))  # Listen on all interfaces, port 8889

# Variable to be updated after delay
busy = False
command_index = 0
delay_seconds = 5  # Set the delay time (in seconds)

cancel_event = threading.Event()  # Event to cancel the delayed task
variable_lock = threading.Lock()


def send_udp_message(ip: str, port: int, message_dict: dict):
    for key, value in message_dict.items():
        message_dict[key] = value

    packed_message = msgpack.packb(message_dict)  # Pack the dictionary
    sock.sendto(packed_message, (ip, port))


def set_variable_later():
    """Sets the variable after a delay, but checks if it should be canceled."""
    print(f"Waiting for {delay_seconds} seconds before updating the variable...")

    # Wait until the delay or the event is set
    for _ in range(delay_seconds):
        if cancel_event.is_set():  # If event is set, cancel the delay
            print("Delay canceled due to new message.")
            return
        time.sleep(1)

    # If not canceled, update the variable
    global busy
    with variable_lock:
        busy = False
    print(f"Variable set: {busy}")


def listen_for_messages():
    global busy
    global command_index
    while True:
        data, addr = sock.recvfrom(1024)  # Buffer size of 1024 bytes
        unpacked_data = msgpack.unpackb(data, raw=False)
        print(f"Received from {addr}: {unpacked_data}")
        with variable_lock:
            busy = True
            print(unpacked_data)
            command_index = unpacked_data["command_index"]
        # Cancel the previous delay (if any) before starting a new one
        cancel_event.set()  # Set the event to cancel the previous delay
        cancel_event.clear()  # Clear the event for the new delay

        # Start a separate thread to set the variable later
        threading.Thread(target=set_variable_later, daemon=True).start()


def periodic_sender():
    global busy
    global command_index
    while True:
        with variable_lock:
            send_udp_message("127.0.0.1", 8889, {"command_index": command_index, "busy": busy})
        time.sleep(0.5)  # Send message every 2 seconds


if __name__ == "__main__":
    # Start listener thread
    listener_thread = threading.Thread(target=listen_for_messages, daemon=True)
    listener_thread.start()
    sender_thread = threading.Thread(target=periodic_sender, daemon=True)
    sender_thread.start()

    # Start periodic sender
    while True:
        time.sleep(1)
