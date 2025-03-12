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
import random
import socket
import threading
import time

import msgpack

# Global socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 8888))  # Listen on all interfaces, port 8889

# Variable to be updated after delay
delay_seconds = 5  # Set the delay time (in seconds)
curr_feedback = {
    "busy": False,
    "stepper_directions": [False, False, False, False],
    "stepper_positions": [0.0, 0.0, 0.0, 0.0],
    "servo_positions": [0.0, 0.0],
    "stepper_endstops": [True, True, True, True],
    "wp_sensor": False,
    "referenced": False,
    "command_index": 0,
}
step_sizes = [8.0, 4.0, 8.0, 8.0]  # so 250mm in 250/8*0.1 = 3,125s  # so 180deg in 4.5s  # so 150mm in 1.875s  # unused
home_pos = {"target_mot_x": 0.0, "target_mot_yaw": 45.0, "target_mot_z": 100.0, "target_mot_u": 0.0}
control_freq = 0.1
feedback_freq = 0.1
thresholds = [0.5, 0.2, 0.5, 0.1]

index_map = {"target_mot_x": 0, "target_mot_yaw": 1, "target_mot_z": 2, "target_mot_u": 3}
reverse_index_map = {v: k for k, v in index_map.items()}
curr_command = {}
target_pos_stepper = {}
cancel_event = threading.Event()  # Event to cancel the delayed task
stop_event = threading.Event()
variable_lock = threading.Lock()


def send_udp_message(ip: str, port: int, message_dict: dict):
    for key, value in message_dict.items():
        message_dict[key] = value

    packed_message = msgpack.packb(message_dict)  # Pack the dictionary
    sock.sendto(packed_message, (ip, port))


def fake_movement():
    """Sets the variable after a delay, but checks if it should be canceled."""
    # Wait until the delay or the event is set
    global target_pos_stepper
    global curr_feedback
    global step_sizes
    global control_freq
    target_reached = False
    while not target_reached:
        if cancel_event.is_set():  # If event is set, cancel the delay
            print("Delay canceled due to new message.", flush=True)
            return
        with variable_lock:
            target_reached = True
            for k in target_pos_stepper.keys():
                index = index_map[k]
                diff = target_pos_stepper[k] - curr_feedback["stepper_positions"][index]
                if abs(diff) > step_sizes[index]:
                    target_reached = False
                    curr_feedback["stepper_positions"][index] += step_sizes[index] if diff > 0 else -step_sizes[index]
                    curr_feedback["stepper_directions"][index] = diff > 0
                elif abs(diff) > thresholds[index]:
                    curr_feedback["stepper_positions"][index] = target_pos_stepper[k] + random.uniform(
                        -thresholds[index], thresholds[index]
                    )
        time.sleep(control_freq)
    with variable_lock:
        if curr_command["command"] == "CALIBRATE":
            curr_feedback["referenced"] = True
        curr_feedback["busy"] = False


def listen_for_messages():
    global curr_feedback
    global curr_command
    global target_pos_stepper
    sock.settimeout(1.0)
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(1024)
            unpacked_data = msgpack.unpackb(data, raw=False)
        except TimeoutError:
            continue
        except socket.error:
            continue
        with variable_lock:
            curr_command = unpacked_data
            print(unpacked_data, flush=True)
            if "command_index" in unpacked_data:
                curr_feedback["command_index"] = unpacked_data["command_index"]
        # Cancel the previous delay (if any) before starting a new one
        cancel_event.set()
        cancel_event.clear()

        target_pos_stepper = {}
        if curr_command["command"] == "MOVE":
            target_pos_stepper = {k: v for k, v in curr_command.items() if k.startswith("target_mot_")}
        if curr_command["command"] == "CALIBRATE":
            target_pos_stepper = {"target_mot_x": 0.0, "target_mot_yaw": 0.0, "target_mot_z": 0.0, "target_mot_u": 0.0}
        if curr_command["command"] == "HOME":
            target_pos_stepper = home_pos
        if curr_command["command"] == "STOP":
            with variable_lock:
                curr_feedback["busy"] = False
        if target_pos_stepper:
            with variable_lock:
                curr_feedback["busy"] = True
            threading.Thread(target=fake_movement, daemon=True).start()


def periodic_sender():
    global curr_feedback
    global feedback_freq
    while not stop_event.is_set():
        with variable_lock:
            for index, pos in enumerate(curr_feedback["stepper_positions"]):
                curr_feedback["stepper_endstops"][index] = pos < thresholds[index]
            send_udp_message("127.0.0.1", 8889, curr_feedback)
        time.sleep(feedback_freq)


if __name__ == "__main__":
    # Start listener thread
    listener_thread = threading.Thread(target=listen_for_messages, daemon=True)
    listener_thread.start()
    sender_thread = threading.Thread(target=periodic_sender, daemon=True)
    sender_thread.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_event.set()
        listener_thread.join()
        sender_thread.join()
        exit(0)
