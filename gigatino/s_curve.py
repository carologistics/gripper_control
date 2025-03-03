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
import matplotlib.pyplot as plt
import numpy as np


class SCurveMotionController:
    def __init__(self, peak_speed, max_accel, max_jerk):
        self.peak_speed = peak_speed
        self.max_accel = max_accel
        self.max_jerk = max_jerk
        self.speed = 0
        self.accel = 0
        self.braking_distance = 0.0

    def set_braking_distance(self, braking_dist):
        self.braking_distance = braking_dist

    def compute(self, target, current, dt):
        distance_remaining = target - current
        if abs(distance_remaining) < 0.5:
            self.speed = 0
            self.accel = 0
            return 0, 0
        elif distance_remaining > 0:
            # Moving forward
            if self.braking_distance >= distance_remaining:
                self.accel -= self.max_jerk * dt  # Deceleration phase
            else:
                self.accel += self.max_jerk * dt  # Acceleration phase
        else:
            # Moving backward
            if self.braking_distance >= -distance_remaining:
                self.accel += self.max_jerk * dt  # Deceleration (negative direction)
            else:
                self.accel -= self.max_jerk * dt  # Acceleration (negative direction)

        # Constrain acceleration within limits
        self.accel = np.clip(self.accel, -self.max_accel, self.max_accel)
        # if distance_remaining > 0:

        # Compute velocity using constrained acceleration
        self.speed += self.accel * dt
        self.speed = np.clip(self.speed, -self.peak_speed, self.peak_speed)

        return self.speed, self.accel


# Simulation Parameters

dt = 0.01  # Time step (10ms)
max_accel = 2000.0  # Maximum acceleration
max_jerk = (max_accel / 10) / dt  # Maximum jerk
target_position = 20.0  # Target position
current_position = 220.0  # Start position
peak_speed = 2000.0
dist = abs(target_position - current_position)
braking_distance = 10.0  # TODO: This is the term we need to calculate based on dist

simulation_time = 10.0  # Total simulation time

# Initialize controller
controller = SCurveMotionController(peak_speed, max_accel, max_jerk)

controller.set_braking_distance(braking_distance)
# Logging for plotting
time_vals = [0]
speed_vals = [0]
accel_vals = [0]
pos_vals = [0]

# Run simulation
t = 0
while t < simulation_time:
    speed, accel = controller.compute(target_position, current_position, dt)
    current_position += speed * dt

    # Store values for plotting
    time_vals.append(t)
    speed_vals.append(speed)
    accel_vals.append(accel)
    pos_vals.append(current_position)

    t += dt

# Plot results
fig, axs = plt.subplots(2, 1, figsize=(8, 8))
ax1 = axs[0]
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Speed (mm/s)", color="tab:blue")
ax1.plot(time_vals, speed_vals, label="Speed", color="tab:blue")
ax1.tick_params(axis="y", labelcolor="tab:blue")


ax2 = ax1.twinx()
ax2.set_ylabel("Acceleration (mm/s²)", color="tab:red")
# ax2.plot(time_vals, np.array(accel_vals) / dt, label="Acceleration (mm/s²)", color='tab:red', linestyle="dashed")
ax2.plot(time_vals, accel_vals, label="Acceleration", color="tab:red", linestyle="dashed")
ax2.tick_params(axis="y", labelcolor="tab:red")

# Plot Position vs Time
axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Position (mm)")
axs[1].plot(time_vals, pos_vals, label="Position", color="tab:green")
axs[1].set_title("S-Curve Position Profile")

fig.tight_layout()
plt.title("S-Curve Speed & Acceleration Profile")
plt.show()
