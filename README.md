# Gripper Control

This repository contains ROS 2 packages related to the Carologistics gripping components.

## Getting Started

### Prerequisites

Ensure you have the following installed:
- ROS 2 (Foxy, Galactic, or Humble)
- Colcon build tool
- Arduino IDE

### Building the Package

To build the package, navigate to the workspace directory and run:

```sh
colcon build --symlink-install --packages-select arduino
```

### Running the Simulator

To start the simulator, open a terminal and run:

```sh
ros2 launch gripper_tf_simulator gripper_tf.launch.py
```

In Rviz2, add the TF display to visualize the transformations.

### Running the Action Server

To run the action server, open a terminal and run:

```sh
ros2 run gripper_tf_simulator gripper_action_server
```

### Testing the Action Server

To test whether the server works, you can run the action client:

```sh
ros2 run gripper_tf_simulator gripper_action_client
```

## Important Notice

Make sure to set the IP range of the Ethernet port to match the IP range of the server:

- IP: `192.168.1.20`
- Subnet: `255.255.255.0`

## Arduino Setup

### Wiring the Ethernet Shield

Ensure the Ethernet shield is properly connected to the Arduino Giga. The pin connections are as follows:

| Ethernet Shield | Arduino Giga |
|-----------------|--------------|
| MISO            | PIN_SPI_MISO (89u) |
| MOSI            | PIN_SPI_MOSI (90u) |
| SCK             | PIN_SPI_SCK (91u) |
| SS              | PIN_SPI_SS (18u) |
| VCC             | 5V or 3.3V |
| GND             | GND |

### Uploading the Sketch

1. Open the Arduino IDE.
2. Load the `ArduinoSketch.ino` file from the `ArduinoSketch` directory.
3. Ensure the correct board and port are selected.
4. Upload the sketch to the Arduino Giga.

### Verifying the Ethernet Connection

After uploading the sketch, open the Serial Monitor in the Arduino IDE. You should see the following output:

```
Arduino Giga Ethernet server started
IP Address: 192.168.1.100
```

You can then ping the Arduino Giga from your computer to verify the connection:

```sh
ping 192.168.1.100
```

## License

This project is licensed under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file for details.

## Maintainers

- Your Name ([your@email.com](mailto:your@email.com))

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.


### For now the Ethernet the libary inlcuded in the ArduinoSketch folder is the only one working for magical reasons.
