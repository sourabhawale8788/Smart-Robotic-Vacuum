# Smart-Robotic-Vacuum
• Engineered fully autonomous indoor navigation using Raspberry Pi + ESP32-CAM with PWM-based DC motor control and real-time
sensor fusion (ultrasonic + IR over GPIO) written in bare-metal Embedded C — no Arduino framework dependency.
• Implemented a Python/OpenCV computer vision pipeline on Raspberry Pi for dynamic obstacle detection, avoidance, and real-time
path re-routing with zero human input.
• Coordinated vision processing and actuator control via UART serial communication between Raspberry Pi and ESP32, achieving
deterministic response times in a hard real-time embedded system.
• Tuned interrupt-driven GPIO handlers for collision avoidance, verified with oscilloscope and serial monitor to ensure consistent
sub-30ms latency under load.
