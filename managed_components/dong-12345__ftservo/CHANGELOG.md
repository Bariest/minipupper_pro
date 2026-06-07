# Changelog

## [1.0.0] - 2026-05-01

### Added

- Initial release of ftservo component for ESP-IDF
- Support for SCSCL series servos (SCS15/SCS20/SCS30)
- Support for SMS_STS series servos (SMS03/SMS05/STS30)
- Support for HLS series servos (HLS15/HLS20)
- Hardware abstraction layer for ESP32 UART interface
- RS485 half-duplex mode support
- One-click initialization API: `ftServo_InitWithType()`
- Position, speed, acceleration, and torque control
- PWM mode for SCSCL servos
- Wheel mode for SMS_STS servos
- Synchronous write for multi-servo control
- Feedback reading (position, speed, load, voltage, temperature, current)
- Comprehensive example project in `examples/basic/`
