# Boardoza Jet Engine Control Interactive Turbofan Project

The **Boardoza Jet Engine Project** is an ESP32-based interactive turbofan engine demonstrator designed to recreate the visual appearance, sound, and operating behavior of a jet engine. The system combines a **4.3-inch capacitive touchscreen**, a **6V 25mm DC gearmotor**, a **VNH5019A motor driver**, and a **VL53L7CX-1 Time-of-Flight (ToF) sensor**. Its software-controlled motor ramp simulates turbine acceleration and deceleration, while proximity detection enables automatic engine activation when a visitor approaches.

The system integrates a **DFPlayer Mini** for synchronized engine audio and music playback, a **PCF8563 RTC** for timekeeping, and a **16-LED addressable RGB strip** for programmable lighting effects. The touchscreen provides control over engine operation, proximity sensitivity, lighting, music, Wi-Fi configuration, and clock settings. The project is suitable for **STEM education, aviation demonstrations, interactive exhibitions, and desktop displays**.

<h2 align="center">Jet Motor</h2>
<p align="center">
  <img src="./assets/Jet Motor.gif" alt="Boardoza Jet Motor" width="400">
</p>

---

## Key Features

- **Realistic Jet Engine Simulation:** Implements a software-controlled motor ramp with dead-band mapping, slew limiting, and a decaying startup pulse to recreate smooth turbine acceleration and deceleration with live estimated RPM feedback.
- **Automatic Proximity Detection:** Uses a 64-zone Time-of-Flight sensor with adjustable distance thresholds, confirmation filtering, and release hysteresis to activate the engine automatically when a visitor approaches.
- **Synchronized Engine Audio:** Integrates a DFPlayer Mini audio module for engine sound effects, automatic audio fading during startup and shutdown, independent music playback, and mute control.
- **Programmable RGB Lighting:** Supports Rainbow, Solid, Fade, Ping, and Chase lighting effects with an interactive hue wheel, saturation adjustment, independent brightness control, and live hexadecimal color preview.
- **Interactive Touchscreen Interface:** Features six navigable pages with horizontal swipe gestures, engine controls, music selection, lighting configuration, proximity settings, and an automatic clock screensaver.
- **Wireless Configuration and Time Synchronization:** Provides on-device Wi-Fi scanning, touchscreen keyboard input, NTP time synchronization, and persistent configuration storage using Non-Volatile Storage (NVS).

---

## System Architecture

The Jet Engine Control system combines an **ESP32-S3 controller**, a capacitive touchscreen, motor control electronics, proximity sensing, audio playback, and addressable RGB lighting to create an interactive turbofan engine demonstrator.

### Main Controller & Display

| Component | Model |
| :--- | :--- |
| Microcontroller |  [Boardoza ESP32-S3 Pulse](https://github.com/Boardoza/Boardoza_ESP32-S3_Pulse_Development_Board) |
| Touchscreen |  [FN043AM030](https://www.ozdisan.com/p/tft-paneller-381/fannal-fn043am030-v1-0-535823?tab=productDetails) |
| Display Driver |  [Boardoza FT800 TFT Driver](https://github.com/Boardoza/Boardoza_FT800_TFT_Driver_Breakout_Board) |

### Motor Control & Power

| Component | Model |
| :--- | :--- |
| DC Gearmotor |  [6V 25mm 130 RPM DC Motor](https://www.ozdisan.com/) |
| Motor Driver |  [Boardoza VNH5019](https://github.com/Boardoza/Boardoza_VNH5019_H_Bridge_Motor_Driver_Module) |
| Voltage Regulator |  [Boardoza TPS62130A](https://github.com/Boardoza/Boardoza_TPS62130A_3V3_3A_Buck_Converter_Breakout_Board) |
| USB Type-C Converter |  [Boardoza USB Type C](https://github.com/Boardoza/Boardoza_USB_Type_C_Breakout_Board) |

### Sensors & Connectivity

| Component | Model |
| :--- | :--- |
| Distance Sensor |  [Boardoza VL53L7CX-1 ToF BB](https://github.com/Boardoza/Boardoza_VL53L7CX-1_Multizone_Time-of-Flight_Distance_Sensor_Breakout_Board) |
| Real-Time Clock |  [Boardoza PCF8563T](https://github.com/Boardoza/Boardoza_PCF8563T_RTC_I2C_Breakout_Board) |
| I²C Hub |  [Boardoza I²C Hub Module](https://github.com/Boardoza/Boardoza_I2C_Hub_I2C_Bus_Distribution_Module) |

### Audio & Lighting

| Component | Model |
| :--- | :--- |
| MP3 Player |  [DFPlayer Mini MP3 Player](https://www.ozdisan.com/p/arduino-sensorleri-ve-modulleri-613/dfrobot-dfr0768-958463?s=Mini+MP3+Player&tab=productDetails) |
| Speaker |  [DXYD57W-32F-8A](https://www.ozdisan.com/p/hoparlorler-431/duoxing-dxyd57w-32f-8a-1595184?tab=productDetails) |
| RGB Lighting |  [Boardoza Phoenix-4](https://github.com/Boardoza/Boardoza_Phoenix4_Modular_ARGB_LED_Breakout_Board) |

---

## Board Pinout

The complete wiring diagram for the Jet Engine Control project is provided below. It includes the connections between the ESP32 controller, motor driver, distance sensor, audio module, touchscreen, and others.

[Boardoza Jet Motor Pinout Diagram](./assets/Jet%20Motor%20Pinout%20Diagram.pdf)

---

## CAD Files

[Boardoza Jet Motor.3mf ](./CAD%20files/Jet_Motor.3mf/)  
[Boardoza Jet Motor.step](./CAD%20files/Jet_Motor.step)

---

## Version History

- v1.0 – Initial release

---

## Support

- For questions or technical support, please contact **support@boardoza.com**

---

## License

This repository contains both hardware and software components:

### **Hardware Design**

[![CC BY-SA 4.0][cc-by-sa-shield]][cc-by-sa]

All hardware design files are licensed under [Creative Commons Attribution-ShareAlike 4.0 International License][cc-by-sa].

[cc-by-sa]: http://creativecommons.org/licenses/by-sa/4.0/
[cc-by-sa-shield]: https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg

### **Software/Firmware**

[![BSD-3-Clause][bsd-shield]][bsd]

All software and firmware are licensed under [BSD 3-Clause License][bsd].

[bsd]: https://opensource.org/licenses/BSD-3-Clause
[bsd-shield]: https://img.shields.io/badge/License-BSD%203--Clause-blue.svg
