# ESP32 LCD + Ultrasonic Sensor + Linear Buzzer

This project uses an ESP32 with:

- 16x2 I2C LCD Display
- DYP-A06BLYT-V1.1 Ultrasonic Sensor
- Linear Active Buzzer

The system measures distance and:

- Displays distance on LCD
- Shows `CLOSER!` or `FAR`
- Changes buzzer beep speed based on distance

## Features

- ESP-IDF based project
- I2C LCD interface
- Real-time ultrasonic distance measurement
- Linear beep speed control
- Closer object = Faster beeps
- Farther object = Slower beeps

---

# Components Used

| Component | Description |
|---|---|
| ESP32 | Main microcontroller |
| 16x2 LCD with I2C | Display |
| DYP-A06BLYT-V1.1 | Ultrasonic sensor |
| Active Buzzer | Audio alert |

---

# GPIO Connections

## LCD I2C

| LCD Pin | ESP32 Pin |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| VCC | 5V |
| GND | GND |

## Ultrasonic Sensor

| Sensor Pin | ESP32 Pin |
|---|---|
| TRIG | GPIO 4 |
| ECHO | GPIO 18 |
| VCC | 5V |
| GND | GND |

## Buzzer

| Buzzer Pin | ESP32 Pin |
|---|---|
| + | GPIO 16 |
| - | GND |

---

# Sensor Range

| Condition | Action |
|---|---|
| < 35 cm | Displays `CLOSER!` |
| >= 35 cm | Displays `FAR` |
| Closer object | Faster beeps |
| Farther object | Slower beeps |

Sensor valid range:

- Minimum: 30 cm
- Maximum: 200 cm

---

# Project Structure

```bash
main/
 ├── main.c
CMakeLists.txt
README.md
```

---

# How to Build

## Set ESP-IDF Environment

```bash
. $HOME/esp/esp-idf/export.sh
```

## Build Project

```bash
idf.py build
```

## Flash to ESP32

```bash
idf.py -p /dev/ttyUSB0 flash
```

## Open Serial Monitor

```bash
idf.py monitor
```

---

# Example Output

LCD Display:

```text
CLOSER!
32cm
```

or

```text
FAR
120cm
```

---

# Working Principle

1. Ultrasonic sensor measures object distance
2. ESP32 calculates distance
3. LCD updates live distance
4. Buzzer interval changes linearly:
   - Near object → Fast beeps
   - Far object → Slow beeps

---

# Future Improvements

- RTC clock integration
- OLED display support
- WiFi monitoring
- Mobile app integration
- Object detection counter

---

# Author

Andrea Monteiro

Built using ESP-IDF and ESP32.
