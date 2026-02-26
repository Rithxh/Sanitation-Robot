# Decontamination Bot

An autonomous line-tracking robot that navigates to designated destinations and dispenses aerosol disinfectant spray. It uses a PID algorithm for line following and a color sensor to identify target locations.

---

## Dependencies

Install via Arduino IDE Library Manager:
- `L298N`
- `QTRSensors`

---

## Pin Configuration

```cpp
// Motors
#define ENA 4   #define AIN1 16  #define AIN2 17
#define ENB 19  #define BIN1 5   #define BIN2 18

// IR Line Sensors
#define IR1 32  #define IR2 25  #define IR3 26
#define IR4 27  #define IR5 14

// Color Sensor
#define s0 13  #define s1 23  #define s2 22
#define s3 21  #define sensorOut 34
```

---

## Calibration

On startup, the built-in LED turns on — slowly move the bot over the line until the LED turns off (~400 cycles). The bot will then begin navigating automatically.

---

## PID Tuning

```cpp
double Kp = 0.4;             // Increase for faster correction
double Kd = 0.0;             // Increase to reduce oscillation
unsigned char MAX_SPEED = 150;
```

---

## Color Codes

| Value | Color  |
|-------|--------|
| 1     | Red    |
| 2     | Green  |
| 3     | Blue   |
| 4     | Black  |
| -1    | None   |

When a color is detected, the bot slows down and triggers the spray mechanism.