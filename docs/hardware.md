# 硬體與接線／Hardware and wiring

本頁整理已確認的裝置與程式接線，原始照片保存在 [`archive/photos/`](../archive/photos/)。未確認的模組額定電流不由照片外觀或程式反推。

This page records confirmed devices and code-level wiring. Original photos are retained in [`archive/photos/`](../archive/photos/). Unconfirmed module current ratings are not inferred from appearance or software.

| 項目／Device | 角色／Role |
| --- | --- |
| Jetson Orin Nano | 視覺推論、傳輸、感測整合與上層控制。Vision, networking, sensor integration, and high-level control. |
| STM32F412ZGT6 | M4 即時控制、PWM、UART 與四向測距保護；型號由 `.ioc` 與 Keil 工程確認。MCU control and protection; part identified in project files. |
| RPLIDAR C1 | 雷達掃描，經原轉接板連 Jetson USB。Lidar connected through the original USB adapter. |
| VL53L0X × 4 | 前、後、左、右距離量測。Front, rear, left, and right ranging. |
| USB 相機／USB camera | 車載影像擷取。Onboard image capture. |
| 4S LiPo、ESC、無刷馬達、舵機 | 動力與轉向。Battery, propulsion, and steering. |
| 車載 MiFi | 5G 行動網路分享。Onboard 5G connectivity. |

## 確認的電源路徑／Confirmed power topology

- 4S 電池 → ESC → 無刷馬達。
- ESC 內建穩壓輸出（BEC）→ 轉向舵機。
- 電池分接 → 藍色 LTC3780 類模組 → Jetson。
- Jetson USB → 相機與雷達轉接板。
- 電池分接 → D-SUN 電源模組 → M4 板的 ST-LINK 供電輸入位置。

The battery feeds the ESC/motor, an LTC3780-type module for Jetson, and a D-SUN module for the M4 board. The servo uses the ESC BEC; the camera and lidar adapter use Jetson USB. The M4 supply enters through the board's ST-LINK power-input location. This records the project wiring, not a universal connector-voltage specification.

## M4 程式接線／Firmware pin assignment

| 介面／Interface | 接線／Assignment |
| --- | --- |
| Jetson → M4 UART | Jetson TX → M4 PA3；M4 PA2 → Jetson RX；共地，板端 3.3 V 邏輯。Common ground and 3.3 V UART logic. |
| 舵機 PWM／Servo PWM | PA8 |
| ESC PWM | PA1 |
| ToF 時脈／SCL | PB8 |
| ToF 資料／SDA | PB9 |
| 前／後／左／右 XSHUT | PG10／PG11／PG12／PG13 |
| 前／後／左／右 7 位元 I²C 位址 | `0x30`／`0x31`／`0x32`／`0x33` |

ToF 使用開漏軟體 I²C 與分別控制的 XSHUT；既有驅動內嵌於 `main.c`。UART、PWM、感測器與電源輸入是不同用途的介面，部署時須以實際板卡接頭定義對照。

The ToF driver uses open-drain software I²C and separate XSHUT lines. It is embedded in `main.c`. UART logic, PWM signals, sensor supply, and board power input are distinct interfaces and must match the actual board connectors.

## 已寫入的車體設定／Stored geometry

車長 47.4 cm、車寬 32.8 cm、軸距 27.5 cm、後懸 14 cm、左右轉彎半徑各 48 cm；雷達從車尾向前 30 cm、橫向置中、原始 0° 朝前且角度順時針。設定來源為 [`vehicle_config.py`](../jetson/minitesla_sensors/vehicle_config.py)。

Stored dimensions are 47.4 cm length, 32.8 cm width, 27.5 cm wheelbase, 14 cm rear overhang, and 48 cm left/right turning radii. The lidar is centered laterally, 30 cm forward of the rear, with raw 0° forward and clockwise angles. These are retained project settings, not measurements newly taken during publication.
