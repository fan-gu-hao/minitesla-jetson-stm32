# 系統架構／System architecture

## 工作分配／Execution model

| 執行單位／Unit | 職責／Responsibility |
| --- | --- |
| Jetson 主程序／Main process | 管理子程序與感測快照，整合影像摘要及控制狀態。Supervises workers and integrates sensor snapshots and control state. |
| 影像推論程序／Inference process | 擷取相機、執行 `best.pt`、追蹤物件、繪製標註與 JPEG 編碼。Captures frames, runs inference/tracking, annotates and encodes JPEG. |
| 影像傳輸程序／Video sender | TCP 連線、影格序號、顯示確認與有效期限。Handles transport, frame sequence, display acknowledgement, and freshness. |
| UART 執行緒／UART thread | 唯一串口持有者與寫入者，接收 M4 遙測、收發遠端控制與狀態。Owns serial I/O and exchanges remote commands/status. |
| 雷達執行緒／Lidar thread | 接收 SDK 資料，更新感測儲存並轉送電腦。Receives SDK samples, updates the store, and forwards scans. |
| 實驗性規劃程序／Experimental planner | 隔離停車幾何與規劃運算，只回傳具有效期限的建議。Runs parking geometry/planning and returns expiring proposals. |
| 電腦媒體子程序／Desktop media process | 影像與雷達接收、影像解碼，與操作視窗分離。Separates media reception/decoding from the UI. |
| 電腦主執行緒／Desktop UI thread | 呈現畫面、控制按鍵、狀態與可見影格確認。Renders UI and manages input/status/display acknowledgements. |
| M4 TIM5 中斷／M4 timer ISR | 每 1 ms 評估保護條件與輸出。Evaluates protection and output on a configured 1 ms timer. |

Jetson 以多程序（multiprocessing）的 `spawn` 建立子程序。佇列容量有限，優先保留最新資料；不為補上落後週期而重播舊油門。影像、規劃與紀錄工作不直接寫入馬達串口。

Jetson uses `multiprocessing` with `spawn`. Bounded queues prioritize recent data. Missed output periods are not recovered by replaying old throttle commands. Vision, planning, and recording do not write directly to the motor UART.

## 控制權與同步／Control authority and synchronization

- `ControlAuthority` 以同一把可重入鎖（RLock）排序模式切換與最終串口輸出；切換先中立，再接受新模式的有效命令。
- `session`、`seq`、`epoch` 與動作識別碼分別處理執行期、封包順序、模式代次及重複請求；這些欄位不是網路加密或身分驗證。
- `SingleInstance` 與串口排他協調新版程式的資源持有；舊測試程式仍應正常關閉。
- 感測資料鎖與控制鎖有各自職責，控制迴圈只取得需要的健康摘要，不持續複製完整影像輪廓。

`ControlAuthority` serializes mode transitions and final UART writes under one RLock. Session, sequence, mode epoch, and action IDs reject stale or duplicate requests; they do not provide cryptographic authentication. Process/serial exclusivity coordinates current applications, while old test processes must still be stopped. The control loop reads compact health information rather than repeatedly copying image polygons.

## 通訊／Communication

| 通道／Channel | 格式與用途／Format and purpose |
| --- | --- |
| 電腦 ↔ Jetson | UDP 5000：控制、心跳與狀態。Control, heartbeats, and status. |
| Jetson → 電腦影像 | TCP 7000：JPEG 影格、序號與確認。JPEG frames, sequence numbers, and acknowledgements. |
| 雷達 SDK → Jetson | UDP `127.0.0.1:6000`，`angle,distance,quality`。Local scan samples. |
| Jetson → 電腦雷達 | 轉送埠以 `sensor_config.json` 為準。Forwarding port is defined by the sensor configuration. |
| Jetson → M4 | UART 115200，7 位元組：`55 AA 02 SEQ speed turn CRC8`。Control frame. |
| M4 → Jetson | 79 位元組：6 位元組標頭、72 位元組資料、CRC8。Telemetry frame. |

M4 遙測包含四向距離、量測狀態與年齡、保護原因、命令與輸出、通訊計數。串流解析器可處理拆包、黏包與損壞封包重新同步。

M4 telemetry reports ranges, validity/age, guard reasons, commands/outputs, and communication counters. The parser handles partial/concatenated frames and resynchronizes after corruption.

### 雷達 C++ 讀取器與 Python 主程式／C++ lidar acquisition and Python application

雷達資料經過兩段介面：C1 先透過原廠 USB／UART 轉接板，由 C++ `ultra_simple` 以 `/dev/ttyUSB0`、460800 鮑率讀取；修改後的 C++ 程式再將掃描點送至本機 UDP `127.0.0.1:6000`。Python 主程式的 `LidarReceiver` 執行緒負責接收。

The C1 reaches the C++ `ultra_simple` application through its original USB/UART adapter, using `/dev/ttyUSB0` at 460800 baud. The modified C++ application then sends scan points to local UDP `127.0.0.1:6000`, where the Python application's `LidarReceiver` thread receives them.

| 處理階段／Stage | 程式行為／Behavior |
| --- | --- |
| 擷取與排序／Acquire and sort | `grabScanDataHq()`、`ascendScanData()` |
| 數值轉換／Conversion | 角度 `angle_z_q14 × 90 / 16384` 度、距離 `dist_mm_q2 / 4` 毫米，以及品質值／angle in degrees, distance in millimeters, and quality |
| 資料傳送／Transmission | `snprintf()` 組成 `angle,distance,quality`，`sendto()` 每點傳送一個 UDP 封包／one UDP datagram per point |
| 主程式接收／Reception | `minitesla.py` 建立並啟動 `LidarReceiver`；`parse_lidar_csv()` 檢查欄位、有限值與範圍／starts a receiver thread and validates fields, finite values, and ranges |
| 感測整合／Integration | `SensorStore.update_lidar()` 保存掃描資料與接收時間，建立區域摘要／stores scan samples and reception times and builds sector summaries |
| 遠端顯示／Remote display | 接收執行緒另送往 `pc_host:lidar_remote_port`；轉送錯誤與本機接收分開處理／forwards to the desktop independently of local reception |
| 編譯與部署／Build and deployment | SDK 根目錄 `make` → AArch64 `ultra_simple`；獨立雷達服務與主程式共同部署／build an AArch64 executable and deploy it as a separate service |

實作入口／Implementation: [C++ 修改](../jetson/rplidar_sdk-master/app/ultra_simple/main.cpp) · [Python 接收](../jetson/minitesla_sensors/io.py) · [感測資料整合](../jetson/minitesla_sensors/core.py) · [編譯與部署](operation.md)。

### 馬達干擾下的 UART 通訊／UART communication under motor EMI

實車整合時遇到馬達運轉造成的電磁干擾（EMI）。在材料與開發時程有限的條件下，我採用 UART 作為 Jetson 與 M4 的控制通道，並將 CRC-8 校驗、錯誤資料捨棄、串流重新同步與逾時保護整合進收發流程。這個方案已讓實車能正常運行，進一步硬體抑制留待後續改版。

CRC-8 用於偵測傳輸資料錯誤；接收端的封包解析與 M4 保護策略共同決定哪些命令可以進入控制流程，以及通訊中斷後的輸出狀態。

Motor-generated EMI was encountered during vehicle integration. Under material and schedule constraints, I used UART for the Jetson–M4 control channel and integrated CRC-8, corrupted-data rejection, stream resynchronization, and timeout protection. This enabled normal vehicle operation; further hardware suppression remains a future revision.

CRC-8 detects transmission errors. The receiver's parser and the M4 protection policy determine which commands enter the control flow and the output state following loss of valid communication.

| 層次／Layer | 實作行為／Implementation |
| --- | --- |
| 控制封包／Command frame | `55 AA 02 SEQ speed turn CRC8`，共 7 位元組／7 bytes |
| CRC-8 | 初值 `0x00`、多項式 `0x07`，涵蓋前 6 位元組／initial value 0x00, polynomial 0x07, covers the first six bytes |
| CRC 錯誤／CRC failure | 累計錯誤並以滑動視窗重新尋找封包；不更新控制命令或有效接收時間／count the error and resynchronize with a sliding window; leave the command and accepted-command timestamp unchanged |
| 不完整封包／Partial frame | 位元組間隔超過 20 ms 時清除未完成封包／discard an incomplete frame after an inter-byte gap over 20 ms |
| UART 硬體錯誤／UART hardware error | 檢查溢位、雜訊、框架與同位元錯誤旗標；記錄錯誤、清除解析狀態並捨棄該位元組／check overrun, noise, framing, and parity error flags; count the error, reset parsing, and discard the byte |
| 命令逾時／Command timeout | 距最後接受命令超過 300 ms，鎖定 ESC 中立值 `152`／latch ESC neutral at 152 after more than 300 ms without an accepted command |
| 遙測／Telemetry | 回報有效封包數、CRC 錯誤數及 UART 硬體錯誤數；遙測封包亦附 CRC-8／report accepted-frame, CRC-error, and UART-error counters; protect telemetry frames with CRC-8 |

程式入口／Implementation: Jetson [`build_command()`](../jetson/minitesla_sensors/io.py) 與 M4 [`crc8()`、`parser_feed()`、`publish_command()`、`USART2_IRQHandler()`、`policy_step()`](../firmware/stm32f4/Core/Src/main.c)。

## 停止與恢復／Stopping and recovery

M4 任一方向距離小於等於 300 mm、量測無效、量測資料超過 120 ms，或控制命令超過 300 ms，會鎖定中立輸出。解除需四向至少 400 mm 穩定 500 ms、收到中立，再有新油門命令。這些是程式設定，不是已量測的整車煞停距離或端到端最壞延遲。

The M4 latches neutral for any range at or below 300 mm, invalid ranging, samples older than 120 ms, or commands older than 300 ms. Release requires all directions at least 400 mm for 500 ms, a neutral command, and then a new throttle command. These are configured thresholds, not measured vehicle stopping distances or worst-case end-to-end latency.

Jetson 另外監督遠端心跳、影像時效、感測狀態與模式交接；故障或重連不會自動恢復舊的自主行駛。M4 看門狗監督主迴圈與控制時基；系統服務的重新啟動則針對程序退出，兩者作用層次不同。

Jetson additionally supervises remote heartbeats, video freshness, sensor state, and mode handoff. Reconnection does not resume a previous autonomous action. The MCU watchdog supervises firmware progress; service restarts handle process exits at the Linux level.
