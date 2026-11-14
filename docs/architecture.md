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

## 停止與恢復／Stopping and recovery

M4 任一方向距離小於等於 300 mm、量測無效、量測資料超過 120 ms，或控制命令超過 300 ms，會鎖定中立輸出。解除需四向至少 400 mm 穩定 500 ms、收到中立，再有新油門命令。這些是程式設定，不是已量測的整車煞停距離或端到端最壞延遲。

The M4 latches neutral for any range at or below 300 mm, invalid ranging, samples older than 120 ms, or commands older than 300 ms. Release requires all directions at least 400 mm for 500 ms, a neutral command, and then a new throttle command. These are configured thresholds, not measured vehicle stopping distances or worst-case end-to-end latency.

Jetson 另外監督遠端心跳、影像時效、感測狀態與模式交接；故障或重連不會自動恢復舊的自主行駛。M4 看門狗監督主迴圈與控制時基；系統服務的重新啟動則針對程序退出，兩者作用層次不同。

Jetson additionally supervises remote heartbeats, video freshness, sensor state, and mode handoff. Reconnection does not resume a previous autonomous action. The MCU watchdog supervises firmware progress; service restarts handle process exits at the Linux level.
