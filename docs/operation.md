# 部署與操作／Deployment and operation

## 目錄對應／Directory mapping

| 儲存庫／Repository | 部署位置／Deployment |
| --- | --- |
| `pc/` | 電腦任意專案資料夾，四個 Python 檔案保持同層。Any desktop project directory, keeping all four Python files together. |
| `jetson/` | `$HOME/Desktop/jetson orin nano`，複製裡面的內容，避免多包一層。Copy the contents to this vehicle directory. |
| `firmware/stm32f4/` | 整個工程供 Keil 開啟；保留子目錄相對關係。Keep the entire project structure for Keil. |

本儲存庫保存程式、模型與紀錄，不包含整台 Jetson 的作業系統映像或 Python 環境。優先沿用原本可運作的環境，勿把歷史筆記中的不同版本指令全部重新執行。

This repository preserves source, the model, and records, not a full Jetson OS image or Python environment. Reuse the established environment. Historical notes document different attempts and are not one script to execute wholesale.

## 電腦端／Desktop

在原 Python 環境執行：／In the existing Python environment:

```bash
cd pc
python pc_dashboard.py
```

| 操作／Input | 行為／Action |
| --- | --- |
| M | 手動接管，完成交接後重新按方向鍵。Manual takeover; press driving keys again after handoff. |
| W／S／A／D | 前進／倒退／左轉／右轉。Forward/reverse/left/right. |
| 空白鍵／Space | 停止待命。Stop and enter standby. |
| P | 啟動反應式巡邏。Start reactive patrol. |
| R | 原因排除後解除故障，回待命。Reset a cleared fault and return to standby. |

視窗失焦、縮小或影像失效會觸發既有停止規則。自主行駛也需要遠端監督，不是斷線後繼續無人行駛。

Focus loss, minimization, and video failure invoke the existing stop rules. Autonomous behavior still requires remote supervision.

## Jetson 服務／Vehicle services

將檔案部署完成，啟用原本的 Python 環境後，以一般使用者執行：

After deployment, activate the existing Python environment and run as the normal user:

```bash
cd "$HOME/Desktop/jetson orin nano"
chmod +x rplidar_sdk-master/output/Linux/Release/ultra_simple
python3 minitesla_service.py install --pc-host 120.126.10.41
python3 minitesla_service.py status
```

安裝工具會檢查環境、裝置、模型與服務設定，再安裝及啟動服務。只有需要系統權限的步驟才自行呼叫 `sudo`，不要使用 `sudo python3`。若原服務已安裝且環境、位置未變，更新後使用 `restart` 即可。

The installer checks the environment, devices, model, and service configuration before installing and starting services. It invokes `sudo` only for privileged steps; do not run it with `sudo python3`. If services are already installed and paths/environment are unchanged, use `restart` after updating files.

```bash
python3 minitesla_service.py stop
python3 minitesla_service.py start
python3 minitesla_service.py restart
python3 minitesla_service.py logs
```

上述是個別管理指令，不是必須依序執行的啟動腳本。開機服務涵蓋主程式、雷達讀取與紀錄器；不要再另開相同車端主程式或雷達讀取器。

These are individual management commands, not a required sequence. Services manage the vehicle application, lidar reader, and recorder. Avoid launching duplicate manual instances.

既定電腦位址為 `120.126.10.41`，M4 為 `/dev/ttyTHS0`、115200，雷達為 `/dev/ttyUSB0`、460800。這些是專案預設值，可依部署環境修改；本儲存庫未新增網路穿透服務的帳號或登入憑證。

The project defaults are desktop `120.126.10.41`, M4 `/dev/ttyTHS0` at 115200, and lidar `/dev/ttyUSB0` at 460800. Adjust them for deployment. No NAT-traversal service account or login credential is included.

## M4 工程／Firmware project

開啟 `firmware/stm32f4/MDK-ARM/wheelcontrol.uvprojx`。`main.c` 已包含四向 ToF 驅動與保護流程，保留工程既有中斷檔及 HAL／CMSIS。若需燒錄，使用原工具鏈重新建置後的產物；歷史 `.hex`／`.axf` 不代表本版原始碼的重新編譯結果。

Open `firmware/stm32f4/MDK-ARM/wheelcontrol.uvprojx`. The current `main.c` embeds ToF drivers and protection; retain the existing interrupt source and HAL/CMSIS. Build with the established toolchain before flashing. Archived `.hex`/`.axf` files are not fresh builds of the current source.

2026-11-16 已依最後版本壓縮檔還原完整工程目錄，包括獨立 ToF 來源及原建置產物。請沿用上述 `.uvprojx` 的來源清單；`main.c` 已內嵌相關實作，不需再把 `Core/Src/tof4.c`、`policy.c`、`vl53l0x_platform.c` 或 `Core/Vendor/VL53L0X/` 的來源重複加入編譯。現存 `.hex`／`.axf` 等是使用者提供的原檔，本次未重新建置。

The final uploaded archive restores the complete project layout, including standalone ToF sources and saved build outputs. Keep the supplied `.uvprojx` source list: `main.c` already embeds the relevant implementations, so do not add the standalone copies again. Existing binaries are the supplied originals, not new builds from this reconciliation.

## 監看與紀錄／Monitoring and records

```bash
python3 sensor_status.py
python3 sensor_status.py --once --json
python3 demo_recorder.py export
```

監看工具只讀取快照，不再占用 UART。紀錄器約每 0.2 秒取樣，可匯出 `samples.csv`、`events.csv`、`records.jsonl` 與摘要；它不是完整影片或每筆控制封包的錄製。程式尚未產生的實車紀錄不會由發布流程虛構補上。

Monitoring reads snapshots without opening UART. The recorder samples about every 0.2 seconds and exports CSV/JSONL plus metadata; it is not video recording or a complete capture of every control packet. Publication does not invent missing driving records.

停車相關仍保留原介面與程式，但屬於[實驗性擴充](experimental.md)，不列入主要展示操作。

Parking UI/code remains available as an [experimental extension](experimental.md), outside the main demonstration scope.
