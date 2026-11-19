# 部署與操作／Deployment and operation

## 目錄對應／Directory mapping

| 儲存庫／Repository | 部署位置／Deployment |
| --- | --- |
| `pc/` | 電腦任意專案資料夾，四個 Python 檔案保持同層。Any desktop project directory, keeping all four Python files together. |
| `jetson/` | `$HOME/Desktop/jetson orin nano`，複製裡面的內容，避免多包一層。Copy the contents to this vehicle directory. |
| `firmware/stm32f4/` | 整個工程供 Keil 開啟；保留子目錄相對關係。Keep the entire project structure for Keil. |

本儲存庫保存程式、模型與紀錄，不包含整台 Jetson 的作業系統映像或 Python 環境。優先沿用原本可運作的環境，勿把歷史筆記中的不同版本指令全部重新執行。

This repository preserves source, the model, and records, not a full Jetson OS image or Python environment. Reuse the established environment. Historical notes document different attempts and are not one script to execute wholesale.

## 模型準備／Model preparation

本專案的 `jetson/best.pt` 是加入空車位圖片重新訓練後保存的模型，並包含自訂 `vacant` 類別。部署時保留它與 `sensor_config.json` 的相對位置即可，`camera.model` 預設為 `best.pt`；使用這份已訓練權重啟動車子，不需要再執行訓練。開發階段的資料加入、重新訓練與模型部署步驟見[模型訓練說明](model-training.md)。停車相關功能仍為測試版。

The supplied `jetson/best.pt` was retained after retraining with added vacant-space images and includes the custom `vacant` class. Keep its location relative to `sensor_config.json`; `camera.model` defaults to `best.pt`. Running the vehicle with these trained weights does not require another training run. See [model training](model-training.md) for the development steps. Parking-related functions remain prototypes.

## 公開版連線位址／Public example address

公開程式與操作範例統一使用 `192.0.2.10`。它屬於 RFC 5737 的文件範例網段 `192.0.2.0/24`（TEST-NET-1），不是實際部署的接收端。執行下列命令前，請把 `--pc-host 192.0.2.10` 改成 Jetson 能連到的電腦接收端位址；這裡填的是接收端，不是車子的位址。

Public source and commands use `192.0.2.10`, from the RFC 5737 documentation range `192.0.2.0/24` (TEST-NET-1). It is not a deployment endpoint. Before running these commands, replace it with the desktop receiver address reachable from the Jetson, not the vehicle address.

手動啟動主程式時，可用 `--pc-host`、環境變數 `MINITESLA_PC_HOST` 或 `sensor_config.json` 的 `network.pc_host` 指定接收端。命令列優先於環境變數與設定檔。開機服務請用 `minitesla_service.py install --pc-host 實際接收端位址` 明確設定；服務已安裝後若要更改接收端，須重新執行安裝指令來更新服務中的啟動參數，單純重新啟動不會改寫該參數。

For manual launches, set `--pc-host`, `MINITESLA_PC_HOST`, or `network.pc_host` in `sensor_config.json`; the command-line argument takes precedence over the environment and config file. For boot services, explicitly pass the receiver to `minitesla_service.py install --pc-host`. Re-run that installer to change an installed service's receiver address; restarting alone does not rewrite its stored launch arguments.

`0.0.0.0` 是電腦程式的監聽位址；`127.0.0.1:6000` 是 Jetson 內部的雷達資料通道，兩者用途不同，不需隨遠端接收端位址一起更改。

`0.0.0.0` is the desktop listen address; `127.0.0.1:6000` is the Jetson-local lidar channel. Neither should be replaced with the remote receiver address.

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
python3 minitesla_service.py install --pc-host 192.0.2.10
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

公開範例電腦位址為 `192.0.2.10`，部署時必須替換。M4 預設為 `/dev/ttyTHS0`、115200，雷達預設為 `/dev/ttyUSB0`、460800，可依實際裝置修改；本儲存庫未新增網路穿透服務的帳號或登入憑證。

Replace the documentation-only desktop address `192.0.2.10` for deployment. Device defaults are M4 `/dev/ttyTHS0` at 115200 and lidar `/dev/ttyUSB0` at 460800. Adjust device paths as needed. No NAT-traversal service account or login credential is included.

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
