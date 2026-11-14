# MiniTesla 第一、二階段：精簡替換套件

本套件只保留 8 個必要檔案，Jetson 統一使用 `minitesla.py`。已移除重複啟動入口、未修改的 M4 中斷檔、測試檔及分開的協定／版本說明。功能與前一份套件相同。

## 替換

1. 將 `M4/Core/Src/main.c` 覆蓋原 `wheelcontrol(main)/Core/Src/main.c`。**原工程 `stm32f4xx_it.c` 保留，不要刪除。**在原 Keil 工程重新建置（Rebuild）並燒錄。
2. 將 `Jetson` 資料夾內全部內容複製到原車端程式資料夾，保留原 `best.pt`。以後只啟動 `minitesla.py`，不再啟動 `minitesla(1).py`、`minitesla(2).py` 或 `minitesla_app.py`。
3. 電腦端與原雷達 SDK 沿用。不要另啟動舊 UART 橋接程式或 `lidar_receive.py`，避免占用相同 UART 或 UDP 6000。

M4 的 ToF 驅動已內嵌，原 HAL／CMSIS 與工程設定沿用，不用額外下載驱動或新增來源檔。不要將獨立 policy／tof4／VL53L0X 再次加入編譯。

## 接線

既有 Jetson UART TX → M4 PA3 保留；回傳使用 **M4 PA2 → Jetson 同一 UART 的 RX**，沿用兩板共地與板端 3.3 V UART。若原本已接雙向線，不必補線。ToF 接線、PWM 與保護門檻不變。

## 啟動

使用原本的 Python 環境，不需新增套件；模型仍使用 `best.pt`。

先啟動原電腦端與雷達程式。原雷達命令為：

```bash
cd ~/Desktop/rplidar_sdk-master/output/Linux/Release
./ultra_simple --channel --serial /dev/ttyUSB0 460800
```

另開終端機，進入新版 Jetson 程式資料夾：

```bash
python3 minitesla.py
```

預設電腦位址沿用原 `minitesla(2).py` 的 `192.168.225.46`。若你原本使用 `minitesla(1).py` 的跨網路位址，使用**同一個程式**：

```bash
python3 minitesla.py --pc-host 120.126.10.41
```

也可將實際位址寫入 `sensor_config.json` 的 `network.pc_host`；命令列 `--pc-host` 可覆寫設定。沒有本地桌面時加 `--no-display`，模型在其他位置時加 `--model /實際路徑/best.pt`。

監看四向距離與感測狀態：

```bash
python3 sensor_status.py
```

監看完整資料一次：

```bash
python3 sensor_status.py --once --json
```

監看只讀取主程式每 0.1 秒更新的 `sensor_snapshot.json`，不會再開 UART。主程式按 `Ctrl+C` 結束。

## 這次提供的資料

- M4 每 50 毫秒嘗試以非阻塞方式回傳四向距離、量測狀態、資料年齡、保護原因及輸出控制值。
- Jetson 統一整理 ToF、雷達點位、影像類別、追蹤編號、像素框及最多 128 點的簡化分割輪廓。
- 無效或過期距離為 `null`，不能當作空曠。人物距離尚未與雷達關聯，因此維持未知。
- `m4.mode` 是原底層保護狀態：0=鎖定、1=中立待起步、2=運行，並非尚未加入的巡邏模式。
- ToF、雷達、相機預設有效期限分別為 120、500、1000 毫秒；相機年齡包含推論耗時。ToF 年齡加上 10 毫秒傳輸餘裕作估計，未宣稱硬體同步或最壞延遲保證。

雷達的 `lidar.yaw_deg` 暫為 `null`，不猜測實際安裝方向。原始角度仍可接收、顯示與轉送；設定後才產生車體方向。車體座標沿用 X 前／Y 右／Z 上、車尾原點，前=0°、右=90°、後=180°、左=270°；`yaw_deg` 是雷達原始 0° 的車身朝向，`clockwise` 指原始角度增加是否朝右。雷達／相機位置沿用之前提供的 (300,181,315)／(395,175,190) 毫米。

目前提供資料回傳及整合，仍沿用原手動控制。模式管理、人物警示、避障、巡邏、停車與開機自啟屬下一階段。

前版 32 項軟體／通訊測試及 C 主機檢查已通過；此次僅精簡包裝與合併入口，感測與通訊模組未改。此處沒有 Keil／ARM 編譯環境或實車連線，未完成新增功能的 ARM 建置與實車驗證，所以提供原始碼，不附舊 HEX。
