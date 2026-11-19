# 智慧車｜Jetson Orin Nano × STM32 嵌入式人工智慧巡邏避障系統：邊緣推論、異質運算與即時控制

> **本專案由我一人在一年內，從零開始獨立完成。**
>
> 我先進行**整車硬體規劃與元件選型**，接著完成**電源分路、穩壓供電配置與實際配線**，最後進行 **STM32 控制韌體與 Jetson 車端軟體開發**；並自行**加入空車位影像重新訓練模型，產出 best.pt 後部署至 Jetson**，整合多感測器、人工智慧推論、遠端操作與整車控制。

[![智慧車整車封面：點擊觀看實機展示影片](docs/images/smart-car-cover.jpg)](https://youtu.be/pAVebk_odPo)

## [▶ 點此觀看智慧車實機展示影片](https://youtu.be/pAVebk_odPo)

[English](README.en.md) · [硬體與接線](docs/hardware.md) · [系統架構](docs/architecture.md) · [模型訓練](docs/model-training.md) · [部署操作](docs/operation.md) · [原始建置紀錄](logs/README.md)

系統整合硬體、Linux 車載運算、微控制器韌體與電腦操作介面：Jetson Orin Nano 執行視覺推論、網路傳輸及上層決策；STM32F412 執行 PWM、UART 封包解析與獨立距離保護。開發從各裝置的單項驗證逐步推進到整車整合，儲存庫同時保留早期程式、編譯紀錄與歷次修改說明，能追溯成品如何形成。

## 獨立完成的整車工程

| 工程領域 | 獨立完成的實作 | 系統成果 |
| --- | --- | --- |
| 硬體與供電整合 | 選配運算板、控制板、感測器與致動器；安排電池分接、穩壓供電、USB 周邊供電、BEC 舵機供電，以及板卡安裝與訊號接線 | 將車用動力、運算與控制負載接成可共同運作的整車系統 |
| Jetson 平台建置 | 建立 Python／GPU 推論環境、處理 PyTorch／torchvision 相容性、自行編譯 torchvision 的 ARM 與 CUDA 擴充 | 讓相機影像與訓練模型能接上實際車端執行環境 |
| STM32 即時控制 | 以 C 實作韌體，配置時脈、GPIO、硬體計時器與 PWM；實作 UART 解析、CRC8、四向 ToF 管理、保護狀態機與看門狗 | 將網路操作命令轉成實際油門／轉向訊號，保留獨立於 Jetson 的控制保護 |
| 自訂資料與模型重新訓練 | 將空車位圖片加入訓練資料，重新訓練含自訂 `vacant` 類別的 YOLOv8 分割模型，產出並部署 `best.pt` | 保存 81 類模型權重，接入車端推論、分割顯示與遠端回傳；[訓練與部署步驟](docs/model-training.md) |
| 感知與資料傳輸 | 整合 USB 相機、`best.pt` 分割模型、ByteTrack、RPLIDAR C1、四顆 VL53L0X；傳送影像、掃描、距離與狀態 | 操作者能同時看見車前影像、辨識結果與周邊距離 |
| 網路與遠端操作 | 使用專用 5G 網路 SIM、車載 MiFi 與既有 NAT 穿透配置；分開處理影像、控制與雷達資料通道 | 把車端裝置接到遠端電腦操作台，處理配對、失聯與恢復接管 |
| 多工與控制權整合 | 分離推論／傳輸程序、UART／雷達執行緒，加入佇列、鎖、排他開啟、資料時效與模式交接 | 避免影像運算阻塞控制，或多個工作同時爭用馬達串口 |
| 部署與操作介面 | 統一電腦操作台、服務安裝與管理、開機啟動、狀態監看與展示紀錄匯出 | 將開發時分散啟動的程式收斂成可部署、可操作、可追查的系統 |

硬體以現成車體、開發板及電源／感測模組整合。採用的原廠 HAL、CMSIS、感測器 API 與雷達 SDK，詳見[第三方來源與授權說明](THIRD_PARTY_NOTICES.md)。

## 從零到整車的開發路徑

開發由硬體與供電規劃開始，依序完成平台建置、控制韌體、模型訓練與整車整合。

| 階段 | 實作內容 | 原始碼與技術文件 |
| --- | --- | --- |
| 1．車體與電子系統規劃 | 決定 Jetson／M4 分工，安裝電池、ESC、舵機、相機、雷達與板卡；安排各路供電、共地與訊號線 | [硬體照片](archive/photos/)、[供電及接線整理](docs/hardware.md) |
| 2．Jetson 到貨後的環境建置 | 建立板端執行環境，檢查 GPU 套件與 Python 相容性，準備 CUDA 編譯環境並自行建置 torchvision | [編譯指令筆記](notes/compile_torchvision.txt)、[完整建置輸出](logs/torchvision_build.log) |
| 3．先讓馬達與轉向受控 | 在 STM32 建立 PWM 輸出與 UART 命令通道，對接 Jetson 串口，確認中立值與控制範圍 | [早期 UART 橋接](archive/early_sources/jetson_motor_uart_verbose.py)、[早期 M4 程式](archive/early_sources/main(1).c)、[目前完整工程](firmware/stm32f4/) |
| 4．加入空車位資料並重新訓練模型 | 納入空車位圖片，重新訓練自訂類別分割模型並產出 `best.pt`，準備車端部署 | [模型權重](jetson/best.pt)、[訓練步驟與模型類別](docs/model-training.md) |
| 5．逐項接通感知與遠端資料 | 分別處理影像推論、電腦鍵盤控制、UART 轉送及雷達接收，再整合資料流 | [早期獨立程式](archive/early_sources/)、[雷達 SDK](jetson/rplidar_sdk-master/) |
| 6．把獨立程式整合成並行系統 | 將相機／推論、影像傳送與感測接收拆成適合的程序或執行緒，整合到車端主程式 | [早期整合版本](archive/early_sources/minitesla(2).py)、[目前車端主程式](jetson/minitesla.py) |
| 7．加入四向測距與控制保護 | 增加四顆 ToF 的獨立 XSHUT、位址分配、有效性檢查與距離回傳，將油門保護放進 M4 | [四向測距整合紀錄](archive/release_notes/01_MiniTesla_Phase1_2_Replace_v1.md)、[M4 韌體](firmware/stm32f4/Core/Src/main.c) |
| 8．處理整合後的失聯與競態 | 補上影像顯示確認、命令期限、控制權交接、單一 UART 寫入者及程序／串口排他 | [多工同步修正紀錄](archive/release_notes/04_MiniTesla_Phase4_ConcurrencyFix_v1.md)、[控制管理](jetson/minitesla_sensors/control.py) |
| 9．收斂成整車操作與部署 | 整合人物警示、反應式避障巡邏、電腦操作台、開機服務與狀態紀錄工具 | [操作台](pc/pc_dashboard.py)、[自主控制](jetson/minitesla_sensors/autonomy.py)、[服務管理](jetson/minitesla_service.py)、[紀錄器](jetson/demo_recorder.py) |

## 硬體實作：電源電路安排、模組安裝與接線

![實車底盤、電池與多層車載電子平台](archive/photos/104652_0.jpg)

*實機組裝照片：電池、車體底盤與以隔離柱固定的多層車載電子平台。[其他原始硬體照片](archive/photos/)*

### 元件、供電電壓與負載

整車由主電池分成**動力、Jetson 運算、M4 控制三路**。電源轉換、板卡供電與訊號接線由我規劃並完成；相機、雷達與感測器使用所連接板卡的電源，舵機使用 ESC 內建的穩壓輸出（BEC）。

| 元件 | 供電來源與電壓 | 規格與整合工作 |
| --- | --- | --- |
| 高能（GNB／GAONENG）4S 高壓鋰電池（LiHV） | 主電源，**標稱 15.2 V** | **6000 mAh、130C**；銘牌標示單節充電上限 4.35 V，四串合計 17.4 V |
| 藍色 LTC3780 升降壓模組 | 4S 電池 → **19 V 設計電源軌** | 供應 Jetson 及其板載、USB 周邊負載 |
| NVIDIA Jetson Orin Nano 開發套件 | LTC3780 → **19 V 直流輸入**（專案設計值） | 執行 Linux、影像推論、雷達整合、網路傳輸與上層控制 |
| D-SUN 降壓模組 | 4S 電池 → **5 V** | [包裝標示 MP1584EN、5 V／3 A](archive/photos/104661_0.jpg)；獨立供應 M4 支路 |
| ST STM32F412ZGT6／M4 控制板 | D-SUN → ST-LINK USB 供電入口 **5 V**；MCU／UART 邏輯 **3.3 V** | 即時 PWM、UART、四向測距與獨立控制保護 |
| 電子變速器（ESC） | **4S 電池母線直接輸入**，隨電池電壓變化 | 驅動無刷馬達；內建 BEC 供應舵機 |
| 無刷馬達（BLDC） | ESC 的**三相換相輸出** | 油門由 M4 的 PWM 控制 ESC，馬達端為三相驅動波形 |
| Hyper GO 轉向舵機 | ESC 內建 BEC；**舵機銘牌 6.0 V** | [銘牌 35 kg·cm、0.13 秒／60°](archive/photos/104651_0.jpg)；M4 另接控制訊號 |
| USB 相機（板號 BL-1080p-S10） | Jetson USB **5 V** | 取像並送入 `best.pt` 推論流程 |
| 雷達原廠 USB 轉接板 | Jetson USB **5 V** | 接收 USB 電源、轉接 UART 並將電源送至雷達 |
| 思嵐（SLAMTEC）RPLIDAR C1 | 轉接板 **5 V**；允許 **4.8–5.2 V** | 5 V、10 Hz 時運轉典型 **230 mA**、最大 **260 mA**；啟動典型 **800 mA** |
| 意法半導體（ST）VL53L0X × 4 | M4 板端供電；**感測器晶片規格 2.6–3.5 V** | 前後左右四向；單顆量測典型 **19 mA**、峰值可達 **40 mA**，載板資料見[硬體文件](docs/hardware.md) |
| Jetson 原廠散熱器與風扇 | 載板風扇接頭 **5 V** | 原廠散熱器與主動風冷，使用板端供電 |
| ESC／馬達原廠散熱組件 | 沿用原車散熱線組；風扇額定電壓未登錄 | 保留原廠散熱配置 |
| 車載行動熱點（MiFi）與專用 5G 網路 SIM | MiFi 供電規格未登錄 | 提供行動網路與既有 NAT 穿透連線 |

### 實際電源分路

主電池與控制支路：

```mermaid
flowchart TD
    BAT["GNB 4S LiHV｜標稱 15.2 V"]
    BAT -->|"4S 電池母線"| ESC["電子變速器 ESC"]
    ESC -->|"三相換相輸出"| MOTOR["無刷馬達"]
    ESC -->|"內建 BEC"| SERVO["Hyper GO 舵機｜銘牌 6.0 V"]
    BAT --> REGJ["LTC3780 升降壓模組"]
    REGJ -->|"19 V（設計值）"| J["Jetson Orin Nano"]
    BAT --> REGM["D-SUN｜標示 5 V／3 A"]
    REGM -->|"5 V，ST-LINK USB 入口"| M["STM32F412 控制板｜3.3 V 邏輯"]
    M -->|"板端感測供電"| TOF["VL53L0X × 4｜晶片 2.6–3.5 V"]
```

Jetson 周邊支路：

```mermaid
flowchart TD
    J["Jetson｜19 V 設計輸入"] -->|"板內轉換"| USB["USB 5 V｜每組雙層接頭 3 A"]
    USB --> CAM["BL-1080p-S10 相機"]
    USB --> AD["原廠雷達轉接板"]
    AD -->|"5 V"| LIDAR["RPLIDAR C1｜運轉 230 mA／啟動 800 mA 典型"]
    J -->|"風扇接頭 5 V"| FAN["Jetson 原廠散熱風扇"]
```

**負載分配：**Jetson 支路承擔運算板、相機、雷達轉接板、雷達及原廠風扇；M4 與四向 ToF 使用獨立控制支路；馬達和舵機的負載留在 ESC 支路。各板卡共地，UART 與 PWM 負責傳送控制訊號。

**電流規格：**Jetson 每組雙層 USB Type-A 接頭共用 **3 A** 輸出限流；雷達的 **800 mA 啟動典型值**與 **230 mA 運轉典型值**分開計入 USB 負載。D-SUN 的 **3 A** 是模組標示輸出能力，M4 板內供電路徑與下游感測器另行分配。[完整電氣規格、規格來源與接線表](docs/hardware.md)。

### 關鍵訊號配線

| 訊號 | 接線／設定 | 實作重點 |
| --- | --- | --- |
| Jetson → M4 | Jetson UART TX → STM32 PA3／USART2 RX | 車端使用 `/dev/ttyTHS0`，115200、8N1 |
| M4 → Jetson | STM32 PA2／USART2 TX → Jetson UART RX | 回傳四向距離與控制狀態；雙方共地，UART 使用 3.3 V 邏輯 |
| 舵機控制 | PA8 → TIM1 CH1 → 舵機訊號線 | 訊號由 M4 產生，供電來自 BEC |
| ESC 控制 | PA1 → TIM2 CH2 → ESC 訊號線 | 速度與中立命令轉換成脈寬 |
| 四向 ToF 共用匯流排 | PB8＝SCL、PB9＝SDA | 開漏軟體 I²C；感測器讀取由 M4 管理 |
| 前／後／左／右 XSHUT | PG10／PG11／PG12／PG13 | 逐顆啟用並分配不同位址 |
| 前／後／左／右七位元 I²C 位址 | `0x30`／`0x31`／`0x32`／`0x33` | 解決四顆同預設位址裝置共用匯流排的衝突 |

接線對應可在 [`main.c`](firmware/stm32f4/Core/Src/main.c) 的 `pwm_init`、`uart_init` 與 ToF 初始化中核對。車體尺寸設定為車長 **47.4 cm**、車寬 **32.8 cm**、軸距 **27.5 cm**；雷達原始 0° 朝前、橫向置中、角度順時針，見 [車體設定](jetson/minitesla_sensors/vehicle_config.py)。

## 模型重新訓練：加入空車位影像並產出 best.pt

我針對智慧車的空車位辨識需求，**將空車位圖片納入訓練資料，重新訓練 YOLOv8 實例分割模型，產出 `best.pt`，再部署至 Jetson Orin Nano**。這項工作包括自訂辨識類別的模型訓練，以及將訓練結果接入車端相機、推論、分割結果顯示與遠端回傳流程。

車端使用 **81 類的 YOLOv8s 實例分割模型**：保留原有 80 個 COCO 類別名稱，新增空車位類別 **`vacant`（類別編號 80）**，部署權重為 [`jetson/best.pt`](jetson/best.pt)。

開發步驟為：**加入空車位訓練影像 → 重新訓練自訂類別模型 → 產出 best.pt → 部署到 Jetson → 接入影像辨識與顯示流程**。訓練與模型部署已完成；受後續開發時程限制，停車相關功能僅完成測試版，保留作為後續擴充。現有停車規劃主要使用雷達與 ToF，尚未將 `vacant` 視覺結果直接接入自動停車決策。

詳見[模型訓練與部署步驟](docs/model-training.md)及[停車測試版](docs/experimental.md)。

## Jetson：從套件環境到車載影像推論

在 Jetson 建立 Linux、Python 與 GPU 推論環境，處理 PyTorch／torchvision 套件相容性，並從官方原始碼編譯 torchvision 的 ARM 與 CUDA 擴充。

- 平台環境：**Linux AArch64、Python 3.8**。
- CUDA 建置：使用 `nvcc`，設定 `TORCH_CUDA_ARCH_LIST="8.7"`，編譯 `compute_87`／`sm_87` 目標。
- 建置設定：`MAX_JOBS=1`，沿用既有 Python 環境完成安裝。
- 完成編譯與安裝的版本：**`torchvision 0.16.1+fdea156`**；完整指令及輸出見[建置紀錄](logs/README.md)。

車端影像流程由 USB 相機取像，直接載入自行訓練的 [`.pt` 模型權重](jetson/best.pt)，執行 Ultralytics 分割推論與 ByteTrack 追蹤，再以 OpenCV 繪製結果、編碼為 JPEG 並傳回電腦操作台。

**實作入口：** [車端影像處理](jetson/minitesla.py) · [編譯指令](notes/compile_torchvision.txt) · [完整編譯紀錄](logs/torchvision_build.log)

## STM32：即時致動控制與四向測距保護

### 硬體 PWM 與雙向 UART

韌體直接配置 TIM1／TIM2 產生兩路 PWM。現行時基使用 16 MHz，預除頻器 `PSC=159`、週期暫存器 `ARR=1013`，得到約 **98.6 Hz**；每個計數為 **10 μs**，中立值 `152` 對應 **1.52 ms** 脈寬。上層速度與轉向命令透過這組計時器設定轉換為致動器脈寬。

| 控制量 | 現行韌體設定 |
| --- | --- |
| 舵機 | `94–210`，中位 `152` |
| ESC | 現行保護版本限制為 `142–160`，中立 `152` |
| UART 命令 | `55 AA 02 SEQ speed turn CRC8`，CRC-8 多項式 `0x07` |
| 命令接收 | 完整封包通過校驗後才更新命令與接收時間；錯誤／不完整資料不視為新控制輸入 |

M4 同時回傳四向距離、有效性與保護狀態，Jetson 不只有「送速度」單向控制，也能監督控制端是否正常。遙測封包為 79 位元組，主迴圈每 50 ms 嘗試透過 TXE 中斷傳送；忙碌時跳過，避免排隊累積過時的狀態。

### 四向測距與獨立保護

四顆 VL53L0X 上電時有相同預設位址。韌體先控制各顆 XSHUT，再逐顆啟用、重新配置位址，最後循環讀取。實作包含 GPIO 啟動順序、I²C 平台介面、原廠 API 整合、錯誤處理與量測時間戳。

開機先輸出 ESC 與舵機中立值，再初始化感測器。保護策略由 **TIM5 的 1 ms 控制時基**檢查，與相機推論及網路傳輸分開。控制中斷不做 I²C 讀取、延遲或列印，優先序高於 UART 中斷；主迴圈負責感測讀取與遙測封包準備，USART2 中斷負責命令接收與位元組收發；獨立看門狗（IWDG）只有在主迴圈與控制時基皆持續推進時才刷新。

| 條件 | 現行程式策略 |
| --- | --- |
| 任一方向距離小於或等於 300 mm | 鎖定油門為 ESC 中立值 |
| 任一必要 ToF 無效，或資料超過 120 ms 未更新 | 不把未知距離當成安全空間，鎖定油門 |
| UART 命令超過 300 ms 未更新 | 鎖定油門 |
| 障礙解除 | 四向皆至少 400 mm 且持續至少 500 ms，確認中立命令後，等待後續新的油門命令才恢復行駛 |

**保護動作為鎖定 ESC 中立油門**，同時可保留有效且未逾時的轉向命令。控制策略每 1 ms 檢查一次，觸發與解除的距離門檻分別為 300 mm 與 400 mm。

## 5G 遠端操作與多通道資料流

```mermaid
flowchart TD
    PC["電腦操作台"] <-->|"5G／MiFi 網路與既有穿透配置"| J["Jetson 車端"]
    CAM["USB 相機"] -->|"影像擷取"| J
    LIDAR["RPLIDAR C1／USB 轉接板"] -->|"SDK 掃描資料"| J
    J <-->|"UART 命令與遙測"| M["STM32F412"]
    TOF["四向 VL53L0X"] -->|"I²C"| M
    M -->|"PWM"| DRIVE["ESC 與轉向舵機"]
```

5G 採用**專用網路 SIM 插入車載 MiFi，再分享網路給車端**的方式。跨網路連線由 NAT 穿透配置提供；應用程式則由 Jetson 主動送出 `HELLO`，讓電腦操作台建立控制對端資訊並完成配對。

| 資料通道 | 功能與整合重點 |
| --- | --- |
| TCP 7000：影像 | 傳送 JPEG 影像與影格序號；以車端擷取時間及接收端顯示確認判斷影像時效 |
| UDP 5000：控制與狀態 | 車端配對、遠端心跳、操作命令、模式與感測狀態回報 |
| UDP 6000：雷達 | 銜接原廠 SDK、車端接收與電腦雷達畫面；雷達串口為 `/dev/ttyUSB0`、460800 |
| UART 115200：Jetson ↔ M4 | 封包校驗、速度／轉向命令與四向測距遙測 |

雷達整合也包含修改原廠 C++ `ultra_simple` 範例：將掃描角度、距離與品質轉成 `angle,distance,quality`，送到本機 UDP，再由車端程式整理與轉送，見 [SDK 範例修改](jetson/rplidar_sdk-master/app/ultra_simple/main.cpp)。

網路恢復後，系統經待命與重新接管流程恢復操作，不直接重送先前油門。這把「連線重新建立」與「允許車輛重新移動」分開處理。

## 多工整合：讓影像、感測與控制能一起工作

| 執行單位 | 負責的工作 | 資源與同步安排 |
| --- | --- | --- |
| 影像推論程序（Process） | 相機擷取、模型推論、追蹤、標註與編碼 | 使用 `multiprocessing`，經有限容量佇列交付資料 |
| 影像傳送程序 | TCP 連線與影格傳輸 | 與推論分開，避免傳輸等待直接堵住相機／推論工作 |
| UART 執行緒（Thread） | M4 收發及控制命令派送 | 單一串口持有者與寫入者，避免多個模式直接爭用裝置 |
| 雷達執行緒 | 接收掃描、更新感測快照與轉送 | 與 UART 接收分離，按資料時間判斷有效性 |
| 控制管理器 | 手動／待命／自主／故障切換 | 模式判斷、健康檢查與 UART 派送共用控制鎖 |
| 電腦媒體子程序與操作介面 | 影像／雷達接收與顯示、鍵盤控制 | 媒體處理與操作視窗分工，納入焦點及影像狀態 |

整合中特別處理過一個競態：**一個工作已取出舊油門，另一個工作剛切成故障狀態，前者卻仍可能把舊油門送出。** 修正方式是把控制判斷和 UART 派送放在同一控制鎖內，取得鎖後重新檢查時間與感測健康；影像推論和長時間網路等待則放在鎖外。

同時加入程序鎖、pySerial 排他開啟與 Linux 終端裝置排他設定，防止重複啟動的新實例再占用串口。監看工具只讀取快照，不另開 UART。這些措施各自處理程序重複、裝置爭用、共享狀態和舊資料問題；已進入串口傳送的封包無法撤回，因此仍保留 M4 的獨立保護。

**實作入口：** [程序與串口排他](jetson/minitesla_sensors/runtime.py) · [控制權管理](jetson/minitesla_sensors/control.py) · [UART／雷達工作](jetson/minitesla_sensors/io.py) · [當時的同步修正說明](archive/release_notes/04_MiniTesla_Phase4_ConcurrencyFix_v1.md)

## 從分散啟動到統一部署

電腦端入口是 [`pc_dashboard.py`](pc/pc_dashboard.py)，整合駕駛操作、辨識影像、雷達、四向距離及狀態。車端 [`minitesla_service.py`](jetson/minitesla_service.py) 管理主程式、雷達讀取與紀錄器的服務安裝、開機啟動、停止、重啟與紀錄查閱。

[`sensor_status.py`](jetson/sensor_status.py) 讀取感測快照；[`demo_recorder.py`](jetson/demo_recorder.py) 可匯出 `samples.csv`、`events.csv`、`records.jsonl` 與摘要，用來追查模式變化、距離及故障原因。

| 主要展示內容 | 對應工程能力 |
| --- | --- |
| 遠端手動駕駛與辨識影像回傳 | 視覺推論、編碼、網路傳輸、操作介面與馬達控制的完整鏈路 |
| 人物警示、雷達及四向距離監看 | 異質感測資料接入、狀態整合與回傳 |
| 反應式避障巡邏 | 以當下距離決策、控制權交接與保護條件整合 |
| 失聯停止與重新接管 | 心跳、影像時效、感測有效性及兩層控制保護 |
| 開機服務與紀錄匯出 | Linux 部署、自動啟動、程序管理與問題追查 |

人物辨識提供警示，雷達與四向 ToF 提供距離資訊；反應式巡邏依即時距離調整行進方向，並由控制權管理與 M4 保護策略監督車輛輸出。

### 實機展示影片

[![智慧車實機展示影片：點擊封面前往 YouTube 播放](https://i.ytimg.com/vi/pAVebk_odPo/maxresdefault.jpg)](https://youtu.be/pAVebk_odPo)

## 原始碼與技術文件

| 內容 | 入口 |
| --- | --- |
| 完整車端執行流程 | [`jetson/minitesla.py`](jetson/minitesla.py) 與 [`minitesla_sensors/`](jetson/minitesla_sensors/) |
| MCU 控制與測距 | [`main.c`](firmware/stm32f4/Core/Src/main.c)、[`wheelcontrol.uvprojx`](firmware/stm32f4/MDK-ARM/wheelcontrol.uvprojx) |
| 電腦端操作 | [`pc/`](pc/) |
| 環境建置紀錄 | [`logs/`](logs/)、[`notes/`](notes/) |
| 開發演進 | [`archive/early_sources/`](archive/early_sources/)、[`archive/release_notes/`](archive/release_notes/) |
| 硬體與原始文件 | [`archive/photos/`](archive/photos/)、[`archive/documents/`](archive/documents/) |
| 部署與版本範圍 | [操作說明](docs/operation.md)、[驗證說明](docs/validation.md) |

### 停車測試版與後續擴充

車位搜尋、自動對位與倒車入庫目前保留測試版。後續將整合空車位模型的 `vacant` 辨識結果與停車規劃，詳見[實驗性擴充](docs/experimental.md)。
