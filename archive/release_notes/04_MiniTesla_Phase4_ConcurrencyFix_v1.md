# MiniTesla 第四階段：多工與串口同步修正

本包接續已安裝的第四階段「影像斷線處理與恢復接管」。共 6 個檔案：4 個修改後的完整程式、1 個新增程式，以及本說明。可直接覆蓋同名檔案，不必手動剪貼程式片段。

## 本次修正

- 新版主程式啟動時先取得固定的程序鎖。同一台 Jetson 即使從不同資料夾啟動，也會拒絕第二份使用同一鎖檔的新版主程式。
- M4 串口由唯一執行緒持有，加入 pySerial 排他開啟與 Linux 終端裝置排他設定，阻止後續一般程式再開啟該串口。
- 模式判斷、感測健康檢查與 UART 寫入共用控制鎖，移除「取出油門後，已切成故障狀態卻仍送出先前油門」的交接空隙。
- 等待鎖結束後重新取得時間與健康狀態，將等待時間算進控制心跳及感測資料的有效期限。
- UART 開啟或通訊失敗時，結束整套車端主程式並清理背景工作；排除原因後重新啟動。

這次仍是控制基礎修正。自動巡邏、人物警示、自動避障決策及停車功能尚未加入。

## 一、替換檔案

先在車端主程式終端機按 `Ctrl+C`，等待程式結束，再複製檔案。

| 壓縮包內檔案 | 部署至車端專案的相對路徑 | 操作 |
| --- | --- | --- |
| `Jetson/minitesla.py` | `minitesla.py` | 覆蓋 |
| `Jetson/minitesla_sensors/core.py` | `minitesla_sensors/core.py` | 覆蓋 |
| `Jetson/minitesla_sensors/control.py` | `minitesla_sensors/control.py` | 覆蓋 |
| `Jetson/minitesla_sensors/io.py` | `minitesla_sensors/io.py` | 覆蓋 |
| `Jetson/minitesla_sensors/runtime.py` | `minitesla_sensors/runtime.py` | 新增 |
| `README_使用說明.md` | 可自行存放 | 操作說明 |

把包內 `Jetson` 資料夾的內容合併進原本的 `jetson orin nano` 專案根目錄；不要把原 `minitesla_sensors` 資料夾整個刪除再重建，也不要多套一層 `Jetson` 資料夾。

更新後，`minitesla_sensors` 內應有以下 5 個程式檔：

- `__init__.py`：原檔保留。
- `core.py`、`control.py`、`io.py`：本次覆蓋。
- `runtime.py`：本次新增。

原 `sensor_config.json`、`sensor_status.py`、`best.pt`、`rplidar_sdk-master` 與既有 Python 環境保留。電腦端繼續使用第四階段的 `pc_control.py`、`pc_receive.py`。M4 韌體與接線不變，不需重新燒錄；本次不需下載新驅動或增加第三方 Python 套件，沿用既有 pySerial。

## 二、啟動電腦端

在原電腦端程式資料夾開兩個終端機，分別執行：

```powershell
py pc_receive.py
```

```powershell
py pc_control.py
```

若這兩支第四階段程式已在執行，可以繼續使用。若原本用 `python`，將 `py` 換成 `python`，仍使用原先安裝 OpenCV、NumPy 的環境。

不要同時執行舊影像接收器 `from_jetson_orin_nano(1).py` 或舊控制程式 `sever(1).py`。通訊仍為控制 UDP 5000、雷達 UDP 6000、影像 TCP 7000；MiFi、5G 與 NAT 穿透沿用既有配置。

## 三、啟動 Jetson

以下假設專案位置為 `$HOME/Desktop/jetson orin nano`。請使用原本可載入 `best.pt` 的 Python 環境。

第一個終端機啟動原雷達 SDK；如果它已在執行，不要再開第二份：

```bash
cd "$HOME/Desktop/jetson orin nano/rplidar_sdk-master/output/Linux/Release"
./ultra_simple --channel --serial /dev/ttyUSB0 460800
```

第二個終端機啟動車端主程式：

```bash
cd "$HOME/Desktop/jetson orin nano"
python3 minitesla.py --pc-host 192.0.2.10
```

沒有 Jetson 本地桌面時，改用：

```bash
python3 minitesla.py --pc-host 192.0.2.10 --no-display
```

需要查看四向 ToF 與模式狀態時，可另外開第三個終端機：

```bash
cd "$HOME/Desktop/jetson orin nano"
python3 sensor_status.py
```

`sensor_status.py` 只讀取快照，可與主程式同時使用。不要另外啟動 `jetson_motor_uart_verbose.py` 等會開啟 M4 串口的舊程式；舊 `lidar_receive.py`、`detect.py` 也不要與整合主程式一起執行。

所有範例電腦端 IP 固定為 `192.0.2.10`，日後只要修改 `--pc-host` 的值。命令列參數會覆寫原設定檔中的目標 IP。

## 四、接管與停止

1. 等模型載入、電腦影像出現，且控制視窗顯示影像接收確認正常、M4／ToF 就緒。
2. 點選控制視窗，按 `M`，等「手動就緒」再按 `W/S/A/D`。操作期間保持控制視窗焦點。
3. 按空白鍵停止並回待命。
4. 一般影像或控制失聯恢復後，按 `R` 解除故障，再按 `M` 接管；必須重新按下方向鍵，不會自動恢復先前油門。
5. 若 Jetson 顯示 M4 通訊已停止，主程式會結束。排除串口占用、裝置或連線原因後，重新執行主程式，再等就緒並按 `M`；只按 `R` 不會重啟已結束的程式。
6. 結束使用時先按空白鍵，再在 Jetson 主程式終端機按 `Ctrl+C`，等待背景工作清理完成。

## 五、新增保護的行為與界線

| 情況 | 新版行為／操作 |
| --- | --- |
| 再開一份新版 `minitesla.py` | 第二份顯示「另一份 MiniTesla 車端主程式仍在執行」，隨即退出；保留原本正在執行的那份 |
| 正常結束後重新啟動 | 程序鎖會釋放，可以重新啟動 |
| 程序異常退出 | 核心在最後一個鎖描述元關閉時釋放鎖，不依賴程式寫入或清除 PID |
| `/tmp/minitesla-vehicle.lock` 仍存在 | 屬正常狀況；檔案存在不代表程式正在執行，請勿手動刪除鎖檔 |
| 串口已被其他遵循排他規則的程式持有 | 開啟失敗，結束這份新車端實例，不強制終止其他程式 |
| 舊程式早已開啟串口 | 新排他設定不會驅逐既有描述元，仍須先正常關閉舊程式 |
| 故障狀態先取得鎖並完成切換 | 後續 UART 派送重新判斷為中立，不送出先前暫存的油門 |
| UART 寫入先取得鎖 | 該次寫入結束後才提交等待中的故障切換；已進入作業系統或導線的封包不能撤回 |

程序鎖協調新版主程式；它不會替原雷達 SDK 加上單一實例保護。UART 排他設定可阻止後續一般開啟，但不是限制具管理權限程式的安全邊界。

車端保留既有多行程（multiprocessing，使用 spawn）處理相機／推論、影像傳送及選用的本地顯示；主行程內以執行緒（threading）處理 M4 串口及雷達接收。模式切換由同一控制管理器處理，不會為每個模式另外建立一支馬達寫入執行緒。

Linux 排程與 UART 寫入逾時不是硬即時保證。控制鎖只包住必要判斷與短逾時寫入，不包含影像推論或網路等待。M4 原有四向 ToF 與命令逾時保護仍獨立保留；目前保護輸出是 ESC 中立值 152，不能據此宣稱已驗證 ESC 主動煞車或保證車輛瞬間停止。

## 六、驗證範圍

本次通過 87 項軟體測試：原第四階段 68 項回歸測試，加上 19 項同步與資源排他測試。包含真實執行緒競爭、跨行程鎖互斥、強制結束持鎖行程後重新取得鎖、等待期間資料過期、UART 寫入失敗與故障切換順序，以及 Linux 偽終端（PTY）排他開啟及清理。

PTY 測試使用模擬的 Serial 介面驗證核心排他操作；此環境沒有安裝 pySerial，未驗證實際 Jetson 串口驅動。另已檢查 Python 3.8 語法及與前版保留檔案合併後的匯入與命令列說明。沒有執行你的 Jetson GPU、Windows 圖形介面或實車測試。
