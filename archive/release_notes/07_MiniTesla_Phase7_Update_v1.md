# MiniTesla 第七階段：開機啟動與展示紀錄

這是接續第六階段的增量更新包。只包含 1 個修改檔、2 個新增程式與本說明；每個程式都是完整檔案。

| 壓縮檔內的檔案 | 放置位置 | 動作 |
| --- | --- | --- |
| `Jetson/minitesla.py` | Jetson 專案根目錄 | 取代現有檔案 |
| `Jetson/minitesla_service.py` | Jetson 專案根目錄 | 新增 |
| `Jetson/demo_recorder.py` | Jetson 專案根目錄 | 新增 |

沿用既有 `minitesla_sensors/`、`sensor_config.json`、`best.pt`、雷達 SDK 與 Python 環境。此次電腦端與 M4 沒有修改，也不需要重新燒錄 M4。

## 這次增加的行為

- Jetson 開機後，由 systemd 啟動車端主程式、既有雷達讀取程式與獨立紀錄程序。無須登入桌面，也不開啟 Jetson 本地影像視窗。
- 初始控制模式仍是待命。電腦端影像確認、控制心跳、M4 與四向 ToF 條件仍適用；車子不因為開機或程式重啟而自行巡邏或倒車。
- 相機推論／影像傳送子程序異常結束、M4 通訊工作失敗或雷達接收執行緒結束時，主程式結束並回報原因。服務在程序退出後等待 3 秒重新啟動；新實例具有新的控制連線識別，須重新選擇模式。
- 一般網路斷線仍由原有重連與控制保護處理，不會因電腦未開啟而一直重載模型。
- 正常停止時，主程式交由原本唯一的 UART 寫入執行緒收尾並嘗試送出中立值。若通訊本身故障，仍由既有 M4 命令逾時保護接手。中立值 152 不等於已驗證的主動煞車。
- 新紀錄程序只讀取狀態快照，不開啟 UART、不產生行駛命令，也不在控制鎖內寫入紀錄。

## 第一次安裝：只做一次

1. 在 Jetson 先正常關閉正在手動執行的車端主程式、雷達 `ultra_simple` 及舊串口測試程式。使用各終端的 `Ctrl+C`。若已安裝本階段，先執行下方 `stop`。
2. 依上表放入 3 個程式。它們應與 `sensor_config.json`、`best.pt` 同層，不要再多包一層 `Jetson/`。
3. 先啟用你原本已能執行車端程式的 Python 環境，再執行：

```bash
cd "$HOME/Desktop/jetson orin nano"
python3 minitesla_service.py install --pc-host 120.126.10.41
```

請以平常操作 Jetson 的一般帳號執行，不要改成 `sudo python3`。工具會記住這個 `python3` 的完整路徑，並保留必要的 `PATH`、`LD_LIBRARY_PATH`、`PYTHONPATH`。只有寫入服務設定與操作服務時會呼叫 `sudo`，可能要求輸入 Jetson 密碼。

安裝會先檢查設定、模型檔、裝置存取權限、既有套件匯入、程式占用與服務語法。通過後才安裝並立即啟動；下次開機也會啟動。不會下載模型、重裝 PyTorch／torchvision，或重新編譯雷達 SDK。

預設使用：

| 項目 | 設定 |
| --- | --- |
| 電腦 IP | `120.126.10.41` |
| 車端設定檔 | 專案根目錄的 `sensor_config.json` |
| M4 | 設定檔中的 `/dev/ttyTHS0`，115200 |
| 雷達裝置 | `/dev/ttyUSB0`，460800 |
| 雷達執行檔 | 專案內 `rplidar_sdk-master/output/Linux/Release/ultra_simple` |
| 雷達傳送至車端 | `127.0.0.1:6000`，對應你現有 SDK 程式 |
| 展示紀錄 | 專案內 `demo_records/` |

若雷達裝置不同，只在安裝指令追加實際路徑：

```bash
python3 minitesla_service.py install --pc-host 120.126.10.41 --lidar-device /dev/ttyUSB1
```

`--config` 與 `--lidar-binary` 也可指定其他位置。換 IP、Python 環境或專案位置後，先 `stop`，再從新位置／新環境執行 `install`，重新產生設定。若只要先看將產生的設定，可在安裝指令加上 `--dry-run`；這不會安裝、啟動或檢查硬體。

## 往後操作的差異

安裝完成後，Jetson 的主程式、雷達與紀錄程序已在背景執行。往後開機不必另外手動執行這三項。

電腦端仍依前一階段開啟影像接收與控制介面，M／P／K／空白鍵的操作維持原樣。服務啟動成功只表示程序開始執行，不代表模型已載入完畢或控制條件已就緒；仍以控制介面的感測與影像狀態為準。

以下指令在 Jetson 專案目錄執行：

| 目的 | 指令 |
| --- | --- |
| 查看全部服務狀態 | `python3 minitesla_service.py status` |
| 暫時停止全部服務 | `python3 minitesla_service.py stop` |
| 再次啟動全部服務 | `python3 minitesla_service.py start` |
| 程式更新後重新啟動 | `python3 minitesla_service.py restart` |
| 查看最近訊息並持續追蹤 | `python3 minitesla_service.py logs` |
| 停止並取消開機啟動 | `python3 minitesla_service.py disable` |
| 移除本工具的服務設定 | `python3 minitesla_service.py uninstall` |

`logs` 畫面按 `Ctrl+C` 只停止觀看訊息。`stop` 只停本次，下一次開機仍會啟動；要取消開機啟動用 `disable`。`uninstall` 保留專案、模型與紀錄。

若要回到手動執行主程式做除錯，先 `stop`，再使用原本的手動啟動方式。不要同時另開車端主程式或雷達 SDK。原有單一實例與 UART 排他保護保留；雷達 SDK 沒有新增跨程序排他機制，因此舊的手動雷達程序須自行正常結束。

## 匯出一次展示紀錄

安裝後會自動紀錄，不需要另外開終端執行 `record`。展示結束後執行：

```bash
cd "$HOME/Desktop/jetson orin nano"
python3 demo_recorder.py export
```

終端會印出產生的 ZIP 完整路徑，預設位於 `demo_records/`，檔名含 UTC 時間。執行中也可匯出，不會暫停控制。若想納入服務正常停止時的最後紀錄，可先看完展示狀態再匯出；同時停止全部服務時，紀錄程序不保證取到車端的最後一筆停止快照。

匯出包含：

| 檔案 | 內容 |
| --- | --- |
| `samples.csv` | 模式、輸出值、四向距離與有效性、人物警示、巡邏及停車狀態 |
| `events.csv` | 取樣時觀察到的模式、人物警示與其他狀態變化 |
| `records.jsonl` | 保留範圍內的原始精簡紀錄，每行一筆 JSON |
| `manifest.json` | 匯出時間、來源檔案、取樣／事件筆數及執行期識別 |
| `README.txt` | 欄位判讀與紀錄限制 |

這些是每 0.2 秒取樣的展示紀錄，不是錄影，也不是每一筆控制封包的完整追蹤。短於取樣間隔的狀態變化可能沒被記到。`person_event_count` 是人物連續出現的警示次數，並非不同人物的人數；它在車端重啟後重新計數，應搭配 `run_id` 判讀。

`valid_at_snapshot` 表示車端快照當下有效；`valid` 與 `age_ms` 另計入紀錄程序的讀取延遲，因此兩者可能不同。過期、不同開機識別與無法讀取的快照不會沿用上一筆健康狀態。紀錄器的有效性判讀不會回寫車端控制。

持續紀錄採輪替檔案：每檔最多 10 MiB、含目前檔共 5 檔，最多約 50 MiB。到容量上限會刪除最舊的輪替檔，應在需要保留的展示完成後匯出。手動匯出的 ZIP 會保留，不納入這個 50 MiB 上限；系統服務的診斷訊息則由 Jetson 原有 journal 管理。

## 問題判讀

| 現象 | 處理 |
| --- | --- |
| 安裝提示另一份主程式執行中 | 關閉手動主程式，或先用 `stop` 停止已安裝服務，再安裝 |
| 找不到模型或 SDK 執行檔 | 放回原本的 `best.pt`／已編譯 SDK，或使用既有設定路徑；本包不重複附這些檔案 |
| 套件匯入失敗 | 確認目前 `python3` 是原先正常運作的環境，切換後重新安裝服務 |
| 裝置權限不足 | 使用原本可操作相機與串口的帳號；工具不會自行改變裝置權限 |
| 車端或雷達服務反覆重啟 | 用 `logs` 查看模型、相機或串口錯誤，修正後用 `restart`；不能把自動重試當作成功 |
| 三個服務 active，但電腦畫面未就緒 | 檢查原有 5G MiFi／NAT 網路與電腦端程式；沿用現有埠號與連線方式 |
| `demo_records` 只有來源不可用事件 | 查看車端服務與快照路徑；確認已取代本包 `minitesla.py` |
| 安裝因同名服務或外部覆寫而停止 | 工具不覆蓋其他服務設定；先確認現有 `minitesla*.service`／target 的來源 |

服務設定使用 systemd 的程序退出重啟機制，不是推論卡死偵測器。SDK 若仍在執行但沒有輸出有效雷達資料，不會單憑此情況被重啟；資料有效性仍由既有感測狀態顯示。

安裝途中若 `sudo` 或系統指令失敗，可能留下部分服務檔；修正後可重新 `install`，或以 `uninstall` 移除本工具已建立的部分設定。

本次通過既有回歸與新增測試共 294 項、Python 3.8 語法檢查，以及實際 `systemd-analyze verify` 設定檢查；另驗證紀錄器的程序啟停與 ZIP／CSV 匯出。

程式採用的服務語義可參考 [systemd 官方 service 說明](https://github.com/systemd/systemd/blob/main/man/systemd.service.xml)。上述軟體測試與服務設定檢查不代表已在你的 Jetson 上完成開機或實車驗證。
