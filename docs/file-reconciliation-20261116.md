# 智慧車：18 份補傳檔案核對紀錄

> 2026-11-18 公開位址整理：本儲存庫目前檔案中的實際接收端位址已改為文件範例 `192.0.2.10`，包括舊版程式、操作筆記及歷史文字紀錄。歷史雜湊與核對結果描述當時匯入版本；經此次位址替換的文字檔不再與原始上傳逐位元組相同。模型、M4 韌體及原建置產物未因此重新建置或修改。
>
> 2026-11-18 publication update: actual receiver addresses in the current repository files were replaced with the documentation placeholder `192.0.2.10`, including legacy source, command notes, and historical text records. Historical hashes and comparison results describe their original import stage; redacted text files are no longer byte-identical to the uploads. The model, M4 firmware, and saved build outputs were not rebuilt or modified by this address update.

核對日期：2026-11-16。核對對象為 `fan-gu-hao/smart-car`，儲存庫識別碼 `1458034118`，分支 `main`。

本次以使用者補傳的 18 份原檔逐一核對，讀取的 GitHub 基準提交為 `a9fbdc789f81185735e2f76bf9f89be7f0c498e9`。檔名括號中的編號不能證明新舊，模型、文件及紀錄均以檔案內容雜湊確認。

## 結果與本次修正

- **7 份**模型、文件、紀錄與其他程式，已在對應正式或文件目錄中逐位元組一致。
- **7 份**Python 程式只有先前依使用者要求將「MiniTesla」改成「智慧車／Smart Car」的顯示文字差異。保留目前命名，控制流程、函式介面與通訊識別均不回退。
- **1 份** `sensor_config.json` 同步為這次提供的原文，包含 `network.pc_host: null` 與原排版。未指定電腦位址時，主程式仍使用既定的 `192.0.2.10`；`--pc-host` 仍可指定其他位址。
- **2 份**停車原檔已完整保存在歷史目錄。它們屬於讀寫 JSON 的設定方式，與現行固定參數介面不同；本次保留已確認的車體參數與現行工具。
- **1 份** `指令.txt` 先前未收錄，補入 `notes/pc_dashboard_windows.txt`，原文完整保留。

本次同步的設定檔原文也已存在 `archive/early_sources/sensor_config_uploaded.json`；這次修正的是正式執行位置的內容，不另外新增重複原檔。

## 逐檔對照

| 本次提供的檔案 | GitHub 對應位置 | 核對與處理 |
| --- | --- | --- |
| `parking_config.json` | `archive/legacy_builds/jetson orin nano/parking_config.json` | 歷史原檔完全一致；現行程式使用固定參數 |
| `pc_receive.py` | `pc/pc_receive.py` | 僅顯示名稱不同；保留智慧車命名 |
| `pc_dashboard.py` | `pc/pc_dashboard.py` | 僅顯示名稱不同；保留智慧車命名 |
| `室內智慧車功能(3).docx` | `archive/documents/室內智慧車功能(2).docx` | 內容完全一致；下載編號不同 |
| `parking_setup(1).py` | `archive/early_sources/parking_setup_phase9_v1.py` | 歷史原檔完全一致；現行固定參數工具保留 |
| `指令.txt` | `notes/pc_dashboard_windows.txt` | 本次補入，原文不變 |
| `demo_recorder.py` | `jetson/demo_recorder.py` | 僅顯示名稱不同；保留智慧車命名 |
| `best(1).pt` | `jetson/best.pt` | 內容完全一致 |
| `sensor_status.py` | `jetson/sensor_status.py` | 僅顯示名稱不同；保留智慧車命名 |
| `pc_control.py` | `pc/pc_control.py` | 僅顯示名稱不同；保留智慧車命名 |
| `compile 指令.txt` | `notes/compile_torchvision.txt` | 內容完全一致 |
| `all instruct(1).txt` | `notes/all_instruct_current.txt` | 內容完全一致 |
| `minitesla(3).py` | `jetson/minitesla.py` | 僅顯示名稱不同；保留智慧車命名 |
| `sensor_config.json` | `jetson/sensor_config.json` | 本次以提供檔案同步；同步後逐位元組一致 |
| `技術文件(2).docx` | `archive/documents/技術文件(1).docx` | 內容完全一致；下載編號不同 |
| `minitesla_service.py` | `jetson/minitesla_service.py` | 僅顯示名稱不同；保留智慧車命名 |
| `torchvision_build(1).log` | `logs/torchvision_build.log` | 內容完全一致 |
| `pc_media.py` | `pc/pc_media.py` | 內容完全一致 |

## 停車檔案的相依差異

補傳的 `parking_setup(1).py` 呼叫 `load_settings(path)`；現行 `minitesla_sensors/parking_config.py` 提供的是不接收參數的 `load_settings()`。以補傳工具執行唯讀的 `status` 操作，即可重現：

```text
TypeError: load_settings() takes 0 positional arguments but 1 was given
```

因此，單獨覆蓋 `jetson/parking_setup.py` 會造成介面不相容。正式位置保留目前的固定參數版；補傳原檔與 `archive/early_sources/parking_setup_phase9_v1.py` 完全一致，沒有遺失。

補傳的 `parking_config.json` 為 `enabled: false`，雷達朝向、橫向位置、後懸及左右轉彎半徑仍有 `null`，也尚未包含後來的巡邏找位任務欄位。現行 `vehicle_config.py` 則保有使用者後來確認的雷達 0° 朝前、橫向置中、後懸 0.14 公尺、左右轉彎半徑各 0.48 公尺。這次維持既定「參數寫在程式內，啟動時不要求輸入」的操作方式。

停車功能維持實驗性保留，未改成主要展示成果。

## 執行的核對

- 從 GitHub 基準提交取得檔案樹，比對 Git 物件雜湊（blob SHA-1）；補傳檔另外計算 SHA-256。
- 檢查 22 份現行 Python 程式的 Python 3.8 語法。
- 對上述 7 份名稱差異程式比較抽象語法樹（AST），確認僅有 24 個顯示／說明字串不同，沒有敘述結構或控制邏輯差異。
- 現行停車工具的唯讀 `status` 執行成功；補傳工具與現行設定介面的不相容已重現。
- 隔離執行主程式的設定載入函式，確認同步後預設電腦位址為 `192.0.2.10`，命令列位址覆寫仍生效。
- 未啟動車輛控制主程式、串列通訊、網路傳送或模型推論；上述結果為檔案及相依核對，並非新的實車測試。

首頁、封面、展示影片、M4 韌體、雷達來源及其他 GitHub 專案不屬於本次修改範圍。既有 `source_manifest.csv` 是初次發布時的來源快照，本次補檔與處理結果記於此頁。

## 本次提供檔案的 SHA-256

```text
7bac3ba5a3de59ae3eb40a92f03f693c256077df65854f87fe9c1fe180ac8321  parking_config.json
38f418218c00134747e9818c8c3dec36f1bcb2599c96e9140125488b23560a70  pc_receive.py
773ad1d9fd0c26bfa07e07eaf2d0881be4883144eccf560e8750579c9735225c  pc_dashboard.py
df6b20456a31850d7e3b411e12b8d5848fc680737399554802be9d164bd3c43c  室內智慧車功能(3).docx
ce56dbd1411c4507256d715e733384711b9866f53a9749d670cc58d49f65dc7d  parking_setup(1).py
23fafccecb78f5d3252406d162b8b6105037589b985fed0306734e317904d35f  指令.txt
af05aad6afb57f56de2425bd3480094db311532b14934b447cabdb4ba3940d46  demo_recorder.py
02fefb70faf6edf8e676316fa72d46611bd166852204c06f0229479a9f196709  best(1).pt
013c4592b6fbbcaacf16a752d90a71eb39e2597dbf51b7b1df7e527ec9ad6a5c  sensor_status.py
17dcc3d9e2619ed35cf2206833bb9692911ef88c0ca677f434abb60208ed4dbf  pc_control.py
5296b436e98c42faf306777d5fcfe2c128c3cc97c4265c51e090dee0073f6027  compile 指令.txt
b5075405cdf47bf57fad9a2f3b391372ed0ffbb6c64a6249d909cba7fb594134  all instruct(1).txt
c1a70d0ec811f5709262fc5f5780029a64993839768bce358ea38703470e7d5b  minitesla(3).py
7a446b6ac30887073e34032751349fde806b12f41763339ec988ec92882d9f83  sensor_config.json
b3d7238306315a50caa48925549083c41bc7c13e75df04fcf190112e9b6bd35f  技術文件(2).docx
bc518c20979c5a0524b857d7f154648dc4179f74f97ee645bcf9870dcbdab236  minitesla_service.py
ac6803e60710a9af4bd3baa3003e85c052623f86a66774a298be7ab4f5db2746  torchvision_build(1).log
918c07d3e9559a934af0dac1e1b38ae52f8ce78e9c2cc765fdb19001b039cbe8  pc_media.py
```
