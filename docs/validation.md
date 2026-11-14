# 版本與驗證範圍／Version and validation scope

整理日期：2026-11-15（臺灣時間）。本頁只描述已完成的來源核對與檢查，不以舊文件中的測試數字取代本次結果。

Prepared on 2026-11-15, Taiwan time. This page distinguishes source checks performed during recovery/publication from historical test reports.

## 來源／Sources

- 原始 M4 完整工程、雷達 SDK、`best.pt` 與第一、二階段至第十階段的更新包。
- 後來找回的 `jetson orin nano.zip`，包含 197 個檔案。
- 原始照片、文件、操作筆記、`torchvision_build.log` 與歷史 M4 建置紀錄。

Sources include the original complete M4 project, lidar SDK, trained model, staged update packages, 197 recovered Jetson files, and original photos/documents/notes/logs.

## 核對結果／Checks performed

| 項目／Item | 結果與範圍／Result and scope |
| --- | --- |
| Jetson 檔案比對 | 197 個找回檔案中，與整併版共有 193 個路徑；191 個內容相同，另 2 個差異為舊 `parking_setup.py` 與電腦位址設定。其餘 4 個是筆記、舊設定與編譯紀錄。Of 193 shared paths, 191 matched; differences were the old helper and host configuration. Four additional files were notes/config/logs. |
| Python | 22 個現行電腦／Jetson Python 檔通過 Python 3.8 語法檢查；十三個感測模組完成匯入與基本建構核對。Syntax and basic import/construction checks. |
| 通訊 | 依 M4 欄位格式建立 79 位元組遙測封包，檢查拆包解析與四向資料入庫。Checked telemetry parsing and sensor-store integration with constructed frames. |
| 控制資料流 | 核對待命輸出、程序間資料介面、單一 UART 寫入來源及任務分派相依關係。Checked standby data flow and ownership/interfaces. |
| M4 工程 | Keil 的 23 個來源路徑與 5 個標頭搜尋目錄存在；目前 `main.c` 符合正式 ToF 更新來源。Checked referenced paths and source provenance. |
| 雷達 | 來源程式使用 `127.0.0.1:6000`；保留的執行檔為 AArch64，發布時設為可執行。Checked destination and executable format/permission. |
| 模型 | `best.pt` 與原上傳逐位元組相同。Byte-identical to the recovered upload. |

模型 SHA-256：／Model SHA-256:

```text
02fefb70faf6edf8e676316fa72d46611bd166852204c06f0229479a9f196709
```

## 發布整理／Publication changes

- 整理成 `pc/`、`jetson/`、`firmware/stm32f4/`、`logs/`、`notes/` 與 `archive/`。
- 採用既有固定參數版 `parking_setup.py`，修正舊版 `load_settings(path)` 與新版零參數介面的不相容。
- `sensor_config.json` 電腦位址明列為先前約定的 `120.126.10.41`；上傳原版另存歷史資料。
- 雷達執行檔補上執行權限；其內容未重新編譯或修改。
- 新增中英文說明；停車相關僅列實驗性保留。

Publication reorganizes paths, restores the previously delivered fixed-parameter helper, pins the established desktop address, restores executable permission, and adds bilingual documentation. It does not introduce new driving algorithms.

本次沒有連接實車、執行 Jetson GPU 推論、重新建置 M4 韌體或重跑歷史全部測試。原作者先前實車使用情況與本次來源檢查是不同層次的證據；後續實測紀錄可再追加。

No physical vehicle connection, Jetson GPU inference run, fresh M4 build, or complete rerun of historical tests was performed during publication. Previous vehicle use and current source checks are distinct evidence categories.

## 保存範圍／Preservation scope

來源清單記錄於 [`source_manifest.csv`](source_manifest.csv)。已保存的來源內容及找回的 197 個檔案內容都有對應保存位置；相同內容可共用一份。舊復原包的清單仍放在 `archive/restoration/`，其路徑描述的是當時目錄，不是本儲存庫的新目錄。

The [source manifest](source_manifest.csv) maps preserved files to their recovery sources. Every recovered source payload and all 197 found-file payloads have a retained counterpart; identical payloads may share a copy. Old restoration manifests describe the historical layout.

未曾提供的 `orin nano pack/`、`paper文件/` 完整內容、完整安裝環境、未上傳的修改或實測影片，不在本次已取得資料範圍內。

Complete unprovided local folders, installed environments, unuploaded changes, and unprovided demonstration videos are outside the available source set.
