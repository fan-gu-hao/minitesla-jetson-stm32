# 智慧車：最後版本壓縮檔完整核對

核對日期：2026-11-16。使用者確認本次提供的兩份壓縮檔為其電腦保存的最後正確版本，故以壓縮檔內的原始內容與相對路徑為準。

The two uploaded archives are the owner's authoritative final local versions. Their file bytes and relative project paths define this reconciliation.

儲存庫：`fan-gu-hao/smart-car`，識別碼 `1458034118`，分支 `main`。核對基準提交：`41abdd533c58c8d9563e7f5b78b63023dcaafc7c`。

## 完整核對範圍／Complete comparison scope

| 壓縮檔／Archive | 檔案數／Files | 正式目錄原已一致／Already identical | 內容不同／Different bytes | 原僅在其他目錄／Only elsewhere |
| --- | ---: | ---: | ---: | ---: |
| `minitesla_sensors.zip` | 13 | 11 | 2 | 0 |
| `wheelcontrol(main).zip` | 1,566 | 1,463 | 1 | 102 |
| 合計／Total | **1,579** | **1,474** | **3** | **102** |

每個檔案都計算 SHA-256 與 Git 物件雜湊（blob SHA-1），逐一與基準提交的完整檔案樹核對，並非僅抽查主程式。完整 1,579 列結果見 [CSV 核對清單](source-audit-20261116.csv)。

Every file was hashed and compared with the pinned repository tree. The linked CSV records all 1,579 archive entries, target paths, hashes, prior locations, and actions.

## 正式目錄還原／Canonical project restoration

| 上傳內容／Uploaded contents | GitHub 正式位置／Canonical location |
| --- | --- |
| `minitesla_sensors.zip` 中的 `minitesla_sensors/` | `jetson/minitesla_sensors/` |
| `wheelcontrol(main).zip` 根目錄下的完整工程 | `firmware/stm32f4/` |

**3 份內容不同的檔案全部依本次原檔還原**：

- `jetson/minitesla_sensors/runtime.py`：一處重複執行提示文字。
- `jetson/minitesla_sensors/__init__.py`：一處模組說明字串。
- `firmware/stm32f4/Core/Src/main.c`：兩處註解名稱。

差異源自先前的智慧車命名整理，沒有控制邏輯差異。本次為遵守原檔優先，連這些文字都還原為壓縮檔原文；README 的智慧車標題、封面與展示影片不受影響。

All three differing files are restored byte-for-byte from the supplied archives, including their original internal naming. The differences were display text or comments, not control logic. The public Smart Car title, cover, and demonstration video remain unchanged.

**102 份內容原已保存、但不在工程原位置的 M4 檔案，補回正式工程**：

- 23 份 `Core/` 下的獨立感測／控制來源、標頭與 VL53L0X 原廠來源。
- 79 份 `MDK-ARM/` 下的編譯產物、工程附屬資料、建置紀錄與原始壓縮備份。

其中 101 份原位於 `archive/legacy_builds/wheelcontrol(main)/`，1 份建置紀錄原位於 `logs/m4/`。原歷史目錄及集中紀錄保留，正式工程新增相同內容的原路徑，讓整個 M4 工程對應本次壓縮檔。

The 102 files were already preserved elsewhere (101 in the historical archive and one build log in `logs/m4/`). Their original project paths are restored without removing the existing historical records.

## 工程與相依檢查／Project and dependency checks

- 13 個感測 Python 模組通過 Python 3.8 語法檢查及匯入檢查。
- 固定車體設定載入成功；左右轉彎半徑各 0.48 公尺、後懸 0.14 公尺與本次原檔一致。
- M4 的 `wheelcontrol.uvprojx` 保持原檔；其中 23 個來源路徑與 5 個標頭搜尋目錄全部存在。
- `main.c` 已內嵌 ToF 平台及原廠來源。補回的獨立來源維持壓縮檔原貌，沒有額外加入工程的編譯清單，避免重複編譯定義。
- 檢查範圍是檔案內容、路徑、語法與模組相依；本次未連接實車、啟動控制主程式、重新編譯或燒錄 M4。
- `.hex`、`.axf`、`.o` 與建置紀錄按上傳原檔保存，並非本次重新產生的建置成果。

The 13 Python modules pass syntax and import checks; fixed geometry loads correctly. All 23 Keil source paths and five include directories exist. The original project file is unchanged, and standalone ToF sources are not added to the compile list because `main.c` already embeds them. This is a source/dependency check, not a new hardware test or firmware build.

## 壓縮檔識別／Archive digests

| 壓縮檔／Archive | SHA-256 |
| --- | --- |
| `minitesla_sensors.zip` | `09fa11c78fb39ea667945c06fdb1b023701d8cdffff8af65ada347fd965e0cea` |
| `wheelcontrol(main).zip` | `b8a493006ba594b56d844cddede69f1f6fe32f89ce51ae7f263bf19717258280` |

本次更新只涉及上述智慧車儲存庫的兩個程式目錄及相關核對／操作說明。歷次核對報告仍保留當時結果；這兩個目錄以本次原檔為最新依據。

This update is limited to the two source directories and related audit/deployment documentation in this repository. Earlier audit reports remain historical records; these uploaded archives are the latest authority for the two directories.
