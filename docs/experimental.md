# 實驗性擴充／Experimental extension

專案另保留車位搜尋、自動對位與倒車入庫的實驗性程式。相關功能仍待完善，僅作為後續擴充與研究參考，未納入本次主要展示成果。

Experimental parking-space search, alignment, and reverse-parking code is retained for future development and research. These functions still need refinement and are excluded from the main demonstration claims.

相關來源包括 `parking.py`、`parking_auto.py`、`parking_geometry.py`、`parking_path.py`、`parking_scout.py`、`parking_config.py`、`vehicle_config.py` 與 `parking_setup.py`，以及控制器／操作台中的對應介面。部分模組仍被現行程式匯入，因此本次保留完整相依關係，不以刪檔方式隱藏功能。

Relevant modules include the parking engine, geometry, path planning, scouting, configuration, and preview tools, with corresponding controller/UI integration. Some remain imported by the current application, so dependencies are retained intact.

此分類調整的是展示定位，不是關閉程式功能的開關。現有程式仍保留 K、G、Y、N 等操作入口；歷次操作細節留在 [`archive/release_notes/`](../archive/release_notes/)。

This classification changes the presentation scope, not the runtime enable switch. Existing K/G/Y/N controls remain in the code. Historical operating details are retained in [`archive/release_notes/`](../archive/release_notes/).
