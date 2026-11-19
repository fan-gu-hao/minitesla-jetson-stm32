# 實驗性擴充／Experimental extension

我曾將空車位圖片加入訓練資料，重新訓練模型產出 `best.pt`，並部署到 Jetson；模型新增的空車位類別為 `vacant`（編號 80）。受後續開發時程限制，車位搜尋、自動對位與倒車入庫僅完成測試版，保留作為後續擴充，未納入主要展示成果。

I added vacant-space images to the training data, retrained the model to produce `best.pt`, and deployed it on Jetson. The model's added class is `vacant` (ID 80). With limited time for further development, parking-space search, alignment, and reverse parking reached a prototype stage and remain extensions outside the main demonstration claims.

相關來源包括 `parking.py`、`parking_auto.py`、`parking_geometry.py`、`parking_path.py`、`parking_scout.py`、`parking_config.py`、`vehicle_config.py` 與 `parking_setup.py`，以及控制器／操作台中的對應介面。部分模組仍被現行程式匯入，因此本次保留完整相依關係，不以刪檔方式隱藏功能。

Relevant modules include the parking engine, geometry, path planning, scouting, configuration, and preview tools, with corresponding controller/UI integration. Some remain imported by the current application, so dependencies are retained intact.

此分類調整的是展示定位，不是關閉程式功能的開關。現有程式仍保留 K、G、Y、N 等操作入口；歷次操作細節留在 [`archive/release_notes/`](../archive/release_notes/)。

This classification changes the presentation scope, not the runtime enable switch. Existing K/G/Y/N controls remain in the code. Historical operating details are retained in [`archive/release_notes/`](../archive/release_notes/).

## 模型訓練與停車規劃／Training and parking planning

[模型重新訓練與部署](model-training.md)是已完成的開發工作。現行停車規劃主要使用雷達與 ToF：`minitesla.py` 的規劃輸入不含相機辨識結果，`parking_auto.py` 的距離來源為 `lidar_tof`。因此，目前未將視覺 `vacant` 類別直接接入自動停車決策；完整串接仍屬後續開發。

[Model retraining and deployment](model-training.md) were completed. The current parking planner primarily uses lidar and ToF: its input in `minitesla.py` excludes camera detections, and `parking_auto.py` identifies the range source as `lidar_tof`. The visual `vacant` class is not yet fed directly into automatic-parking decisions; completing that integration remains future work.
