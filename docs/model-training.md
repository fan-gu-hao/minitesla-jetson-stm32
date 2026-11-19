# 模型重新訓練與車端部署／Model retraining and onboard deployment

[返回中文首頁](../README.md) · [English homepage](../README.en.md) · [部署操作／Deployment](operation.md) · [停車測試版／Parking prototype](experimental.md)

## 開發目的與完成範圍／Goal and completed work

我原先規劃讓智慧車辨識空車位，因此將空車位圖片加入訓練資料，重新訓練 YOLOv8 實例分割模型，產出 `best.pt`，並接入 Jetson Orin Nano 的相機推論、分割結果顯示與遠端回傳流程。受後續開發時程限制，停車相關功能僅完成測試版，保留作為後續擴充。

I planned to add vacant-parking-space recognition to the vehicle, included vacant-space images in the training data, retrained a YOLOv8 instance-segmentation model, and produced `best.pt`. I integrated these weights into camera inference, segmentation visualization, and remote video return on the Jetson Orin Nano. With limited time for further development, parking-related functions reached a prototype stage and remain extensions.

## 開發步驟／Development steps

下表整理這部分的工程流程；模型訓練是車端模型部署之前的獨立開發階段。它不代表每次啟動車子都要重新訓練。

The steps describe the development workflow. Model training is a separate stage before onboard deployment, not a task repeated whenever the vehicle starts.

| 步驟／Step | 實作內容／Work performed | 產出與用途／Output and purpose |
| --- | --- | --- |
| 1．確定辨識目標／Define the target | 將空車位辨識納入智慧車的擴充目標。Include vacant-space recognition as a vehicle extension. | 模型需包含空車位類別。Define the need for a vacant-space class. |
| 2．加入訓練影像／Add training images | 將空車位圖片納入訓練資料；模型保存的資料設定名稱為 `fanuse.yaml`。Include vacant-space images; the saved training configuration references `fanuse.yaml`. | 準備自訂任務的訓練輸入。Prepare training inputs for the custom task. |
| 3．重新訓練並保存模型／Retrain and save | 重新訓練 YOLOv8 分割模型，產出 `best.pt`。Retrain the YOLOv8 segmentation model and produce `best.pt`. | 保存含 `vacant` 類別的模型權重。Retain weights with the `vacant` class. |
| 4．部署到車端／Deploy onboard | 將模型放在 `jetson/best.pt`，由相機設定指定模型路徑。Place the weights at `jetson/best.pt` and select the model through the camera configuration. | 車端可載入自訂模型進行推論。Make the custom model available to the vehicle runtime. |
| 5．整合影像處理與回傳／Integrate vision and transport | 在相機程序執行模型、追蹤物件、繪製結果，將影像與辨識摘要交給其他工作程序。Run inference/tracking, draw results, and hand off video and detection metadata. | 完成模型到車載影像流程的整合。Integrate the model into onboard vision. |
| 6．保留停車測試版／Retain the parking prototype | 保留車位搜尋、對位與倒車入庫程式；視覺類別與停車決策的完整串接仍待後續開發。Retain scouting, alignment, and reverse-parking code; full integration with the vision class remains future work. | 以實驗性擴充呈現。Present parking as an experimental extension. |

## 目前模型檔可核對的內容／Evidence in the retained model

以下資訊來自 [`jetson/best.pt`](../jetson/best.pt) 保存的類別表及模型／訓練中繼資料；核對時只解析封裝內容，未執行模型。這些資訊描述所保存的模型，不代表空車位辨識的準確率或停車成功率。

These values come from the class map and model/training metadata retained in [`jetson/best.pt`](../jetson/best.pt), inspected without executing the model. They describe the artifact, not vacant-space recognition accuracy or parking success rates.

| 項目／Item | 保存內容／Retained value |
| --- | --- |
| 任務／Task | 實例分割（instance segmentation），`task=segment`、`SegmentationModel` |
| 訓練設定中的模型／Model referenced by training configuration | `yolov8s-seg.pt`；模型結構名稱／architecture name：`yolov8s-seg.yaml` |
| 資料設定名稱／Dataset configuration name | `fanuse.yaml` |
| 類別數／Class count | 81 |
| 類別表／Class map | 編號 0–79 保留原有 COCO 類別名稱，編號 80 為 `vacant`。IDs 0–79 retain the original COCO class names; ID 80 is `vacant`. |
| 保存的工具版本／Saved framework version | Ultralytics `8.3.127` |
| 部署格式／Deployment format | 直接載入 `.pt` 權重。Direct `.pt` model loading. |

類別表證明模型包含新增空車位類別；它不等於公開了原始訓練資料集，也不能單獨證明用了哪一版 COCO 資料集或所有類別的訓練品質。舊預測筆記中的 `yolov8x-seg.pt` 是另一條歷史預測命令；此處以實際保存的 `best.pt` 中繼資料為準。

The class map establishes that the model includes the added vacant-space class. It does not provide the original dataset, identify a particular COCO dataset release, or establish per-class quality. A separate historical prediction command references `yolov8x-seg.pt`; this page follows the metadata of the actual retained `best.pt`.

## 程式如何使用訓練結果／How the code uses the trained model

[`sensor_config.json`](../jetson/sensor_config.json) 的 `camera.model` 指向 `best.pt`。[`minitesla.py`](../jetson/minitesla.py) 的 `camera_worker()` 載入該模型，使用 ByteTrack 追蹤，並讀取模型回傳的類別名稱、信心分數、邊界框及分割輪廓。

The camera configuration points to `best.pt`. The vehicle application's `camera_worker()` loads those weights, uses ByteTrack tracking, and reads the returned class names, confidence scores, bounding boxes, and segmentation contours.

關鍵呼叫摘錄／Key calls:

```python
model = YOLO(camera["model"])

results = model.track(
    source=frame,
    tracker="bytetrack.yaml",
    persist=True,
    stream=False,
    conf=float(camera.get("confidence", 0.25)),
    iou=float(camera.get("iou", 0.7)),
    verbose=False,
)
```

程式從 `result.names` 取得類別名稱，沒有將顯示範圍固定在原有 80 類；因此自訂類別可沿用既有繪圖與傳送流程。推論在相機工作程序內進行，不會在此重新訓練模型。現行部署使用 `best.pt`，未把 ONNX、TensorRT 或量化列為本專案已完成成果。

Class names are read from `result.names` rather than restricting visualization to the original 80 classes, so the custom class can use the existing drawing and transport path. This camera process performs inference, not retraining. The deployed artifact is `best.pt`; ONNX, TensorRT, and quantization are not claimed as completed work in this project.

## 空車位辨識與停車控制的關係／Recognition and parking control

目前已完成的是加入空車位影像的模型重新訓練、保存權重，以及車端推論整合。停車相關功能因開發時程限制保留測試版。

Completed work includes retraining with vacant-space images, retaining the weights, and integrating onboard inference. Parking-related functionality remains a prototype because of the development schedule.

現行 `minitesla.py` 給停車規劃程序的輸入只有時間資訊、M4、ToF 與雷達資料，不包含相機辨識結果；`parking_auto.py` 的距離來源標記為 `lidar_tof`，`parking_geometry.py` 使用雷達幾何建立車位候選。因此目前的 `vacant` 視覺結果尚未直接驅動自動停車決策，後續可在此基礎上整合。詳見[停車測試版範圍](experimental.md)。

The current planner input contains timing, M4, ToF, and lidar data, excluding camera detections. `parking_auto.py` identifies its range source as `lidar_tof`, and `parking_geometry.py` builds parking candidates from lidar geometry. The `vacant` vision output therefore does not yet directly drive automatic-parking decisions; connecting these parts is future integration work.

## 公開資料與重建範圍／Published material and reproducibility

儲存庫已保存訓練完成的模型與車端部署程式。原始訓練影像、標註、`fanuse.yaml` 實體檔、完整訓練腳本及評估報告尚未收錄，不能僅靠目前檔案完整重跑原訓練。影像張數、標註工具、實際訓練硬體、完整訓練輪數及辨識準確率不在此補寫為已核實數據。

The repository includes trained weights and deployment code. Original training images, annotations, the actual `fanuse.yaml` file, full training scripts, and evaluation reports are not included, so the original training run cannot be fully reproduced from the published files alone. This page does not invent image counts, annotation tools, training hardware, completed epochs, or accuracy metrics.

執行目前車端版本可直接使用既有 `best.pt`，無須先重新訓練；操作方式見[部署與操作](operation.md)。[`archive/documents/yolov8 command`](../archive/documents/yolov8%20command) 保存的是預測命令，並非訓練指令。

The current vehicle deployment can use the supplied `best.pt` without retraining. See [deployment and operation](operation.md). The archived YOLO command note contains prediction commands, not training commands.
