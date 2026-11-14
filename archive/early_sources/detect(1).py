from ultralytics import YOLO
import cv2
import numpy as np

CLASS_NAMES = [  # 原始類別名照舊
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
    "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
    "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet",
    "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush", "vacant"
]

model = YOLO("best.pt")
cap = cv2.VideoCapture(0)

def get_color(idx):
    """根據 ID 生成獨特 RGB 顏色"""
    np.random.seed(idx)
    return tuple(int(x) for x in np.random.randint(0, 255, 3))

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # YOLOv8 tracking + segmentation 推理
    results = model.track(
        source=frame,
        tracker="bytetrack.yaml",
        persist=True,
        stream=True,
        conf=0.25,  # YOLOv8 預設
        iou=0.7     # YOLOv8 預設
    )

    for r in results:
        boxes = r.boxes
        ids = r.boxes.id if r.boxes.id is not None else []

        # ➤ segmentation 遮罩繪製（每個用不同顏色）
        if hasattr(r, "masks") and r.masks is not None:
            for i, seg in enumerate(r.masks.xy):
                color = get_color(int(ids[i]) if len(ids) > i else i)
                pts = seg.reshape(-1, 1, 2).astype(int)
                cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)

        for i, box in enumerate(boxes):
            xyxy = box.xyxy[0].cpu().numpy()
            conf = box.conf[0].item()
            cls = int(box.cls[0])
            class_name = CLASS_NAMES[cls] if cls < len(CLASS_NAMES) else f"class_{cls}"
            track_id = int(ids[i]) if len(ids) > i else -1
            color = get_color(track_id if track_id != -1 else cls)

            # ➤ 終端輸出
            print(f"[ID {track_id}] 類別={class_name} | 信心={conf:.2f} | 框={xyxy.astype(int)}")

            # ➤ 畫框與標籤
            cv2.rectangle(frame,
                          (int(xyxy[0]), int(xyxy[1])),
                          (int(xyxy[2]), int(xyxy[3])),
                          color, 2)
            cv2.putText(frame,
                        f"ID {track_id} {class_name} {conf:.2f}",
                        (int(xyxy[0]), int(xyxy[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        color, 1)

    cv2.imshow("YOLOv8 Seg+Track", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

