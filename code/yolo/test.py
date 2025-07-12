import cv2
from ultralytics import YOLO

# 加载训练好的模型
model = YOLO("D:\\vscode\\HumanRecognition\\models\\last.pt")  # 修改为你的模型路径

# 打开摄像头（默认摄像头索引为0）
cap = cv2.VideoCapture(0)

while cap.isOpened():
    # 读取摄像头帧
    success, frame = cap.read()
    
    if not success:
        print("无法读取摄像头画面")
        break

    # 使用YOLOv8进行人形检测
    results = model.predict(
        source=frame,
        conf=0.5,       # 置信度阈值
        imgsz=640,      # 输入尺寸（根据训练配置调整）
        classes=[0],    # 仅检测类别0（person）
        verbose=False   # 关闭冗余输出
    )

    # 在画面中绘制检测结果
    annotated_frame = results[0].plot()

    # 显示实时画面
    cv2.imshow("YOLOv8 人形检测", annotated_frame)

    # 按'q'退出
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()