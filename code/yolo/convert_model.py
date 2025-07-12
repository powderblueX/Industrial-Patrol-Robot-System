from ultralytics import YOLO
import openvino as ov
import os

# 加载YOLOv8模型
model = YOLO('models/last.pt')

# 导出为ONNX格式
model.export(format='onnx', imgsz=640)

# 导出为OpenVINO IR格式
model.export(format='openvino', imgsz=640)

print("模型转换完成！")
print(f"OpenVINO模型保存在: {os.path.abspath('models/last_openvino_model')}") 