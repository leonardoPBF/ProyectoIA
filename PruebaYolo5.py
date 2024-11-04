import torch
import cv2

# Carga el modelo YOLOv5 desde torch.hub
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model.to(device)
model.eval()

# Configura la captura de video
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Realiza la detección
    results = model(frame)  # YOLOv5 maneja la conversión automática a tensor y preprocesado
    detections = results.pandas().xyxy[0]  # Obtiene resultados en formato de DataFrame

    # Procesar y mostrar los resultados
    for _, row in detections.iterrows():
        x1, y1, x2, y2, conf, cls = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax']), row['confidence'], row['name']
        if conf > 0.5:  # Filtrar detecciones con confianza mayor a 0.5
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{cls}: {conf:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Muestra el fotograma con detecciones
    cv2.imshow("YOLOv5 Detección", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
