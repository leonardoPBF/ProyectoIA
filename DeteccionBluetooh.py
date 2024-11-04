import torch
import cv2
import numpy as np
import pathlib
import serial
import time

# Redefine PosixPath para entornos Windows
pathlib.PosixPath = pathlib.WindowsPath

# Ruta del modelo personalizado
weights = "D:/Leonardo/Universidad/Ciclo-8/IA/Proy/Deteccion/modelo.pt"
yolo_path = "D:/Leonardo/Universidad/Ciclo-8/IA/Proy/Deteccion/yolov5"
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Cargar el modelo YOLOv5 a PyTorch
model = torch.hub.load(yolo_path, 'custom', path=weights, source='local', force_reload=True)
model.to(device).eval()

print("Modelo cargado en", device)

# Lista de nombres de las clases
class_names = ['G', 'C', '3', 'S']

# Configura la conexión Bluetooth
bluetooth_port = "COM8"  # Cambia esto al puerto COM donde esté tu módulo Bluetooth
baud_rate = 9600
bluetooth = serial.Serial(bluetooth_port, baud_rate)

# Configura la captura de video
camaraip = "http://192.168.0.5:8080/video"  # Aplicación IP Webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Redimensiona el fotograma antes de procesarlo y mostrarlo
    frame = cv2.resize(frame, (500, 380))

    # Realiza la detección en el fotograma
    results = model(frame)

    # Procesar y mostrar resultados
    for det in results.xyxy[0]:  # [x1, y1, x2, y2, conf, cls]
        x1, y1, x2, y2, conf, cls = det
        if conf > 0.7:
            class_id = int(cls)
            class_name = class_names[class_id] if class_id < len(class_names) else f"Clase {class_id}"
            label = f"{class_name}: {conf:.2f}"

            # Dibuja la detección en el fotograma
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Enviar el nombre de la clase si la confianza supera el 85%
            if conf > 0.94:
                try:
                    # Enviar el nombre de la clase a través de Bluetooth
                    bluetooth.write((class_name + "\n").encode())
                    print(f"Enviado al ESP8266 vía Bluetooth: {class_name}")

                    time.sleep(1.5)  # Pequeño delay para evitar saturación
                except Exception as e:
                    print(f"Error al enviar datos vía Bluetooth: {class_name}", e)

    # Muestra el fotograma con detecciones
    cv2.imshow("YOLOv5 Detección", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cerrar conexiones
cap.release()
cv2.destroyAllWindows()
bluetooth.close()
