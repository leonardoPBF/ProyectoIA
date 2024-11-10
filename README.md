## <div align="center">Proyecto deteccion de señales de trancito con YOLOv5 🚀</div>

Para la realizacion del proyecto se utilizaron las siguientes tecnologias:

- python v3.9
- roboflow -> para el etiquetado y exportacion del dataset
- google colab -> para el entrenamiento del dataset
- vscode -> implementacion de opencv para la camara en vivo
- arduino IDE

### <div align="center">Accede a mi desarrollo</div>

<div align="center">  
    <a href="https://colab.research.google.com/drive/15BO9J9km48Zsapa2_s0zK_kZMLrXM4bV?usp=sharing"><img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Open train colab"></a>    
    <a  href="https://universe.roboflow.com/signal-stop/stop-signal"><img src="https://avatars.githubusercontent.com/u/120690883?s=280&v=4" width="10%" alt="Open In Kaggle"></a>
  </div>
  <br>

Para mas informacion de YOLO <a href="https://docs.ultralytics.com/yolov5/">Documentacion web oficial</a> 

## <div align="center">Configuracion basica</div>


<details open>
<summary>Install</summary>
Recomenado 

- PyTorch>=1.8
- python>=3.9

Descomprimir stop signal.v3i.yolov5pytorch.zip que es el dataset personalizado

```bash
git clone https://github.com/ultralytics/yolov5  # clonar
cd yolov5
pip install -r yolov5/requirements.txt  # instalara todas las librerias necesarias
```

Librerias utilizadas

```bash
import torch
import cv2
import numpy as np
import pathlib
import socket
import time
```

```bash
#Importante configurar tus propias rutas de entorno

weights = "D:/./././modelo.pt" #modificar
yolo_path = "D:/././yolov5" #modificar
```

</details>

#### Posibles errores

Para el problema de **yolov5\models\common.py:892: FutureWarning: torch.cuda.amp.autocast(args...) is deprecated. Please use torch.amp.autocast('cuda', args...) instead**. En la ruta yolov5\models\common.py configurar: 
```bash
line 892 -> with torch.amp.autocast('cuda')
linea 865 -> with torch.amp.autocast('cuda'):
```

## <div align="center">Codigo microcontroladores</div>

Configuracion usada en los microcontroladores

<details close>
<summary>Arduino Uno</summary>

```
#include <AFMotor.h>
#include <SoftwareSerial.h>

SoftwareSerial EspSerial(0, 1); // RX, TXmarron

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

void setup() {
  Serial.begin(9600);
  EspSerial.begin(9600);
  Serial.println("Sistema Iniciado. Esperando comandos...");

  // Configura velocidades iniciales de los motores
  setMotorSpeed(200);
}

void loop() {
  if (EspSerial.available() > 0) {
    char comando = EspSerial.read(); // Leer comando como carácter
    Serial.print("Datos recibidos: ");
    Serial.println(comando);

    handleCommand(comando);
  }
}

void handleCommand(char comando) {

  if(comando == '.'){delay(4000);}

  
    switch (comando) {
      case 'F': forward(); break;
      case 'B': back(); break;
      case 'L': left(); break;
      case 'R': right(); break;
      case '0': Stop(); break;   

      case 'G': green(); break;
      case 'C': red(); break;
      case '3': semaforo(); break;    
      case 'S': s(); break;  
      default:
        Serial.println("Comando no reconocido");
        break;
      
 }
}

void setMotorSpeed(int speed) {
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}

void forward() {
  Serial.println("Avanzar");
  setMotorSpeed(255);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void back() {
  Serial.println("Retroceder");
  setMotorSpeed(255);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void left() {
  Serial.println("Izquierda");
  setMotorSpeed(255);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void right() {
  Serial.println("Derecha");
  setMotorSpeed(255);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void Stop() {
  Serial.println("Detener");
  setMotorSpeed(0);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
void green() {
  Serial.println("Avanzar, estamos en verde");
  setMotorSpeed(255);
}
void red() {
  Serial.println("Detener, estamos en rojo");
  setMotorSpeed(0);

}
void semaforo() {
  Serial.println("Se detecto un semaforo");  
}

void s() {
  Serial.println("Se detecto Stop");  
  setMotorSpeed(0);
  delay(3000);
}

```

</details>

<details close>
<summary>ESP8266 nodemcu</summary>

```
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

// Configuración de la red Wi-Fi
const char* ssid = "Configurar propia cuenta";
const char* password = "lConfigurar propia cuenta";

WiFiServer server(80);
SoftwareSerial BTSerial(2, 0);  // GPIO 2: RX, GPIO 0: TX

void setup() {
  Serial.begin(9600); // Monitor Serial
  BTSerial.begin(9600); // Bluetooth Serial

  // Conexión a Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado. Dirección IP: ");
  Serial.println(WiFi.localIP());

  server.begin(); // Inicia el servidor Wi-Fi
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
      client.setTimeout(2000);  // Configura un timeout de 2 segundos
      while (client.connected()) {
          if (client.available()) {
            char class_name = client.read();
            Serial.println(class_name);
            client.flush();
          }
          yield();  // Permite al sistema manejar otras tareas
      }
      client.stop();  // Cierra la conexión del cliente
  }


  if (BTSerial.available()) {
    char command = BTSerial.read(); // Leer comando Bluetooth
    Serial.println("");
    Serial.write(command); // Imprimir comando en Serial Monitor
  }
}

```

</details>

