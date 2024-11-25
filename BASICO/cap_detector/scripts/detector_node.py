#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import torch  # type: ignore
import time
import os

class CacaoDetectorNode(Node):
    def __init__(self):
        super().__init__('cacao_detector_node2')

        # Inicializar la cámara en /dev/video0
        self.cap = cv2.VideoCapture('/dev/video2') 
        if not self.cap.isOpened():
            self.get_logger().error("No se puede acceder a la cámara en /dev/video2") #/dev/video0
            return

        # Cargar el modelo YOLOv5 entrenado
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/christopher/yolov5/runs/train/exp2/weights/best.pt', force_reload=True)
        self.model.eval()

        # Publicador para detecciones
        self.publisher_ = self.create_publisher(String, '/cacao_detection', 10)

        # Directorio donde se guardarán las imágenes capturadas
        self.save_dir = '/home/christopher/cacao_capturas2'
        os.makedirs(self.save_dir, exist_ok=True)

        # Control de tiempo para guardar una imagen cada segundo
        self.last_capture_time = time.time()

        # Timer para procesar la cámara en intervalos regulares
        self.timer = self.create_timer(0.1, self.process_frame)  # Procesar la imagen cada 0.1 segundos

    def process_frame(self):
        # Leer el frame de la cámara
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('No se pudo capturar la imagen de la cámara')
            return

        # Realizar detección con YOLOv5
        results = self.model(frame)

        # Extraer los datos de detección
        detected_classes = results.pandas().xyxy[0]['name'].tolist()
        boxes = results.pandas().xyxy[0][['xmin', 'ymin', 'xmax', 'ymax']].values

        # Dibujar rectángulos alrededor de las detecciones
        for i, box in enumerate(boxes):
            (x_min, y_min, x_max, y_max) = box
            label = detected_classes[i]
            if label == 'cacao enfermo':
                color = (0, 0, 255)  # Rojo para cacao enfermo
            elif label == 'cacao sano':
                color = (0, 255, 0)  # Verde para cacao sano
            else:
                color = (255, 255, 255)  # Blanco si no es cacao

            # Dibujar el rectángulo en el frame
            cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, 2)
            # Escribir la clase detectada encima del rectángulo
            cv2.putText(frame, label, (int(x_min), int(y_min) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # Publicar detección
        detection_msg = String()
        if 'cacao enfermo' in detected_classes:
            detection_msg.data = 'Cacao enfermo detectado'
        elif 'cacao sano' in detected_classes:
            detection_msg.data = 'Cacao sano detectado'
        else:
            detection_msg.data = 'No se detectó cacao'

        self.publisher_.publish(detection_msg)
        self.get_logger().info(f'Detección publicada: {detection_msg.data}')

        # Guardar una imagen cada segundo
        current_time = time.time()
        if current_time - self.last_capture_time >= 2.0:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            image_path = os.path.join(self.save_dir, f"captura_{timestamp}.jpg")
            cv2.imwrite(image_path, frame)
            self.get_logger().info(f'Imagen guardada: {image_path}')
            self.last_capture_time = current_time

        # Mostrar el frame con las detecciones en una ventana
        cv2.imshow('Cacao Detection', frame)
        
        # Esperar por 1 milisegundo y permitir salir del bucle con la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

    def destroy_node(self):
        # Liberar el recurso de la cámara al finalizar el nodo
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CacaoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
