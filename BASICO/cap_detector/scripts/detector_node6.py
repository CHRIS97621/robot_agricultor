#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import time
import os
import sys

class CacaoDetectorNode(Node):
    def __init__(self):
        super().__init__('cacao_detector_node_v8')

        # Configuración de los parametros de la cámara
        self.declare_parameter('camera_device', 'dev/video2') # valor por defecto video2
        camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        
        # Inicializar la cámara con el dispositivo especificado
        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error('No se puede acceder a la camara en dev/video2')
            return
       
        # Añadir el entorno pipx al sys.path
        pipx_venv_path = "/home/christopher/.local/share/pipx/venvs/ultralytics/lib/python3.12/site-packages"
        sys.path.append(pipx_venv_path)

        # Cargamos el modelo YOLOv8 en este caso
        self.model = self.load_yolov8_model()

        # Publicador de detecciones
        self.publisher_ =self.create_publisher(String, '/cacao_detection', 10)

        # Directorio donde se guardarán la imágenes capturadas
        self.save_dir = '/home/christopher/cacao_capturas2'
        os.makedirs(self.save_dir, exist_ok=True)

        #Control de tiempo para guardar imagen
        self.last_capture_time = time.time()

        # Timer 
        self.timer = self.create_timer(0.1, self.process_frame)  # Procesar la imagen cada 0.1 segundos 
    
    def load_yolov8_mode (self):
        try:
            # Importar torch y ultralytics del entorno
            import torch                   # type: ignore
            from ultralytics import YOLO   # type: ignore

            # cargar el modelo YOLOv8
            model = YOLO('/home/christopher/yolo_results/train3/weights/best.pt')
            return model
        except Exception as e:
            self.get_logger().error(f"Error al cargar el modelo YOLOv8: {e}")
            return None
    
    def process_frame(self):
        # Leer el frame de la cámara
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Nose puedo capturar la imagen')
            return
        # Ajustar el umbral de confianza y el umbral IoU
        if self.model is not None:
            results = self.model(frame, conf=0.7, iou=0.5)  # Ajustar conf y IoU

            # Filtrar por clases específicas ('cacao sano' y 'cacao enfermo')
            detections = results[0].boxes.xyxy.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy()  # Las clases detectadas
            class_names = self.model.model.names

            filtered_results = []
            cacao_sano_count = 0
            cacao_enfermo_count = 0

            for i, box in enumerate(detections):
                cls_id = int(classes[i])  # ID de la clase detectada
                if cls_id in [0, 1]:  # Solo cacao sano (0) y cacao enfermo (1)
                    filtered_results.append((box, cls_id))
                    if cls_id == 0:
                        cacao_sano_count += 1
                    elif cls_id == 1:
                        cacao_enfermo_count += 1

            # Calcular porcentajes
            total_detections = cacao_sano_count + cacao_enfermo_count
            if total_detections > 0:
                cacao_sano_percentage = (cacao_sano_count / total_detections) * 100
                cacao_enfermo_percentage = (cacao_enfermo_count / total_detections) * 100
            else:
                cacao_sano_percentage = 0
                cacao_enfermo_percentage = 0
            
            # Invertir los porcentajes para que coincidan con las etiquetas

            cacao_sano_percentage, cacao_enfermo_percentage = cacao_enfermo_percentage, cacao_sano_percentage

            self.get_logger().info(f'Porcentaje de Cacao sano: {cacao_sano_percentage:.2f}%')
            self.get_logger().info(f'Porcentaje de Cacao enfermo: {cacao_enfermo_percentage:.2f}%')

            # Postprocesamiento: eliminar detecciones con cuadros demasiado pequeños
            min_box_area = 1000  # Área mínima para considerar la detección
            final_results = []
            for box, cls_id in filtered_results:
                x_min, y_min, x_max, y_max = box[:4]
                box_area = (x_max - x_min) * (y_max - y_min)
                if box_area >= min_box_area:
                    final_results.append((box, cls_id))

            # Dibujar rectángulos alrededor de las detecciones
            for box, cls_id in final_results:
                x_min, y_min, x_max, y_max = box[:4]
                label = class_names[cls_id]

                # Invertimos las etiquetas manualmente
                if label == 'cacao sano':
                    label = 'cacao enfermo'
                elif label == 'cacao enfermo':
                    label = 'cacao sano'

                # Colores para las etiquetas (Verde para sano, Rojo para enfermo)
                color = (0, 255, 0) if label == 'cacao sano' else (0, 0, 255)

                cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, 2)
                cv2.putText(frame, label, (int(x_min), int(y_min) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            # Publicar detección
            detection_msg = String()
            detection_msg.data = 'Cacao detectado'
            self.publisher_.publish(detection_msg)
            self.get_logger().info(f'Detección publicada: {detection_msg.data}')

            # Mostrar el frame con las detecciones y los porcentajes en tiempo real
            cv2.putText(frame, f'Cacao sano: {cacao_sano_percentage:.2f}%', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.putText(frame, f'Cacao enfermo: {cacao_enfermo_percentage:.2f}%', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            # Guardar una imagen solo si se detecta un 100% de cacao sano o cacao enfermo
            
            if cacao_sano_percentage == 100.0 or cacao_enfermo_percentage == 100.0:
                # Añadir el texto de los porcentajes en la imagen antes de guardarla
                cv2.putText(frame, f'Cacao sano: {cacao_sano_percentage:.2f}%', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                cv2.putText(frame, f'Cacao enfermo: {cacao_enfermo_percentage:.2f}%', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)


                # Guardar la imagen con los porcentajes
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                image_path = os.path.join(self.save_dir, f"captura_{timestamp}.jpg")
                cv2.imwrite(image_path, frame)
                self.get_logger().info(f'Imagen guardada: {image_path}')

            # Mostrar el frame con las detecciones y los porcentajes en la pantalla
            cv2.imshow('Cacao Detection', frame)
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