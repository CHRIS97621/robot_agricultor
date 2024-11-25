import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
from PySide2.QtWidgets import QApplication, QMainWindow, QSlider, QVBoxLayout, QWidget, QLabel
from PySide2.QtCore import Qt, QTimer

class SerialReader(Node):
    def __init__(self, cmd_freq=10):
        super().__init__('serial_reader')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)  # Asegúrate de que el puerto sea el correcto
        
        # Configuración de frecuencia de envío de comandos
        self.cmd_freq = cmd_freq  # Frecuencia en Hz
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            data = dict(item.split(":") for item in line.split(","))
            msg = Twist()
            msg.linear.x = float(data["vel_left"])
            msg.angular.z = float(data["vel_right"])
            self.publisher_.publish(msg)

    def update_velocity(self, linear, angular):
        # Actualiza las velocidades deseadas
        self.linear_velocity = linear
        self.angular_velocity = angular

    def send_velocity_command(self):
        # Envía los valores actuales de velocidad al ESP32
        target_msg = f"linear:{self.linear_velocity},angular:{self.angular_velocity}\n"
        self.ser.write(target_msg.encode('utf-8'))


class MainWindow(QMainWindow):
    def __init__(self, serial_reader):
        super().__init__()
        self.serial_reader = serial_reader
        self.setWindowTitle("Control de Velocidad")

        # Slider de velocidad lineal
        self.linear_slider = QSlider(Qt.Horizontal)
        self.linear_slider.setMinimum(-100)
        self.linear_slider.setMaximum(100)
        self.linear_slider.setValue(0)
        self.linear_slider.valueChanged.connect(self.update_velocity)

        # Slider de velocidad angular
        self.angular_slider = QSlider(Qt.Horizontal)
        self.angular_slider.setMinimum(-100)
        self.angular_slider.setMaximum(100)
        self.angular_slider.setValue(0)
        self.angular_slider.valueChanged.connect(self.update_velocity)

        # Etiquetas
        self.linear_label = QLabel("Velocidad Lineal")
        self.angular_label = QLabel("Velocidad Angular")

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.linear_label)
        layout.addWidget(self.linear_slider)
        layout.addWidget(self.angular_label)
        layout.addWidget(self.angular_slider)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Configuración del temporizador para enviar comandos a intervalos regulares
        self.timer = QTimer()
        self.timer.setInterval(1000 / self.serial_reader.cmd_freq)  # Intervalo en milisegundos
        self.timer.timeout.connect(self.serial_reader.send_velocity_command)
        self.timer.start()

    def update_velocity(self):
        linear = self.linear_slider.value() / 100.0
        angular = self.angular_slider.value() / 100.0
        self.serial_reader.update_velocity(linear, angular)


def main(args=None):
    rclpy.init(args=args)
    
    # Crear el nodo de SerialReader con cmd_freq configurado
    cmd_freq = 10  # Frecuencia deseada de envío de comandos en Hz
    serial_reader = SerialReader(cmd_freq=cmd_freq)

    # Iniciar GUI
    app = QApplication([])
    window = MainWindow(serial_reader)
    window.show()

    try:
        while rclpy.ok():
            serial_reader.read_serial()
            rclpy.spin_once(serial_reader)
            app.processEvents()
    except KeyboardInterrupt:
        pass

    serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
