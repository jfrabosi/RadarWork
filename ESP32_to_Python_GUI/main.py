import sys
import serial
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtWidgets import QProgressBar


class SerialReader(QObject):
    data_received = pyqtSignal(str)

    def __init__(self, port, baudrate):
        super().__init__()
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.running = True

    def read_serial_data(self):
        while self.running:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    self.data_received.emit(line)
            except Exception as e:
                print(f"Error reading serial data: {e}")

    def stop(self):
        self.running = False
        self.serial_port.close()


class ESP32DataGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Data GUI")
        self.setGeometry(100, 100, 600, 400)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        # Motion indicator
        self.motion_label = QLabel("No Motion")
        self.motion_label.setAlignment(Qt.AlignCenter)
        self.motion_label.setStyleSheet("background-color: red; color: white; font-size: 20px; padding: 10px;")
        layout.addWidget(self.motion_label)

        # Gauges
        gauges_layout = QHBoxLayout()
        self.intra_gauge = self.create_gauge("Intra Presence Score")
        self.inter_gauge = self.create_gauge("Inter Presence Score")
        self.distance_gauge = self.create_gauge("Distance (mm)")
        gauges_layout.addWidget(self.intra_gauge)
        gauges_layout.addWidget(self.inter_gauge)
        gauges_layout.addWidget(self.distance_gauge)
        layout.addLayout(gauges_layout)

        # Serial communication
        self.serial_reader = SerialReader('COM5', 115200)
        self.serial_reader.data_received.connect(self.process_data)
        self.serial_thread = threading.Thread(target=self.serial_reader.read_serial_data, daemon=True)
        self.serial_thread.start()

    def create_gauge(self, title):
        gauge_widget = QWidget()
        gauge_layout = QVBoxLayout()
        gauge_widget.setLayout(gauge_layout)

        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignCenter)
        gauge_layout.addWidget(title_label)

        progress_bar = QProgressBar()
        progress_bar.setRange(0, 100)
        progress_bar.setTextVisible(False)
        gauge_layout.addWidget(progress_bar)

        value_label = QLabel("0")
        value_label.setAlignment(Qt.AlignCenter)
        gauge_layout.addWidget(value_label)

        return gauge_widget

    def process_data(self, data):
        try:
            if "Intra presence score:" in data:
                parts = data.split(',')
                intra_score = int(parts[0].split(':')[1].strip())
                inter_score = int(parts[1].split(':')[1].strip())
                distance = int(parts[2].split(':')[1].strip())

                self.update_gauge(self.intra_gauge, intra_score, 2000)  # Assuming max intra score is 2000
                self.update_gauge(self.inter_gauge, inter_score, 15000)  # Assuming max inter score is 15000
                self.update_gauge(self.distance_gauge, distance, 1000)  # Assuming max distance is 1000mm

            elif "Motion" in data:
                self.motion_label.setText("Motion")
                self.motion_label.setStyleSheet(
                    "background-color: green; color: white; font-size: 20px; padding: 10px;")
            elif "No motion" in data:
                self.motion_label.setText("No Motion")
                self.motion_label.setStyleSheet("background-color: red; color: white; font-size: 20px; padding: 10px;")
        except Exception as e:
            print(f"Error processing data: {e}")

    def update_gauge(self, gauge, value, max_value):
        progress_bar = gauge.findChild(QProgressBar)
        value_label = gauge.findChildren(QLabel)[-1]

        progress_value = min(int((value / max_value) * 100), 100)
        progress_bar.setValue(progress_value)
        value_label.setText(str(value))

    def closeEvent(self, event):
        self.serial_reader.stop()
        super().closeEvent(event)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = ESP32DataGUI()
    gui.show()
    sys.exit(app.exec_())