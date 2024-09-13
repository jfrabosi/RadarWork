import sys
import serial
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel
from PyQt5.QtGui import QPixmap, QColor
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QProgressBar

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
        self.serial_port = serial.Serial('COM5', 115200, timeout=1)
        self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.serial_thread.start()

        # Update timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # Update every 100ms

    def create_gauge(self, title):
        gauge_widget = QWidget()
        gauge_layout = QVBoxLayout()
        gauge_widget.setLayout(gauge_layout)

        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        gauge_layout.addWidget(label)

        progress_bar = QProgressBar()
        progress_bar.setRange(0, 100)
        progress_bar.setTextVisible(False)
        gauge_layout.addWidget(progress_bar)

        value_label = QLabel("0")
        value_label.setAlignment(Qt.AlignCenter)
        gauge_layout.addWidget(value_label)

        return gauge_widget

    def read_serial_data(self):
        while True:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    self.process_data(line)
            except Exception as e:
                print(f"Error reading serial data: {e}")

    def process_data(self, data):
        if "Intra presence score:" in data:
            parts = data.split(',')
            intra_score = int(parts[0].split(':')[1].strip())
            inter_score = int(parts[1].split(':')[1].strip())
            distance = int(parts[2].split(':')[1].strip())

            self.intra_gauge.findChild(QProgressBar).setValue(min(intra_score // 100, 100))
            self.intra_gauge.findChild(QLabel, "", Qt.FindChildOption.FindChildrenRecursively).setText(str(intra_score))

            self.inter_gauge.findChild(QProgressBar).setValue(min(inter_score // 100, 100))
            self.inter_gauge.findChild(QLabel, "", Qt.FindChildOption.FindChildrenRecursively).setText(str(inter_score))

            self.distance_gauge.findChild(QProgressBar).setValue(min(distance // 10, 100))
            self.distance_gauge.findChild(QLabel, "", Qt.FindChildOption.FindChildrenRecursively).setText(str(distance))
        elif "Motion" in data:
            self.motion_label.setText("Motion")
            self.motion_label.setStyleSheet("background-color: green; color: white; font-size: 20px; padding: 10px;")
        elif "No motion" in data:
            self.motion_label.setText("No Motion")
            self.motion_label.setStyleSheet("background-color: red; color: white; font-size: 20px; padding: 10px;")

    def update_gui(self):
        # This method is called by the timer to update the GUI
        # We don't need to do anything here as the GUI is updated in process_data
        pass

    def closeEvent(self, event):
        self.serial_port.close()
        super().closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = ESP32DataGUI()
    gui.show()
    sys.exit(app.exec_())