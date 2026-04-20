import sys
import os
import time
import math
import random
import serial
import serial.tools.list_ports
from collections import deque
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QPushButton, 
                             QTextEdit, QMessageBox, QGroupBox, QFormLayout, 
                             QSplitter, QTabWidget)
from PyQt6.QtCore import pyqtSignal, QObject, QTimer, Qt, QThread, QUrl
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEngineSettings
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
import pyqtgraph as pg

# OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *

# WebEngine için Hardware Acceleration / Chromium flag'leri (Mac/M1 çakışmalarını önlemek için)
os.environ["QSG_RHI_BACKEND"] = "opengl"
os.environ["QT_WEBENGINE_CHROMIUM_FLAGS"] = "--disable-gpu-compositing"

# ----------------- PYQTGRAPH -----------------
pg.setConfigOption('background', '#1E1E1E')
pg.setConfigOption('foreground', '#D4D4D4')
pg.setConfigOptions(antialias=True)

# ----------------- HARİTA HTML -----------------
MAP_HTML = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style> 
        body { margin: 0; padding: 0; background-color: #000; } 
        #map { height: 100vh; width: 100vw; background: #000; } 
    </style>
</head>
<body>
    <div id="map"></div>
    <script>
        var map = L.map('map').setView([38.835, 33.393], 5);
        
        // Profesyonel Siyah Harita (CartoDB Dark Matter - Engel yemez)
        L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png', {
            maxZoom: 19, attribution: '© OpenStreetMap © CartoDB'
        }).addTo(map);

        var rocketIcon = L.icon({
            iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-blue.png',
            shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
            iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34], shadowSize: [41, 41]
        });

        var payloadIcon = L.icon({
            iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-orange.png',
            shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
            iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34], shadowSize: [41, 41]
        });

        var rocketMarker = null; var payloadMarker = null;
        var rFirst = true; var pFirst = true;

        function updateRocket(lat, lon) {
            if (rocketMarker === null) {
                rocketMarker = L.marker([lat, lon], {icon: rocketIcon}).addTo(map).bindPopup('<b>🚀 Roket</b>').openPopup();
            } else { rocketMarker.setLatLng([lat, lon]); }
            if (rFirst) { map.setView([lat, lon], 16); rFirst = false; }
        }

        function updatePayload(lat, lon) {
            if (payloadMarker === null) {
                payloadMarker = L.marker([lat, lon], {icon: payloadIcon}).addTo(map).bindPopup('<b>🛰️ Görev Yükü</b>');
            } else { payloadMarker.setLatLng([lat, lon]); }
            if (pFirst) { map.setView([lat, lon], 16); pFirst = false; }
        }
    </script>
</body>
</html>
"""

# HTML Datasini Local Dosyaya Yaz (WebEngine Guvenlik Duvarini Asmak Icin)
map_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "map_internal.html"))
try:
    with open(map_path, "w", encoding="utf-8") as f:
        f.write(MAP_HTML)
except Exception:
    pass

# ----------------- SERIAL WORKER (QTHREAD) -----------------
class SerialWorker(QThread):
    raw_data_signal = pyqtSignal(str, str) 
    parsed_data_signal = pyqtSignal(str, list, float) 
    error_signal = pyqtSignal(str, str)
    disconnected_signal = pyqtSignal(str)

    def __init__(self, port, baudrate, identifier, start_time):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.identifier = identifier
        self.start_time = start_time
        self.is_running = True
        self.serial_conn = None

    def run(self):
        if self.port == "Simülatör":
            self.run_simulator()
            return

        try:
            self.serial_conn = serial.Serial(self.port, int(self.baudrate), timeout=1)
        except Exception as e:
            self.error_signal.emit(self.identifier, f"Bağlantı açılamadı: {e}")
            self.disconnected_signal.emit(self.identifier)
            return

        while self.is_running and self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    raw_data = self.serial_conn.readline().decode('utf-8', errors='replace').strip()
                    if raw_data:
                        self.raw_data_signal.emit(self.identifier, raw_data)
                        parts = raw_data.split(',')
                        if len(parts) > 1:
                            t = time.time() - self.start_time
                            self.parsed_data_signal.emit(self.identifier, parts, t)
            except Exception as e:
                self.error_signal.emit(self.identifier, f"Okuma hatası: {e}")
                self.disconnected_signal.emit(self.identifier)
                break

    def run_simulator(self):
        alt = 0.0
        vel = 0.0
        r, p, y = 0.0, 0.0, 0.0
        lat, lon = 38.835, 33.393

        while self.is_running:
            t = time.time() - self.start_time
            r = (r + random.uniform(-5, 5)) % 360
            p = (p + random.uniform(-2, 2)) % 360
            y = (y + random.uniform(-1, 1)) % 360
            alt += 15.0 + random.uniform(-2, 2)
            vel = 120.0 + random.uniform(-5, 5)
            acc = 1.2 + random.uniform(-0.1, 0.1)
            lat += 0.0001
            lon += 0.0001
            
            if self.identifier == "rocket":
                raw_data = f"ROKET,{alt:.1f},{vel:.1f},{acc:.2f},Ucussim,{lat:.4f}|{lon:.4f},{r:.1f},{p:.1f},{y:.1f}"
            else:
                raw_data = f"YUK,{alt:.1f},24.5,101325,Ayrildisim,45.2,{lat-0.002:.4f}|{lon-0.002:.4f}"
                
            self.raw_data_signal.emit(self.identifier, raw_data)
            parts = raw_data.split(',')
            self.parsed_data_signal.emit(self.identifier, parts, t)
            self.msleep(50)

    def stop(self):
        self.is_running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.wait()

# ----------------- 3D OPENGL WIDGET -----------------
class Rocket3DWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.cam_rot_x = 30.0
        self.cam_rot_y = -45.0
        self.last_mouse_pos = None

    def set_angles(self, r, p, y):
        self.roll = r
        self.pitch = p
        self.yaw = y
        self.update()

    def mousePressEvent(self, event):
        self.last_mouse_pos = event.position()

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos is not None:
            dx = event.position().x() - self.last_mouse_pos.x()
            dy = event.position().y() - self.last_mouse_pos.y()
            if event.buttons() & Qt.MouseButton.LeftButton:
                self.cam_rot_x += dy * 0.5
                self.cam_rot_y += dx * 0.5
                self.update()
        self.last_mouse_pos = event.position()

    def initializeGL(self):
        glClearColor(0.12, 0.12, 0.14, 1.0)
        glEnable(GL_DEPTH_TEST)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        if h == 0: h = 1
        gluPerspective(45.0, w / float(h), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, -1.0, -8.0) 
        
        glRotatef(self.cam_rot_x, 1.0, 0.0, 0.0)
        glRotatef(self.cam_rot_y, 0.0, 1.0, 0.0)

        glLineWidth(1.0)
        glBegin(GL_LINES)
        glColor3f(0.25, 0.25, 0.25)
        for i in range(-15, 16):
            glVertex3f(-15, -2.5, i); glVertex3f( 15, -2.5, i)
            glVertex3f(i, -2.5, -15); glVertex3f(i, -2.5,  15)
        glEnd()

        glLoadIdentity()
        glTranslatef(-3.0, -2.5, -8.0) 
        
        glRotatef(self.cam_rot_x, 1.0, 0.0, 0.0)
        glRotatef(self.cam_rot_y, 0.0, 1.0, 0.0)
        
        glRotatef(self.yaw, 0.0, 1.0, 0.0)
        glRotatef(self.pitch, 1.0, 0.0, 0.0)
        glRotatef(self.roll, 0.0, 0.0, 1.0)

        glLineWidth(3.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(1.0, 0.0, 0.0)
        glColor3f(0.0, 1.0, 0.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 1.0, 0.0)
        glColor3f(0.0, 0.0, 1.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 1.0)
        glEnd()

        glLoadIdentity()
        glTranslatef(0.0, -1.0, -8.0) 

        glRotatef(self.cam_rot_x, 1.0, 0.0, 0.0)
        glRotatef(self.cam_rot_y, 0.0, 1.0, 0.0)

        glRotatef(self.yaw, 0.0, 1.0, 0.0)    
        glRotatef(self.pitch, 1.0, 0.0, 0.0)  
        glRotatef(self.roll, 0.0, 0.0, 1.0)   

        glTranslatef(0.0, 0.0, -1.0)
        
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)

        quadric = gluNewQuadric()
        gluQuadricDrawStyle(quadric, GLU_FILL)
        
        glColor3f(0.9, 0.9, 0.9)
        gluCylinder(quadric, 0.3, 0.3, 2.0, 32, 2)
        
        glPushMatrix()
        glColor3f(0.3, 0.3, 0.3)
        gluDisk(quadric, 0.0, 0.3, 32, 1)
        glPopMatrix()

        glPushMatrix()
        glTranslatef(0.0, 0.0, 2.0)
        glColor3f(0.8, 0.1, 0.1)
        gluCylinder(quadric, 0.3, 0.0, 0.8, 32, 2)
        glPopMatrix()

        glPushMatrix()
        glColor3f(0.2, 0.2, 0.2)
        gluCylinder(quadric, 0.3, 0.2, -0.4, 32, 2)
        glPopMatrix()

        glDisable(GL_LIGHTING) 
        glColor3f(0.8, 0.1, 0.1) 
        glBegin(GL_TRIANGLES)
        for i in range(4):
            angle = math.radians(i * 90.0)
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            glVertex3f(0.3 * cos_a, 0.3 * sin_a, 0.6)
            glVertex3f(0.3 * cos_a, 0.3 * sin_a, 0.0)
            glVertex3f(0.8 * cos_a, 0.8 * sin_a, -0.2)
        glEnd()
        glEnable(GL_LIGHTING)
        gluDeleteQuadric(quadric)
        glDisable(GL_LIGHTING)


# ----------------- ANA SİSTEM -----------------
class SerialViewerApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Yer İstasyonu (Pro-Aviyonik, QThread, Harita, 3D)")
        self.resize(1920, 1080)
        
        self.start_time = time.time()
        
        self.rocket_worker = None
        self.payload_worker = None

        self.MAX_POINTS = 300
        self.r_t = deque(maxlen=self.MAX_POINTS)
        self.r_alt = deque(maxlen=self.MAX_POINTS)
        self.r_vel = deque(maxlen=self.MAX_POINTS)
        self.r_acc = deque(maxlen=self.MAX_POINTS)
        
        self.p_t = deque(maxlen=self.MAX_POINTS)
        self.p_alt = deque(maxlen=self.MAX_POINTS)
        self.p_temp = deque(maxlen=self.MAX_POINTS)
        self.p_press = deque(maxlen=self.MAX_POINTS)

        self._setup_ui()
        self.refresh_ports()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)

    def _setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)
        
        # 1. SOL PANEL
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(5, 5, 5, 5)
        
        self.refresh_btn = QPushButton("🔄 Tüm COM Portlarını Yenile")
        self.refresh_btn.setStyleSheet("font-weight: bold; padding: 10px; background-color: #333333; color: white;")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        left_layout.addWidget(self.refresh_btn)
        
        # ROKET GRUBU
        rocket_group = QGroupBox("🚀 Roket")
        rocket_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 15px; }")
        rv_layout = QVBoxLayout()
        rc_layout = QHBoxLayout()
        self.rocket_cb = QComboBox()
        self.rocket_baud = QComboBox()
        self.rocket_baud.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.rocket_baud.setCurrentText("9600")
        self.rocket_connect_btn = QPushButton("Bağlan")
        self.rocket_connect_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.rocket_disconnect_btn = QPushButton("Kes")
        self.rocket_disconnect_btn.setStyleSheet("background-color: #F44336; color: white; font-weight: bold;")
        self.rocket_disconnect_btn.setEnabled(False)
        self.rocket_connect_btn.clicked.connect(self.connect_rocket)
        self.rocket_disconnect_btn.clicked.connect(lambda: self.disconnect_system("rocket"))
        
        rc_layout.addWidget(QLabel("Port:"))
        rc_layout.addWidget(self.rocket_cb)
        rc_layout.addWidget(QLabel("Baud:"))
        rc_layout.addWidget(self.rocket_baud)
        
        rc2_layout = QHBoxLayout()
        rc2_layout.addWidget(self.rocket_connect_btn)
        rc2_layout.addWidget(self.rocket_disconnect_btn)

        r_form_layout = QFormLayout()
        self.rocket_labels = {"İrtifa (m)": QLabel("-"), "Hız (m/s)": QLabel("-"), "İvme (g)": QLabel("-"), "Durum": QLabel("-"), "GPS": QLabel("-"), "Avionik (R,P,Y)": QLabel("-")}
        for key, lbl in self.rocket_labels.items():
            lbl.setStyleSheet("color: #2196F3; font-weight: bold; font-size: 15px;")
            r_form_layout.addRow(key + ":", lbl)
            
        rv_layout.addLayout(rc_layout)
        rv_layout.addLayout(rc2_layout)
        rv_layout.addLayout(r_form_layout)
        rocket_group.setLayout(rv_layout)
        left_layout.addWidget(rocket_group)
        
        # GÖREV YÜKÜ GRUBU
        payload_group = QGroupBox("🛰️ Görev Yükü")
        payload_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 15px; }")
        pv_layout = QVBoxLayout()
        pc_layout = QHBoxLayout()
        self.payload_cb = QComboBox()
        self.payload_baud = QComboBox()
        self.payload_baud.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.payload_baud.setCurrentText("9600")
        self.payload_connect_btn = QPushButton("Bağlan")
        self.payload_connect_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.payload_disconnect_btn = QPushButton("Kes")
        self.payload_disconnect_btn.setStyleSheet("background-color: #F44336; color: white; font-weight: bold;")
        self.payload_disconnect_btn.setEnabled(False)
        self.payload_connect_btn.clicked.connect(self.connect_payload)
        self.payload_disconnect_btn.clicked.connect(lambda: self.disconnect_system("payload"))
        
        pc_layout.addWidget(QLabel("Port:"))
        pc_layout.addWidget(self.payload_cb)
        pc_layout.addWidget(QLabel("Baud:"))
        pc_layout.addWidget(self.payload_baud)
        
        pc2_layout = QHBoxLayout()
        pc2_layout.addWidget(self.payload_connect_btn)
        pc2_layout.addWidget(self.payload_disconnect_btn)

        p_form_layout = QFormLayout()
        self.payload_labels = {"İrtifa (m)": QLabel("-"), "Sıcaklık (°C)": QLabel("-"), "Basınç (Pa)": QLabel("-"), "Durum": QLabel("-"), "Nem (%)": QLabel("-"), "GPS": QLabel("-")}
        for key, lbl in self.payload_labels.items():
            lbl.setStyleSheet("color: #FF9800; font-weight: bold; font-size: 15px;")
            p_form_layout.addRow(key + ":", lbl)
            
        pv_layout.addLayout(pc_layout)
        pv_layout.addLayout(pc2_layout)
        pv_layout.addLayout(p_form_layout)
        payload_group.setLayout(pv_layout)
        left_layout.addWidget(payload_group)
        
        # TERMİNAL
        left_layout.addWidget(QLabel("Terminal Log:"))
        self.text_area = QTextEdit()
        self.text_area.setReadOnly(True)
        self.text_area.setStyleSheet("background-color: #121212; color: #D4D4D4; font-family: monospace; font-size: 12px;")
        left_layout.addWidget(self.text_area)
        
        self.clear_btn = QPushButton("Terminali Temizle")
        self.clear_btn.clicked.connect(self.text_area.clear)
        left_layout.addWidget(self.clear_btn)

        # =========================================================
        # 2. SAĞ PANEL (SEKMELER)
        # =========================================================
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(5, 5, 5, 5)

        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("QTabBar::tab { height: 35px; width: 250px; font-weight: bold; font-size: 14px; }")
        right_layout.addWidget(self.tabs)

        # ---------------- SEKME 1: GRAFİKLER ----------------
        self.tab_graphs = QWidget()
        graph_layout = QVBoxLayout(self.tab_graphs)

        self.plot_alt = pg.PlotWidget(title="<span style='font-size: 14pt; color: #FFFFFF;'>İrtifa Karşılaştırması</span>")
        self.plot_alt.showGrid(x=True, y=True, alpha=0.3)
        self.plot_alt.addLegend(offset=(10, 10))
        self.plot_alt.setLabel('left', 'İrtifa', units='m')
        self.curve_r_alt = self.plot_alt.plot(pen=pg.mkPen('#2196F3', width=3), name='Roket İrtifa')
        self.curve_p_alt = self.plot_alt.plot(pen=pg.mkPen('#FF9800', width=3), name='GY İrtifa')
        graph_layout.addWidget(self.plot_alt)

        self.plot_r_kin = pg.PlotWidget(title="<span style='font-size: 14pt; color: #FFFFFF;'>Roket Hız ve İvme</span>")
        self.plot_r_kin.showGrid(x=True, y=True, alpha=0.3)
        self.plot_r_kin.addLegend(offset=(10, 10))
        self.curve_r_vel = self.plot_r_kin.plot(pen=pg.mkPen('#F44336', width=3), name='Hız (m/s)')
        
        self.plot_r_acc_view = pg.ViewBox()
        self.plot_r_kin.showAxis('right')
        self.plot_r_kin.scene().addItem(self.plot_r_acc_view)
        self.plot_r_kin.getAxis('right').linkToView(self.plot_r_acc_view)
        self.plot_r_acc_view.setXLink(self.plot_r_kin)
        self.curve_r_acc = pg.PlotCurveItem(pen=pg.mkPen('#9C27B0', width=3), name='İvme (g)')
        self.plot_r_acc_view.addItem(self.curve_r_acc)
        def update_acc_view():
            self.plot_r_acc_view.setGeometry(self.plot_r_kin.getViewBox().sceneBoundingRect())
            self.plot_r_acc_view.linkedViewChanged(self.plot_r_kin.getViewBox(), self.plot_r_acc_view.XAxis)
        self.plot_r_kin.getViewBox().sigResized.connect(update_acc_view)
        graph_layout.addWidget(self.plot_r_kin)

        self.plot_p_sens = pg.PlotWidget(title="<span style='font-size: 14pt; color: #FFFFFF;'>Görev Yükü Çevresel Veriler</span>")
        self.plot_p_sens.showGrid(x=True, y=True, alpha=0.3)
        self.plot_p_sens.addLegend(offset=(10, 10))
        self.curve_p_temp = self.plot_p_sens.plot(pen=pg.mkPen('#E91E63', width=3), name='Sıcaklık (°C)')
        self.plot_p_press_view = pg.ViewBox()
        self.plot_p_sens.showAxis('right')
        self.plot_p_sens.scene().addItem(self.plot_p_press_view)
        self.plot_p_sens.getAxis('right').linkToView(self.plot_p_press_view)
        self.plot_p_press_view.setXLink(self.plot_p_sens)
        self.curve_p_press = pg.PlotCurveItem(pen=pg.mkPen('#00BCD4', width=3), name='Basınç (Pa)')
        self.plot_p_press_view.addItem(self.curve_p_press)
        def update_press_view():
            self.plot_p_press_view.setGeometry(self.plot_p_sens.getViewBox().sceneBoundingRect())
            self.plot_p_press_view.linkedViewChanged(self.plot_p_sens.getViewBox(), self.plot_p_press_view.XAxis)
        self.plot_p_sens.getViewBox().sigResized.connect(update_press_view)
        graph_layout.addWidget(self.plot_p_sens)

        self.tabs.addTab(self.tab_graphs, "📈 Sensör Grafikleri")

        # ---------------- SEKME 2: HARİTA ----------------
        self.tab_map = QWidget()
        map_layout = QVBoxLayout(self.tab_map)
        map_layout.setContentsMargins(0, 0, 0, 0)
        
        self.web_view = QWebEngineView()
        # Security/CORS kısıtlamalarını esnetelim
        settings = self.web_view.settings()
        settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
        settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
        
        if os.path.exists(map_path):
            self.web_view.setUrl(QUrl.fromLocalFile(map_path))
        else:
            # Fallback olarak html string bas (Eski usul)
            self.web_view.setHtml(MAP_HTML, QUrl("http://localhost"))
            
        map_layout.addWidget(self.web_view)
        self.tabs.addTab(self.tab_map, "🗺️ Canlı GPS Haritası")

        # ---------------- SEKME 3: 3D YÖNELİM ----------------
        self.tab_3d = QWidget()
        t3d_layout = QVBoxLayout(self.tab_3d)
        t3d_layout.setContentsMargins(0, 0, 0, 0) 

        self.gl_widget = Rocket3DWidget()
        t3d_layout.addWidget(self.gl_widget)
        self.tabs.addTab(self.tab_3d, "🎯 3D Aviyonik")
        
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setSizes([380, 1540])

    def refresh_ports(self):
        curr_rocket = self.rocket_cb.currentText()
        curr_payload = self.payload_cb.currentText()
        self.rocket_cb.clear()
        self.payload_cb.clear()
        
        self.rocket_cb.addItem("Simülatör")
        self.payload_cb.addItem("Simülatör")

        for p in serial.tools.list_ports.comports():
            self.rocket_cb.addItem(p.device)
            self.payload_cb.addItem(p.device)
            
        if curr_rocket: self.rocket_cb.setCurrentText(curr_rocket)
        if curr_payload: self.payload_cb.setCurrentText(curr_payload)

    def connect_rocket(self):
        port = self.rocket_cb.currentText()
        baudrate = self.rocket_baud.currentText()
        if not port: return
        
        self.rocket_worker = SerialWorker(port, baudrate, "rocket", self.start_time)
        self.rocket_worker.raw_data_signal.connect(self.on_raw_data)
        self.rocket_worker.parsed_data_signal.connect(self.on_parsed_data)
        self.rocket_worker.error_signal.connect(self.on_sys_error)
        self.rocket_worker.disconnected_signal.connect(lambda i: self.disconnect_system(i))
        
        self.rocket_worker.start()

        self.rocket_connect_btn.setEnabled(False)
        self.rocket_disconnect_btn.setEnabled(True)
        self.rocket_cb.setEnabled(False)
        self.rocket_baud.setEnabled(False)

        self.on_raw_data("rocket", f"=== {port} BAĞLANDI ===")

    def connect_payload(self):
        port = self.payload_cb.currentText()
        baudrate = self.payload_baud.currentText()
        if not port: return
        
        self.payload_worker = SerialWorker(port, baudrate, "payload", self.start_time)
        self.payload_worker.raw_data_signal.connect(self.on_raw_data)
        self.payload_worker.parsed_data_signal.connect(self.on_parsed_data)
        self.payload_worker.error_signal.connect(self.on_sys_error)
        self.payload_worker.disconnected_signal.connect(lambda i: self.disconnect_system(i))
        
        self.payload_worker.start()

        self.payload_connect_btn.setEnabled(False)
        self.payload_disconnect_btn.setEnabled(True)
        self.payload_cb.setEnabled(False)
        self.payload_baud.setEnabled(False)

        self.on_raw_data("payload", f"=== {port} BAĞLANDI ===")

    def disconnect_system(self, identifier):
        if identifier == "rocket":
            if self.rocket_worker:
                self.rocket_worker.stop()
                self.rocket_worker = None
            self.rocket_connect_btn.setEnabled(True)
            self.rocket_disconnect_btn.setEnabled(False)
            self.rocket_cb.setEnabled(True)
            self.rocket_baud.setEnabled(True)
            self.on_raw_data("rocket", "=== BAĞLANTI KESİLDİ ===")
        elif identifier == "payload":
            if self.payload_worker:
                self.payload_worker.stop()
                self.payload_worker = None
            self.payload_connect_btn.setEnabled(True)
            self.payload_disconnect_btn.setEnabled(False)
            self.payload_cb.setEnabled(True)
            self.payload_baud.setEnabled(True)
            self.on_raw_data("payload", "=== BAĞLANTI KESİLDİ ===")

    def on_raw_data(self, identifier, raw_str):
        if identifier == "rocket":
            self.append_text(f'<span style="color:#2196F3; font-weight:bold;">[ROKET]</span> {raw_str}')
        else:
            self.append_text(f'<span style="color:#FF9800; font-weight:bold;">[G.YÜKÜ]</span> {raw_str}')

    def on_parsed_data(self, identifier, parts, t):
        if parts[0] in ["ROKET", "YUK", "GOREVYUKU"]:
            parts = parts[1:]

        if identifier == "rocket":
            try:
                if len(parts) > 0:
                    self.rocket_labels["İrtifa (m)"].setText(parts[0])
                    self.r_t.append(t); self.r_alt.append(float(parts[0]))
                if len(parts) > 1:
                    self.rocket_labels["Hız (m/s)"].setText(parts[1])
                    self.r_vel.append(float(parts[1]))
                if len(parts) > 2:
                    self.rocket_labels["İvme (g)"].setText(parts[2])
                    self.r_acc.append(float(parts[2]))
                if len(parts) > 3: 
                    self.rocket_labels["Durum"].setText(parts[3])
                if len(parts) > 4: 
                    gps_str = parts[4]
                    self.rocket_labels["GPS"].setText(gps_str)
                    self.process_gps(gps_str, target="rocket")
                if len(parts) > 7:
                    try:
                        roll = float(parts[5])
                        pitch = float(parts[6])
                        yaw = float(parts[7])
                        self.rocket_labels["Avionik (R,P,Y)"].setText(f"{roll:.1f}, {pitch:.1f}, {yaw:.1f}")
                        self.gl_widget.set_angles(roll, pitch, yaw)
                    except ValueError: pass
            except Exception: pass

        elif identifier == "payload":
            try:
                if len(parts) > 0:
                    self.payload_labels["İrtifa (m)"].setText(parts[0])
                    self.p_t.append(t); self.p_alt.append(float(parts[0]))
                if len(parts) > 1:
                    self.payload_labels["Sıcaklık (°C)"].setText(parts[1])
                    self.p_temp.append(float(parts[1]))
                if len(parts) > 2:
                    self.payload_labels["Basınç (Pa)"].setText(parts[2])
                    self.p_press.append(float(parts[2]))
                if len(parts) > 3: 
                    self.payload_labels["Durum"].setText(parts[3])
                if len(parts) > 4: 
                    self.payload_labels["Nem (%)"].setText(parts[4])
                if len(parts) > 5:
                    gps_str = parts[5]
                    self.payload_labels["GPS"].setText(gps_str)
                    self.process_gps(gps_str, target="payload")
            except Exception: pass

    def on_sys_error(self, identifier, err_msg):
        self.append_text(f'<span style="color:#F44336; font-weight:bold;">[HATA-{identifier.upper()}] {err_msg}</span>')

    def process_gps(self, gps_str, target="rocket"):
        for char in ['|', ';', ':']:
            if char in gps_str:
                coords = gps_str.split(char)
                if len(coords) >= 2:
                    try:
                        lat, lon = float(coords[0].strip()), float(coords[1].strip())
                        if target == "rocket":
                            self.web_view.page().runJavaScript(f"updateRocket({lat}, {lon});")
                        elif target == "payload":
                            self.web_view.page().runJavaScript(f"updatePayload({lat}, {lon});")
                    except ValueError: pass
                break

    def update_plots(self):
        if len(self.r_t) > 0:
            self.curve_r_alt.setData(list(self.r_t), list(self.r_alt))
            self.curve_r_vel.setData(list(self.r_t), list(self.r_vel))
            self.curve_r_acc.setData(list(self.r_t), list(self.r_acc))
        if len(self.p_t) > 0:
            self.curve_p_alt.setData(list(self.p_t), list(self.p_alt))
            self.curve_p_temp.setData(list(self.p_t), list(self.p_temp))
            self.curve_p_press.setData(list(self.p_t), list(self.p_press))

    def append_text(self, text):
        formatted_text = text.replace('\n', '<br/>').replace('\r', '') + '<br/>'
        scrollbar = self.text_area.verticalScrollBar()
        is_scrolled_to_bottom = scrollbar.value() == scrollbar.maximum()
        self.text_area.moveCursor(self.text_area.textCursor().MoveOperation.End)
        self.text_area.insertHtml(formatted_text)
        if is_scrolled_to_bottom:
            scrollbar.setValue(scrollbar.maximum())

    def closeEvent(self, event):
        self.disconnect_system("rocket")
        self.disconnect_system("payload")
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerialViewerApp()
    window.show()
    sys.exit(app.exec())
