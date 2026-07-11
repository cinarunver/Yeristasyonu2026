# ==============================================================================
# TODO LİSTESİ (Güncelleme: 2026-07-02)
#
# --- 📋 YAPILACAKLAR ---
# 1. HYİ (Hakem Yer İstasyonu) entegrasyonu — TEKNOFEST hakem paketi formatı
#    + ayrı COM port üzerinden gönderim (başlanmadı).
# 2. Loglama iyileştirme: CSV format, timestamp, tüm telemetri alanları
#    (şu an sadece özet satırı düz txt).
# 3. Harita internete bağımlı (Leaflet CDN + CartoDB tile) — sahada internet
#    yoksa boş ekran; offline tile fallback gerekli.
# 4. "Veri kesildi" uyarısı (son paketten bu yana X sn geçtiyse görsel ikaz).
# 5. kalman_monitor.py'nin commit edilmesi.
#
# --- ✓ YAPILDI (2026-07-09 nihai tur — firmware ile birebir hizalama) ---
# - Görev yükü paketi gerçek firmware'e (gorevyuku.cpp) göre kesinleştirildi:
#   GorevYukuPaket = 6 float '<6f' (basinc hPa, sicaklik, nem, irtifa,
#   gpsEnlem, gpsBoylam), LEN=24, çerçeve=29B. Firmware IMU/uçuş durumu
#   GÖNDERMEZ; hayali '<15fB' (IMU+durum) formatı geri alındı — donanımla
#   hiç eşleşmiyordu (LEN=24 vs 61). Arayüzden de fazladan alanlar kaldırıldı.
# - Roket paketi doğrulandı: TelemetryPacket '<14f3B' = 59B, LEN=59, çerçeve=64B
#   (main.cpp struct'ı ile birebir, little-endian).
# - Nem (%) grafiği eklendi; grafik eksen etiketleri ve ikincil eksen legend'ları
#   düzeltildi; simülatör irtifa gürültüsü gerçekçi (±2 m) yapıldı.
# --- ✓ YAPILDI (2026-07-02 düzeltme turu) ---
# - Payload protokolü gerçek firmware'e uyarlandı: GorevYukuPaket 24B '<6f'
#   (basinc hPa, sicaklik, nem, irtifa, gpsEnlem, gpsBoylam) — eski 71B format
#   gerçek donanımla hiç eşleşmiyordu.
# - Okuma modu (Binary/String) log formatından (Parsed/Raw) ayrıldı.
# - String parser jenerikleştirildi (payload string modunda KeyError kalktı).
# - CRC hatasında 2 bayt ilerleme (çerçeve kayması düzeltildi) + paket OK/hata
#   sayaçları arayüze eklendi.
# - Terminal (10 Hz) ve harita (2 Hz) güncellemeleri throttle edildi.
# - Grafik verileri deque(maxlen=10000) yapıldı.
# - stop() thread-güvenli hale getirildi; port okuma thread'inde kapanıyor.
# - Buffer taşma koruması kuyruk koruyacak şekilde düzeltildi.
# - Varsayılan baud 9600 (E32 LoRa alıcısı); update_plots hatası görünür.
# - Lokal Loglama (TXT kaydı, commit 412d250)
# ==============================================================================
# YER İSTASYONU v3.1 — FRAMED BINARY TELEMETRİ PROTOKOLü
# Roket : ESP32 UcusYazilimi/src/main.cpp → E32-433T30D LoRa @9600, ~10 Hz
#         [0xAA][0x55][LEN=59][TelemetryPacket 59B][CRC16_HI][CRC16_LO] = 64B
# G.Yükü: ESP32 GorevYukuYazilimi/gorevyuku.cpp → E32-433T30D LoRa @9600, ~10 Hz
#         [0xAA][0x55][LEN=48][GorevYukuPaket 48B][CRC16_HI][CRC16_LO] = 53B
#         GorevYukuPaket = BME280(basinc,sicaklik,nem,irtifa) + GPS(enlem,boylam)
#                        + BNO055 ivme(X,Y,Z) + BNO055 gyro(X,Y,Z) = 12 float.
#                        roll/pitch/yaw ve uçuş durumu GÖNDERİLMEZ (bkz. gorevyuku.cpp).
# ==============================================================================

import sys
import os
import csv
import time
import math
import random
import struct
import serial
import serial.tools.list_ports
from datetime import datetime
from collections import deque

# --- PAKET SABİTLERİ ---
# Kaynak: UcusYazilimi/src/main.cpp (TelemetryWire) ve
#         UcusYazilimi/GorevYukuYazilimi/gorevyuku.cpp (GorevYukuWire)
# FIXED-POINT WIRE FORMAT (paket kucultme): firmware float -> packed int quantize.
# Roket TelemetryWire (33B): ivmeX/Y/Z, gyroX/Y/Z, roll, pitch (8x int16),
#   yaw (uint16), irtifa, dikeyHiz, eglim (3x int16), gpsEnlem, gpsBoylam
#   (2x int32), durum (uint8 bitfield). Olcekler asagidaki WIRE_OLCEK_* ile ters cevrilir.
ROCKET_PACKET_FORMAT = '<8hH3h2iB'
ROCKET_PACKET_SIZE = struct.calcsize(ROCKET_PACKET_FORMAT) # 33 byte
ROCKET_FRAME_SIZE = 2 + 1 + ROCKET_PACKET_SIZE + 2 # 38 byte

# GorevYukuWire (28B): basinc (uint16 hPa x10), sicaklik (int16 x100),
#   nem (uint16 x100), irtifa (int16 x10), gpsEnlem/gpsBoylam (2x int32 x1e7),
#   ivmeX/Y/Z (3x int16 x100), gyroX/Y/Z (3x int16 x10).
# NOT: BNO055'ten yalnız ivme + gyro gönderilir; roll/pitch/yaw ve uçuş durumu GÖNDERİLMEZ.
PAYLOAD_PACKET_FORMAT = '<HhHh2i6h'
PAYLOAD_PACKET_SIZE = struct.calcsize(PAYLOAD_PACKET_FORMAT) # 28 byte
PAYLOAD_FRAME_SIZE = 2 + 1 + PAYLOAD_PACKET_SIZE + 2 # 33 byte

# --- FIXED-POINT OLCEKLER (firmware ile birebir; ters cevirmek icin bolunur) ---
WIRE_OLCEK_IVME   = 100.0
WIRE_OLCEK_GYRO   = 10.0
WIRE_OLCEK_ACI    = 100.0
WIRE_OLCEK_IRTIFA = 10.0
WIRE_OLCEK_HIZ    = 10.0
WIRE_OLCEK_GPS    = 1e7
WIRE_OLCEK_BASINC = 10.0    # hPa x10
WIRE_OLCEK_SICAK  = 100.0
WIRE_OLCEK_NEM    = 100.0

# Firmware q16/qu16/q32 ile birebir clamp (simulasyon paketleme overflow'unu onler).
def _qi16(x): return max(-32768, min(32767, int(round(x))))
def _qu16(x): return max(0, min(65535, int(round(x))))
def _qi32(x): return max(-2147483648, min(2147483647, int(round(x))))

SYNC_1, SYNC_2 = 0xAA, 0x55

# --- CSV LOG SÜTUNLARI ---
# (CSV kolon başlığı, packet sözlüğü anahtarı, ondalık basamak sayısı).
#   ondalık = None  → hesaplanan/tam sayı alan (zaman, saat, bayraklar, durum)
#   ondalık = int   → float alan bu kadar haneye yuvarlanır (okunaklı + küçük dosya)
# GPS 7 hane (~1 cm), irtifa/hız/basınç/sıcaklık 2 hane, açı/ivme/gyro 3 hane.
# Roket ve görev yükü ayrı şemaya sahip → her kaynak kendi CSV dosyasına yazılır.
ROCKET_CSV_ALANLARI = [
    ('zaman_s', None, 3), ('saat', None, None),
    ('irtifa_m', 'irtifa', 2), ('dikeyHiz_ms', 'dikeyHiz', 2), ('eglim_derece', 'eglimAcisi', 2),
    ('ivmeX', 'ivmeX', 3), ('ivmeY', 'ivmeY', 3), ('ivmeZ', 'ivmeZ', 3),
    ('gyroX', 'gyroX', 3), ('gyroY', 'gyroY', 3), ('gyroZ', 'gyroZ', 3),
    ('roll', 'roll', 2), ('pitch', 'pitch', 2), ('yaw', 'yaw', 2),
    ('gpsEnlem', 'gpsEnlem', 7), ('gpsBoylam', 'gpsBoylam', 7),
    ('ayrilma1', 'ayrilma1_durum', None), ('ayrilma2', 'ayrilma2_durum', None),
    ('ucus_durumu', 'ucus_durumu', None),
]
PAYLOAD_CSV_ALANLARI = [
    ('zaman_s', None, 3), ('saat', None, None),
    ('basinc_hPa', 'basinc', 2), ('sicaklik_C', 'bmeSicaklik', 2), ('nem_pct', 'nem', 2),
    ('irtifa_m', 'irtifa', 2), ('gpsEnlem', 'gpsEnlem', 7), ('gpsBoylam', 'gpsBoylam', 7),
    ('ivmeX', 'ivmeX', 3), ('ivmeY', 'ivmeY', 3), ('ivmeZ', 'ivmeZ', 3),
    ('gyroX', 'gyroX', 3), ('gyroY', 'gyroY', 3), ('gyroZ', 'gyroZ', 3),
]

DURUM_ETIKET = {
    0: 'HAZIR',
    1: 'YÜKSELİYOR',
    2: 'İNİŞ_1 (Drogue)',
    3: 'İNİŞ_2 (Ana Paraşüt)',
    4: 'İNDİ ✓',
}

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def parse_rocket_frame(raw: bytes):
    if len(raw) != ROCKET_FRAME_SIZE: return None
    if raw[0] != SYNC_1 or raw[1] != SYNC_2: return None
    if raw[2] != ROCKET_PACKET_SIZE: return None
    payload = raw[3:3 + ROCKET_PACKET_SIZE]
    crc_recv = (raw[3 + ROCKET_PACKET_SIZE] << 8) | raw[3 + ROCKET_PACKET_SIZE + 1]
    if crc16_ccitt(payload) != crc_recv: return None
    v = struct.unpack(ROCKET_PACKET_FORMAT, payload)
    durum = v[14]   # '<8hH3h2iB' -> 15 deger: v[14] = durum bitfield byte
    return {
        'ivmeX': v[0] / WIRE_OLCEK_IVME, 'ivmeY': v[1] / WIRE_OLCEK_IVME, 'ivmeZ': v[2] / WIRE_OLCEK_IVME,
        'gyroX': v[3] / WIRE_OLCEK_GYRO, 'gyroY': v[4] / WIRE_OLCEK_GYRO, 'gyroZ': v[5] / WIRE_OLCEK_GYRO,
        'roll':  v[6] / WIRE_OLCEK_ACI,  'pitch': v[7] / WIRE_OLCEK_ACI,  'yaw':   v[8] / WIRE_OLCEK_ACI,
        'irtifa':     v[9]  / WIRE_OLCEK_IRTIFA,
        'dikeyHiz':   v[10] / WIRE_OLCEK_HIZ,
        'eglimAcisi': v[11] / WIRE_OLCEK_ACI,
        'gpsEnlem':   v[12] / WIRE_OLCEK_GPS,
        'gpsBoylam':  v[13] / WIRE_OLCEK_GPS,
        'ayrilma1_durum': bool(durum & 0x01),
        'ayrilma2_durum': bool(durum & 0x02),
        'ucus_durumu':    (durum >> 2) & 0x07,
    }

def parse_payload_frame(raw: bytes):
    if len(raw) != PAYLOAD_FRAME_SIZE: return None
    if raw[0] != SYNC_1 or raw[1] != SYNC_2: return None
    if raw[2] != PAYLOAD_PACKET_SIZE: return None
    payload = raw[3:3 + PAYLOAD_PACKET_SIZE]
    crc_recv = (raw[3 + PAYLOAD_PACKET_SIZE] << 8) | raw[3 + PAYLOAD_PACKET_SIZE + 1]
    if crc16_ccitt(payload) != crc_recv: return None
    v = struct.unpack(PAYLOAD_PACKET_FORMAT, payload)
    return {
        'basinc':      v[0] / WIRE_OLCEK_BASINC,   # hPa
        'bmeSicaklik': v[1] / WIRE_OLCEK_SICAK,
        'nem':         v[2] / WIRE_OLCEK_NEM,
        'irtifa':      v[3] / WIRE_OLCEK_IRTIFA,
        'gpsEnlem':    v[4] / WIRE_OLCEK_GPS,
        'gpsBoylam':   v[5] / WIRE_OLCEK_GPS,
        'ivmeX': v[6] / WIRE_OLCEK_IVME, 'ivmeY': v[7] / WIRE_OLCEK_IVME, 'ivmeZ': v[8] / WIRE_OLCEK_IVME,
        'gyroX': v[9] / WIRE_OLCEK_GYRO, 'gyroY': v[10] / WIRE_OLCEK_GYRO, 'gyroZ': v[11] / WIRE_OLCEK_GYRO,
    }

# (String modu tamamen kaldırıldı)
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QPushButton, 
                             QTextEdit, QMessageBox, QGroupBox, QFormLayout, 
                             QSplitter, QTabWidget, QFileDialog, QRadioButton,
                             QLineEdit, QButtonGroup, QCheckBox)
from PyQt6.QtCore import pyqtSignal, QObject, QTimer, Qt, QThread, QUrl
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEngineSettings
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
import pyqtgraph as pg

# OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *

# Çapraz Platform (Cross-Platform) Grafik Motoru Ayarlamaları
# Sadece Mac (Apple Silicon) üzerinde WebEngine ve OpenGL çakışmasını önlemek için uygulanır.
if sys.platform == "darwin":
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
    <link rel="stylesheet" href="assets/leaflet.css"/>
    <script src="assets/leaflet.js"></script>
    <style> 
        body { margin: 0; padding: 0; background-color: #000; } 
        #map { height: 100vh; width: 100vw; background: #000; } 
    </style>
</head>
<body>
    <div id="map"></div>
    <script>
        var map = L.map('map').setView([38.835, 33.393], 5);
        
        // 1. Harita Katmanları (Base Layers)
        var darkLayer = L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png', {
            maxZoom: 19, attribution: '© OpenStreetMap © CartoDB'
        });

        var topoLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}', {
            maxZoom: 19, attribution: 'Tiles © Esri'
        });

        var satelliteLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
            maxZoom: 19, attribution: 'Tiles © Esri'
        });

        // Varsayılan olarak Koyu Katmanı ekle
        darkLayer.addTo(map);

        // Katman Seçim Kontrolü (Layer Switcher Pop-up)
        var baseMaps = {
            "Koyu Harita": darkLayer,
            "Karasal Harita": topoLayer,
            "Uydu Haritası": satelliteLayer
        };
        L.control.layers(baseMaps, null, { position: 'topright' }).addTo(map);

        var rocketIcon = L.icon({
            iconUrl: 'assets/marker-icon-2x-blue.png',
            shadowUrl: 'assets/marker-shadow.png',
            iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34], shadowSize: [41, 41]
        });

        var payloadIcon = L.icon({
            iconUrl: 'assets/marker-icon-2x-orange.png',
            shadowUrl: 'assets/marker-shadow.png',
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
    parsed_data_signal = pyqtSignal(str, dict, float)
    error_signal = pyqtSignal(str, str)
    disconnected_signal = pyqtSignal(str)
    stats_signal = pyqtSignal(str, int, int)  # identifier, geçerli paket, hatalı paket

    def __init__(self, port, baudrate, identifier, start_time, mode="binary"):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.identifier = identifier
        self.start_time = start_time
        self.mode = mode
        self.is_running = True
        self.serial_conn = None
        self.paket_ok = 0
        self.paket_hata = 0

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

        buf = bytearray()
        sync = bytes([SYNC_1, SYNC_2])
        frame_size = ROCKET_FRAME_SIZE if self.identifier == "rocket" else PAYLOAD_FRAME_SIZE
        parse_fn = parse_rocket_frame if self.identifier == "rocket" else parse_payload_frame
        son_stats = time.time()
        try:
            while self.is_running and self.serial_conn and self.serial_conn.is_open:
                try:
                    waiting = self.serial_conn.in_waiting
                    if waiting > 0:
                        buf.extend(self.serial_conn.read(waiting))
                        # Taşma koruması: hepsini silme, kuyruğu koru (kısmi çerçeve kaybolmasın)
                        if len(buf) > frame_size * 20:
                            del buf[:len(buf) - frame_size * 2]
                        while len(buf) >= frame_size:
                            idx = buf.find(sync)
                            if idx == -1:
                                # SYNC yok; son bayt 0xAA olabilir, onu sakla
                                del buf[:-1]
                                break
                            if idx > 0:
                                del buf[:idx]
                            if len(buf) < frame_size:
                                break
                            frame = bytes(buf[:frame_size])
                            packet = parse_fn(frame)
                            if packet is not None:
                                del buf[:frame_size]
                                self.paket_ok += 1
                                t = time.time() - self.start_time
                                self.raw_data_signal.emit(self.identifier, frame.hex())
                                self.parsed_data_signal.emit(self.identifier, packet, t)
                            else:
                                # LEN/CRC tutmadı: sahte SYNC olabilir — çerçeveyi atma,
                                # sadece SYNC'i geç ki kaymış gerçek çerçeve yakalanabilsin
                                del buf[:2]
                                self.paket_hata += 1
                    else:
                        self.msleep(5)

                    now = time.time()
                    if now - son_stats >= 1.0:
                        self.stats_signal.emit(self.identifier, self.paket_ok, self.paket_hata)
                        son_stats = now
                except Exception as e:
                    if self.is_running:
                        self.error_signal.emit(self.identifier, f"Okuma hatası: {e}")
                        self.disconnected_signal.emit(self.identifier)
                    break
        finally:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
            except Exception:
                pass

    def run_simulator(self):
        alt, vel, acc = 0.0, 0.0, 0.0
        r, p, yaw_a = 0.0, 0.0, 0.0
        lat, lon = 41.0082, 28.9784 # İstanbul
        ucus_durumu = 0
        ayrilma1 = 0
        ayrilma2 = 0
        dt = 0.1

        while self.is_running:
            t = time.time() - self.start_time
            
            # --- FİZİK MOTORU (Hedef: ~3800m Apogee) ---
            if t < 3.0:
                ucus_durumu = 0
                acc = 0.0 + random.uniform(-0.02, 0.02)
                vel = 0.0
                alt = 0.0
            elif t < 8.3:
                ucus_durumu = 1
                acc = 5.2 + random.uniform(-0.2, 0.2)
                vel += (acc * 9.81) * dt
                alt += vel * dt
                lat += 0.00005
                lon += 0.00002
            elif vel > 0:
                ucus_durumu = 1
                acc = -1.0 + random.uniform(-0.05, 0.05)
                vel -= 9.81 * dt + (0.002 * vel**2 * dt)
                alt += vel * dt
                if vel <= 0:
                    ucus_durumu = 2
                    ayrilma1 = 1
            elif alt > 600:
                ucus_durumu = 2
                vel = -25.0 + random.uniform(-1.0, 1.0)
                alt += vel * dt
                acc = -0.1
                if alt <= 600:
                    ucus_durumu = 3
                    ayrilma2 = 1
            elif alt > 0:
                ucus_durumu = 3
                vel = -5.0 + random.uniform(-0.5, 0.5)
                alt += vel * dt
                acc = -0.05
                if alt < 0: alt = 0
            else:
                ucus_durumu = 4
                vel = 0.0
                alt = 0.0
                acc = 0.0

            # Fıldır Fıldır Dönme
            r = (r + random.uniform(30.0, 90.0)) % 360 
            p = (p + random.uniform(15.0, 45.0)) % 360 
            yaw_a = (yaw_a + random.uniform(20.0, 60.0)) % 360
            
            # Gürültüsüz (noise-free) irtifa ve ivmeler
            sent_alt = alt

            if self.identifier == "rocket":
                # Firmware quantize esdegeri: float -> packed int (paket kucultme, clamp'li)
                payload = struct.pack(ROCKET_PACKET_FORMAT,
                    0, 0, _qi16(acc * WIRE_OLCEK_IVME),                      # ivme x100
                    0, 0, 0,                                                # gyro x10
                    _qi16(r * WIRE_OLCEK_ACI), _qi16(p * WIRE_OLCEK_ACI),   # roll,pitch x100
                    _qu16((yaw_a % 360) * WIRE_OLCEK_ACI),                  # yaw x100 (uint16)
                    _qi16(sent_alt * WIRE_OLCEK_IRTIFA),                    # irtifa x10
                    _qi16(vel * WIRE_OLCEK_HIZ),                            # dikeyHiz x10
                    _qi16((p % 90) * WIRE_OLCEK_ACI),                       # eglim x100
                    _qi32(lat * WIRE_OLCEK_GPS), _qi32(lon * WIRE_OLCEK_GPS),  # gps x1e7
                    (int(ayrilma1) & 1) | ((int(ayrilma2) & 1) << 1) | ((int(ucus_durumu) & 0x07) << 2)
                )
                crc = crc16_ccitt(payload)
                frame = bytes([SYNC_1, SYNC_2, ROCKET_PACKET_SIZE]) + payload + bytes([(crc>>8)&0xFF, crc&0xFF])
                packet = parse_rocket_frame(frame)
                if packet:
                    self.raw_data_signal.emit(self.identifier, frame.hex())
                    self.parsed_data_signal.emit(self.identifier, packet, t)
            else:
                # GorevYukuWire (28B): BME280(basinc,sicaklik,nem,irtifa) + GPS
                #  + BNO055 ivme(X,Y,Z) + gyro(X,Y,Z). roll/pitch/yaw göndermez.
                gy_ivmeZ = acc
                basinc_hpa = 1013.25 - sent_alt * 0.12
                sicaklik_c = 25.0 - sent_alt * 0.006
                payload = struct.pack(PAYLOAD_PACKET_FORMAT,
                    _qu16(basinc_hpa * WIRE_OLCEK_BASINC),                  # basinc hPa x10 (uint16)
                    _qi16(sicaklik_c * WIRE_OLCEK_SICAK),                   # sicaklik x100
                    _qu16(45.0 * WIRE_OLCEK_NEM),                          # nem x100 (uint16)
                    _qi16(sent_alt * WIRE_OLCEK_IRTIFA),                    # irtifa x10
                    _qi32(lat * WIRE_OLCEK_GPS), _qi32(lon * WIRE_OLCEK_GPS),  # gps x1e7
                    0, 0, _qi16(gy_ivmeZ * WIRE_OLCEK_IVME),                # ivme x100
                    0, 0, 0                                                  # gyro x10
                )
                crc = crc16_ccitt(payload)
                frame = bytes([SYNC_1, SYNC_2, PAYLOAD_PACKET_SIZE]) + payload + bytes([(crc>>8)&0xFF, crc&0xFF])
                packet = parse_payload_frame(frame)
                if packet:
                    self.raw_data_signal.emit(self.identifier, frame.hex())
                    self.parsed_data_signal.emit(self.identifier, packet, t)

            self.paket_ok += 1
            now = time.time()
            if now - getattr(self, '_sim_son_stats', 0.0) >= 1.0:
                self.stats_signal.emit(self.identifier, self.paket_ok, self.paket_hata)
                self._sim_son_stats = now
            self.msleep(100)

    def stop(self):
        # Portu buradan (GUI thread) kapatma: okuma thread'i hâlâ kullanıyor olabilir.
        # Bayrağı indir, thread kendi döngüsünden çıkıp portu finally'de kapatsın.
        self.is_running = False
        if not self.wait(2000):
            # Thread takıldıysa (ör. readline blokta) portu kapatarak okumayı kır
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
            except Exception:
                pass
            self.wait(1000)

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

        # Grafik verileri: maxlen ile sınırlı — sınırsız büyüyüp GUI'yi yavaşlatmasın
        # (10 Hz LoRa'da ~16 dk pencere)
        VERI_PENCERESI = 10000
        self.r_t_alt = deque(maxlen=VERI_PENCERESI)
        self.r_alt = deque(maxlen=VERI_PENCERESI)
        self.r_t_vel = deque(maxlen=VERI_PENCERESI)
        self.r_vel = deque(maxlen=VERI_PENCERESI)
        self.r_t_acc = deque(maxlen=VERI_PENCERESI)
        self.r_acc = deque(maxlen=VERI_PENCERESI)

        self.p_t_alt = deque(maxlen=VERI_PENCERESI)
        self.p_alt = deque(maxlen=VERI_PENCERESI)
        self.p_t_temp = deque(maxlen=VERI_PENCERESI)
        self.p_temp = deque(maxlen=VERI_PENCERESI)
        self.p_t_press = deque(maxlen=VERI_PENCERESI)
        self.p_press = deque(maxlen=VERI_PENCERESI)
        self.p_t_hum = deque(maxlen=VERI_PENCERESI)
        self.p_hum = deque(maxlen=VERI_PENCERESI)

        self.is_logging = False
        self.log_file_path = ""
        # Kaynak başına açık CSV dosya tanıtıcıları ve writer'ları (Parsed mod)
        self._csv_files = {}    # identifier -> file handle
        self._csv_writers = {}  # identifier -> csv.writer

        # Terminal/harita throttle zaman damgaları (100 Hz veri GUI'yi kilitlemesin)
        self._son_terminal = {}
        self._son_harita = {}
        self._plot_hata_gosterildi = False

        # Çizim dirty flag'leri ve son paket zamanları
        self.rocket_dirty = False
        self.payload_dirty = False
        self.last_rocket_packet_time = 0.0
        self.last_payload_packet_time = 0.0
        self.rocket_timeout_active = False
        self.payload_timeout_active = False

        self._setup_ui()
        self.refresh_ports()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)

        # Periyodik disk flush zamanlayıcısı (2 saniyede bir)
        self.flush_timer = QTimer()
        self.flush_timer.timeout.connect(self.flush_logs)
        self.flush_timer.start(2000)

    def _form_ayarla(self, form: QFormLayout):
        """Telemetri değer formlarının satırlarını okunaklı hale getirir.
        Dar sol panelde 'Aviyonik (R,P,Y)' gibi uzun başlıklar sığmayıp satır
        yükseklikleri çakışıyordu → başlık uzunsa etiketi ALTA sar (WrapAllRows)
        ve satırlar arası dikey boşluk ver ki değerler üst üste binmesin."""
        form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        form.setVerticalSpacing(8)
        form.setHorizontalSpacing(8)
        form.setContentsMargins(4, 4, 4, 4)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        form.setFormAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)

    def _form_baslik(self, key: str) -> QLabel:
        """Form satır başlığı (etiket) — değerle aynı dikey yükseklikte, sabit
        renkte. QLabel kullanıyoruz ki min-height verip çakışmayı önleyebilelim."""
        lbl = QLabel(key + ":")
        lbl.setMinimumHeight(22)
        lbl.setStyleSheet("color: #D4D4D4; font-weight: bold; font-size: 14px;")
        return lbl

    def _setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # Düzen: SOL veri kolonu üstten alta tek parça uzar; SAĞ taraf dikey bölünür
        # → üstte sekmeler, altta [Terminal (orta) | Log Kaydı (sağ)] şeridi.
        #   h_splitter (yatay): [ sol veri kolonu | sağ dikey blok ]
        #   right_v_splitter (dikey): [ sekmeler / alt şerit ]
        h_splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(h_splitter)

        right_v_splitter = QSplitter(Qt.Orientation.Vertical)

        # 1. SOL PANEL
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(5, 5, 5, 5)
        
        self.refresh_btn = QPushButton("🔄 Tüm COM Portlarını Yenile")
        self.refresh_btn.setStyleSheet("font-weight: bold; padding: 10px; background-color: #333333; color: white;")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        left_layout.addWidget(self.refresh_btn)

        # ROKET GRUBU
        self.rocket_group = QGroupBox("🚀 Roket")
        self.rocket_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 15px; }")
        rv_layout = QVBoxLayout()
        rc_layout = QHBoxLayout()
        self.rocket_cb = QComboBox()
        self.rocket_baud = QComboBox()
        self.rocket_baud.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.rocket_baud.setCurrentText("9600")  # E32-433T30D LoRa alıcısı 9600 (firmware BAUD_LORA)
        self.rocket_connect_btn = QPushButton("🚀 Roketi Bağla")
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
        self._form_ayarla(r_form_layout)
        self.rocket_labels = {
            "İrtifa (m)":      QLabel("-"),
            "Dikey Hız (m/s)": QLabel("-"),
            "İvme Z (m/s²)":   QLabel("-"),
            "Eğim Açısı (°)":  QLabel("-"),
            "Durum":           QLabel("-"),
            "GPS":             QLabel("-"),
            "Aviyonik (R,P,Y)": QLabel("-"),
            "Paket (OK/Hata)": QLabel("-"),
        }
        for key, lbl in self.rocket_labels.items():
            lbl.setStyleSheet("color: #2196F3; font-weight: bold; font-size: 14px;")
            lbl.setMinimumHeight(22)
            r_form_layout.addRow(self._form_baslik(key), lbl)
            
        rv_layout.addLayout(rc_layout)
        rv_layout.addLayout(rc2_layout)
        rv_layout.addLayout(r_form_layout)
        self.rocket_group.setLayout(rv_layout)
        left_layout.addWidget(self.rocket_group)
        
        # GÖREV YÜKÜ GRUBU
        self.payload_group = QGroupBox("🛰️ Görev Yükü")
        self.payload_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 15px; }")
        pv_layout = QVBoxLayout()
        pc_layout = QHBoxLayout()
        self.payload_cb = QComboBox()
        self.payload_baud = QComboBox()
        self.payload_baud.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.payload_baud.setCurrentText("9600")  # E32-433T30D LoRa alıcısı 9600 (firmware BAUD_LORA)
        self.payload_connect_btn = QPushButton("🛰️ Görev Yükünü Bağla")
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
        self._form_ayarla(p_form_layout)
        # GorevYukuPaket: BME280 + GPS + BNO055 (ivme+gyro). Firmware roll/pitch/yaw
        # ve uçuş durumu GÖNDERMEZ (gorevyuku.cpp), o alanlar arayüzde yok.
        self.payload_labels = {
            "İrtifa (m)":       QLabel("-"),
            "Sıcaklık (°C)":    QLabel("-"),
            "Basınç (hPa)":     QLabel("-"),
            "Nem (%)":          QLabel("-"),
            "İvme XYZ (m/s²)":  QLabel("-"),
            "Gyro XYZ (rad/s)": QLabel("-"),
            "GPS":              QLabel("-"),
            "Paket (OK/Hata)":  QLabel("-"),
        }
        for key, lbl in self.payload_labels.items():
            lbl.setStyleSheet("color: #FF9800; font-weight: bold; font-size: 14px;")
            lbl.setMinimumHeight(22)
            p_form_layout.addRow(self._form_baslik(key), lbl)
            
        pv_layout.addLayout(pc_layout)
        pv_layout.addLayout(pc2_layout)
        pv_layout.addLayout(p_form_layout)
        self.payload_group.setLayout(pv_layout)
        left_layout.addWidget(self.payload_group)
        
        left_layout.addStretch()

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
        # DİKKAT: units='m' verilirse pyqtgraph autoSIPrefix ile küçük irtifaları
        # (ör. 0.2 m) mm'ye çevirip ×1000 ÖLÇEKLER → 0.2 m grafikte "200" görünür.
        # Bu yüzden units KULLANMA, düz etiket ver ve autoSIPrefix'i kapat.
        self.plot_alt.setLabel('left', 'İrtifa (m)')
        self.plot_alt.getAxis('left').enableAutoSIPrefix(False)
        self.plot_alt.setLabel('bottom', 'Zaman', units='s')
        self.curve_r_alt = self.plot_alt.plot(pen=pg.mkPen('#2196F3', width=3), name='Roket İrtifa')
        self.curve_p_alt = self.plot_alt.plot(pen=pg.mkPen('#FF9800', width=3), name='GY İrtifa')
        graph_layout.addWidget(self.plot_alt)

        self.plot_r_kin = pg.PlotWidget(title="<span style='font-size: 14pt; color: #FFFFFF;'>Roket Hız ve İvme</span>")
        self.plot_r_kin.showGrid(x=True, y=True, alpha=0.3)
        r_kin_legend = self.plot_r_kin.addLegend(offset=(10, 10))
        # units='m/s' de autoSIPrefix ile küçük hızları ölçekler → düz etiket kullan
        self.plot_r_kin.setLabel('left', 'Hız (m/s)')
        self.plot_r_kin.getAxis('left').enableAutoSIPrefix(False)
        self.plot_r_kin.setLabel('bottom', 'Zaman', units='s')
        self.plot_r_kin.getAxis('right').setLabel('İvme (m/s²)')
        self.plot_r_kin.getAxis('right').enableAutoSIPrefix(False)
        self.curve_r_vel = self.plot_r_kin.plot(pen=pg.mkPen('#F44336', width=3), name='Hız (m/s)')

        self.plot_r_acc_view = pg.ViewBox()
        self.plot_r_kin.showAxis('right')
        self.plot_r_kin.scene().addItem(self.plot_r_acc_view)
        self.plot_r_kin.getAxis('right').linkToView(self.plot_r_acc_view)
        self.plot_r_acc_view.setXLink(self.plot_r_kin)
        self.curve_r_acc = pg.PlotCurveItem(pen=pg.mkPen('#9C27B0', width=3), name='İvme (g)')
        self.plot_r_acc_view.addItem(self.curve_r_acc)
        # İvme eğrisi ayrı ViewBox'ta olduğu için legend onu otomatik yakalayamaz — elle ekle
        r_kin_legend.addItem(self.curve_r_acc, 'İvme (g) [sağ eksen]')
        def update_acc_view():
            self.plot_r_acc_view.setGeometry(self.plot_r_kin.getViewBox().sceneBoundingRect())
            self.plot_r_acc_view.linkedViewChanged(self.plot_r_kin.getViewBox(), self.plot_r_acc_view.XAxis)
        self.plot_r_kin.getViewBox().sigResized.connect(update_acc_view)
        graph_layout.addWidget(self.plot_r_kin)

        self.plot_p_sens = pg.PlotWidget(title="<span style='font-size: 14pt; color: #FFFFFF;'>Görev Yükü Çevresel Veriler</span>")
        self.plot_p_sens.showGrid(x=True, y=True, alpha=0.3)
        p_sens_legend = self.plot_p_sens.addLegend(offset=(10, 10))
        self.plot_p_sens.setLabel('left', 'Sıcaklık (°C) / Nem (%)')
        self.plot_p_sens.getAxis('left').enableAutoSIPrefix(False)
        self.plot_p_sens.setLabel('bottom', 'Zaman', units='s')
        self.plot_p_sens.getAxis('right').setLabel('Basınç (hPa)')
        self.plot_p_sens.getAxis('right').enableAutoSIPrefix(False)
        self.curve_p_temp = self.plot_p_sens.plot(pen=pg.mkPen('#E91E63', width=3), name='Sıcaklık (°C)')
        self.curve_p_hum = self.plot_p_sens.plot(pen=pg.mkPen('#4CAF50', width=3, style=Qt.PenStyle.DashLine), name='Nem (%)')
        self.plot_p_press_view = pg.ViewBox()
        self.plot_p_sens.showAxis('right')
        self.plot_p_sens.scene().addItem(self.plot_p_press_view)
        self.plot_p_sens.getAxis('right').linkToView(self.plot_p_press_view)
        self.plot_p_press_view.setXLink(self.plot_p_sens)
        self.curve_p_press = pg.PlotCurveItem(pen=pg.mkPen('#00BCD4', width=3), name='Basınç (hPa)')
        self.plot_p_press_view.addItem(self.curve_p_press)
        # Basınç eğrisi ayrı ViewBox'ta olduğu için legend onu otomatik yakalayamaz — elle ekle
        p_sens_legend.addItem(self.curve_p_press, 'Basınç (hPa) [sağ eksen]')
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
        
        # Sol veri kolonu doğrudan yatay splitter'a; sekmeler ise sağ dikey bloğun ÜST
        # yarısına girer (alt yarıya aşağıda terminal+log şeridi eklenir).
        h_splitter.addWidget(left_widget)
        right_v_splitter.addWidget(right_widget)
        h_splitter.addWidget(right_v_splitter)

        # 3. ALT ŞERİT (TERMİNAL ORTADA + LOG KAYDI SAĞDA) — sağ dikey bloğun altına
        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout(bottom_widget)
        bottom_layout.setContentsMargins(0, 0, 0, 0)
        
        # 3.1 Terminal
        terminal_widget = QWidget()
        terminal_layout = QVBoxLayout(terminal_widget)
        terminal_layout.setContentsMargins(0, 0, 0, 0)
        
        terminal_layout.addWidget(QLabel("Terminal Log:"))
        self.text_area = QTextEdit()
        self.text_area.setReadOnly(True)
        self.text_area.document().setMaximumBlockCount(1000)
        self.text_area.setStyleSheet("background-color: #121212; color: #D4D4D4; font-family: monospace; font-size: 12px;")
        terminal_layout.addWidget(self.text_area)
        
        self.clear_btn = QPushButton("Terminali Temizle")
        self.clear_btn.clicked.connect(self.text_area.clear)
        terminal_layout.addWidget(self.clear_btn)
        
        # 3.2 Log Ayarları
        log_group = QGroupBox("💾 Log Kaydı Ayarları")
        log_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 15px; }")
        log_layout = QVBoxLayout()
        
        file_layout = QHBoxLayout()
        self.log_path_input = QLineEdit()
        self.log_path_input.setReadOnly(True)
        self.log_path_input.setPlaceholderText("Log dosyası seçin...")
        self.log_browse_btn = QPushButton("Gözat...")
        self.log_browse_btn.clicked.connect(self.browse_log_file)
        file_layout.addWidget(self.log_path_input)
        file_layout.addWidget(self.log_browse_btn)
        log_layout.addLayout(file_layout)
        
        format_layout = QHBoxLayout()
        self.rb_parsed = QRadioButton("Parsed (CSV)")
        self.rb_parsed.setToolTip("Her kaynak ayrı .csv dosyasına yazılır:\n"
                                  "<taban>_ukb.csv ve <taban>_gorevyuku.csv\n"
                                  "Başlık satırı + her paket bir satır (zaman + tüm alanlar).")
        self.rb_raw = QRadioButton("Raw (String/Hex)")
        self.rb_raw.setToolTip("Seçilen tek dosyaya ham hex/string satırları yazılır.")
        self.rb_parsed.setChecked(True)

        self.log_format_group = QButtonGroup()
        self.log_format_group.addButton(self.rb_parsed)
        self.log_format_group.addButton(self.rb_raw)
        format_layout.addWidget(self.rb_parsed)
        format_layout.addWidget(self.rb_raw)
        log_layout.addLayout(format_layout)

        # CSV ayraç seçimi — Türkçe Excel ondalık ayırıcı olarak virgül bekler.
        # İşaretli: ';' sütun ayracı + ',' ondalık (TR Excel). İşaretsiz: standart ',' + '.'.
        self.cb_tr_excel = QCheckBox("Türkçe Excel (; ayraç, virgül ondalık)")
        self.cb_tr_excel.setChecked(True)
        self.cb_tr_excel.setToolTip(
            "İşaretli: sütunlar ';' ile ayrılır, ondalık ayırıcı ','.\n"
            "  → Türkçe Excel'de çift tıklayınca sütunlara düzgün oturur.\n"
            "İşaretsiz: standart CSV (sütun ',' , ondalık '.').")
        log_layout.addWidget(self.cb_tr_excel)

        # --- İKİ AYRI KAYNAK DURUM BÖLÜMÜ ---
        # Tek dosya seçimi + tek Başlat düğmesi ikisini birlikte kaydeder, ancak
        # hangi dosyaya hangi kaynağın yazdığını ayrı ayrı görebilmek için her
        # kaynağın kendi durum kutusu var (dosya adı + yazıyor/boş + satır sayısı).
        #   UKB   = 'rocket' worker (ana roket portundan gelen veri) → _ukb.csv
        #   G.YÜKÜ = 'payload' worker (ayrı bağlanan görev yükü)     → _gorevyuku.csv
        self.log_durum_labels = {}   # identifier -> {'dosya','durum','sayac'} QLabel'ler
        self.log_satir_sayaci = {"rocket": 0, "payload": 0}
        for identifier, baslik, renk in (
                ("rocket", "📦 UKB (Ana Roketten)", "#2196F3"),
                ("payload", "🛰️ Görev Yükü (Bağlanan)", "#FF9800")):
            box = QGroupBox(baslik)
            box.setStyleSheet(f"QGroupBox {{ font-weight: bold; font-size: 13px; color: {renk}; }}")
            box_layout = QVBoxLayout()
            box_layout.setContentsMargins(8, 4, 8, 6)
            box_layout.setSpacing(2)
            dosya_lbl = QLabel("Dosya: -")
            dosya_lbl.setStyleSheet("color: #B0BEC5; font-weight: normal; font-size: 12px;")
            durum_lbl = QLabel("○ Kayıt kapalı")
            durum_lbl.setStyleSheet("color: #9E9E9E; font-weight: bold; font-size: 12px;")
            sayac_lbl = QLabel("Yazılan satır: 0")
            sayac_lbl.setStyleSheet("color: #B0BEC5; font-weight: normal; font-size: 12px;")
            box_layout.addWidget(dosya_lbl)
            box_layout.addWidget(durum_lbl)
            box_layout.addWidget(sayac_lbl)
            box.setLayout(box_layout)
            log_layout.addWidget(box)
            self.log_durum_labels[identifier] = {
                'dosya': dosya_lbl, 'durum': durum_lbl, 'sayac': sayac_lbl}

        self.log_active_btn = QPushButton("▶ Log Kaydını Başlat")
        self.log_active_btn.setCheckable(True)
        self.log_active_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        self.log_active_btn.toggled.connect(self.toggle_logging)
        log_layout.addWidget(self.log_active_btn)
        
        log_layout.addStretch()
        log_group.setLayout(log_layout)
        log_group.setFixedWidth(380)
        
        bottom_layout.addWidget(terminal_widget)
        bottom_layout.addWidget(log_group)

        # Alt şerit, sağ dikey bloğun alt yarısına: üstte sekmeler, altta terminal+log
        right_v_splitter.addWidget(bottom_widget)
        right_v_splitter.setSizes([800, 280])
        # Yatay: sol veri kolonu | sağ dikey blok
        h_splitter.setSizes([380, 1540])

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
        
        # Eski worker tam kapanmadıysa (Kes'ten hemen sonra Bağlan) önce temizle,
        # yoksa port hâlâ OS'te açık olabilir ve yeni bağlantı hata verir.
        if self.rocket_worker is not None:
            self._teardown_worker("rocket")
 
        self.rocket_worker = SerialWorker(port, baudrate, "rocket", self.start_time)
        self.rocket_worker.raw_data_signal.connect(self.on_raw_data)
        self.rocket_worker.parsed_data_signal.connect(self.on_parsed_data)
        self.rocket_worker.stats_signal.connect(self.on_stats)
        self.rocket_worker.error_signal.connect(self.on_sys_error)
        self.rocket_worker.disconnected_signal.connect(self.on_worker_disconnected)
 
        # Bağlantı başladığı an zaman damgasını sıfırla ve stilleri sıfırla
        self.last_rocket_packet_time = time.time()
        self.rocket_timeout_active = False
        self.rocket_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 15px; }")

        self.rocket_worker.start()
 
        self.rocket_connect_btn.setEnabled(False)
        self.rocket_disconnect_btn.setEnabled(True)
        self.rocket_cb.setEnabled(False)
        self.rocket_baud.setEnabled(False)
 
        self.on_raw_data("rocket", f"=== {port} BAĞLANDI ===")
        if self.is_logging and self.rb_parsed.isChecked():
            yollar = self._csv_yollari()
            self.append_text(
                f'<span style="color:#8BC34A;">[LOG] 📦 UKB (ana roket) bağlandı → '
                f'artık {os.path.basename(yollar["rocket"])} dosyasına yazılıyor.</span>')
 
    def connect_payload(self):
        port = self.payload_cb.currentText()
        baudrate = self.payload_baud.currentText()
        if not port: return
        
        if self.payload_worker is not None:
            self._teardown_worker("payload")
 
        self.payload_worker = SerialWorker(port, baudrate, "payload", self.start_time)
        self.payload_worker.raw_data_signal.connect(self.on_raw_data)
        self.payload_worker.parsed_data_signal.connect(self.on_parsed_data)
        self.payload_worker.stats_signal.connect(self.on_stats)
        self.payload_worker.error_signal.connect(self.on_sys_error)
        self.payload_worker.disconnected_signal.connect(self.on_worker_disconnected)

        # Bağlantı başladığı an zaman damgasını sıfırla ve stilleri sıfırla
        self.last_payload_packet_time = time.time()
        self.payload_timeout_active = False
        self.payload_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 15px; }")

        self.payload_worker.start()

        self.payload_connect_btn.setEnabled(False)
        self.payload_disconnect_btn.setEnabled(True)
        self.payload_cb.setEnabled(False)
        self.payload_baud.setEnabled(False)

        self.on_raw_data("payload", f"=== {port} BAĞLANDI ===")
        if self.is_logging and self.rb_parsed.isChecked():
            yollar = self._csv_yollari()
            self.append_text(
                f'<span style="color:#8BC34A;">[LOG] 🛰️ Görev Yükü bağlandı → '
                f'artık {os.path.basename(yollar["payload"])} dosyasına yazılıyor.</span>')

    def _teardown_worker(self, identifier):
        """Worker'ı durdurur, sinyallerini söker ve referansı temizler.
        Sinyalleri sökmek şart: aksi halde durdurulmakta olan worker'ın
        gecikmeli disconnected_signal'i, hemen ardından açılan yeni bağlantıyı
        keser (asıl 'tekrar bağlanmıyor' hatasının kaynağı)."""
        worker = self.rocket_worker if identifier == "rocket" else self.payload_worker
        if worker is None:
            return
        try:
            worker.disconnected_signal.disconnect()
            worker.raw_data_signal.disconnect()
            worker.parsed_data_signal.disconnect()
            worker.stats_signal.disconnect()
            worker.error_signal.disconnect()
        except (TypeError, RuntimeError):
            # Zaten sökülmüş / silinmiş olabilir — sorun değil
            pass
        worker.stop()
        worker.deleteLater()
        if identifier == "rocket":
            self.rocket_worker = None
        else:
            self.payload_worker = None

    def on_worker_disconnected(self, identifier):
        """Worker kendi kendine kesildiğinde (hata/port kopması) çağrılır.
        Yalnızca sinyali gönderen worker HÂLÂ güncel worker ise işleme al —
        eski bir worker'ın gecikmeli sinyali yeni bağlantıyı kesmesin."""
        sender = self.sender()
        guncel = self.rocket_worker if identifier == "rocket" else self.payload_worker
        if sender is not None and sender is not guncel:
            return
        self.disconnect_system(identifier)

    def disconnect_system(self, identifier):
        if identifier == "rocket":
            # Zaten kesildiyse (ör. hem stop() hem worker sinyali tetiklendi) tekrarlama
            if self.rocket_worker is None and self.rocket_connect_btn.isEnabled():
                return
            self._teardown_worker("rocket")
            self.rocket_connect_btn.setEnabled(True)
            self.rocket_disconnect_btn.setEnabled(False)
            self.rocket_cb.setEnabled(True)
            self.rocket_baud.setEnabled(True)
            self.on_raw_data("rocket", "=== BAĞLANTI KESİLDİ ===")
        elif identifier == "payload":
            if self.payload_worker is None and self.payload_connect_btn.isEnabled():
                return
            self._teardown_worker("payload")
            self.payload_connect_btn.setEnabled(True)
            self.payload_disconnect_btn.setEnabled(False)
            self.payload_cb.setEnabled(True)
            self.payload_baud.setEnabled(True)
            self.on_raw_data("payload", "=== BAĞLANTI KESİLDİ ===")

    def on_raw_data(self, identifier, raw_str):
        if self.is_logging and self.rb_raw.isChecked():
            self.write_log(f"[{identifier.upper()}] RAW: {raw_str}")

        # Terminal throttle: kaynak başına en fazla 10 satır/sn; sistem mesajları (===) muaf
        now = time.monotonic()
        if not raw_str.startswith("===") and now - self._son_terminal.get(identifier, 0.0) < 0.1:
            return
        self._son_terminal[identifier] = now

        if identifier == "rocket":
            self.append_text(f'<span style="color:#2196F3; font-weight:bold;">[ROKET]</span> {raw_str}')
        else:
            self.append_text(f'<span style="color:#FF9800; font-weight:bold;">[G.YÜKÜ]</span> {raw_str}')

    def on_parsed_data(self, identifier: str, packet: dict, t: float):
        if identifier == "rocket":
            self.last_rocket_packet_time = time.time()
            self.rocket_dirty = True
            try:
                self.rocket_labels["İrtifa (m)"].setText(f"{packet['irtifa']:.1f}")
                self.r_alt.append(packet['irtifa']); self.r_t_alt.append(t)
 
                self.rocket_labels["Dikey Hız (m/s)"].setText(f"{packet['dikeyHiz']:.2f}")
                self.r_vel.append(packet['dikeyHiz']); self.r_t_vel.append(t)
 
                self.rocket_labels["İvme Z (m/s²)"].setText(f"{packet['ivmeZ']:.2f}")
                self.r_acc.append(packet['ivmeZ']); self.r_t_acc.append(t)
 
                self.rocket_labels["Eğim Açısı (°)"].setText(f"{packet['eglimAcisi']:.1f}")
                self.rocket_labels["Durum"].setText(DURUM_ETIKET.get(packet['ucus_durumu'], '?'))
                self.rocket_labels["GPS"].setText(f"{packet['gpsEnlem']:.5f}, {packet['gpsBoylam']:.5f}")
                self.rocket_labels["Aviyonik (R,P,Y)"].setText(
                    f"{packet['roll']:.1f}, {packet['pitch']:.1f}, {packet['yaw']:.1f}")
 
                self.gl_widget.set_angles(packet['roll'], packet['pitch'], packet['yaw'])
                now = time.monotonic()
                if (packet['gpsEnlem'] != 0.0 or packet['gpsBoylam'] != 0.0) and \
                        now - self._son_harita.get("rocket", 0.0) >= 0.5:
                    self._son_harita["rocket"] = now
                    self.web_view.page().runJavaScript(
                        f"if (typeof updateRocket === 'function') updateRocket({packet['gpsEnlem']}, {packet['gpsBoylam']});")
 
                summary = (f"Alt:{packet['irtifa']:.1f}m | Hiz:{packet['dikeyHiz']:.1f}m/s | "
                           f"Durum:{DURUM_ETIKET.get(packet['ucus_durumu'],'?')} | "
                           f"GPS:{packet['gpsEnlem']:.4f},{packet['gpsBoylam']:.4f}")
                if self.is_logging and self.rb_parsed.isChecked():
                    self.write_csv_row("rocket", packet, t)
                # Terminale parse özetini throttle'lı bas
                if now - self._son_terminal.get("rocket_parse", 0.0) >= 0.1:
                    self._son_terminal["rocket_parse"] = now
                    self.append_text(f'<span style="color:#8BC34A;">  └─ [PARSE] {summary}</span>')
 
            except Exception as e:
                self.append_text(f'<span style="color:#F44336;">[PARSE HATASI] {e}</span>')
 
        elif identifier == "payload":
            self.last_payload_packet_time = time.time()
            self.payload_dirty = True
            try:
                self.payload_labels["İrtifa (m)"].setText(f"{packet['irtifa']:.1f}")
                self.p_alt.append(packet['irtifa']); self.p_t_alt.append(t)

                self.payload_labels["Sıcaklık (°C)"].setText(f"{packet['bmeSicaklik']:.1f}")
                self.p_temp.append(packet['bmeSicaklik']); self.p_t_temp.append(t)

                self.payload_labels["Basınç (hPa)"].setText(f"{packet['basinc']:.1f}")
                self.p_press.append(packet['basinc']); self.p_t_press.append(t)

                self.payload_labels["Nem (%)"].setText(f"{packet['nem']:.1f}")
                self.p_hum.append(packet['nem']); self.p_t_hum.append(t)

                # BNO055 — görev yükü artık ivme + gyro gönderiyor (roll/pitch/yaw yok)
                self.payload_labels["İvme XYZ (m/s²)"].setText(
                    f"{packet['ivmeX']:.2f}, {packet['ivmeY']:.2f}, {packet['ivmeZ']:.2f}")
                self.payload_labels["Gyro XYZ (rad/s)"].setText(
                    f"{packet['gyroX']:.2f}, {packet['gyroY']:.2f}, {packet['gyroZ']:.2f}")

                self.payload_labels["GPS"].setText(f"{packet['gpsEnlem']:.5f}, {packet['gpsBoylam']:.5f}")

                now = time.monotonic()
                if (packet['gpsEnlem'] != 0.0 or packet['gpsBoylam'] != 0.0) and \
                        now - self._son_harita.get("payload", 0.0) >= 0.5:
                    self._son_harita["payload"] = now
                    self.web_view.page().runJavaScript(
                        f"if (typeof updatePayload === 'function') updatePayload({packet['gpsEnlem']}, {packet['gpsBoylam']});")

                summary = (f"Alt:{packet['irtifa']:.1f}m | Sic:{packet['bmeSicaklik']:.1f}°C | "
                           f"Bas:{packet['basinc']:.1f}hPa | "
                           f"GPS:{packet['gpsEnlem']:.4f},{packet['gpsBoylam']:.4f}")
                if self.is_logging and self.rb_parsed.isChecked():
                    self.write_csv_row("payload", packet, t)
                if now - self._son_terminal.get("payload_parse", 0.0) >= 0.1:
                    self._son_terminal["payload_parse"] = now
                    self.append_text(f'<span style="color:#CDDC39;">  └─ [PARSE] {summary}</span>')

            except Exception as e:
                self.append_text(f'<span style="color:#F44336;">[PAYLOAD PARSE HATASI] {e}</span>')



    def browse_log_file(self):
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Log Dosyasını Kaydet", "",
            "CSV Dosyaları (*.csv);;Metin Dosyaları (*.txt);;Tümü (*)")
        if file_path:
            self.log_path_input.setText(file_path)
            self.log_file_path = file_path
            self._log_durum_guncelle()  # dosya adlarını kutulara hemen yansıt

    def _csv_yollari(self):
        """Seçilen taban yoldan UKB (ana roket portu) ve görev yükü için ayrı CSV
        yolları üretir. Örn. 'ucus.csv' -> 'ucus_ukb.csv' + 'ucus_gorevyuku.csv'.
        NOT: 'rocket' worker'ı UKB'ye (ana rokete) bağlı olduğu için dosya adı _ukb."""
        taban, uzanti = os.path.splitext(self.log_file_path)
        if uzanti.lower() != '.csv':
            uzanti = '.csv'
        return {
            'rocket': f"{taban}_ukb{uzanti}",
            'payload': f"{taban}_gorevyuku{uzanti}",
        }

    def _log_durum_guncelle(self):
        """Her kaynağın log durum kutusunu (dosya adı / yazıyor-boş / satır sayısı)
        güncel duruma göre yeniler. Parsed CSV modunda: kayıt aktif + worker bağlı
        ise 'Yazılıyor', kayıt aktif ama worker bağlı değilse 'Boş — bağlı değil'.
        RAW modunda kaynak başına ayrı dosya yok → bilgi amaçlı tek dosya gösterilir."""
        worker_var = {"rocket": self.rocket_worker is not None,
                      "payload": self.payload_worker is not None}
        parsed = self.rb_parsed.isChecked()
        yollar = self._csv_yollari() if (parsed and self.log_file_path) else None

        for identifier, lbls in self.log_durum_labels.items():
            sayac = self.log_satir_sayaci.get(identifier, 0)
            lbls['sayac'].setText(f"Yazılan satır: {sayac}")

            if not parsed:
                lbls['dosya'].setText("Dosya: (RAW — tek dosya)")
            elif yollar:
                lbls['dosya'].setText(f"Dosya: {os.path.basename(yollar[identifier])}")
            else:
                lbls['dosya'].setText("Dosya: -")

            if not self.is_logging:
                lbls['durum'].setText("○ Kayıt kapalı")
                lbls['durum'].setStyleSheet("color: #9E9E9E; font-weight: bold; font-size: 12px;")
            elif not parsed:
                lbls['durum'].setText("● RAW kaydı açık")
                lbls['durum'].setStyleSheet("color: #FFEB3B; font-weight: bold; font-size: 12px;")
            elif worker_var[identifier]:
                lbls['durum'].setText("● Yazılıyor")
                lbls['durum'].setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 12px;")
            else:
                lbls['durum'].setText("○ Boş — bağlı değil")
                lbls['durum'].setStyleSheet("color: #FF9800; font-weight: bold; font-size: 12px;")

    def _csv_ac(self):
        """Kayıt başlarken kaynak başına CSV dosyasını açar, header'ı (yeni/boş
        dosyaysa) yazar. Zaten veri varsa append eder, header tekrar yazılmaz."""
        yollar = self._csv_yollari()
        semalar = {'rocket': ROCKET_CSV_ALANLARI, 'payload': PAYLOAD_CSV_ALANLARI}
        # Ayracı kayıt başında SABİTLE (kayıt ortasında değişmesin → dosya bütünlüğü).
        # Türkçe Excel: sütun ';', ondalık ',' — aksi halde standart ',' ve '.'.
        self._csv_tr_excel = self.cb_tr_excel.isChecked()
        self._csv_delimiter = ';' if self._csv_tr_excel else ','
        for identifier, yol in yollar.items():
            try:
                yeni = not os.path.exists(yol) or os.path.getsize(yol) == 0
                f = open(yol, 'a', newline='', encoding='utf-8')
                w = csv.writer(f, delimiter=self._csv_delimiter)
                if yeni:
                    w.writerow([sutun for sutun, _, _ in semalar[identifier]])
                    f.flush()
                self._csv_files[identifier] = f
                self._csv_writers[identifier] = w
            except Exception as e:
                self.append_text(f'<span style="color:#F44336;">[CSV AÇMA HATASI] {yol}: {e}</span>')

    def _csv_kapat(self):
        for f in self._csv_files.values():
            try:
                f.flush(); f.close()
            except Exception:
                pass
        self._csv_files.clear()
        self._csv_writers.clear()

    def flush_logs(self):
        if self.is_logging and self._csv_files:
            for identifier, f in list(self._csv_files.items()):
                try:
                    f.flush()
                except Exception as e:
                    self.append_text(f'<span style="color:#F44336;">[CSV FLUSH HATASI] {identifier}: {e}</span>')

    def toggle_logging(self, checked):
        if checked:
            if not self.log_file_path:
                QMessageBox.warning(self, "Uyarı", "Lütfen önce bir log dosyası seçin!")
                self.log_active_btn.setChecked(False)
                return
            self.is_logging = True
            self.log_satir_sayaci = {"rocket": 0, "payload": 0}  # sayaçları sıfırla
            # Parsed mod → kaynak başına CSV dosyaları aç (header dahil)
            if self.rb_parsed.isChecked():
                self._csv_ac()
                yollar = self._csv_yollari()
                self.append_text(
                    f'<span style="color:#FFEB3B;">[LOG] CSV kaydı başladı → 📦 {os.path.basename(yollar["rocket"])} , '
                    f'🛰️ {os.path.basename(yollar["payload"])}</span>')
                # Hangi kaynaklar bağlı? Bağlı olmayan kaynağın dosyası boş kalır —
                # kullanıcı "sadece bir kaynağın logu geliyor" sanmasın, açıkça uyar.
                roket_bagli = self.rocket_worker is not None
                gy_bagli    = self.payload_worker is not None
                self.append_text(
                    f'<span style="color:#90CAF9;">[LOG] Bağlı kaynaklar: '
                    f'📦 UKB={"EVET" if roket_bagli else "HAYIR"} , '
                    f'🛰️ Görev Yükü={"EVET" if gy_bagli else "HAYIR"}</span>')
                if not roket_bagli or not gy_bagli:
                    eksik = []
                    if not roket_bagli: eksik.append("📦 UKB")
                    if not gy_bagli:    eksik.append("🛰️ Görev Yükü")
                    self.append_text(
                        f'<span style="color:#FF9800; font-weight:bold;">[LOG UYARI] '
                        f'{" ve ".join(eksik)} henüz BAĞLI DEĞİL → ilgili CSV dosyası boş kalır. '
                        f'İlgili bölümdeki yeşil "Bağlan" düğmesine bas.</span>')
            else:
                self.append_text(f'<span style="color:#FFEB3B;">[LOG] RAW kaydı başladı: {self.log_file_path}</span>')
            self.log_active_btn.setText("⏹ Log Kaydını Durdur")
            self.log_active_btn.setStyleSheet("background-color: #F44336; color: white; font-weight: bold; padding: 10px;")
        else:
            self.is_logging = False
            self._csv_kapat()
            self.log_active_btn.setText("▶ Log Kaydını Başlat")
            self.log_active_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
            self.append_text('<span style="color:#FFEB3B;">[LOG] Kayıt durduruldu.</span>')
        self._log_durum_guncelle()  # başlat/durdur sonrası kutuları anında tazele

    def write_log(self, data_str):
        """RAW modu: seçilen tek dosyaya düz metin satırı ekler."""
        if not self.is_logging or not self.log_file_path:
            return
        try:
            with open(self.log_file_path, "a", encoding="utf-8") as f:
                f.write(data_str + "\n")
        except Exception as e:
            self.append_text(f'<span style="color:#F44336;">[LOG YAZMA HATASI] {e}</span>')

    def write_csv_row(self, identifier, packet, t):
        """Parsed modu: paketi, kaynağın CSV şemasına göre bir satır olarak yazar.
        Her float alan şemada tanımlı ondalık basamağa yuvarlanır → okunaklı,
        hizalı sütunlar ve küçük dosya (ham 17 haneli float yerine 2-7 hane).
        Türkçe Excel modunda ondalık ayırıcı '.' yerine ',' kullanılır (yalnız
        SAYISAL alanlarda — 'saat' alanındaki nokta korunur)."""
        writer = self._csv_writers.get(identifier)
        if writer is None:
            return
        tr = getattr(self, '_csv_tr_excel', False)  # ondalık virgül mü?

        def sayi(s):
            return s.replace('.', ',') if tr else s

        semalar = {'rocket': ROCKET_CSV_ALANLARI, 'payload': PAYLOAD_CSV_ALANLARI}
        satir = []
        for sutun, anahtar, ondalik in semalar[identifier]:
            if anahtar is None:
                if sutun == 'zaman_s':
                    satir.append(sayi(f"{t:.3f}"))
                elif sutun == 'saat':
                    # saat metnindeki '.' ondalık değil (sn.ms) — DÖNÜŞTÜRME
                    satir.append(datetime.now().strftime("%H:%M:%S.%f")[:-3])
                else:
                    satir.append('')
            else:
                deger = packet.get(anahtar)
                if isinstance(deger, bool):
                    satir.append(int(deger))
                elif ondalik is not None and isinstance(deger, (int, float)):
                    # sabit ondalıkla yaz (ör. 0.20 → '0.20', 3421.5 → '3421.50')
                    satir.append(sayi(f"{deger:.{ondalik}f}"))
                elif deger is None:
                    satir.append('')
                else:
                    satir.append(deger)
        try:
            writer.writerow(satir)
            # (f.flush() kaldırıldı, flush_timer tarafından 2 saniyede bir yapılacak)
            # Yazılan satır sayacını artır; kutudaki canlı sayaç update_plots'ta
            # (50 ms) tazelenir — her satırda label güncellemek GUI'yi meşgul eder.
            self.log_satir_sayaci[identifier] = self.log_satir_sayaci.get(identifier, 0) + 1
        except Exception as e:
            self.append_text(f'<span style="color:#F44336;">[CSV YAZMA HATASI] {e}</span>')

    def on_stats(self, identifier, ok, hata):
        metin = f"{ok} ✓ / {hata} ✗"
        if identifier == "rocket":
            self.rocket_labels["Paket (OK/Hata)"].setText(metin)
        else:
            self.payload_labels["Paket (OK/Hata)"].setText(metin)

    def on_sys_error(self, identifier, err_msg):
        self.append_text(f'<span style="color:#F44336; font-weight:bold;">[HATA-{identifier.upper()}] {err_msg}</span>')

    def update_plots(self):
        # 1. Bağlantı Kesildi (Timeout) Kontrolü
        now_time = time.time()
        
        # Roket için kontrol
        if self.rocket_worker is not None:
            if now_time - self.last_rocket_packet_time > 3.0:
                if not self.rocket_timeout_active:
                    self.rocket_timeout_active = True
                    self.rocket_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 15px; border: 2px solid #F44336; color: #F44336; }")
                    self.rocket_labels["Paket (OK/Hata)"].setText("BAĞLANTI KESİLDİ ⚠️")
                    self.rocket_labels["Paket (OK/Hata)"].setStyleSheet("color: #F44336; font-weight: bold; font-size: 14px;")
            else:
                if self.rocket_timeout_active:
                    self.rocket_timeout_active = False
                    self.rocket_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 15px; }")
                    self.rocket_labels["Paket (OK/Hata)"].setStyleSheet("color: #2196F3; font-weight: bold; font-size: 14px;")
        
        # Görev Yükü için kontrol
        if self.payload_worker is not None:
            if now_time - self.last_payload_packet_time > 3.0:
                if not self.payload_timeout_active:
                    self.payload_timeout_active = True
                    self.payload_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 15px; border: 2px solid #F44336; color: #F44336; }")
                    self.payload_labels["Paket (OK/Hata)"].setText("BAĞLANTI KESİLDİ ⚠️")
                    self.payload_labels["Paket (OK/Hata)"].setStyleSheet("color: #F44336; font-weight: bold; font-size: 14px;")
            else:
                if self.payload_timeout_active:
                    self.payload_timeout_active = False
                    self.payload_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 15px; }")
                    self.payload_labels["Paket (OK/Hata)"].setStyleSheet("color: #FF9800; font-weight: bold; font-size: 14px;")

        # 2. Grafik Güncellemeleri (Yalnızca dirty flag aktifse çizilir -> CPU dostu)
        try:
            if self.rocket_dirty:
                if len(self.r_t_alt) == len(self.r_alt) and len(self.r_alt) > 0:
                    self.curve_r_alt.setData(list(self.r_t_alt), list(self.r_alt))
                if len(self.r_t_vel) == len(self.r_vel) and len(self.r_vel) > 0:
                    self.curve_r_vel.setData(list(self.r_t_vel), list(self.r_vel))
                if len(self.r_t_acc) == len(self.r_acc) and len(self.r_acc) > 0:
                    self.curve_r_acc.setData(list(self.r_t_acc), list(self.r_acc))
                self.rocket_dirty = False
            
            if self.payload_dirty:
                if len(self.p_t_alt) == len(self.p_alt) and len(self.p_alt) > 0:
                    self.curve_p_alt.setData(list(self.p_t_alt), list(self.p_alt))
                if len(self.p_t_temp) == len(self.p_temp) and len(self.p_temp) > 0:
                    self.curve_p_temp.setData(list(self.p_t_temp), list(self.p_temp))
                if len(self.p_t_press) == len(self.p_press) and len(self.p_press) > 0:
                    self.curve_p_press.setData(list(self.p_t_press), list(self.p_press))
                if len(self.p_t_hum) == len(self.p_hum) and len(self.p_hum) > 0:
                    self.curve_p_hum.setData(list(self.p_t_hum), list(self.p_hum))
                self.payload_dirty = False
        except Exception as e:
            # Sessizce yutma: ilk hatayı terminale bas (spam olmasın diye bir kez)
            if not self._plot_hata_gosterildi:
                self._plot_hata_gosterildi = True
                self.append_text(f'<span style="color:#F44336;">[GRAFİK HATASI] {e}</span>')

        # Log durum kutularını (dosya/durum/satır sayacı) canlı tut
        self._log_durum_guncelle()

    def append_text(self, text):
        formatted_text = text.replace('\n', '<br/>').replace('\r', '')
        scrollbar = self.text_area.verticalScrollBar()
        is_scrolled_to_bottom = scrollbar.value() == scrollbar.maximum()
        self.text_area.append(formatted_text)
        if is_scrolled_to_bottom:
            scrollbar.setValue(scrollbar.maximum())

    def closeEvent(self, event):
        self.disconnect_system("rocket")
        self.disconnect_system("payload")
        self._csv_kapat()  # açık CSV dosyalarını flush edip kapat
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerialViewerApp()
    window.show()
    sys.exit(app.exec())
