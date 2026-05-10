import time
import math
import random
import struct
import sys

# --- PROTOKOL SABİTLERİ (YerIstasyonu2026.py ile aynı olmalı) ---
OUTPUT_MODE = "binary" # "hex", "string" veya "binary" seçebilirsiniz.
PACKET_FORMAT = '<14f3B'  # 14 float + 3 uint8
PACKET_SIZE   = struct.calcsize(PACKET_FORMAT) # 59 byte
SYNC_1, SYNC_2 = 0xAA, 0x55
FRAME_SIZE    = 2 + 1 + PACKET_SIZE + 2 # 64 byte

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def run_simulation():
    # Temel Fiziksel Değerler
    time_elapsed = 0.0
    dt = 0.1 # Saniyede 10 veri (10 Hz)
    
    altitude = 0.0
    velocity = 0.0
    acceleration_z = 1.0 # Dünya yerçekimi 1G
    
    # Açısal Değerler
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    
    # Başlangıç GPS (İstanbul)
    lat = 41.0082
    lon = 28.9784
    
    ucus_durumu = 0 # 0: HAZIR, 1: YÜKSELİYOR, 2: İNİŞ_1, 3: İNİŞ_2, 4: İNDİ
    ayrilma1 = 0
    ayrilma2 = 0
    
    print(f"🚀 Gerçekçi Roket Simülatörü Başlıyor")
    print(f"MOD: {OUTPUT_MODE.upper()}")
    print("-" * 70)
    
    try:
        while True:
            # --- FİZİK MOTORU (Hedef: ~3800m Apogee) ---
            if time_elapsed < 3.0:
                ucus_durumu = 0 # HAZIR
                acceleration_z = 0.0 + random.uniform(-0.02, 0.02)
                velocity = 0.0
                altitude = 0.0
            elif time_elapsed < 8.3:
                # Motor ateşlemesi (Kalkış): ivme ~5G
                ucus_durumu = 1 # YÜKSELİYOR
                acceleration_z = 5.2 + random.uniform(-0.2, 0.2)
                velocity += (acceleration_z * 9.81) * dt
                altitude += velocity * dt
                lat += 0.00005
                lon += 0.00002
            elif velocity > 0:
                # Motor bitti, serbest süzülüş (Coasting)
                ucus_durumu = 1 # YÜKSELİYOR
                acceleration_z = -1.0 + random.uniform(-0.05, 0.05) # Sadece yerçekimi ve sürtünme
                velocity -= 9.81 * dt + (0.002 * velocity**2 * dt) # Sürüklenme kuvveti (basit)
                altitude += velocity * dt
                
                # Apogee tespiti
                if velocity <= 0:
                    ucus_durumu = 2 # İNİŞ_1 (Drogue)
                    ayrilma1 = 1
            elif altitude > 600:
                # Birinci ayrılma sonrası düşüş
                ucus_durumu = 2
                velocity = -25.0 + random.uniform(-1.0, 1.0) # Sürüklenme paraşütüyle hızlı iniş
                altitude += velocity * dt
                acceleration_z = -0.1
                
                # İkinci ayrılma tespiti (600m)
                if altitude <= 600:
                    ucus_durumu = 3 # İNİŞ_2 (Ana)
                    ayrilma2 = 1
            elif altitude > 0:
                # Ana paraşütle yavaş iniş
                ucus_durumu = 3
                velocity = -5.0 + random.uniform(-0.5, 0.5)
                altitude += velocity * dt
                acceleration_z = -0.05
                if altitude < 0: altitude = 0
            else:
                # Yere inildi
                ucus_durumu = 4 # İNDİ
                velocity = 0.0
                altitude = 0.0
                acceleration_z = 0.0

            # Yönelim Simülasyonu (Fıldır Fıldır)
            roll = (roll + random.uniform(30.0, 90.0)) % 360 
            pitch = (pitch + random.uniform(15.0, 45.0)) % 360 
            yaw = (yaw + random.uniform(20.0, 60.0)) % 360
            
            # İrtifa saçmalaması (Gerçek fizik etkilenmez, sadece ekrana giden bozuk)
            sent_altitude = altitude + random.uniform(-800.0, 800.0)

            # --- PAKETLEME (Binary Struct) ---
            # Format: 14 float + 3 uint8 (UKB Formatı)
            # ivmeX, ivmeY, ivmeZ, gyroX, gyroY, gyroZ, roll, pitch, yaw, 
            # irtifa, dikeyHiz, eglimAcisi, gpsLat, gpsLon,
            # ayrilma1, ayrilma2, ucusDurumu
            payload = struct.pack(PACKET_FORMAT,
                random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1), acceleration_z, # Ivme (G)
                random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1),    # Gyro
                roll, pitch, yaw,
                sent_altitude, velocity, abs(pitch), # Saçmalayan İrtifa, Dikey Hız, Eğim Açısı
                lat, lon,
                ayrilma1, ayrilma2, ucus_durumu
            )
            
            # CRC Hesapla
            crc = crc16_ccitt(payload)
            
            # Çerçeveyi Oluştur: [SYNC1][SYNC2][LEN][PAYLOAD][CRC_HI][CRC_LO]
            frame = bytes([SYNC_1, SYNC_2, PACKET_SIZE]) + payload + struct.pack('>H', crc)
            
            if OUTPUT_MODE == "hex":
                print(f"[{time_elapsed:5.1f}s] Frame: {frame.hex().upper()}")
            elif OUTPUT_MODE == "string":
                print(f"Irtifa: {altitude:.2f} | Vz: {velocity:.2f} | Eglim: {abs(pitch):.2f} | Pitch: {pitch:.2f} | Durum: {ucus_durumu}")
            elif OUTPUT_MODE == "binary":
                sys.stdout.buffer.write(frame)
                sys.stdout.buffer.flush()
            
            time_elapsed += dt
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[!] Simülasyon durduruldu.")

if __name__ == "__main__":
    run_simulation()
