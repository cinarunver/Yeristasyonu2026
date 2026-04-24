import time
import math
import random
import struct

# --- PROTOKOL SABİTLERİ (YerIstasyonu2026.py ile aynı olmalı) ---
PACKET_FORMAT = '<17f3B'  # 17 float + 3 uint8
PACKET_SIZE   = struct.calcsize(PACKET_FORMAT) # 71 byte
SYNC_1, SYNC_2 = 0xAA, 0x55
FRAME_SIZE    = 2 + 1 + PACKET_SIZE + 2 # 76 byte

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
    
    # Başlangıç GPS (Tuz Gölü)
    lat = 38.83510
    lon = 33.39320
    
    ucus_durumu = 0 # 0: HAZIR, 1: YÜKSELİYOR, 2: İNİŞ_1, 3: İNİŞ_2, 4: İNDİ
    ayrilma1 = 0
    ayrilma2 = 0
    
    print(f"🚀 Gerçekçi Roket BINARY Simülatörü Başlıyor (76 Byte Frame)")
    print(f"Protokol: [AA 55][LEN={PACKET_SIZE}][PAYLOAD][CRC16]")
    print("-" * 70)
    
    try:
        while True:
            # --- FİZİK MOTORU ---
            if time_elapsed < 3.0:
                ucus_durumu = 0 # HAZIR
                acceleration_z = 1.0 + random.uniform(-0.02, 0.02)
                velocity = 0.0
                altitude = 0.0
            elif time_elapsed < 15.0:
                ucus_durumu = 1 # YÜKSELİYOR
                acceleration_z = 15.0 + random.uniform(-1.0, 1.0)
                velocity += (acceleration_z - 1.0) * 9.81 * dt
                altitude += velocity * dt
                lat += 0.00005
                lon += 0.00002
            elif time_elapsed < 30.0:
                ucus_durumu = 1 # HALA YÜKSELİYOR (Coasting)
                acceleration_z = -0.5 + random.uniform(-0.1, 0.1)
                velocity += (acceleration_z - 1.0) * 9.81 * dt # -g etkisi
                altitude += velocity * dt
                if velocity < 0 and ucus_durumu == 1:
                    ucus_durumu = 2 # APOGEE - INIS_1
                    ayrilma1 = 1
            elif altitude > 500:
                ucus_durumu = 2 # INIS_1 (Drogue)
                velocity = -15.0 + random.uniform(-1.0, 1.0)
                altitude += velocity * dt
                acceleration_z = 0.0
            elif altitude > 0:
                if ucus_durumu == 2:
                    ucus_durumu = 3 # INIS_2 (Ana Paraşüt)
                    ayrilma2 = 1
                velocity = -6.0 + random.uniform(-0.5, 0.5)
                altitude += velocity * dt
                if altitude < 0: altitude = 0
            else:
                ucus_durumu = 4 # İNDİ
                velocity = 0.0
                altitude = 0.0

            # Yönelim ve Sensörler
            roll = (roll + 5.0) % 360 if ucus_durumu == 1 else roll
            pitch = math.sin(time_elapsed) * 10.0
            yaw = (yaw + 1.0) % 360
            
            # Barometrik formül (yaklaşık)
            basinc = 101325.0 * (1 - 2.25577e-5 * altitude)**5.25588
            sicaklik = 25.0 - (altitude * 0.0065)
            nem = 45.0 + random.uniform(-5, 5)

            # --- PAKETLEME (Binary Struct) ---
            # Format: 17 float + 3 uint8
            # ivmeX, ivmeY, ivmeZ, gyroX, gyroY, gyroZ, roll, pitch, yaw, 
            # basinc, sicaklik, irtifa, nem, dikeyHiz, eglimAcisi, gpsLat, gpsLon,
            # ayrilma1, ayrilma2, ucusDurumu
            payload = struct.pack(PACKET_FORMAT,
                random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1), acceleration_z, # Ivme
                random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1),    # Gyro
                roll, pitch, yaw,
                basinc, sicaklik, altitude, nem,
                velocity, abs(pitch), # Dikey Hız, Eğim Açısı
                lat, lon,
                ayrilma1, ayrilma2, ucus_durumu
            )
            
            # CRC Hesapla
            crc = crc16_ccitt(payload)
            
            # Çerçeveyi Oluştur (Header + Payload + CRC)
            # YerIstasyonu2026.py parse_frame bekler: [SYNC1][SYNC2][LEN][PAYLOAD][CRC_HI][CRC_LO]
            frame = bytes([SYNC_1, SYNC_2, PACKET_SIZE]) + payload + struct.pack('>H', crc)
            
            # Ekrana Yazdır (Hex string olarak yer istasyonuna kopyalanabilir)
            print(f"[{time_elapsed:5.1f}s] Frame: {frame.hex().upper()}")
            
            time_elapsed += dt
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[!] Simülasyon durduruldu.")

if __name__ == "__main__":
    run_simulation()
