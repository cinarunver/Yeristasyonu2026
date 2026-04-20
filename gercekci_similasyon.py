import time
import math
import random

def run_simulation():
    # Temel Fiziksel Değerler
    time_elapsed = 0.0
    dt = 0.1 # Saniyede 10 veri (10 Hz)
    
    altitude = 0.0
    velocity = 0.0
    acceleration = 1.0 # Dünya yerçekimi 1G
    
    # Açısal Değerler
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    
    # Başlangıç GPS (Örnek: Tuz Gölü)
    lat = 38.83510
    lon = 33.39320
    
    state = "BEKLEMEDE"
    
    print("🚀 Gerçekçi Roket Uçuş Simülatörü Başlıyor (10 Hz)...")
    print("Çıkış için CTRL+C tuşlarına basabilirsiniz.\n")
    print("-" * 70)
    
    try:
        while True:
            # FİZİK MOTORU SİMÜLASYONU
            if time_elapsed < 5.0:
                # 1. Aşama: Rampada bekleme
                state = "RAMPADA"
                altitude = 0.0
                velocity = 0.0
                acceleration = 1.0 + random.uniform(-0.01, 0.01)
                
            elif time_elapsed < 18.0:
                # 2. Aşama: Motor Ateşlemesi (Sert ivmelenme)
                state = "MOTOR_ATESLENDI"
                acceleration = 12.5 + random.uniform(-0.5, 0.5) # 12.5g itki
                velocity += (acceleration * 9.81) * dt
                altitude += velocity * dt
                
                # GPS Hızla güncellenir
                lat += 0.00008
                lon += 0.00003
                
            elif time_elapsed < 35.0:
                # 3. Aşama: Motor Susması / Serbest Uçuş (Sürtünme ile yavaşlama)
                state = "SERBEST_UCUS_(COAST)"
                acceleration = -0.8 + random.uniform(-0.1, 0.1) # Serbest düşüş / sürtünme
                velocity += (acceleration * 9.81) * dt
                
                # Hız sıfırın altına düşerse tepe noktası aşılmış demektir
                altitude += velocity * dt
                if altitude < 0: altitude = 0
                
                lat += 0.00004
                lon += 0.00001
                
            elif time_elapsed < 65.0:
                # 4. Aşama: Paraşüt Açılması (Sabit limit hızda düşüş)
                state = "PARASUT_ACILDI"
                acceleration = 0.0 + random.uniform(-0.05, 0.05)
                velocity = -8.0 + random.uniform(-0.2, 0.2) # Saniyede 8m sabit düşüş
                altitude += velocity * dt
                
                if altitude <= 0:
                    altitude = 0.0
                    state = "KURTARMA_BEKLENIYOR"
                    
                # Rüzgarla sürüklenme
                lat -= 0.00001
                lon += 0.00002
                
            else:
                # 5. Aşama: İniş Tamamlandı
                state = "YERDE"
                altitude = 0.0
                velocity = 0.0
                acceleration = 1.0

            # HAVADA SARSINTI SİMÜLASYONU (Smooth Sine Waves + Biraz Gürültü)
            if state in ["MOTOR_ATESLENDI", "SERBEST_UCUS_(COAST)"]:
                roll = math.sin(time_elapsed * 2.0) * 15.0 + random.uniform(-2, 2)
                pitch = math.sin(time_elapsed) * 5.0 + random.uniform(-1, 1)
                yaw = (yaw + 5.0 * dt) % 360  # Yavaşça kendi ekseninde dönme
            elif state == "PARASUT_ACILDI":
                roll = math.sin(time_elapsed) * 45.0 # Paraşütte beşik gibi sallanma
                pitch = math.cos(time_elapsed) * 10.0
                yaw = (yaw + 2.0 * dt) % 360
            else:
                roll = 0.0; pitch = 0.0; yaw = 0.0
                
            # VERİYİ PAKETLE (Araya | koyacağız GPS için)
            gps_str = f"{lat:.5f}|{lon:.5f}"
            
            # Format: ROKET,İrtifa,Hız,İvme,Durum,Enlem|Boylam,Roll,Pitch,Yaw
            packet = f"ROKET,{altitude:.1f},{velocity:.1f},{acceleration:.2f},{state},{gps_str},{roll:.1f},{pitch:.1f},{yaw:.1f}"
            
            print(packet)
            
            # Zamanı ilerlet
            time_elapsed += dt
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("\n[!] Simülasyon kullanıcı tarafından durduruldu.")

if __name__ == "__main__":
    run_simulation()
