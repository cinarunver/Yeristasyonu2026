# Trakya Roket 2026 — Telemetri Protokolü Kılavuzu

Bu belge, **UcusYazilimi2026** (ESP32 Uçuş Bilgisayarı) ve **YerIstasyonu2026** arasındaki telemetri veri formatını açıklamaktadır.

> **⚠️ ÖNEMLİ DEĞIŞIKLIK:** Bu sistem artık CSV/String formatı **kullanmamaktadır**.  
> Veriler **ham binary (raw binary)** olarak `TelemetryPacket` struct formatında iletilir.  
> Yer istasyonu bu binary veriyi `struct.unpack()` ile ayrıştırmalıdır.

---

## 1. İletim Mimarisi

```
[ESP32 - Core 0]                     [ESP32 - Core 1]
  Sensör Okuma                          Haberleşme Görevi
  + Kalman Filtresi        Queue ──▶   Serial.write()  ──▶  PC/TTL (115200)
  + Uçuş Algoritması       (10 paket)  Serial1.write() ──▶  LoRa (9600)
  + TelemetryPacket doldur
```

| Kanal  | Arayüz   | Baud   | Kullanım                |
|--------|----------|--------|-------------------------|
| TTL    | UART0    | 115200 | PC / Yer İstasyonu      |
| LoRa   | UART1    | 9600   | Kablosuz uzak alım      |

**Gönderim yöntemi:** `Serial.write((uint8_t*)&packet, sizeof(packet))`  
**Frekans:** ~100 Hz (Core 0 döngü hızıyla senkronize)

---

## 2. TelemetryPacket Struct Tanımı

```cpp
#pragma pack(1)  // Hizalama boşlukları olmadan, sıkıştırılmış binary
struct TelemetryPacket {
    float    ivmeX;           // [0]  m/s²  — X ekseni ivme (yerçekimsiz)
    float    ivmeY;           // [1]  m/s²  — Y ekseni ivme (yerçekimsiz)
    float    ivmeZ;           // [2]  m/s²  — Z ekseni ivme (yerçekimsiz)
    float    gyroX;           // [3]  rad/s — X ekseni açısal hız
    float    gyroY;           // [4]  rad/s — Y ekseni açısal hız
    float    gyroZ;           // [5]  rad/s — Z ekseni açısal hız
    float    roll;            // [6]  derece — Euler roll açısı
    float    pitch;           // [7]  derece — Euler pitch açısı
    float    yaw;             // [8]  derece — Euler yaw açısı
    float    basinc;          // [9]  Pascal — Atmosferik basınç
    float    bmeSicaklik;     // [10] °C     — Hava sıcaklığı
    float    irtifa;          // [11] metre  — Ground-relative irtifa
    float    nem;             // [12] %      — Bağıl nem
    float    dikeyHiz;        // [13] m/s   — Dikey hız (yukarı = pozitif)
    float    eglimAcisi;      // [14] derece — Yerden eğim (0° = tam dik)
    float    gpsEnlem;        // [15] decimal degrees — GPS enlem
    float    gpsBoylam;       // [16] decimal degrees — GPS boylam
    bool     ayrilma1_durum;  // [17] bool  — 1. Fünye ateşlendi mi? (Drogue)
    bool     ayrilma2_durum;  // [18] bool  — 2. Fünye ateşlendi mi? (Ana paraşüt)
    uint8_t  ucus_durumu;     // [19] 0–4   — Aktif uçuş evresi (bkz. §4)
};
// Toplam boyut: 17×4 + 2×1 + 1×1 = 71 byte (#pragma pack(1) ile)
```

> **Not:** `#pragma pack(1)` direktifi **hem ESP32 tarafında hem Python tarafında** dikkate alınmalıdır.  
> Python'da `struct.calcsize(FORMAT)` ile boyutu doğrulayın.

---

## 3. Paket Boyutu ve Binary Layout

| Alan           | Tip       | Boyut  | Offset |
|----------------|-----------|--------|--------|
| ivmeX          | float32   | 4 byte | 0      |
| ivmeY          | float32   | 4 byte | 4      |
| ivmeZ          | float32   | 4 byte | 8      |
| gyroX          | float32   | 4 byte | 12     |
| gyroY          | float32   | 4 byte | 16     |
| gyroZ          | float32   | 4 byte | 20     |
| roll           | float32   | 4 byte | 24     |
| pitch          | float32   | 4 byte | 28     |
| yaw            | float32   | 4 byte | 32     |
| basinc         | float32   | 4 byte | 36     |
| bmeSicaklik    | float32   | 4 byte | 40     |
| irtifa         | float32   | 4 byte | 44     |
| nem            | float32   | 4 byte | 48     |
| dikeyHiz       | float32   | 4 byte | 52     |
| eglimAcisi     | float32   | 4 byte | 56     |
| gpsEnlem       | float32   | 4 byte | 60     |
| gpsBoylam      | float32   | 4 byte | 64     |
| ayrilma1_durum | bool/uint8 | 1 byte | 68    |
| ayrilma2_durum | bool/uint8 | 1 byte | 69    |
| ucus_durumu    | uint8      | 1 byte | 70    |
| **TOPLAM**     |            | **71 byte** |   |

---

## 4. Uçuş Durumu (ucus_durumu) Değerleri

| Değer | Sabit      | Açıklama                              |
|-------|------------|---------------------------------------|
| `0`   | HAZIR      | Kalkış bekleniyor                     |
| `1`   | YUKSELIYOR | İvme eşiği aşıldı, roket yükseliyor  |
| `2`   | INIS_1     | Apogee tespit edildi, Drogue açıldı   |
| `3`   | INIS_2     | 550m altına inildi, Ana paraşüt açıldı|
| `4`   | INDI       | Yere iniş tamamlandı, sistem pasif    |

---

## 5. Python Yer İstasyonu — Parse Yöntemi

### Format String

```python
import struct

# '<' = little-endian (ESP32 varsayılan byte sırası)
# 17f = 17 adet float (her biri 4 byte)
# 3B  = 3 adet unsigned byte (ayrilma1, ayrilma2, ucus_durumu)
PACKET_FORMAT = '<17f3B'
PACKET_SIZE   = struct.calcsize(PACKET_FORMAT)  # → 71 byte olmalı
```

### Tek Paket Okuma (Serial)

```python
import serial, struct

PACKET_FORMAT = '<17f3B'
PACKET_SIZE   = struct.calcsize(PACKET_FORMAT)

ser = serial.Serial('COM3', 115200, timeout=1)  # port ve baud ayarlayın

def read_packet(ser):
    raw = ser.read(PACKET_SIZE)
    if len(raw) != PACKET_SIZE:
        return None
    values = struct.unpack(PACKET_FORMAT, raw)
    return {
        'ivmeX':           values[0],
        'ivmeY':           values[1],
        'ivmeZ':           values[2],
        'gyroX':           values[3],
        'gyroY':           values[4],
        'gyroZ':           values[5],
        'roll':            values[6],
        'pitch':           values[7],
        'yaw':             values[8],
        'basinc':          values[9],
        'bmeSicaklik':     values[10],
        'irtifa':          values[11],
        'nem':             values[12],
        'dikeyHiz':        values[13],
        'eglimAcisi':      values[14],
        'gpsEnlem':        values[15],
        'gpsBoylam':       values[16],
        'ayrilma1_durum':  bool(values[17]),
        'ayrilma2_durum':  bool(values[18]),
        'ucus_durumu':     values[19],
    }
```

### ⚠️ Senkronizasyon Uyarısı

Binary akışta paket sınırları kayabilir (özellikle seri port açılışında buffer'da yarım paket olabilir). Senkronizasyon kaybını önlemek için `YerIstasyonu2026.py`'deki mevcut baud ve timeout ayarlarını koruyun; gerekirse `ser.read_until()` veya sabit boyut okuma ile senkronizasyon döngüsü ekleyin.

---

## 6. Arduino Tarafı — Gönderim Kodu

```cpp
// Core 1 - Task2code() içinde
TelemetryPacket packet;
if (xQueueReceive(telemetryQueue, &packet, portMAX_DELAY) == pdTRUE) {
    // TTL — PC / Yer İstasyonu
    Serial.write((uint8_t*)&packet, sizeof(packet));
    // LoRa — Kablosuz
    Serial1.write((uint8_t*)&packet, sizeof(packet));
}
```

> TX Buffer'lar 1024 byte olarak ayarlandığından `.write()` anlık döner;  
> UART donanımı gönderimi interrupt ile arka planda tamamlar.

---

## 7. Kalman Filtresi Parametreleri

Gönderilen tüm float değerler Kalman filtresiyle yumuşatılmıştır:

| Sensör Grubu          | Ölçüm Hatası | Tahmin Hatası | Süreç Gürültüsü |
|-----------------------|-------------|--------------|-----------------|
| İvme / Jiroskop / Euler | 0.1       | 0.1          | 0.01            |
| Basınç                | 2.0         | 2.0          | 0.1             |
| Sıcaklık              | 0.5         | 0.5          | 0.01            |
| İrtifa                | 1.5         | 1.5          | 0.1             |
| Nem                   | 1.0         | 1.0          | 0.1             |

---

## 8. GPS Koordinatları

- `gpsEnlem` ve `gpsBoylam` **float** olarak decimal degrees formatında iletilir.
- Yer istasyonunda harita gösterimi için doğrudan `Leaflet.js` koordinatı olarak kullanılabilir.
- GPS güncellenmemişse son geçerli değer korunur (TinyGPS++ `isUpdated()` kontrolü).

---

## 9. Sürüm Geçmişi

| Versiyon | Tarih      | Değişiklik                                           |
|----------|------------|------------------------------------------------------|
| v1.0     | 2026-04-20 | İlk sürüm — CSV/String format (`ROKET,...`)          |
| v2.0     | 2026-04-22 | **Breaking:** Ham binary `TelemetryPacket` struct'a geçiş, Kalman filtresi, state machine, Drogue/Ana paraşüt durumları eklendi |
