# Trakya Roket 2026 — Telemetri Protokolü Kılavuzu v3.0

Bu belge, **UcusYazilimi2026** (ESP32 Uçuş Bilgisayarı) ve **YerIstasyonu2026** arasındaki telemetri veri formatını açıklamaktadır.

> **v3.0 DEĞİŞİKLİĞİ:** Ham binary'e ek olarak artık **SYNC marker + CRC16-CCITT çerçevesi** kullanılmaktadır.  
> Yer istasyonu bu çerçeveyi SYNC arayarak senkronize olur, CRC ile bütünlüğü doğrular.

---

## 1. İletim Mimarisi

```
[ESP32 - Core 0]                        [ESP32 - Core 1]
  Sensör Okuma                             Haberleşme Görevi
  + Kalman Filtresi        Queue ──▶     gonder_paket_framed(Serial)  ──▶  TTL @ 115200 baud (~100 Hz)
  + Uçuş Algoritması       (10 paket)   gonder_paket_framed(Serial1) ──▶  E32-433T30D @ 9600 baud (~10 Hz)
  + TelemetryPacket doldur
```

| Kanal | Modül        | Arayüz | Baud   | Gönderim Hızı | Kullanım              |
|-------|--------------|--------|--------|---------------|-----------------------|
| TTL   | UART0        | Serial | 115200 | ~100 Hz       | PC / Yer İstasyonu    |
| LoRa  | E32-433T30D  | UART1  | 9600   | ~10 Hz        | Kablosuz uzak alım    |

> **E32-433T30D Notu:** SX1278 tabanlı, 433 MHz, 30 dBm. Transparent mod — UART'a yazılan baytlar doğrudan RF olarak iletilir. Modül kendi RF katmanında LoRa FEC/CRC yapar; uygulama katmanı CRC'si UART framing güvenliği içindir.

---

## 2. Çerçeve Formatı (Framed Packet)

```
┌──────────┬──────────┬──────────┬─────────────────────────┬───────────────┐
│ SYNC1[1B]│ SYNC2[1B]│ LEN [1B] │  TelemetryPacket [71B]  │ CRC16  [2B]  │
│  0xAA    │  0x55    │   71     │  (pragma pack 1 binary) │ HI    LO     │
└──────────┴──────────┴──────────┴─────────────────────────┴───────────────┘
Toplam: 2 + 1 + 71 + 2 = 76 byte/çerçeve
```

### CRC16-CCITT

- **Polinom:** 0x1021  
- **Başlangıç değeri:** 0xFFFF  
- **Hesaplama kapsamı:** Yalnızca 71 byte `TelemetryPacket` payload'ı (SYNC ve LEN dahil değil)

```python
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc
```

---

## 3. TelemetryPacket Struct Tanımı

```cpp
#pragma pack(push, 1)
struct TelemetryPacket {
    float    ivmeX;           // [0]  m/s²  — X ekseni ivme (yerçekimsiz, Kalman)
    float    ivmeY;           // [1]  m/s²  — Y ekseni ivme
    float    ivmeZ;           // [2]  m/s²  — Z ekseni ivme (kalkış tespiti için)
    float    gyroX;           // [3]  rad/s — X açısal hız (Kalman)
    float    gyroY;           // [4]  rad/s — Y açısal hız
    float    gyroZ;           // [5]  rad/s — Z açısal hız
    float    roll;            // [6]  derece — Euler roll (Kalman)
    float    pitch;           // [7]  derece — Euler pitch
    float    yaw;             // [8]  derece — Euler yaw
    float    irtifa;          // [11] metre  — Ground-relative irtifa (Kalman)
    float    dikeyHiz;        // [13] m/s    — Dikey hız (irtifadan türev)
    float    eglimAcisi;      // [14] derece — Yerden eğim (0° = tam dik)
    float    gpsEnlem;        // [15] decimal degrees
    float    gpsBoylam;       // [16] decimal degrees
    bool     ayrilma1_durum;  // [17] Drogue fünye ateşlendi mi?
    bool     ayrilma2_durum;  // [18] Ana paraşüt fünye ateşlendi mi?
    uint8_t  ucus_durumu;     // [19] 0–4 (bkz. §5)
};
#pragma pack(pop)
// Toplam: 17×4 + 2×1 + 1×1 = 71 byte
```

---

## 4. Binary Layout (Offset Tablosu)

| Alan           | Tip        | Boyut  | Offset |
|----------------|------------|--------|--------|
| ivmeX          | float32 LE | 4      | 0      |
| ivmeY          | float32 LE | 4      | 4      |
| ivmeZ          | float32 LE | 4      | 8      |
| gyroX          | float32 LE | 4      | 12     |
| gyroY          | float32 LE | 4      | 16     |
| gyroZ          | float32 LE | 4      | 20     |
| roll           | float32 LE | 4      | 24     |
| pitch          | float32 LE | 4      | 28     |
| yaw            | float32 LE | 4      | 32     |
| irtifa         | float32 LE | 4      | 36     |
| dikeyHiz       | float32 LE | 4      | 40     |
| eglimAcisi     | float32 LE | 4      | 44     |
| gpsEnlem       | float32 LE | 4      | 48     |
| gpsBoylam      | float32 LE | 4      | 52     |
| ayrilma1_durum | uint8      | 1      | 56     |
| ayrilma2_durum | uint8      | 1      | 57     |
| ucus_durumu    | uint8      | 1      | 58     |
| **PAYLOAD TOPLAM** |        | **59** |        |

---

## 5. Uçuş Durumu Değerleri

| Değer | Sabit      | Açıklama                                |
|-------|------------|-----------------------------------------|
| `0`   | HAZIR      | Kalkış bekleniyor                       |
| `1`   | YUKSELIYOR | İvme eşiği aşıldı, yükseliyor          |
| `2`   | INIS_1     | Apogee tespit edildi, Drogue açıldı     |
| `3`   | INIS_2     | 550m altına inildi, Ana paraşüt açıldı  |
| `4`   | INDI       | Yere iniş tamamlandı, sistem pasif      |

---

## 6. Python Yer İstasyonu — Parse Yöntemi

### Sabitler

```python
import struct

PACKET_FORMAT = '<17f3B'                       # little-endian
PACKET_SIZE   = struct.calcsize(PACKET_FORMAT) # 59 byte
SYNC_1, SYNC_2 = 0xAA, 0x55
FRAME_SIZE    = 2 + 1 + PACKET_SIZE + 2        # 64 byte
```

### Çerçeve Ayrıştırma

```python
def parse_frame(raw: bytes):
    if len(raw) != FRAME_SIZE: return None
    if raw[0] != SYNC_1 or raw[1] != SYNC_2: return None
    if raw[2] != PACKET_SIZE: return None
    payload  = raw[3:3 + PACKET_SIZE]
    crc_recv = (raw[-2] << 8) | raw[-1]
    if crc16_ccitt(payload) != crc_recv: return None
    v = struct.unpack(PACKET_FORMAT, payload)
    return { 'ivmeX': v[0], ..., 'ucus_durumu': v[19] }
```

### Senkronize Okuma (Serial)

```python
buf = bytearray()
while running:
    buf.extend(ser.read(ser.in_waiting or 1))
    if len(buf) > FRAME_SIZE * 10:
        buf = bytearray()  # overflow koruması
    while len(buf) >= FRAME_SIZE:
        # SYNC ara
        idx = next((i for i in range(len(buf)-1)
                    if buf[i]==SYNC_1 and buf[i+1]==SYNC_2), -1)
        if idx == -1: buf = buf[-1:]; break
        buf = buf[idx:]
        if len(buf) < FRAME_SIZE: break
        packet = parse_frame(bytes(buf[:FRAME_SIZE]))
        buf = buf[FRAME_SIZE:]
        if packet: process(packet)
```

---

## 7. ESP32 Gönderim Kodu (Core 1)

```cpp
// gonder_paket_framed(port, packet) fonksiyonu:
// [0xAA][0x55][71][...TelemetryPacket...][CRC16_HI][CRC16_LO]

void Task2code(void *pvParameters) {
    TelemetryPacket pkt;
    uint32_t lora_sayac = 0;
    for (;;) {
        if (xQueueReceive(telemetryQueue, &pkt, portMAX_DELAY) == pdTRUE) {
            gonder_paket_framed(Serial,  pkt);          // TTL → her paket
            if (++lora_sayac >= LORA_GONDERIM_ORANI) {
                gonder_paket_framed(Serial1, pkt);      // LoRa → her 10. paket
                lora_sayac = 0;
            }
        }
    }
}
```

---

## 8. Kalman Filtresi Parametreleri

| Sensör Grubu            | Ölçüm Hatası | Tahmin Hatası | Süreç Gürültüsü |
|-------------------------|-------------|--------------|-----------------|
| İvme / Jiroskop / Euler | 0.1         | 0.1          | 0.01            |
| İrtifa                  | 1.5         | 1.5          | 0.1             |

---

## 9. Sürüm Geçmişi

| Versiyon | Tarih      | Değişiklik                                                              |
|----------|------------|-------------------------------------------------------------------------|
| v1.0     | 2026-04-20 | İlk sürüm — CSV/String format (`ROKET,...`)                             |
| v2.0     | 2026-04-22 | Ham binary `TelemetryPacket` struct, Kalman, state machine              |
| v3.0     | 2026-04-25 | **Framed Protocol:** SYNC(0xAA 0x55) + CRC16-CCITT, E32-433T30D desteği, LoRa 10 Hz rate limiting |
| v4.0     | 2026-04-27 | **Framed Protocol:** SYNC(0xAA 0x55) + CRC16-CCITT, E32-433T30D desteği, LoRa 10 Hz
