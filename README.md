# Yer İstasyonu 2026 — Trakya Roket

TEKNOFEST Roket Yarışması için geliştirilen **yer istasyonu yazılımı**. Roket
(UKB) ve görev yükünden (BGY) LoRa üzerinden gelen telemetriyi çözer, canlı
gösterir, haritada izler, 3B yönelim/yörünge olarak çizer ve CSV'ye kaydeder.

PyQt6 + pyqtgraph + OpenGL ile yazılmıştır; masaüstü (Windows / macOS / Linux)
üzerinde tek dosya olarak çalışır.

```
┌──────────────┐  E32-433T30D LoRa @9600, ~10 Hz   ┌────────────────┐
│  ROKET (UKB) │ ────────────────────────────────► │                │
│   ESP32      │        [AA 55][LEN=23][...]       │  YER İSTASYONU │
└──────────────┘                                   │   (bu proje)   │
┌──────────────┐  E32-433T30D LoRa @9600, ~10 Hz   │                │
│ GÖREV YÜKÜ   │ ────────────────────────────────► │  2 ayrı COM    │
│   ESP32      │        [AA 55][LEN=32][...]       │     portu      │
└──────────────┘                                   └────────────────┘
```

---

## Özellikler

| | |
|---|---|
| **Çift sistem** | Roket ve görev yükü **ayrı COM portlarından** eş zamanlı bağlanır, her biri kendi QThread'inde okunur — arayüz donmaz. |
| **Çerçeveli binary protokol** | `[0xAA][0x55][LEN][payload][CRC16-CCITT]`. CRC hatasında çerçeve atılmaz, yalnızca SYNC atlanır → kaymış gerçek çerçeve yakalanır. Paket OK/hata sayaçları canlı görünür. |
| **Canlı telemetri paneli** | İrtifa, dikey hız, eğim, bileşke ivme, yönelim (R,P,Y), GPS, ayrılma bayrakları, uçuş durumu; görev yükünde ek olarak basınç/sıcaklık/nem/hava yoğunluğu/gyro. |
| **Sensör grafikleri** | Kinematik (irtifa + hız + ivme, ikincil eksen) ve çevresel (sıcaklık + nem + basınç, ikincil eksen) grafikler, `deque(maxlen=10000)` tamponla. |
| **Canlı GPS haritası** | Leaflet; **assets/ içinde yerel** leaflet.js/css. Karanlık / topo / uydu katmanı seçilebilir, yer istasyonu (atış alanı) konumu işaretli, 2 Hz throttle. |
| **Kurtarma pusulası** | Haritanın sol-alt köşesinde **roket ve görev yükü için ayrı** kadran: yer istasyonundan hedefe yön (`KD 45°`) + mesafe (`1.20 km`). Kadran kuzeye sabit — elde pusulayı kuzeye hizala, ok nereyi gösteriyorsa oraya yürü. |
| **3B Aviyonik** | İki bağımsız panel: **roket** → quaternion ile sürülen 3B model + yapay ufuk + yaw pusulası + dönüş hızı barları; **görev yükü** → 3B parametrik uçuş yörüngesi (fare ile döndür/zoom). Gimbal lock'tan kaçınmak için 3B illüstrasyon Euler'e hiç uğramaz. |
| **CSV loglama** | Roket ve görev yükü **ayrı şemaya sahip → ayrı dosyaya** yazılır (`<taban>_ukb.csv`, `<taban>_gorevyuku.csv`). Alan başına ayarlı ondalık hassasiyet (GPS 7 hane ≈ 1 cm). Raw (hex) log modu da var. |
| **Dahili simülatör** | Donanım olmadan test için port listesinden `Simülatör` seçilir. Gerçek fizik + gerçekçi gürültü üretir ve **gerçek uçuşla aynı parse/3B yolundan** geçer. |

---

## Kurulum

Python 3.10+ önerilir.

```bash
git clone https://github.com/cinarunver/Yeristasyonu2026.git
cd Yeristasyonu2026

python3 -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate

pip install -r requirements.txt
```

Bağımlılıklar: `pyserial`, `PyQt6`, `PyQt6-WebEngine`, `pyqtgraph`, `numpy`, `PyOpenGL`

### CARTO anahtarı (Koyu / Voyager katmanları için)

CARTO basemap'leri anahtarsız istendiğinde tile'ı `HTTP 200` ile **ama üzerinde
"API KEY REQUIRED" filigranıyla** döndürür — yani sessizce bozulur. Anahtar
koda gömülmez; şu sırayla aranır:

1. `CARTO_API_KEY` ortam değişkeni
2. proje kökündeki `carto_key.txt` (`.gitignore`'da)

```bash
echo "SENIN_ANAHTARIN" > carto_key.txt
```

Anahtar yoksa uygulama yine çalışır: CARTO katmanları filigranlı gelir,
**uydu katmanı etkilenmez** (Esri + offline tile).

### Offline harita (saha için)

```bash
python3 tile_indir.py --tahmin --kademeli   # önce ne kadar sürecek/tutacak gör
python3 tile_indir.py --kademeli            # indir (~24 bin tile, ~285 MB)
```

Sahada **Uydu Haritası (offline)** katmanını seç — internetsiz çalışan tek katman odur.

> **Not:** `kalman_monitor.py` ayrı bir araçtır ve **PyQt5** kullanır. Yalnızca onu
> çalıştıracaksanız ek olarak `pip install PyQt5` gerekir.

---

## Kullanım

```bash
python3 YerIstasyonu2026.py
```

1. **🔄 Tüm COM Portlarını Yenile** ile portları tara.
2. Roket ve görev yükü için port seç → baud **9600** (E32 LoRa alıcısı varsayılanı).
3. **🚀 Roketi Bağla** / **🛰️ Görev Yükünü Bağla**.
4. Log için: **Gözat…** ile dosya taban adını seç → format **Parsed (CSV)** →
   kaydı başlat.
5. Donanım yoksa port listesinden **Simülatör**'ü seç.

Sekmeler: `📈 Sensör Grafikleri` · `🗺️ Canlı GPS Haritası` · `🎯 3D Aviyonik`

---

## Telemetri Protokolü

Firmware tarafında float değerler **fixed-point** olarak paketlenir (paket
küçültme); yer istasyonu aşağıdaki ölçeklerle geri çevirir.

### Çerçeve

```
[0xAA][0x55][LEN][payload: LEN byte][CRC16_HI][CRC16_LO]
```

CRC16-CCITT (`poly=0x1021`, `init=0xFFFF`), yalnızca payload üzerinden.
Paket tipi **LEN byte'ından** ayrılır: roket `23`, görev yükü `32`.

### Roket — `TelemetryWire`, 23 B, çerçeve 28 B

Kaynak: `UcusYazilimi/src/main.cpp` · format `'<7h2iB'`

| Alan | Tip | Ölçek |
|---|---|---|
| ivmeToplam | int16 | ÷100 |
| qx, qy, qz | 3× int16 | ÷10000 |
| irtifa | int16 | ÷10 |
| dikeyHiz | int16 | ÷10 |
| eğim | int16 | ÷100 |
| gpsEnlem, gpsBoylam | 2× int32 | ÷1e7 |
| durum | uint8 bitfield | bit0 ayrılma1 · bit1 ayrılma2 · bit2-4 uçuş durumu |

### Görev Yükü — `GorevYukuWire`, 32 B, çerçeve 37 B

Kaynak: `UcusYazilimi/GorevYukuYazilimi/gorevyuku.cpp` · format `'<HhHhH2i7h'`

| Alan | Tip | Ölçek |
|---|---|---|
| basınç (hPa) | uint16 | ÷10 |
| sıcaklık | int16 | ÷100 |
| nem | uint16 | ÷100 |
| irtifa | int16 | ÷10 |
| hava yoğunluğu (kg/m³) | uint16 | ÷1000 |
| gpsEnlem, gpsBoylam | 2× int32 | ÷1e7 |
| ivmeToplam | int16 | ÷100 |
| qx, qy, qz | 3× int16 | ÷10000 |
| gyroX, gyroY, gyroZ | 3× int16 | ÷10 |

Görev yükü **uçuş durumu göndermez** (bkz. `gorevyuku.cpp`), bu alan arayüzde yok.

### Havadan gelmeyen veriler

Bant genişliğini korumak için bazı değerler yayınlanmaz; **uçuş
bilgisayarlarının SD kartında tam çözünürlükte** kayıtlıdır:

- **Ham ivme eksenleri (X/Y/Z)** — yerine tek slotta bileşke büyüklük
  `sqrt(x²+y²+z²)` gelir.
- **Roket gyro'su** (görev yükü gyro'su gelir).
- **Tetens ara değerleri** (`es`, `pv`) — hava yoğunluğu firmware'de nemli hava
  formülüyle hesaplanıp hazır gönderilir.

### Quaternion → Euler

Firmware `w ≥ 0` garanti eder (aksi halde üç bileşenin işaretini çevirir), bu
yüzden `w` birim kuaterniyon koşulundan tek anlamlı geri hesaplanır:

```
w = sqrt(1 - (x² + y² + z²))
```

Roll/Pitch/Yaw yalnızca **gösterge ve CSV** için türetilir.

### Uçuş durumları

| Kod | Durum |
|---|---|
| 0 | HAZIR |
| 1 | YÜKSELİYOR |
| 2 | İNİŞ_1 (Drogue) |
| 3 | İNİŞ_2 (Ana Paraşüt) |
| 4 | İNDİ ✓ |

---

## Dosyalar

| Dosya | Açıklama |
|---|---|
| [YerIstasyonu2026.py](YerIstasyonu2026.py) | Ana yer istasyonu uygulaması (parser + arayüz + 3B + loglama + simülatör). |
| [tile_indir.py](tile_indir.py) | **Offline harita tile indirici** — atış alanı çevresinin uydu görüntüsünü `assets/tiles/` altına indirir. `--test` (10 km), `--kademeli` (önerilen), `--tahmin` (sadece hesapla). Kesilirse kaldığı yerden devam eder. |
| [hz_olcer.py](hz_olcer.py) | Bağımsız, tek dosyalık **LoRa telemetri Hz ölçer**. Geçerli çerçeve/sn ve CRC-hata sayacı basar. |
| [kalman_monitor.py](kalman_monitor.py) | **Kalman filtre monitörü** — BNO055/BME280 için HAM vs KAL karşılaştırma arayüzü (USB seri, metin blok formatı, PyQt5). |
| [gercekci_similasyon.py](gercekci_similasyon.py) | Harici uçuş simülatörü *(bkz. bilinen sorunlar)*. |
| [map_internal.html](map_internal.html) | Harita sayfası (çalışma anında üretilir, git'te izlenmez). |
| [assets/](assets/) | Yerel Leaflet js/css + marker ikonları (+ `tiles/` offline uydu önbelleği). |
| `carto_key.txt` | CARTO basemap API anahtarı — **git'e girmez**, her makinede ayrıca oluşturulur (bkz. Kurulum). |
| [YerIstasyonu2026.spec](YerIstasyonu2026.spec) | PyInstaller yapılandırması (Windows / Linux / macOS ortak). |
| [.github/workflows/build.yml](.github/workflows/build.yml) | GitHub Actions CI/CD — üç platformda derler, test eder, sürüm yayınlar. |

## Derleme (CI/CD)

Her `main` push'unda ve PR'da GitHub Actions üç hedefte paket üretir:
**Windows x64**, **Linux x64**, **macOS Apple Silicon**.

> Intel Mac paketi üretilmiyor: GitHub'ın `macos-13` (Intel) runner kuyruğu
> çok uzun. Apple Silicon paketi Intel Mac'te **çalışmaz**; gerekirse
> `.github/workflows/build.yml` içindeki matrise `macos-13` geri eklenir.

Derlenen paketler ilgili çalışmanın **Artifacts** bölümünden indirilir
(30 gün saklanır). Sürüm yayınlamak için etiket atmak yeterli:

```bash
git tag v1.0.0 && git push origin v1.0.0
```

Bu, üç paketi de zip'leyip GitHub Release'e ekler.

**Dumansız test.** CI yalnızca "dosya oluştu mu" diye bakmaz; üretilen ikiliyi
`--check` ile gerçekten çalıştırır. Qt yüklenir, tüm sekmeler (3B + WebEngine
haritası dahil) kurulur ve harita kaynaklarının pakete girdiği doğrulanır.
Eksik varsa derleme kırmızıya döner. Yerelde de çalıştırılabilir:

```bash
python3 YerIstasyonu2026.py --check
```

**Yerel paketleme:**

```bash
pip install pyinstaller
pyinstaller --noconfirm --clean YerIstasyonu2026.spec
# -> dist/YerIstasyonu2026/  (macOS'ta ayrıca dist/YerIstasyonu2026.app)
```

> `assets/tiles/` (~300 MB) git'e girmediği için CI paketlerinde **offline
> tile'lar bulunmaz**; harita çevrimiçi Esri katmanına düşer. Sahada offline
> harita gerekiyorsa `python3 tile_indir.py --kademeli` çalıştırıp paketi
> yerelde üretin.

---

### hz_olcer.py

```bash
python3 hz_olcer.py                              # portları listeler ve sorar
python3 hz_olcer.py /dev/tty.usbserial-0001      # doğrudan
python3 hz_olcer.py COM5 9600
```

Her saniye: `saat | toplam Hz | roket Hz | görevY Hz | CRC-hata/sn`

---

## Bilinen Sorunlar / Yapılacaklar

- [ ] **`gercekci_similasyon.py` protokolü eski.** Hâlâ `'<14f3B'` (59 B / çerçeve
      64 B) üretiyor; güncel protokol roket için `'<7h2iB'` (23 B / 28 B).
      Bu dosya şu an yer istasyonuyla **uyumsuz** — bunun yerine uygulamanın
      **dahili simülatörünü** (port listesinden `Simülatör`) kullanın.
- [ ] **HYİ (Hakem Yer İstasyonu) entegrasyonu** — TEKNOFEST hakem paketi formatı
      + ayrı COM port üzerinden gönderim (başlanmadı).
- [x] ~~**Offline harita**~~ — **uydu katmanı için yapıldı** (2026-09-04):
      `tile_indir.py` ile tile'lar `assets/tiles/` altına indirilir, harita önce
      oraya bakar, yoksa CDN'e düşer. Koyu/topo katmanları hâlâ CDN'e bağımlı —
      sahada **uydu katmanını** seç.
- [ ] **"Veri kesildi" uyarısı** — son paketten bu yana X saniye geçtiyse görsel
      ikaz.

## Sürüm Notları

**2026-09-04 — CARTO anahtarı + Voyager katmanı**
- CARTO basemap'leri anahtarsız iken tile'ları **"API KEY REQUIRED" filigranıyla**
  (HTTP 200 ile, yani sessizce) döndürüyordu. Anahtar desteği eklendi:
  `CARTO_API_KEY` ortam değişkeni veya `carto_key.txt` — **koda gömülmez,
  git'e girmez**. Anahtar yoksa uygulama çalışmaya devam eder, yalnız CARTO
  katmanları filigranlı olur; uydu katmanı etkilenmez.
- **Voyager** katmanı eklendi (yol/yer adları okunaklı, `maxZoom` 20).
- Katman adları netleştirildi: *Uydu Haritası (offline)* — sahada seçilecek olan.

**2026-09-04 — offline harita (uydu)**
- `tile_indir.py` eklendi: Esri World Imagery tile'larını `assets/tiles/{z}/{x}/{y}.jpg`
  düzeninde indirir. Paralel (8 iş parçacığı), hız sınırlı, 3 denemeli, kesintiden
  sonra kaldığı yerden devam eder (indirilmiş tile atlanır).
- Uydu katmanı **offline-öncelikli** hale getirildi: önce `assets/tiles/` okunur,
  tile yoksa Esri CDN'ine düşülür. İnternet varken de yokken de çalışır.
- Ölçüm: uydu tile ortalama **12.9 KB**. 10 km z12-16 = 2.470 tile / 28 MB;
  kademeli (40km z12-14 + 15km z15-17) = 23.544 tile / ~300 MB.
- `assets/tiles/` ve `map_internal.html` git takibinden çıkarıldı.

**2026-09-04 — atış alanı koordinatı + kurtarma pusulası**
- Yer istasyonu konumu **atış alanına** alındı: `38.401831, 33.704852`
  (önceki değer Mühendislik binasıydı). Harita açılışta buraya odaklanır ve
  pusula mesafe/yön hesabı bu noktayı referans alır.
- Haritanın sol-alt köşesine **roket ve görev yükü için ayrı pusula kadranı**
  eklendi: yön (`KD 45°`) + mesafe (`1.20 km`). Kadran kuzeye sabittir — elde
  pusula kuzeye hizalanıp okun gösterdiği yöne yürünür. Yön **gerçek kuzeye**
  göredir (manyetik deklinasyon Türkiye'de ~+5-6° doğu). Mevcut 2 Hz harita
  akışından beslenir, ek paket alanı veya protokol değişikliği gerektirmez.

**2026-09-04 — taşma korumasında sessiz paket kaybı düzeltildi**
- 🐞 Taşma koruması (`buf > frame_size*20` → son 2 çerçeveye kırp) ayrıştırma
  döngüsünden **önce** çalışıyordu. Eşik ~2 saniyelik telemetriye denk geliyor;
  arayüz o kadar süre takılırsa (ağır harita/3B render, GC duraklaması) birikmiş
  ama **tamamen çözülebilir** ~23 paket ayrıştırılmadan siliniyordu. Kayıp
  `paket_hata` sayacına da yansımadığından arayüzde iz bırakmıyor, apoje/ayrılma
  anına denk gelirse o kritik saniyeler kaybolabiliyordu. Kırpma ayrıştırmadan
  **sonraya** alındı — döngü çözülebilen her çerçeveyi zaten tükettiği için
  geriye yalnızca kısmi çerçeve veya SYNC'siz çöp kalır, dolayısıyla artık asla
  geçerli paket düşmez. Bellek üst sınırı korunuyor.

**v3.2 — 2026-07-29 (hava yoğunluğu + gerçek zamanlı yönelim illüstrasyonu)**
- 🐞 **Kritik parse hatası düzeltildi:** roket paketinin quaternion'ı Euler
  sanılıyordu. Format `'<3hH3h2iB'` ile `v[1..3]` roll/pitch/yaw (×100) olarak
  çözülüyordu; firmware o slotlara `qx,qy,qz` (×10000) basıyor. Boyut tesadüfen
  eşit (23 B) olduğundan **CRC tutuyor ve paket geçerli görünüyordu**, ama
  `qx=0.35` ekranda `roll=35.00°` yazıyor, işaretli `qz` unsigned okunduğu için
  yaw 600°+ saçma değerler alıyordu. Format `'<7h2iB'` yapıldı.
- Görev yükü paketi 24 B → 32 B: hava yoğunluğu + yönelim quaternion'ı eklendi.
- 3B sekmesi iki bağımsız yönelim paneline ayrıldı (roket | görev yükü).

**2026-07-09 — firmware ile birebir hizalama**
- Görev yükü paketi gerçek firmware'e göre kesinleştirildi; hayali `'<15fB'`
  (IMU + uçuş durumu) formatı geri alındı — donanımla hiç eşleşmiyordu (LEN=24
  vs 61). Nem (%) grafiği eklendi, eksen etiketleri ve ikincil eksen legend'ları
  düzeltildi.

**2026-07-02 — düzeltme turu**
- Payload protokolü gerçek firmware'e uyarlandı (eski 71 B format uyumsuzdu).
- CRC hatasında 2 bayt ilerleme (çerçeve kayması düzeltildi) + paket OK/hata
  sayaçları arayüze eklendi.
- Terminal (10 Hz) ve harita (2 Hz) güncellemeleri throttle edildi.
- `stop()` thread-güvenli hale getirildi; buffer taşma koruması kuyruğu koruyor.

---

## Lisans

[GNU GPL v3](LICENSE)
