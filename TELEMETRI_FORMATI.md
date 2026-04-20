# Trakya Roket 2026 - Telemetri ve Veri Ayrıştırıcı (Parser) Kılavuzu

Bu belge, **Roket** ve **Görev Yükü** (Payload) üzerinden haberleşme modüllenize (LoRa, XBee vb.) yazacağınız Arduino/C++ kodunun **Yer İstasyonu V2.0** uygulamasına verileri hangi standartta göndermesi gerektiğini açıklamaktadır.

Yer istasyonunda tasarladığımız Veri Ayrıştırıcı (Parser) metodu, verileri **virgül (`,`)** karakterine göre böler. Bu sebeple veriler arasında sadece virgül olmalıdır. (Ondalıklı sayılarda virgül DEĞİL, nokta `.` kullanmalısınız örn: `41.5` gibi). 

---

## 🚀 1. ROKET Veri Paketi Formatı

Roket uygulamasından saniyede X frekansla (örn: 10 Hz veya 20 Hz) basılacak `.print()` diziliminiz aşağıdaki sırayı **kesinlikle** takip etmelidir:

```text
ROKET,İrtifa,Hız,İvme,Durum,Enlem|Boylam,Roll,Pitch,Yaw
```

### Örnek (Arduino `Serial.println` Çıktısı):
`ROKET,1250.5,120.2,1.55,UCUSTA,38.8351|33.3932,15.2,-4.0,90.5`

### Parser İndeks Haritası (Yer İstasyonu Karşılığı):
* **Bölüm 0 (Prefix):** `ROKET` *(Uygulama ROKET yazısını gördüğünde bunun roket verisi olduğunu anlar, eğer yoksa doğrudan sayıdan okumaya başlar ama yazılması güvenlidir).*
* **Bölüm 1:** İrtifa / Yükseklik (Float / metre)
* **Bölüm 2:** Hız (Float / m/s)
* **Bölüm 3:** İvme (Float / g)
* **Bölüm 4:** Durum Mesajı (String - Örn: "Uçuşta", "Ayrıldı", "Paraşüt")
* **Bölüm 5:** GPS / Harita (String)
  * **[!] KRİTİK BİLGİ:** Enlem ve Boylam arasında KESİNLİKLE virgül kullanılmamalıdır! Virgül kullanırsanız paketiniz yanlış parçalanır. Aralarına Dik Çizgi `|` veya Noktalı Virgül `;` koymalısınız. *(Örn: `41.2|28.5`)*
* **Bölüm 6:** Roll Açısı (Float / Derece)
* **Bölüm 7:** Pitch Açısı (Float / Derece)
* **Bölüm 8:** Yaw Açısı (Float / Derece) *(Not: 6,7,8. kısımlar 3D Aviyonik ekranını hareket ettirir.)*

---

## 🛰️ 2. GÖREV YÜKÜ Veri Paketi Formatı

Cansat veya Payload tarafından gelecek olan verilerdeki sıralama da tamamen kendisine özgüdür.

```text
YUK,İrtifa,Sıcaklık,Basınç,Durum,Nem,Enlem|Boylam
```

### Örnek (Arduino `Serial.println` Çıktısı):
`YUK,1200.0,24.5,101325,AYRILDI,45.2,38.8349|33.3930`

### Parser İndeks Haritası (Yer İstasyonu Karşılığı):
* **Bölüm 0 (Prefix):** `YUK` veya `GOREVYUKU` *(Uygulama bu ön ekten veriyi filtreler)*
* **Bölüm 1:** İrtifa (Float / metre)
* **Bölüm 2:** Sıcaklık (Float / Derece)
* **Bölüm 3:** Basınç (Float / Pascal veya hPa)
* **Bölüm 4:** Durum Mesajı (String - Örn: "Uyku", "Ayrıldı", "İnişte")
* **Bölüm 5:** Nem (Float / Yüzdelik %)
* **Bölüm 6:** GPS / Harita (String)
  * Aynı kural geçerli, virgül yerine `|` işareti (Örn: `41.2|28.5`). Turuncu işaretçiyi oynatır.

## 💡 Arduino İçin Örnek Kod (Tavsiye Edilen)
```cpp
void sendRocketTelemetry() {
    Serial.print("ROKET,");
    Serial.print(getAltitude()); Serial.print(",");
    Serial.print(getVelocity()); Serial.print(",");
    Serial.print(getAcceleration()); Serial.print(",");
    Serial.print("UCUS_MODU,");
    
    // GPS Bölümü (Dik çizgi ile)
    Serial.print(getLat()); Serial.print("|"); Serial.print(getLon()); Serial.print(",");
    
    // 3D Aviyonik
    Serial.print(getRoll()); Serial.print(",");
    Serial.print(getPitch()); Serial.print(",");
    Serial.println(getYaw()); // SON VERİDE 'println' kullanılır.
}
```
