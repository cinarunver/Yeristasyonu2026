#!/usr/bin/env python3
# ============================================================
#  TRAKYA ROKET 2026 — LORA TELEMETRI HZ OLCER
# ============================================================
#  Yer istasyonundan BAGIMSIZ, tek dosyalik kucuk arac.
#  Seri porttan gelen gecerli cerceveleri (CRC dogrulamali) sayar ve
#  her saniye "kac Hz veri aliyoruz" bilgisini yazar.
#
#  Cerceve: [0xAA][0x55][LEN][payload LEN byte][CRC16_HI][CRC16_LO]
#  LEN byte'indan tip otomatik ayrilir:  roket=23,  gorev yuku=32.
#  (Firmware fixed-point wire format ile birebir — bkz. lora-wire-fixed-point.)
#
#  Kullanim:
#     python3 hz_olcer.py                 -> portlari listeler ve SORAR
#     python3 hz_olcer.py <port> [baud=9600]   -> dogrudan verilebilir
#     python3 hz_olcer.py /dev/tty.usbserial-0001
#     python3 hz_olcer.py COM5 9600
#
#  Cikti (her 1 sn):  saat | toplam Hz | roket Hz | gorevY Hz | CRC-hata/sn
#  Ctrl+C ile cikilir.
# ============================================================
import sys
import time

SYNC_1, SYNC_2 = 0xAA, 0x55
ROCKET_LEN  = 23   # TelemetryWire (ivme tek slot=toplam; yonelim quaternion)
PAYLOAD_LEN = 32   # GorevYukuWire (+ hava yogunlugu, + quaternion; 24B->32B)
MAX_LEN     = 64   # guvenlik ust siniri (sahte SYNC'leri elemek icin)


def crc16_ccitt(data: bytes) -> int:
    """Firmware/yer istasyonu ile birebir CRC16-CCITT (poly=0x1021, init=0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def cerceveleri_ayikla(buf: bytearray):
    """
    buf icindeki TAM cerceveleri ayiklar; tuketilen baytlari buf'tan siler.
    Yarim kalan cerceve buf'ta birakilir (bir sonraki okumada tamamlanir).

    Her bulunan cerceve icin (uzunluk, gecerli_mi) tuple'i doner:
      - gecerli=True  -> CRC tuttu (tam cerceve tuketildi)
      - gecerli=False -> CRC tutmadi; SAHTE SYNC olabilir, cerceve atilmaz,
                         sadece 2 baytlik SYNC atlanir ki kaymis gercek
                         cerceve yakalanabilsin (yer istasyonuyla ayni mantik).
    """
    sonuc = []
    hedef = bytes([SYNC_1, SYNC_2])
    while True:
        i = buf.find(hedef)
        if i < 0:
            # SYNC yok; son bayti sakla (0xAA olabilir), gerisini at
            if len(buf) > 1:
                del buf[:len(buf) - 1]
            break
        if i > 0:
            del buf[:i]              # SYNC oncesi cop baytlari at
        if len(buf) < 3:
            break                    # LEN henuz gelmedi
        ln = buf[2]
        if ln == 0 or ln > MAX_LEN:
            del buf[:2]              # sahte SYNC; ilerle
            continue
        frame_size = 3 + ln + 2
        if len(buf) < frame_size:
            break                    # tam cerceve henuz gelmedi
        payload  = bytes(buf[3:3 + ln])
        crc_recv = (buf[3 + ln] << 8) | buf[3 + ln + 1]
        gecerli  = (crc16_ccitt(payload) == crc_recv)
        sonuc.append((ln, gecerli))
        if gecerli:
            del buf[:frame_size]     # tam cerceveyi tuket
        else:
            del buf[:2]              # sahte SYNC olabilir; sadece SYNC'i gec
    return sonuc


def port_sec():
    """Kullanilabilir seri portlari listeler ve kullaniciya sordurur."""
    from serial.tools import list_ports
    portlar = list(list_ports.comports())
    if not portlar:
        print("HATA: hic seri port bulunamadi. Cihaz takili mi?")
        sys.exit(1)
    print("Kullanilabilir portlar:")
    for i, p in enumerate(portlar):
        aciklama = ""
        if p.description and p.description.lower() != "n/a":
            aciklama = f"  ({p.description})"
        print(f"  [{i}] {p.device}{aciklama}")
    while True:
        secim = input("Hangi port? (numara ya da tam port adi): ").strip()
        if not secim:
            continue
        if secim.isdigit():
            idx = int(secim)
            if 0 <= idx < len(portlar):
                return portlar[idx].device
            print("  Gecersiz numara, tekrar dene.")
            continue
        return secim  # dogrudan port adi da girilebilir


def main():
    try:
        import serial  # pyserial (yer istasyonu da bunu kullaniyor)
    except ImportError:
        print("HATA: pyserial yok. Kur:  pip install pyserial")
        sys.exit(1)

    # Port: komut satirindan verildiyse onu kullan, yoksa acilista SOR.
    port = sys.argv[1] if len(sys.argv) > 1 else port_sec()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"HATA: port acilamadi ({e})")
        sys.exit(1)

    print(f"# {port} @ {baud} baud — dinleniyor (Ctrl+C ile cik)")
    print(f"# {'saat':>8} | {'toplam':>7} | {'roket':>6} | {'gorevY':>6} | {'CRC-hata':>8}")
    print(f"# {'':>8} | {'Hz':>7} | {'Hz':>6} | {'Hz':>6} | {'/sn':>8}")

    buf = bytearray()
    win_start = time.time()
    ok_total = ok_rocket = ok_payload = crc_fail = 0

    try:
        while True:
            data = ser.read(256)
            if data:
                buf.extend(data)
                for ln, gecerli in cerceveleri_ayikla(buf):
                    if gecerli:
                        ok_total += 1
                        if ln == ROCKET_LEN:
                            ok_rocket += 1
                        elif ln == PAYLOAD_LEN:
                            ok_payload += 1
                    else:
                        crc_fail += 1

            now = time.time()
            dt = now - win_start
            if dt >= 1.0:
                print(f"  {time.strftime('%H:%M:%S'):>8} | "
                      f"{ok_total / dt:7.1f} | {ok_rocket / dt:6.1f} | "
                      f"{ok_payload / dt:6.1f} | {crc_fail / dt:8.1f}")
                win_start = now
                ok_total = ok_rocket = ok_payload = crc_fail = 0
    except KeyboardInterrupt:
        print("\n# durduruldu")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
