#!/usr/bin/env python3
# ==============================================================================
# TILE INDIRICI — offline harita icin uydu goruntusu indirir (2026-09-04)
#
# Sahada internet olmadigi icin harita tile'lari onceden indirilip
# assets/tiles/ altina z/x/y.jpg duzeninde saklanir. YerIstasyonu2026.py
# haritasi once BURAYA bakar, tile yoksa CDN'e duser (bkz. MAP_HTML).
#
# Kaynak: Esri World Imagery (anahtar gerektirmez).
#
# KULLANIM
#   python3 tile_indir.py --test                 # 10 km z12-16  (~2.5k tile, ~30 MB)
#   python3 tile_indir.py --kademeli             # ONERILEN: 40km z12-14 + 15km z15-17
#   python3 tile_indir.py --km 40 --zoom 12-16   # elle
#   python3 tile_indir.py --kademeli --tahmin    # indirmeden once sadece hesapla
#
# Kesilirse tekrar calistir: indirilmis tile atlanir, kaldigi yerden devam eder.
# ==============================================================================
import argparse
import math
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor

# Atis alani (YerIstasyonu2026.py icindeki GS_LAT/GS_LON ile ayni olmali)
GS_LAT, GS_LON = 38.401831, 33.704852

TILE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}")
HEDEF_DIZIN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "tiles")

# Esri'yi zorlamamak icin: es zamanli istek + istekler arasi minimum bekleme.
ES_ZAMANLI = 8
ISTEK_ARALIGI = 0.012      # saniye/istek (thread basina) -> ~8-10 istek/sn toplam
DENEME = 3                 # basarisiz tile icin tekrar sayisi
ZAMAN_ASIMI = 20

# Kademeli plan: uzagi kaba, arama alanini detayli al (bkz. README).
KADEMELI_PLAN = [
    (40, 12, 14),   # genis cevre — yon bulma, roket uzaga duserse
    (15, 15, 17),   # asil arama alani — z17 ~0.94 m/piksel
]

_kilit = threading.Lock()
_sayac = {"indi": 0, "atlandi": 0, "hata": 0, "bayt": 0}


def deg2tile(lat, lon, z):
    """WGS84 derece -> XYZ tile indeksi (Web Mercator, Leaflet ile ayni)."""
    n = 2 ** z
    x = int((lon + 180.0) / 360.0 * n)
    lat = max(-85.05112878, min(85.05112878, lat))
    y = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
    return max(0, min(n - 1, x)), max(0, min(n - 1, y))


def tile_listesi(km, zmin, zmax, merkez=(GS_LAT, GS_LON)):
    """Merkez etrafinda +-km kare alani kaplayan (z,x,y) listesi."""
    lat, lon = merkez
    dlat = km / 111.32
    dlon = km / (111.32 * math.cos(math.radians(lat)))
    işler = []
    for z in range(zmin, zmax + 1):
        x1, y1 = deg2tile(lat + dlat, lon - dlon, z)   # sol-ust
        x2, y2 = deg2tile(lat - dlat, lon + dlon, z)   # sag-alt
        for x in range(min(x1, x2), max(x1, x2) + 1):
            for y in range(min(y1, y2), max(y1, y2) + 1):
                işler.append((z, x, y))
    return işler


def tile_yolu(z, x, y):
    return os.path.join(HEDEF_DIZIN, str(z), str(x), f"{y}.jpg")


def indir_bir(is_):
    z, x, y = is_
    yol = tile_yolu(z, x, y)
    if os.path.exists(yol) and os.path.getsize(yol) > 0:
        with _kilit:
            _sayac["atlandi"] += 1
        return
    os.makedirs(os.path.dirname(yol), exist_ok=True)
    url = TILE_URL.format(z=z, x=x, y=y)
    for deneme in range(DENEME):
        try:
            time.sleep(ISTEK_ARALIGI)
            req = urllib.request.Request(url, headers={"User-Agent": "TrakyaRoket-YerIstasyonu/1.0"})
            with urllib.request.urlopen(req, timeout=ZAMAN_ASIMI) as r:
                veri = r.read()
            if not veri:
                raise ValueError("bos yanit")
            # .part'a yaz, sonra tasi: yarim dosya kalirsa tekrar calistirinca duzelir
            gecici = yol + ".part"
            with open(gecici, "wb") as f:
                f.write(veri)
            os.replace(gecici, yol)
            with _kilit:
                _sayac["indi"] += 1
                _sayac["bayt"] += len(veri)
            return
        except Exception:
            if deneme == DENEME - 1:
                with _kilit:
                    _sayac["hata"] += 1
            else:
                time.sleep(1.5 * (deneme + 1))   # artan bekleme


def ilerleme(toplam, baslangic):
    bitti = _sayac["indi"] + _sayac["atlandi"] + _sayac["hata"]
    if bitti == 0:
        return
    gecen = time.time() - baslangic
    hiz = bitti / gecen if gecen > 0 else 0
    kalan = (toplam - bitti) / hiz if hiz > 0 else 0
    mb = _sayac["bayt"] / 1024 / 1024
    sys.stdout.write(
        f"\r  {bitti:,}/{toplam:,} (%{100*bitti/toplam:5.1f})  "
        f"indi:{_sayac['indi']:,} atlandi:{_sayac['atlandi']:,} hata:{_sayac['hata']:,}  "
        f"{mb:.0f} MB  {hiz:.0f}/sn  kalan ~{kalan/60:.0f} dk   ")
    sys.stdout.flush()


def calistir(işler, baslik):
    # Ayni tile birden fazla bolgede olabilir -> tekille
    işler = sorted(set(işler))
    toplam = len(işler)
    print(f"\n{baslik}")
    print(f"  {toplam:,} tile  (~{toplam*12.9/1024:.0f} MB tahmini)")
    baslangic = time.time()
    son = [0.0]

    def sarmal(is_):
        indir_bir(is_)
        şimdi = time.time()
        if şimdi - son[0] > 0.3:
            son[0] = şimdi
            ilerleme(toplam, baslangic)

    with ThreadPoolExecutor(max_workers=ES_ZAMANLI) as ex:
        list(ex.map(sarmal, işler))
    ilerleme(toplam, baslangic)
    print()
    return time.time() - baslangic


def main():
    ap = argparse.ArgumentParser(description="Offline harita icin uydu tile indirir")
    ap.add_argument("--test", action="store_true", help="10 km z12-16 (hizli deneme)")
    ap.add_argument("--kademeli", action="store_true", help="40km z12-14 + 15km z15-17 (onerilen)")
    ap.add_argument("--km", type=float, help="yaricap (km)")
    ap.add_argument("--zoom", type=str, help="zoom araligi, or. 12-16")
    ap.add_argument("--tahmin", action="store_true", help="indirme, sadece hesapla")
    a = ap.parse_args()

    if a.test:
        planlar = [(10, 12, 16)]
        ad = "TEST — 10 km, z12-16"
    elif a.kademeli:
        planlar = KADEMELI_PLAN
        ad = "KADEMELI — 40km z12-14 (genis cevre) + 15km z15-17 (arama alani)"
    elif a.km and a.zoom:
        zmin, zmax = (int(v) for v in a.zoom.split("-"))
        planlar = [(a.km, zmin, zmax)]
        ad = f"ELLE — {a.km} km, z{zmin}-{zmax}"
    else:
        ap.print_help()
        print("\nOnerilen sira:  once --test  (dogrula),  sonra --kademeli")
        return 1

    print("=" * 68)
    print(f"UYDU TILE INDIRICI — {ad}")
    print(f"Merkez : {GS_LAT}, {GS_LON}  (atis alani)")
    print(f"Hedef  : {HEDEF_DIZIN}")
    print("=" * 68)

    tum = []
    for km, zmin, zmax in planlar:
        p = tile_listesi(km, zmin, zmax)
        print(f"  {km:>4.0f} km  z{zmin}-{zmax}: {len(p):>8,} tile")
        tum += p
    benzersiz = sorted(set(tum))
    print(f"  {'TOPLAM':>9}: {len(benzersiz):>8,} tile  ~{len(benzersiz)*12.9/1024:.0f} MB"
          f"  ~{len(benzersiz)/9/60:.0f} dk")

    if a.tahmin:
        print("\n(--tahmin: indirme yapilmadi)")
        return 0

    var = sum(1 for z, x, y in benzersiz if os.path.exists(tile_yolu(z, x, y)))
    if var:
        print(f"  {var:,} tile zaten mevcut -> atlanacak")

    print("\nBaslamak icin ENTER, iptal icin Ctrl+C...", end="")
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        print("\niptal edildi.")
        return 1

    try:
        sure = calistir(benzersiz, "INDIRILIYOR")
    except KeyboardInterrupt:
        print("\n\nDurduruldu. Tekrar calistirirsan kaldigi yerden devam eder.")
        return 1

    mb = _sayac["bayt"] / 1024 / 1024
    print("=" * 68)
    print(f"BITTI — {_sayac['indi']:,} indi, {_sayac['atlandi']:,} atlandi, "
          f"{_sayac['hata']:,} hata")
    print(f"  {mb:.0f} MB  /  {sure/60:.1f} dakika")
    if _sayac["hata"]:
        print(f"  {_sayac['hata']:,} tile alinamadi — tekrar calistirinca yalniz onlar denenir.")
    print(f"  Konum: {HEDEF_DIZIN}")
    print("Harita artik once yerel tile'a bakacak, yoksa CDN'e dusecek.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
