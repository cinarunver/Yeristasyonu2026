# -*- mode: python ; coding: utf-8 -*-
#
# YerIstasyonu2026 — PyInstaller yapilandirmasi (Windows / Linux / macOS ortak)
#
# NOT: assets/tiles/ (offline harita onbellegi, ~300 MB) .gitignore'dadir.
# CI'da bulunmaz; varsa pakete girer, yoksa harita cevrimici Esri katmanina
# duser. Leaflet dosyalari ve isaretci ikonlari repoda oldugu icin her zaman
# paketlenir.

import os
import sys
from PyInstaller.utils.hooks import collect_data_files

block_cipher = None

# assets/ agacini oldugu gibi tasi (varsa tile'lar dahil).
datas = []
if os.path.isdir('assets'):
    datas.append(('assets', 'assets'))

# pyqtgraph ve OpenGL bazi kaynaklarini dinamik yukler.
datas += collect_data_files('pyqtgraph', include_py_files=False)

# macOS: PyInstaller, QtWebEngineCore.framework icerigini yanlis yere —
# Versions/Resources/ altina — koyuyor. Qt ise framework kurallarina gore
# Helpers/ ve Resources/ sembolik baglarini (-> Versions/A/...) izliyor ve
# dosyalari bulamiyor ("could not find Qt WebEngine resources/process").
# Cozum: COLLECT sonrasi icerigi Versions/A/ altina tasimak (bkz. asagida
# _webengine_duzelt). Ek olarak Helpers/ agacini datas'a ekliyoruz; cunku
# PyInstaller bu dizini hic kopyalamiyor.
if sys.platform == 'darwin':
    import PyQt6
    _helpers = os.path.join(
        os.path.dirname(PyQt6.__file__),
        'Qt6', 'lib', 'QtWebEngineCore.framework', 'Helpers',
    )
    if os.path.isdir(_helpers):
        for _kok, _dizinler, _dosyalar in os.walk(_helpers):
            for _d in _dosyalar:
                _tam = os.path.join(_kok, _d)
                _bagil = os.path.relpath(_tam, _helpers)
                _hedef = os.path.join(
                    'PyQt6', 'Qt6', 'lib', 'QtWebEngineCore.framework',
                    'Versions', 'A', 'Helpers', os.path.dirname(_bagil),
                )
                datas.append((_tam, _hedef))


hiddenimports = [
    'serial.tools.list_ports',
    'OpenGL.platform.egl',
    'OpenGL.platform.glx',
    'OpenGL.platform.win32',
    'OpenGL.arrays.ctypesarrays',
    'OpenGL.arrays.numpymodule',
    'OpenGL.arrays.lists',
    'OpenGL.arrays.numbers',
    'OpenGL.arrays.strings',
    'PyQt6.QtWebEngineWidgets',
    'PyQt6.QtWebEngineCore',
    'PyQt6.QtOpenGLWidgets',
]

a = Analysis(
    ['YerIstasyonu2026.py'],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # Uygulama yalniz Widgets + WebEngine + OpenGL kullanir. Qt'nin Quick/QML,
    # 3D, Multimedia, Sensors... modulleri paketi ~200 MB sisiriyor; disla.
    excludes=[
        'tkinter', 'matplotlib', 'PyQt5', 'PySide6',
        'PyQt6.QtQuick', 'PyQt6.QtQuick3D', 'PyQt6.QtQml',
        'PyQt6.QtQuickWidgets', 'PyQt6.QtQuickControls2',
        'PyQt6.QtMultimedia', 'PyQt6.QtMultimediaWidgets',
        'PyQt6.QtSensors', 'PyQt6.QtCharts', 'PyQt6.QtDataVisualization',
        'PyQt6.QtBluetooth', 'PyQt6.QtNfc', 'PyQt6.QtPdf', 'PyQt6.QtPdfWidgets',
        'PyQt6.QtSql', 'PyQt6.QtTest', 'PyQt6.QtDesigner', 'PyQt6.QtHelp',
        'PyQt6.QtSpatialAudio', 'PyQt6.QtRemoteObjects', 'PyQt6.QtScxml',
        'PyQt6.QtTextToSpeech', 'PyQt6.QtSerialBus', 'PyQt6.QtWebSockets',
        'PyQt6.Qt3DCore',
        # NOT: QtWebChannel ve QtPositioning DISLANMAZ — QtWebEngine bunlari
        # calisma aninda import eder (ModuleNotFoundError ile cakiliyor).
    ],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='YerIstasyonu2026',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    # UPX, PyQt6/WebEngine ikililerini bozabildigi icin kapali.
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name='YerIstasyonu2026',
)

app = BUNDLE(
    coll,
    name='YerIstasyonu2026.app',
    icon=None,
    bundle_identifier='tr.edu.trakya.roket.yeristasyonu2026',
    info_plist={
        'NSHighResolutionCapable': True,
        # Seri port erisimi icin (macOS Ventura+ USB cihaz izni).
        'NSCameraUsageDescription': 'Kullanilmiyor.',
    },
)


# --- macOS framework duzeltmesi ---------------------------------------------
# PyInstaller QtWebEngineCore.framework icerigini Versions/Resources/ altina
# birakiyor; Qt ise Helpers/ ve Resources/ baglarini izleyerek Versions/A/
# altinda ariyor. Iceriyi dogru yere tasi, stray dizini temizle.
if sys.platform == 'darwin':
    import shutil

    def _webengine_duzelt(kok):
        fw = os.path.join(
            kok, '_internal', 'PyQt6', 'Qt6', 'lib',
            'QtWebEngineCore.framework',
        )
        stray = os.path.join(fw, 'Versions', 'Resources')
        hedef_a = os.path.join(fw, 'Versions', 'A')
        if not os.path.isdir(stray) or not os.path.isdir(hedef_a):
            return
        for ad in os.listdir(stray):
            kaynak = os.path.join(stray, ad)
            hedef = os.path.join(hedef_a, ad)
            if os.path.isdir(kaynak):
                os.makedirs(hedef, exist_ok=True)
                for alt in os.listdir(kaynak):
                    s_alt = os.path.join(kaynak, alt)
                    h_alt = os.path.join(hedef, alt)
                    if os.path.exists(h_alt):
                        continue
                    if os.path.isdir(s_alt):
                        shutil.copytree(s_alt, h_alt, symlinks=True)
                    else:
                        shutil.copy2(s_alt, h_alt)
            elif not os.path.exists(hedef):
                shutil.copy2(kaynak, hedef)
        shutil.rmtree(stray, ignore_errors=True)

    _webengine_duzelt(os.path.join(DISTPATH, 'YerIstasyonu2026'))
