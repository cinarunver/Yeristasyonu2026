"""
Kalman Filtre Monitörü — BNO055 + BME280
pip install pyqtgraph PyQt5 pyserial numpy
"""
import sys, re, collections
import numpy as np
import serial, serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QTabWidget, QStatusBar,
    QSplitter, QFrame, QScrollArea, QGridLayout, QPlainTextEdit, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QFont, QTextCursor, QColor
import pyqtgraph as pg

# ── Renkler ──────────────────────────────────────────────────────────────────
BG     = "#0d1117"
PANEL  = "#161b22"
CARD   = "#1c2128"
BORDER = "#30363d"
TEXT   = "#e6edf3"
MUTED  = "#8b949e"
HAM    = "#ff6b6b"
KAL    = "#4ecdc4"
ACCENT = "#58a6ff"
GREEN  = "#3fb950"
RED    = "#f85149"

pg.setConfigOption("background", BG)
pg.setConfigOption("foreground", TEXT)
MAXPTS = 300   # deque boyutu — büyütmek belleği ve render süresini artırır

# ── Alan tanımları ────────────────────────────────────────────────────────────
FIELDS = [
    ("ivmeX","İvme X","m/s²"), ("ivmeY","İvme Y","m/s²"), ("ivmeZ","İvme Z","m/s²"),
    ("gyroX","Gyro X","°/s"),  ("gyroY","Gyro Y","°/s"),  ("gyroZ","Gyro Z","°/s"),
    ("roll","Roll","°"),       ("pitch","Pitch","°"),      ("yaw","Yaw","°"),
    ("bsn","Basınç","hPa"),    ("sck","Sıcaklık","°C"),
    ("nem","Nem","%"),         ("irt","İrtifa","m"),
]
KEYS = [f[0] for f in FIELDS]
FMAP = {f[0]: f for f in FIELDS}

TABS = {
    "İvme":  ["ivmeX","ivmeY","ivmeZ"],
    "Gyro":  ["gyroX","gyroY","gyroZ"],
    "Euler": ["roll","pitch","yaw"],
    "BME":   ["bsn","sck","nem","irt"],
}

# ── Parser: satır satır, tek-değer regex ─────────────────────────────────────
_FLT = r"[-+]?\d+(?:\.\d+)?"

def _val(pattern, text):
    m = re.search(pattern, text)
    return float(m.group(1)) if m else None

def parse_block(buf: str):
    """
    Blok örneği (6 satır):
      [t=12345 ms]
        BNO | ivmeX: HAM=  0.123  KAL=  0.456  | ivmeY: ...
             gyroX: HAM=...
             roll:  HAM=...
        BME | bsn:   HAM=...
             irt:   HAM=...
    """
    t_m = re.search(r"\[t=(\d+)\s*ms\]", buf)
    if not t_m:
        return None
    pkt = {"t": int(t_m.group(1))}

    # Her alan için ayrı regex — HAM/KAL arasında isteğe bağlı boşluk var
    pairs = [
        ("ivmeX", r"ivmeX:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("ivmeY", r"ivmeY:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("ivmeZ", r"ivmeZ:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("gyroX", r"gyroX:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("gyroY", r"gyroY:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("gyroZ", r"gyroZ:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("roll",  r"roll:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("pitch", r"pitch:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("yaw",   r"yaw:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("bsn",   r"bsn:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("sck",   r"sck:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("nem",   r"nem:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
        ("irt",   r"irt:\s*HAM=\s*(" + _FLT + r")\s*KAL=\s*(" + _FLT + r")"),
    ]
    for key, pat in pairs:
        m = re.search(pat, buf)
        if not m:
            return None   # eksik alan → bloğu atla
        pkt[key+"_ham"] = float(m.group(1))
        pkt[key+"_kal"] = float(m.group(2))
    return pkt

# ── Serial Worker ─────────────────────────────────────────────────────────────
class SerialWorker(QObject):
    data_ready = pyqtSignal(dict)
    raw_line   = pyqtSignal(str)
    status_msg = pyqtSignal(str)
    finished   = pyqtSignal()

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self._run = True
        self._buf = ""

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            self.status_msg.emit(f"✓ Bağlandı → {self.port}  @{self.baud}")
        except Exception as e:
            self.status_msg.emit(f"✗ Bağlantı hatası: {e}")
            self.finished.emit()
            return

        while self._run:
            try:
                raw = ser.readline()
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                if line:
                    self.raw_line.emit(line)
                    self._buf += line + "\n"
                    if "---" in line:
                        pkt = parse_block(self._buf)
                        if pkt:
                            self.data_ready.emit(pkt)
                        self._buf = ""
            except Exception as e:
                self.status_msg.emit(f"Okuma hatası: {e}")
                break

        ser.close()
        self.finished.emit()

    def stop(self):
        self._run = False

# ── MetricCard ────────────────────────────────────────────────────────────────
class MetricCard(QFrame):
    def __init__(self, key):
        super().__init__()
        _, label, unit = FMAP[key]
        self.setObjectName("MC")
        self.setStyleSheet(f"#MC{{background:{CARD};border:1px solid {BORDER};border-radius:8px;}}")
        self.setFixedHeight(72)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 6, 10, 6)
        lay.setSpacing(2)

        top = QLabel(f"{label} <small style='color:{MUTED}'>{unit}</small>")
        top.setTextFormat(Qt.RichText)
        top.setStyleSheet(f"color:{MUTED};font-size:11px;font-weight:600;")
        lay.addWidget(top)

        row = QHBoxLayout()
        row.setSpacing(0)

        self.h_lbl = QLabel("—")
        self.h_lbl.setStyleSheet(f"color:{HAM};font-size:16px;font-weight:700;")
        self.k_lbl = QLabel("—")
        self.k_lbl.setStyleSheet(f"color:{KAL};font-size:16px;font-weight:700;")

        for tag, lbl, clr in [("HAM", self.h_lbl, HAM), ("KAL", self.k_lbl, KAL)]:
            col = QVBoxLayout()
            col.setSpacing(0)
            col.addWidget(QLabel(tag, styleSheet=f"color:{clr};font-size:8px;"))
            col.addWidget(lbl)
            row.addLayout(col)
            row.addSpacing(16)

        row.addStretch()
        lay.addLayout(row)

    def update_vals(self, h, k):
        self.h_lbl.setText(f"{h:+.3f}")
        self.k_lbl.setText(f"{k:+.3f}")

# ── ChartPanel ────────────────────────────────────────────────────────────────
class ChartPanel(QWidget):
    def __init__(self, keys):
        super().__init__()
        self.setStyleSheet(f"background:{BG};")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)

        self.plots = {}
        # numpy float32 ring buffer — deque→list kopyasından çok daha hızlı
        self.bufs  = {k: {"h": np.empty(MAXPTS, dtype=np.float32),
                          "k": np.empty(MAXPTS, dtype=np.float32),
                          "n": 0, "head": 0} for k in keys}
        self._dirty = False

        for key in keys:
            _, label, unit = FMAP[key]
            pw = pg.PlotWidget()
            pw.setMinimumHeight(170)
            pw.setLabel("left",  label, units=unit, color=MUTED)
            pw.setLabel("bottom", "örnek", color=MUTED)
            pw.showGrid(x=True, y=True, alpha=0.12)
            pw.getAxis("left").setTextPen(MUTED)
            pw.getAxis("bottom").setTextPen(MUTED)
            pw.setTitle(f"<span style='color:{TEXT};font-size:12px'>{label} ({unit})</span>")

            hc = pw.plot(pen=pg.mkPen(HAM, width=1.5))
            kc = pw.plot(pen=pg.mkPen(KAL, width=2.2))

            leg = pw.addLegend(offset=(-10, 10))
            leg.addItem(hc, f"<span style='color:{HAM}'>HAM</span>")
            leg.addItem(kc, f"<span style='color:{KAL}'>KAL</span>")

            self.plots[key] = (pw, hc, kc)
            lay.addWidget(pw)

    def push(self, key, h, k):
        b = self.bufs[key]
        idx = b["head"] % MAXPTS
        b["h"][idx] = h
        b["k"][idx] = k
        b["head"] += 1
        if b["n"] < MAXPTS:
            b["n"] += 1
        self._dirty = True

    def refresh(self):
        if not self._dirty:
            return
        self._dirty = False
        for key, (_, hc, kc) in self.plots.items():
            b = self.bufs[key]
            n, head = b["n"], b["head"]
            if n < MAXPTS:
                hc.setData(b["h"][:n])
                kc.setData(b["k"][:n])
            else:
                # ring buffer → doğrusal sıralı view (kopyasız)
                start = head % MAXPTS
                idx = np.arange(start, start + MAXPTS) % MAXPTS
                hc.setData(b["h"][idx])
                kc.setData(b["k"][idx])

# ── Terminal ──────────────────────────────────────────────────────────────────
class TerminalWidget(QWidget):
    """
    Performans: QPlainTextEdit + setMaximumBlockCount → reflow yok.
    Satırlar bir buffer'a alınır, 200ms'de bir toplu yazılır.
    """
    def __init__(self):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(4)

        hdr = QHBoxLayout()
        t = QLabel("📡 Ham Seri Çıktı")
        t.setStyleSheet(f"color:{ACCENT};font-weight:700;font-size:13px;")
        hdr.addWidget(t)
        hdr.addStretch()
        clr = QPushButton("Temizle")
        clr.setStyleSheet(f"background:{BORDER};color:{TEXT};border:none;border-radius:5px;padding:3px 10px;")
        clr.clicked.connect(self.clear)
        hdr.addWidget(clr)
        lay.addLayout(hdr)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(800)   # eski satırları otomatik siler — O(1)
        self.log.setFont(QFont("Menlo", 11))
        self.log.setStyleSheet(f"""
            QPlainTextEdit {{
                background:{BG}; color:{TEXT};
                border:1px solid {BORDER}; border-radius:6px; padding:6px;
            }}
        """)
        self.log.setLineWrapMode(QPlainTextEdit.NoWrap)
        lay.addWidget(self.log)

        self._buf = []   # batch buffer
        self._flush_timer = QTimer()
        self._flush_timer.setInterval(200)   # 5 fps — terminal için yeterli
        self._flush_timer.timeout.connect(self._flush)
        self._flush_timer.start()

    def append_line(self, line: str):
        self._buf.append(line)   # sadece buffer'a ekle, UI'ya dokunma

    def _flush(self):
        if not self._buf:
            return
        # Toplu ekleme — tek seferinde setBlockCount tetiklenir
        self.log.appendPlainText("\n".join(self._buf))
        self._buf.clear()

    def clear(self):
        self.log.clear()
        self._buf.clear()

# ── Ana Pencere ───────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🚀 Kalman Filtre Monitörü — BNO055 + BME280")
        self.resize(1500, 900)
        self._worker = None
        self._thread = None
        self._pkt_cnt = 0
        self._cards  = {}          # key → MetricCard
        self._panels = {}          # tab_name → ChartPanel
        self._key_panel = {}       # key → ChartPanel  (O(1) lookup)
        self._pending_pkt = None
        self._dirty = False

        self._apply_theme()
        self._build_ui()

        self._timer = QTimer()
        self._timer.setInterval(60)
        self._timer.timeout.connect(self._tick)

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        cw = QWidget()
        self.setCentralWidget(cw)
        root = QVBoxLayout(cw)
        root.setContentsMargins(10, 8, 10, 6)
        root.setSpacing(6)
        root.addWidget(self._mk_toolbar())

        # Ana splitter: sol=metrikler, sağ=sekmeler
        main_split = QSplitter(Qt.Horizontal)
        main_split.addWidget(self._mk_metrics())
        main_split.addWidget(self._mk_right_panel())
        main_split.setSizes([290, 1210])
        root.addWidget(main_split, stretch=1)

        sb = QStatusBar()
        sb.setStyleSheet(f"background:{PANEL};color:{MUTED};font-size:11px;")
        self.setStatusBar(sb)
        self.status_bar = sb
        sb.showMessage("Port seç → Bağlan")

    def _mk_toolbar(self):
        bar = QWidget()
        bar.setFixedHeight(50)
        bar.setStyleSheet(f"background:{PANEL};border-radius:10px;")
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(14, 6, 14, 6)

        title = QLabel("🚀 Kalman Filtre Monitörü")
        title.setStyleSheet(f"color:{ACCENT};font-size:15px;font-weight:700;")
        lay.addWidget(title)
        lay.addStretch()

        # Port
        lay.addWidget(QLabel("Port:", styleSheet=f"color:{MUTED};font-size:12px;"))
        self.port_cb = QComboBox()
        self.port_cb.setMinimumWidth(180)
        self.port_cb.setStyleSheet(self._cb_style())
        lay.addWidget(self.port_cb)

        # Baud
        lay.addWidget(QLabel("Baud:", styleSheet=f"color:{MUTED};font-size:12px;"))
        self.baud_cb = QComboBox()
        for b in ["9600","19200","38400","57600","115200","230400"]:
            self.baud_cb.addItem(b)
        self.baud_cb.setCurrentText("115200")
        self.baud_cb.setMinimumWidth(95)
        self.baud_cb.setStyleSheet(self._cb_style())
        lay.addWidget(self.baud_cb)

        scan_btn = QPushButton("⟳ Tara")
        scan_btn.setStyleSheet(self._btn(BORDER))
        scan_btn.clicked.connect(self._scan_ports)
        lay.addWidget(scan_btn)

        self.conn_btn = QPushButton("▶  Bağlan")
        self.conn_btn.setStyleSheet(self._btn("#238636"))
        self.conn_btn.clicked.connect(self._toggle)
        lay.addWidget(self.conn_btn)

        self.pkt_lbl = QLabel("Paket: 0")
        self.pkt_lbl.setStyleSheet(f"color:{MUTED};font-size:11px;margin-left:14px;")
        lay.addWidget(self.pkt_lbl)

        self._scan_ports()
        return bar

    def _mk_metrics(self):
        outer = QWidget()
        outer.setStyleSheet(f"background:{PANEL};border-radius:10px;")
        lay = QVBoxLayout(outer)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(4)

        hdr = QLabel("Anlık Değerler")
        hdr.setStyleSheet(f"color:{TEXT};font-size:13px;font-weight:700;margin-bottom:4px;")
        lay.addWidget(hdr)

        # legend
        leg = QHBoxLayout()
        for clr, txt in [(HAM,"HAM – Ham"), (KAL,"KAL – Kalman")]:
            dot = QLabel("●")
            dot.setStyleSheet(f"color:{clr};font-size:14px;")
            lbl = QLabel(txt)
            lbl.setStyleSheet(f"color:{clr};font-size:11px;")
            leg.addWidget(dot); leg.addWidget(lbl); leg.addSpacing(8)
        leg.addStretch()
        lay.addLayout(leg)

        sc = QScrollArea()
        sc.setWidgetResizable(True)
        sc.setStyleSheet("border:none;background:transparent;")
        inner = QWidget()
        inner.setStyleSheet("background:transparent;")
        grid = QGridLayout(inner)
        grid.setSpacing(5)
        grid.setContentsMargins(0, 0, 0, 0)

        sections = [
            ("BNO — İvme", ["ivmeX","ivmeY","ivmeZ"]),
            ("BNO — Gyro", ["gyroX","gyroY","gyroZ"]),
            ("BNO — Euler",["roll","pitch","yaw"]),
            ("BME",        ["bsn","sck","nem","irt"]),
        ]
        row = 0
        for sec, keys in sections:
            sl = QLabel(sec)
            sl.setStyleSheet(f"color:{ACCENT};font-size:10px;font-weight:700;margin-top:6px;")
            grid.addWidget(sl, row, 0, 1, 2); row += 1
            for k in keys:
                c = MetricCard(k)
                grid.addWidget(c, row, 0, 1, 2)
                self._cards[k] = c
                row += 1

        sc.setWidget(inner)
        lay.addWidget(sc)
        return outer

    def _mk_right_panel(self):
        # Dikey splitter: üstte grafikler, altta terminal
        vs = QSplitter(Qt.Vertical)
        vs.addWidget(self._mk_charts())
        vs.addWidget(self._mk_terminal())
        vs.setSizes([620, 220])
        return vs

    def _mk_charts(self):
        outer = QWidget()
        outer.setStyleSheet(f"background:{PANEL};border-radius:10px;")
        lay = QVBoxLayout(outer)
        lay.setContentsMargins(6, 6, 6, 6)

        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(f"""
            QTabWidget::pane {{border:1px solid {BORDER};background:{BG};border-radius:6px;}}
            QTabBar::tab {{background:{CARD};color:{MUTED};padding:5px 16px;
                           border-radius:4px;margin-right:3px;font-size:12px;}}
            QTabBar::tab:selected {{background:{ACCENT};color:#000;font-weight:700;}}
            QTabBar::tab:hover {{background:{BORDER};color:{TEXT};}}
        """)

        for name, keys in TABS.items():
            panel = ChartPanel(keys)
            sc = QScrollArea()
            sc.setWidgetResizable(True)
            sc.setWidget(panel)
            sc.setStyleSheet("border:none;")
            self.tabs.addTab(sc, name)
            self._panels[name] = panel
            for k in keys:
                self._key_panel[k] = panel   # doğrudan arama — panel iterasyonu yok

        lay.addWidget(self.tabs)
        return outer

    def _mk_terminal(self):
        outer = QWidget()
        outer.setStyleSheet(f"background:{PANEL};border-radius:10px;")
        lay = QVBoxLayout(outer)
        lay.setContentsMargins(6, 6, 6, 6)
        self.terminal = TerminalWidget()
        lay.addWidget(self.terminal)
        return outer

    # ── Bağlantı ─────────────────────────────────────────────────────────────
    def _toggle(self):
        if self._worker:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_cb.currentText()
        baud = int(self.baud_cb.currentText())
        if not port:
            self.status_bar.showMessage("Port seçilmedi!")
            return
        self._thread = QThread()
        self._worker = SerialWorker(port, baud)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.data_ready.connect(self._on_data)
        self._worker.raw_line.connect(self.terminal.append_line)
        self._worker.status_msg.connect(self.status_bar.showMessage)
        self._worker.finished.connect(self._on_done)
        self._thread.start()
        self._timer.start()
        self.conn_btn.setText("■  Durdur")
        self.conn_btn.setStyleSheet(self._btn(RED))

    def _disconnect(self):
        if self._worker:
            self._worker.stop()
        self._timer.stop()

    def _on_done(self):
        self._thread.quit(); self._thread.wait()
        self._worker = None; self._thread = None
        self.conn_btn.setText("▶  Bağlan")
        self.conn_btn.setStyleSheet(self._btn("#238636"))
        self.status_bar.showMessage("Bağlantı kesildi.")

    # ── Veri ─────────────────────────────────────────────────────────────────
    def _on_data(self, pkt):
        self._pkt_cnt += 1
        self.pkt_lbl.setText(f"Paket: {self._pkt_cnt}")
        for k in KEYS:
            h, kv = pkt[k+"_ham"], pkt[k+"_kal"]
            self._cards[k].update_vals(h, kv)        # metrik kart güncelle
            self._key_panel[k].push(k, h, kv)        # O(1) — panel arama yok
        self._dirty = True

    def _tick(self):
        if not self._dirty:
            return
        self._dirty = False
        active_name = self.tabs.tabText(self.tabs.currentIndex())
        panel = self._panels.get(active_name)
        if panel:
            panel.refresh()   # sadece görünen sekmeyi güncelle

    def _scan_ports(self):
        cur = self.port_cb.currentText()
        self.port_cb.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_cb.addItem(p.device)
        if cur:
            idx = self.port_cb.findText(cur)
            if idx >= 0:
                self.port_cb.setCurrentIndex(idx)

    # ── Stil ─────────────────────────────────────────────────────────────────
    def _apply_theme(self):
        self.setStyleSheet(f"""
            QMainWindow,QWidget{{background:{BG};color:{TEXT};
                font-family:'SF Pro Display','Segoe UI',Arial,sans-serif;}}
            QSplitter::handle{{background:{BORDER};}}
            QSplitter::handle:horizontal{{width:2px;}}
            QSplitter::handle:vertical{{height:2px;}}
            QScrollBar:vertical{{background:{PANEL};width:5px;border-radius:2px;}}
            QScrollBar::handle:vertical{{background:{BORDER};border-radius:2px;}}
            QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{{height:0;}}
            QScrollBar:horizontal{{background:{PANEL};height:5px;border-radius:2px;}}
            QScrollBar::handle:horizontal{{background:{BORDER};border-radius:2px;}}
            QScrollBar::add-line:horizontal,QScrollBar::sub-line:horizontal{{width:0;}}
        """)

    def _cb_style(self):
        return f"""
            QComboBox{{background:{BG};color:{TEXT};border:1px solid {BORDER};
                       border-radius:6px;padding:4px 8px;font-size:12px;}}
            QComboBox::drop-down{{border:none;width:20px;}}
            QComboBox QAbstractItemView{{background:{PANEL};color:{TEXT};
                                         border:1px solid {BORDER};outline:none;}}
        """

    def _btn(self, bg):
        return f"""
            QPushButton{{background:{bg};color:{TEXT};border:none;border-radius:6px;
                         padding:5px 14px;font-size:12px;font-weight:600;}}
            QPushButton:hover{{background:{bg}cc;}}
            QPushButton:pressed{{background:{bg}88;}}
        """

    def closeEvent(self, e):
        self._disconnect()
        super().closeEvent(e)

# ── Giriş ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("SF Pro Display,Segoe UI,Arial", 10))
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
