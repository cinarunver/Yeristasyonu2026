import sys
import os

# THE FIX
os.environ["QSG_RHI_BACKEND"] = "opengl"

from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtOpenGLWidgets import QOpenGLWidget

app = QApplication(sys.argv)
win = QMainWindow()
w = QWidget()
l = QVBoxLayout(w)

gl = QOpenGLWidget()
web = QWebEngineView()
web.setHtml("<html><body><h1>Hello WebEngine</h1></body></html>")

l.addWidget(gl)
l.addWidget(web)
win.setCentralWidget(w)
win.show()

from PyQt6.QtCore import QTimer
QTimer.singleShot(2000, app.quit)

sys.exit(app.exec())
