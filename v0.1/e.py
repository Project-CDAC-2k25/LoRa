"""
LoRa-Based Serial Peer-to-Peer Chat (PyQt5)
------------------------------------------
•  Dark colour-palette matching the screenshot provided
•  Updated flexible command format:
      TYPE:<REQ/SEND/CONN/PEERLIST/UPDATE/NOTIF>; … ;
•  Dynamic peer list with “last-seen” timestamps
•  2-second top-banner notification for TYPE:NOTIF
•  Cross-platform: Windows & Linux (PyQt5 + pyserial)
"""

import sys, time, serial, serial.tools.list_ports
from datetime import datetime
from PyQt5.QtCore   import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui    import QPalette, QColor, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QListWidget, QListWidgetItem, QTextEdit, QLineEdit, QPushButton,
    QLabel, QComboBox, QSplitter, QFrame, QMessageBox, QStatusBar
)

# ───────────────────────── Serial Worker ──────────────────────────
class SerialWorker(QThread):
    data_received       = pyqtSignal(str)
    error_occurred      = pyqtSignal(str)
    connection_changed  = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self._port = None
        self._running = False

    # ---------- Public API ----------
    def connect_serial(self, port:str, baud:int)->bool:
        """Open serial port in background thread."""
        try:
            if self._port and self._port.is_open:
                self._port.close()
            self._port = serial.Serial(port, baud, timeout=1)
            self._running = True
            self.connection_changed.emit(True)
            self.start()
            return True
        except Exception as e:
            self.error_occurred.emit(f"Connection failed: {e}")
            return False

    def disconnect_serial(self):
        self._running = False
        if self._port and self._port.is_open:
            self._port.close()
        self.connection_changed.emit(False)

    def send_data(self, line:str)->bool:
        """Write <line> + newline to UART."""
        if self._port and self._port.is_open:
            try:
                self._port.write((line+'\n').encode())
                return True
            except Exception as e:
                self.error_occurred.emit(f"Send failed: {e}")
        return False

    # ---------- Thread loop ----------
    def run(self):
        while self._running and self._port and self._port.is_open:
            try:
                if self._port.in_waiting:
                    line = self._port.readline().decode(errors="ignore").strip()
                    if line:
                        self.data_received.emit(line)
            except Exception as e:
                self.error_occurred.emit(f"Read error: {e}")
                break
            self.msleep(10)

# ────────────────────────── Peer List Item ─────────────────────────
class PeerItem(QListWidgetItem):
    def __init__(self, node_id:str, status:str="Online"):
        super().__init__()
        self.node_id   = node_id
        self.last_seen = datetime.now()
        self.status    = status
        self.refresh()

    # —— called whenever new activity / timer tick ——
    def touch(self, status:str="Online"):
        self.last_seen = datetime.now()
        self.status    = status
        self.refresh()

    def refresh(self):
        ago = self._ago_string()
        colour = {"Online":"#57b956","Recent":"#e0a800","Offline":"#888888"}.get(self.status,"#57b956")
        self.setText(f"{self.node_id}\n"
                     f"<span style='color:{colour};font-size:10px'>Last seen: {ago}</span>")
        self.setData(Qt.UserRole, self.node_id)

    def _ago_string(self)->str:
        diff = int((datetime.now() - self.last_seen).total_seconds())
        if diff < 60:      return "just now"
        if diff < 3600:    return f"{diff//60}m ago"
        if diff < 86400:   return f"{diff//3600}h ago"
        return f"{diff//86400}d ago"

# ────────────────────────── Main Window ────────────────────────────
class SerialChatApp(QMainWindow):
    # —— UI initialisation ——
    def __init__(self):
        super().__init__()
        self.worker       = SerialWorker()
        self.peers        = {}      # node_id -> PeerItem
        self.chat_history = {}      # node_id -> list[dict]
        self.current_peer = None

        self.worker.data_received.connect(self.on_serial)
        self.worker.error_occurred.connect(lambda m: self.system_msg(f"⚠️ {m}"))
        self.worker.connection_changed.connect(self.on_conn_changed)

        self._build_ui()
        self._apply_palette()
        self._start_timers()

    def _build_ui(self):
        self.setWindowTitle("Enhanced Serial Chat Monitor")
        self.resize(1000, 650)
        cw = QWidget(); self.setCentralWidget(cw)
        outer = QVBoxLayout(cw)

        # —— Connection bar ——
        con_bar = QFrame(); con_bar.setFixedHeight(80)
        hl = QHBoxLayout(con_bar)
        hl.addWidget(QLabel("Port:"))
        self.port_cmb = QComboBox(); self._refresh_ports()
        hl.addWidget(self.port_cmb)
        hl.addWidget(QLabel("Baud:"))
        self.baud_cmb = QComboBox(); self.baud_cmb.addItems(["9600","115200","230400"])
        self.baud_cmb.setCurrentText("115200")
        hl.addWidget(self.baud_cmb)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._toggle_conn)
        hl.addWidget(self.btn_connect)
        btn_refresh = QPushButton("🔄"); btn_refresh.clicked.connect(self._refresh_ports)
        hl.addWidget(btn_refresh)
        self.btn_get = QPushButton("Get Peers"); self.btn_get.setEnabled(False)
        self.btn_get.clicked.connect(self._request_peers)
        hl.addWidget(self.btn_get); hl.addStretch()
        outer.addWidget(con_bar)

        # —— Splitter: peer list ─ chat —— 
        split = QSplitter(Qt.Horizontal); outer.addWidget(split,1)

        # left panel
        left = QWidget(); lyt = QVBoxLayout(left)
        lbl = QLabel("📋 Peer List"); lbl.setFont(QFont("Arial",14, QFont.Bold)); lbl.setAlignment(Qt.AlignCenter)
        lyt.addWidget(lbl)
        self.peer_count = QLabel("0 peers online"); self.peer_count.setAlignment(Qt.AlignCenter)
        self.peer_count.setStyleSheet("color:#88c0d0;font-size:11px")
        lyt.addWidget(self.peer_count)
        self.peer_list = QListWidget(); self.peer_list.itemClicked.connect(self.on_peer_select)
        lyt.addWidget(self.peer_list,1)
        split.addWidget(left); left.setFixedWidth(280)

        # right panel
        right = QWidget(); r = QVBoxLayout(right)
        self.chat_header = QLabel("💬 Select a peer to start chatting")
        self.chat_header.setFont(QFont("Arial",14,QFont.Bold)); self.chat_header.setAlignment(Qt.AlignCenter)
        r.addWidget(self.chat_header)
        self.chat_box = QTextEdit(); self.chat_box.setReadOnly(True); self.chat_box.setFont(QFont("Consolas",10))
        r.addWidget(self.chat_box,1)
        hl2 = QHBoxLayout()
        self.msg_edit = QLineEdit(); self.msg_edit.setPlaceholderText("Type message and press Enter")
        self.msg_edit.returnPressed.connect(self._send_msg)
        hl2.addWidget(self.msg_edit,1)
        btn_send = QPushButton("Send ➤"); btn_send.clicked.connect(self._send_msg)
        hl2.addWidget(btn_send)
        r.addLayout(hl2)
        split.addWidget(right); split.setSizes([300,700])

        # status bar
        self.status = QStatusBar(); self.setStatusBar(self.status); self.status.showMessage("Disconnected")

    # —— Palette ——
    def _apply_palette(self):
        pal = QPalette()
        pal.setColor(QPalette.Window, QColor(32,32,32))
        pal.setColor(QPalette.WindowText,QColor(220,220,220))
        pal.setColor(QPalette.Base, QColor(24,24,24))
        pal.setColor(QPalette.AlternateBase, QColor(38,38,38))
        pal.setColor(QPalette.Text,QColor(220,220,220))
        pal.setColor(QPalette.Button,QColor(38,38,38))
        pal.setColor(QPalette.ButtonText,QColor(200,200,200))
        pal.setColor(QPalette.Highlight,QColor(58,120,190))
        pal.setColor(QPalette.HighlightedText,QColor(32,32,32))
        self.setPalette(pal)

        # CSS for widgets
        self.setStyleSheet("""
            QMainWindow{background:#202324;}
            QTextEdit,QListWidget{border:1px solid #444;border-radius:4px;background:#1e1e1e;color:#ddd;}
            QLineEdit{border:1px solid #555;border-radius:3px;padding:6px;background:#272727;color:white;}
            QPushButton{background:#1f6c80;color:white;border:none;padding:8px;border-radius:4px;}
            QPushButton:hover{background:#2489a3;} QPushButton:disabled{background:#444;}
        """)

    # —— Timers ——
    def _start_timers(self):
        self.timer = QTimer(); self.timer.timeout.connect(self._tick)
        self.timer.start(30000)   # 30-second last-seen refresh

    def _tick(self):
        for item in self.peers.values():
            # mark as Recent/Offline if needed
            diff = (datetime.now() - item.last_seen).total_seconds()
            if diff>600:   item.status="Offline"
            elif diff>300: item.status="Recent"
            item.refresh()
        self._update_peercount()

    # ────────────────── Serial  helpers ──────────────────
    def _refresh_ports(self):
        self.port_cmb.clear()
        self.port_cmb.addItems([p.device for p in serial.tools.list_ports.comports()])

    def _toggle_conn(self):
        if self.btn_connect.text()=="Connect":
            port = self.port_cmb.currentText()
            baud = int(self.baud_cmb.currentText())
            if not port: QMessageBox.warning(self,"Select Port","Choose a serial port"); return
            if self.worker.connect_serial(port, baud):
                self.system_msg(f"🔗 Attempting connection {port}@{baud}")
        else:
            self.worker.disconnect_serial()
            self.system_msg("❌ Disconnected")

    def on_conn_changed(self, ok:bool):
        if ok:
            self.btn_connect.setText("Disconnect")
            self.btn_get.setEnabled(True)
            self.status.showMessage(f"Connected: {self.port_cmb.currentText()}")
            self.system_msg("✅ Connected – click Get Peers")
        else:
            self.btn_connect.setText("Connect")
            self.btn_get.setEnabled(False)
            self.status.showMessage("Disconnected")
            self.peer_list.clear(); self.peers.clear(); self._update_peercount()
            self.current_peer=None; self.chat_header.setText("💬 Select a peer…")

    # ─────────────────── Outbound commands ──────────────────
    def _request_peers(self):
        self.worker.send_data("TYPE:REQ;DATA:PeerList;")

    def _send_msg(self):
        txt = self.msg_edit.text().strip()
        if not (txt and self.current_peer): return
        self.worker.send_data(f"TYPE:SEND;DATA:{txt};ID:{self.current_peer};")
        self.add_chat("You", txt, outgoing=True)
        self.msg_edit.clear()

    # ─────────────────── Serial inbound ─────────────────────
    def on_serial(self, line:str):
        try:
            parts = [p for p in line.split(";") if ":" in p]
            fields = {k.strip().upper():v.strip() for k,v in (p.split(":",1) for p in parts)}
            t = fields.get("TYPE","").upper()
            if   t=="PEERLIST": self._handle_peerlist(fields.get("DATA",""))
            elif t=="UPDATE"  : self._handle_update(fields)
            elif t=="NOTIF"   : self._notify(fields.get("DATA",""))
            else: self.system_msg(f"📨 {line}")
        except Exception as e:
            self.system_msg(f"⚠️ Parse error: {e}")

    # —— specific handlers ——
    def _handle_peerlist(self, data:str):
        nodes = [n.strip() for n in data.split(",") if n.strip()]
        for nid in nodes:
            if nid in self.peers:
                self.peers[nid].touch()
            else:
                it = PeerItem(nid); self.peers[nid]=it; self.peer_list.addItem(it)
        # remove vanished peers
        vanished = set(self.peers)-set(nodes)
        for nid in vanished:
            row = self.peer_list.row(self.peers[nid])
            self.peer_list.takeItem(row); del self.peers[nid]
        self._update_peercount()
        self.system_msg(f"📋 Peer list updated ({len(nodes)} online)")

    def _handle_update(self, f:dict):
        nid = f.get("ID"); msg = f.get("MSG","")
        status = f.get("STATUS","Online")
        if nid:
            if nid not in self.peers:
                self.peers[nid]=PeerItem(nid,status); self.peer_list.addItem(self.peers[nid])
            self.peers[nid].touch(status)
        if msg: self.add_chat(nid, msg, outgoing=False)

    # ─────────────────── UI helpers ─────────────────────────
    def on_peer_select(self,item:QListWidgetItem):
        self.current_peer = item.data(Qt.UserRole)
        self.chat_header.setText(f"💬 Chat with {self.current_peer}")
        self.chat_box.clear()
        for m in self.chat_history.get(self.current_peer,[]):
            self._append_chat_html(**m)

    def add_chat(self, sender:str, text:str, outgoing:bool):
        ts = time.strftime("%H:%M:%S")
        self.chat_history.setdefault(self.current_peer, []).append(
            dict(sender=sender,text=text,time=ts,outgoing=outgoing))
        self._append_chat_html(sender,text,ts,outgoing)

    def _append_chat_html(self, sender:str, text:str, time:str, outgoing:bool):
        colour = "#EBCB8B" if outgoing else "#A3BE8C"
        align  = "right" if outgoing else "left"
        self.chat_box.append(
            f"<div style='text-align:{align};margin:4px'>"
            f"<span style='color:#888;font-size:10px'>[{time}]</span><br>"
            f"<b>{sender}:</b> <span style='color:{colour}'>{text}</span></div>"
        )

    def system_msg(self, txt:str):
        self.chat_box.append(f"<div style='text-align:center;color:#d08770;font-style:italic'>{txt}</div>")

    def _update_peercount(self):
        self.peer_count.setText(f"{len(self.peers)} peers online")

    # ─────────────────── Notification banner ─────────────────
    def _notify(self, msg:str):
        banner = QLabel(msg, self)
        banner.setStyleSheet("""
            QLabel{background:#23272f;color:#ffd700;font-weight:bold;
                   border:2px solid #88c0d0;border-radius:6px;padding:10px;}
        """)
        banner.adjustSize()
        banner.move((self.width()-banner.width())//2, 12)
        banner.setWindowFlags(Qt.ToolTip)
        banner.show()
        QTimer.singleShot(2000, banner.close)

# ─────────────────────────── main ───────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = SerialChatApp(); win.show()
    sys.exit(app.exec_())
