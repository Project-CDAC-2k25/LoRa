import sys, time, serial, serial.tools.list_ports
from datetime import datetime
from PyQt5.QtCore   import Qt, QThread, pyqtSignal, QTimer, QSize
from PyQt5.QtGui    import QPalette, QColor, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QListWidget, QListWidgetItem, QTextEdit, QLineEdit, QPushButton,
    QLabel, QComboBox, QSplitter, QFrame, QMessageBox, QStatusBar,QStackedWidget
)

# ------------ Serial Worker (same as your previous code) --------------
class SerialWorker(QThread):
    data_received       = pyqtSignal(str)
    error_occurred      = pyqtSignal(str)
    connection_changed  = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self._port = None
        self._running = False

    def connect_serial(self, port:str, baud:int)->bool:
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
        if self._port and self._port.is_open:
            try:
                self._port.write((line).encode())       #   Line which sends data to serial port
                return True
            except Exception as e:
                self.error_occurred.emit(f"Send failed: {e}")
        return False

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

# ------------- Custom Peer Widget (for WhatsApp-style peer row) ------------

from PyQt5.QtWidgets import QSizePolicy

class PeerWidget(QWidget):
    def __init__(self, name, preview, last_seen, status_color, unread=False):
        super().__init__()
        main = QVBoxLayout(self)
        main.setSpacing(5)                       # <-- more vertical space between rows
        main.setContentsMargins(10, 6, 10, 6)  # <-- more top/bottom margin
        self.setStyleSheet("""
            background-color: #23282f;
            border-radius: 8px;
            margin: 0px;
        """)

        # Top row: name + dot
        row1 = QHBoxLayout()
        name_lbl = QLabel(name)
        font = QFont("Arial", 12, QFont.Bold if unread else QFont.Normal)
        name_lbl.setFont(font)
        name_lbl.setStyleSheet("color: #fafafa;")
        dot_lbl = QLabel("●") # Unicode dot
        dot_lbl.setStyleSheet(f"color: {status_color}; font-size:15px; margin-left:6px;")
        row1.addWidget(name_lbl)
        row1.addStretch(1)
        row1.addWidget(dot_lbl)
        main.addLayout(row1)

        # Preview
        preview_lbl = QLabel(preview)
        preview_lbl.setFont(QFont("Arial", 11))
        preview_lbl.setStyleSheet("color: #7fc4ff; margin-top:1px;")
        main.addWidget(preview_lbl)

        # Last seen, right bottom
        row2 = QHBoxLayout()
        row2.addStretch(1)
        last_lbl = QLabel(last_seen)
        last_lbl.setFont(QFont("Arial", 5))
        last_lbl.setStyleSheet("color: #888; margin-bottom:2px; margin-right:1px;")
        row2.addWidget(last_lbl)
        main.addLayout(row2)

        if unread:
            self.setStyleSheet(self.styleSheet() + "background: #203e57;")

        self.setMinimumHeight(54)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)


# ------------- PeerItem for app logic only -------------
class PeerItem:
    def __init__(self, node_id:str):
        self.node_id = node_id
        self.last_seen = datetime.now()
        self.status = "Online"
        self.latest_preview = ""
        self.unread = False

    def touch(self, status:str="Online"):
        self.last_seen = datetime.now()
        self.status = status

    def set_preview(self, text:str, unread=True):
        self.latest_preview = text
        self.unread = unread

    def clear_preview(self):
        self.latest_preview = ""
        self.unread = False

    def ago_string(self):
        diff = int((datetime.now() - self.last_seen).total_seconds())
        if diff < 60:      return "just now"
        if diff < 3600:    return f"{diff//60}m ago"
        if diff < 86400:   return f"{diff//3600}h ago"
        return f"{diff//86400}d ago"

# ------------- Main Window -------------
class SerialChatApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker       = SerialWorker()
        self.peers        = {}      # node_id -> PeerItem
        self.chat_history = {}      # node_id -> list[dict]
        self.msgid_table  = {}      # per-peer outgoing msg: id counter
        self.current_peer = None

        self.worker.data_received.connect(self.on_serial)
        self.worker.error_occurred.connect(lambda m: self.system_msg(f"⚠️ {m}"))
        self.worker.connection_changed.connect(self.on_conn_changed)

        self._build_ui()
        self._apply_palette()
        self._start_timers()

    def _build_ui(self):
        self.setWindowTitle("Enhanced Serial Chat Monitor")
        self.resize(1000, 660)
        cw = QWidget(); self.setCentralWidget(cw)
        outer = QVBoxLayout(cw)

        self.banner = QLabel("", self)
        self.banner.setAlignment(Qt.AlignCenter)
        self.banner.setStyleSheet("""
            QLabel{background:#23272f;color:#ffd700;font-weight:bold;
                   border:2px solid #88c0d0;border-radius:6px;padding:10px;
                   font-size:15px;}
        """)
        self.banner.setVisible(False)
        outer.addWidget(self.banner)

        con_bar = QFrame(); con_bar.setFixedHeight(80)
        hl = QHBoxLayout(con_bar)
        port=QLabel("Port:"); port.setStyleSheet("color:#ffd700")
        hl.addWidget(port)
        self.port_cmb = QComboBox(); self._refresh_ports()
        hl.addWidget(self.port_cmb)
        baud=QLabel("Baud:"); baud.setStyleSheet("color:#eeeeee")
        hl.addWidget(baud)
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
        self.btn_debug = QPushButton("Debug")
        self.btn_debug.setCheckable(True)
        self.btn_debug.setStyleSheet("background:#454f5b;color:#ffc107;border-radius:4px;")
        self.btn_debug.clicked.connect(self._toggle_debug_panel)
        hl.addWidget(self.btn_debug)

        outer.addWidget(con_bar)

        split = QSplitter(Qt.Horizontal); outer.addWidget(split,1)

        # left panel - peer list
        left = QWidget(); lyt = QVBoxLayout(left)
        lbl = QLabel("📋 Peer List"); lbl.setFont(QFont("Arial",14, QFont.Bold)); lbl.setAlignment(Qt.AlignCenter)
        lyt.addWidget(lbl)
        lbl.setStyleSheet("color:#bbbbbb")
        self.peer_count = QLabel("0 peers online"); self.peer_count.setAlignment(Qt.AlignCenter)
        self.peer_count.setStyleSheet("color:#88c0d0;font-size:11px")
        lyt.addWidget(self.peer_count)
        self.peer_list = QListWidget(); self.peer_list.itemClicked.connect(self.on_peer_select)
        lyt.addWidget(self.peer_list,1)
        split.addWidget(left); left.setFixedWidth(280)

        # right - chat
        right = QWidget(); r = QVBoxLayout(right)
        self.right_stack = QStackedWidget()
        
        # Chat Panel (existing code)
        self.chat_panel = QWidget()
        r = QVBoxLayout(self.chat_panel)  #... add chat_header, chat_box, etc.
        self.chat_header = QLabel("💬 Select a peer to start chatting")
        self.chat_header.setFont(QFont("Arial",14,QFont.Bold)); 
        self.chat_header.setStyleSheet("color:#88c0d0")
        self.chat_header.setAlignment(Qt.AlignCenter)
        r.addWidget(self.chat_header)
        self.chat_box = QTextEdit(); self.chat_box.setReadOnly(True)
        
        r.addWidget(self.chat_box,1)
        hl2 = QHBoxLayout()
        self.msg_edit = QLineEdit(); self.msg_edit.setPlaceholderText("Type and press Enter")
        self.msg_edit.returnPressed.connect(self._send_msg)
        hl2.addWidget(self.msg_edit,1)
        btn_send = QPushButton("Send ➤"); btn_send.clicked.connect(self._send_msg)
        hl2.addWidget(btn_send)
        r.addLayout(hl2)

        self.right_stack.addWidget(self.chat_panel)

        # Debug Panel
        self.debug_panel = QWidget()
        dlyt = QVBoxLayout(self.debug_panel)
        self.debug_header = QLabel("🛠️ UART Debug Console"); self.debug_header.setFont(QFont("Arial",13,QFont.Bold))
        self.debug_header.setStyleSheet("color:#ffc107;margin-bottom:6px;")
        dlyt.addWidget(self.debug_header)
        self.debug_console = QTextEdit(); self.debug_console.setReadOnly(True)
        self.debug_console.setFont(QFont("Consolas",11))
        self.debug_console.setStyleSheet("background:#23232c;color:#7fffd4;border-radius:6px;")
        dlyt.addWidget(self.debug_console,1)
        self.right_stack.addWidget(self.debug_panel)

        # Now the right panel swaps between chat_panel [0] & debug_panel [1]
        self.right_stack.setCurrentIndex(0)
        split.addWidget(self.right_stack)
        self.status = QStatusBar(); self.status.setStyleSheet("color:#fff");self.setStatusBar(self.status); self.status.showMessage("Disconnected")

    def _toggle_debug_panel(self):
        if self.btn_debug.isChecked():
            self.right_stack.setCurrentIndex(1)
            self.btn_debug.setText("Close Debug")
        else:
            self.right_stack.setCurrentIndex(0)
            self.btn_debug.setText("Debug")

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
        self.setStyleSheet("""
            QTextEdit,QListWidget{border:1px solid #444;border-radius:4px;background:#181d1e;color:#eee;}
            QLineEdit{border:1px solid #555;border-radius:3px;padding:6px;background:#232727;color:white;}
            QPushButton{background:#247ca3;color:white;border:none;padding:8px;border-radius:4px;}
            QPushButton:hover{background:#2c97ca;} QPushButton:disabled{background:#444;}
        """)

    def _start_timers(self):
        self.timer = QTimer(); self.timer.timeout.connect(self._tick)
        self.timer.start(30000)

    def _tick(self):
        for item in self.peers.values():
            diff = (datetime.now() - item.last_seen).total_seconds()
            if diff>600:   item.status="Offline"
            elif diff>300: item.status="Recent"
        self._refresh_peerlist()
        self._update_peercount()

    def _refresh_ports(self):
        self.port_cmb.clear()
        self.port_cmb.addItems([p.device for p in serial.tools.list_ports.comports()])

    def _toggle_conn(self):
        if self.btn_connect.text()=="Connect":
            port = self.port_cmb.currentText()
            baud = int(self.baud_cmb.currentText())
            if not port: QMessageBox.warning(self,"Select Port","Choose a serial port"); return
            if self.worker.connect_serial(port, baud):
                self.system_msg(f" Attempting connection {port}@{baud} ....")
        else:
            self.worker.disconnect_serial()
            self.system_msg(" Disconnected ... !")

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

    def _request_peers(self):
        self.worker.send_data("TYPE:REQ;DATA:PeerList;;")

    def _send_msg(self):
        txt = self.msg_edit.text().strip()
        if not (txt and self.current_peer): return
        msgid = self.msgid_table.get(self.current_peer,1)
        send_str = f"TYPE:SEND;DATA:{txt};ID:{self.current_peer};MSGID:{msgid};;"
        self.worker.send_data(send_str)
        self.add_chat("You", txt, outgoing=True, msgid=msgid)
        self.msg_edit.clear()
        self.msgid_table[self.current_peer] = (msgid+1)%256

    def on_serial(self, line:str):
        try:
            # Log EVERY incoming UART line, raw as received
            ts = datetime.now().strftime("[%H:%M:%S]")
            self.debug_console.append(f"<span style='color:#888'>{ts}</span> <span style='color:#ffd700'>{line}</span>")
            
            raw = line.rstrip(';')
            fields = {}
            for part in raw.split(";"):
                if ":" in part:
                    k,v = part.split(":",1)
                    fields[k.strip().upper()] = v.strip()
            t = fields.get("TYPE","").upper()
            if   t=="PEERLIST": self._handle_peerlist(fields.get("DATA",""))
            elif t=="UPDATE"  : self._handle_update(fields)
            elif t=="NOTIF"   : self._notify(fields.get("DATA",""))
            elif t=="ACK"     : self._handle_ack(fields)
            else: self.system_msg(f"📨 {line}")
        except Exception as e:
            self.system_msg(f"⚠️ Parse error: {e}")

    def _handle_peerlist(self, data:str):
        nodes = [n.strip() for n in data.split(",") if n.strip()]
        for nid in nodes:
            if nid in self.peers:
                self.peers[nid].touch()
            else:
                self.peers[nid]=PeerItem(nid)
        vanished = set(self.peers)-set(nodes)
        for nid in vanished:
            del self.peers[nid]
        self._refresh_peerlist()
        self._update_peercount()
        self.system_msg(f"📋 Peer list updated ({len(nodes)} online)")

    def _handle_update(self, f:dict):
        nid = f.get("ID"); msg = f.get("MSG",""); status = f.get("STATUS","Online")
        msgid = f.get("MSGID",None)
        if not nid: return
        if nid not in self.peers:
            self.peers[nid]=PeerItem(nid)
        self.peers[nid].touch(status)
        if nid not in self.chat_history:
            self.chat_history[nid] = []
        self.chat_history[nid].append(dict(sender=nid, text=msg, time=time.strftime("%H:%M:%S"),
                                           outgoing=False, msgid=msgid, acked=False))
        # Preview for non-viewed peer
        if self.current_peer != nid:
            self.peers[nid].set_preview(msg, unread=True)
        else:
            # Show in chat window and clear preview if right peer
            self._append_chat_html(nid, msg, time.strftime("%H:%M:%S"), False, msgid, False)
            self.peers[nid].clear_preview()
        self._refresh_peerlist()
        self._update_peercount()

    def on_peer_select(self, item:QListWidgetItem):
        self.current_peer = item.data(Qt.UserRole)
        self.chat_header.setText(f"💬 Chat with {self.current_peer}")
        self.peers[self.current_peer].clear_preview()
        self._refresh_peerlist()
        self.chat_box.clear()
        for m in self.chat_history.get(self.current_peer, []):
            self._append_chat_html(**m)

    def add_chat(self, sender, text, outgoing, msgid=None, acked=False):
        ts = time.strftime("%H:%M:%S")
        peer = self.current_peer
        if peer not in self.chat_history:
            self.chat_history[peer] = []
        self.chat_history[peer].append(dict(sender=sender, text=text, time=ts,
                                            outgoing=outgoing, msgid=msgid, acked=acked))
        if self.current_peer == peer:
            self._append_chat_html(sender, text, ts, outgoing, msgid, acked)

    def _append_chat_html(self, sender, text, time, outgoing, msgid=None, acked=False):
        # WhatsApp-style bubbles (LEFT: incoming, RIGHT: outgoing)
        incoming_style = (
            "background:#223026;border-radius:10px 10px 10px 2px;color:#d8eed4;"
            "display:inline-block;padding:10px 14px;font-size:15px;max-width:75%;"
        )
        outgoing_style = (
            "background:#2d3948;border-radius:10px 10px 2px 10px;color:#ffe2b8;"
            "display:inline-block;padding:10px 14px;font-size:15px;max-width:75%;"
        )
        ticks = ""
        if outgoing:
            ticks = '<span style="color:#57b956;font-size:14px;padding-left:5px;">✔✔</span>' if acked else \
                    '<span style="color:#bbb;font-size:14px;padding-left:5px;">✔</span>'
        if outgoing:
            html = f"""<table width='100%'><tr>
                <td width='50%'></td>
                <td width='50%' align='right'>
                    <span style='color:#aaa;font-size:10px'>[{time}]</span>{ticks}<br>
                    <span style="{outgoing_style}">{text}</span>
                </td></tr></table>
            """
        else:
            html = f"""<table width='100%'><tr>
                <td width='50%' align='left'>
                    <span style='color:#aaa;font-size:10px'>[{time}]</span><br>
                    <span style="{incoming_style}">{text}</span>
                </td>
                <td width='50%'></td>
                </tr></table>
            """
        self.chat_box.append(html)

    def system_msg(self, txt:str):
        #ts = datetime.now().strftime("[%H:%M:%S]")
        # Color/style as you wish (here yellow-ish for system/info)
        #self.debug_console.append(
          #  f"<span style='color:#d08770;font-style:italic'>{ts} {txt}</span>")
        pass

    def _update_peercount(self):
        self.peer_count.setText(f"{len(self.peers)} peers online")

    def _refresh_peerlist(self):
        self.peer_list.clear()
        for nid in sorted(self.peers):
            obj = self.peers[nid]
            # Status dot color, preview, last seen
            status_col = {"Online": "#57b956", "Recent": "#e0a800", "Offline": "#888888"}.get(obj.status, "#888888")
            preview = obj.latest_preview or ""
            preview = (preview[:13] + "...") if len(preview) > 13 else preview
            last_seen = obj.ago_string()
            unread = obj.unread
            item = QListWidgetItem()
            item.setSizeHint(QSize(210, 90))
            self.peer_list.addItem(item)
            w = PeerWidget(obj.node_id, preview, last_seen, status_col, unread)
            self.peer_list.setItemWidget(item, w)
            item.setData(Qt.UserRole, nid)

    def _notify(self, msg:str):
        self.banner.setText(msg)
        self.banner.setVisible(True)
        QTimer.singleShot(2000, lambda: self.banner.setVisible(False))

    def _handle_ack(self, fields):
        # Get peer ID and MSGID from ACK string
        peer_id = fields.get("ID")
        ackmsgid = fields.get("MSGID")
        
        if not peer_id or ackmsgid is None:
            return
        # Look up correct peer's chat history
        chs = self.chat_history.get(peer_id, [])
        # Find outgoing message with matching MSGID
        for entry in reversed(chs):
            if entry.get('outgoing') and str(entry.get('msgid'))==str(ackmsgid) and not entry.get('acked',False):
                entry['acked'] = True
                # If you're viewing that peer, refresh bubbles
                if self.current_peer == peer_id:
                    self._refresh_current_chat(peer_id)
                break

    def _refresh_current_chat(self, peer):
        self.chat_box.clear()
        for m in self.chat_history.get(peer,[]):
            self._append_chat_html(**m)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = SerialChatApp(); win.show()
    sys.exit(app.exec_())
