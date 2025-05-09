import sys, time, json
import serial
import serial.tools.list_ports
from threading import Thread
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget,
    QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QPushButton, QTextEdit, QLineEdit,
    QTableWidget, QHeaderView, QAbstractItemView,
    QInputDialog, QTableWidgetItem, QMessageBox,
    QToolButton, QDialog,QTextBrowser
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont,QIcon
import markdown
MONOKAI_QSS = r"""
#central {
    background-color: #f8f8f2;
    border-radius: 12px;
}
/* Titlebar buttons */
QToolButton {
    background: transparent;
    color: #272822;
    border: none;
    font-size: 14pt;
    margin: 0 4px;
}
QToolButton:hover {
    color: #FF5555;
}

QTabBar::tab {
    background: #3E3D32;
    color: #A6E22E;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
    padding: 8px;
    margin: 2px;
}
QTabBar::tab:selected {
    background: #A6E22E;
    color: #272822;
}
QComboBox, QPushButton, QLineEdit {
    background-color: #272822;
    color: #F8F8F2;
    border: 1px solid #A6E22E;
    border-radius: 8px;
    padding: 4px;
}
QComboBox QAbstractItemView {
    background-color: #272822;
    selection-background-color: #A6E22E;
    color: #F8F8F2;
    border: 1px solid #A6E22E;
    border-radius: 8px;
}
QPushButton {
    background-color: #A6E22E;
    color: #272822;
}
QPushButton:hover {
    background-color: #C0FF3E;
}
QPushButton#checkRS485Btn {
    background-color: #FF5555;
    color: #272822;
    border-radius: 8px;
}
QPushButton#checkRS485Btn:hover {
    background-color: #FF7777;
}

QLabel#statusDot {
    font-size: 20px;
}
QTextEdit {
    background-color: #272822;
    color: #F8F8F2;
    border: 1px solid #A6E22E;
    border-radius: 8px;
}
QTableWidget {
    background-color: #272822;
    color: #F8F8F2;
    border: 1px solid #A6E22E;
    border-radius: 8px;
}
QHeaderView::section {
    background-color: #3E3D32;
    color: #A6E22E;
    padding: 4px;
    border: none;
}
"""

class SensorDataGUI(QMainWindow):
    # Signals for UI updates
    log_line = pyqtSignal(str)
    wifi_update = pyqtSignal(bool)
    mqtt_update = pyqtSignal(bool)
    bus_config_received = pyqtSignal(dict)
    slave_list_received = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Config GUI")
        self.resize(1000, 700)
        self.serial_conn = None
        self._populating = False
        self.init_ui()
        # Connect signals
        self.log_line.connect(self.log.append)
        self.wifi_update.connect(self._on_wifi_update)
        self.mqtt_update.connect(self._on_mqtt_update)
        self.bus_config_received.connect(self._on_bus_config)
        self.slave_list_received.connect(self._on_slave_list)
        # Table edits
        self.slave_table.setEditTriggers(QAbstractItemView.AllEditTriggers)
        self.slave_table.itemChanged.connect(self._on_slave_item_changed)

    def init_ui(self):
        central = QWidget();central.setObjectName("central"); self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        tabs = QTabWidget(); layout.addWidget(tabs)
        tabs.addTab(self._create_serial_tab(), "Serial")
        tabs.addTab(self._create_wifi_tab(), "Wi-Fi")
        tabs.addTab(self._create_mqtt_tab(), "MQTT")
        tabs.addTab(self._create_rs485_tab(), "RS-485")
        help_btn = QToolButton()
        help_btn.setText("?")
        help_btn.setToolTip("Help")
        help_btn.clicked.connect(self.open_help)    # new slot
        tabs.setCornerWidget(help_btn, Qt.TopRightCorner)
        
        self.setStyleSheet(MONOKAI_QSS)
        self.setFont(QFont("Courier New", 10))

    def _create_serial_tab(self):
        tab = QWidget(); layout = QVBoxLayout(tab)
        h = QHBoxLayout()
        self.wifi_dot = QLabel('●'); self.wifi_dot.setStyleSheet('color:red')
        self.wifi_lbl = QLabel('WiFi: Disconnected'); h.addWidget(self.wifi_dot); h.addWidget(self.wifi_lbl)
        self.mqtt_dot = QLabel('●'); self.mqtt_dot.setStyleSheet('color:red')
        self.mqtt_lbl = QLabel('MQTT: Disconnected'); h.addWidget(self.mqtt_dot); h.addWidget(self.mqtt_lbl)
        h.addStretch(); layout.addLayout(h)
        h = QHBoxLayout()
        self.btn_scan = QPushButton('Scan'); self.btn_scan.clicked.connect(self.scan_ports)
        self.combo_port = QComboBox(); self.combo_baud = QComboBox()
        for b in ["9600","19200","38400","57600","115200"]: self.combo_baud.addItem(b)
        self.btn_conn = QPushButton('Connect'); self.btn_conn.clicked.connect(self.connect_serial)
        self.btn_disconn = QPushButton('Disconnect'); self.btn_disconn.clicked.connect(self.disconnect_serial)
        h.addWidget(self.btn_scan); h.addWidget(self.combo_port); h.addWidget(self.combo_baud)
        h.addWidget(self.btn_conn); h.addWidget(self.btn_disconn); h.addStretch(); layout.addLayout(h)
        self.log = QTextEdit(); self.log.setReadOnly(True); layout.addWidget(self.log)
        return tab

    
    def _create_wifi_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        layout.addSpacing(100)

        # Create Wi-Fi Settings section
        wifi_layout = QVBoxLayout()
        wifi_layout.setAlignment(Qt.AlignCenter)

        # Create input fields for SSID and Password
        self.ssid_le = QLineEdit()
        self.ssid_le.setPlaceholderText('SSID')
        self.ssid_le.setStyleSheet("padding: 10px; margin-bottom: 20px;")

        self.psw_le = QLineEdit()
        self.psw_le.setPlaceholderText('Password')
        self.psw_le.setEchoMode(QLineEdit.Password)
        self.psw_le.setStyleSheet("padding: 10px; margin-bottom: 20px;")

        # Create buttons for SET_WIFI and DISCONNECT_WIFI
        self.btn_set_wifi = QPushButton('SET_WIFI')
        self.btn_set_wifi.clicked.connect(self.set_wifi)
        self.btn_set_wifi.setStyleSheet("padding: 12px; margin-bottom: 10px;")

        self.btn_disconn_wifi = QPushButton('DISCONNECT_WIFI')
        self.btn_disconn_wifi.clicked.connect(lambda: self.send_cmd('DISCONNECT_WIFI'))
        self.btn_disconn_wifi.setStyleSheet("padding: 12px; margin-bottom: 20px;")

        # Add widgets to the layout
        wifi_layout.addWidget(self.ssid_le)
        wifi_layout.addWidget(self.psw_le)
        wifi_layout.addWidget(self.btn_set_wifi)
        #wifi_layout.addWidget(self.btn_disconn_wifi)

        layout.addLayout(wifi_layout)
        layout.addStretch(1)  # Add stretch to center the layout in the remaining space

        return tab

    def set_wifi(self):
        ssid = self.ssid_le.text()
        password = self.psw_le.text()

        # Check if either of the inputs is empty
        if not ssid or not password:
            # Show a warning message box
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle("Input Error")
            msg.setText("Please fill in both SSID and Password.")
            msg.exec_()
        else:
            # Proceed with sending the command
            self.send_cmd(f'SET_WIFI:{ssid},{password}')

    def _create_mqtt_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        layout.addSpacing(100)

        # Create MQTT Settings section
        mqtt_layout = QVBoxLayout()
        mqtt_layout.setAlignment(Qt.AlignCenter)

        # Create input fields for Server and Port
        self.mqtt_srv_le = QLineEdit()
        self.mqtt_srv_le.setPlaceholderText('Server')
        self.mqtt_srv_le.setStyleSheet("padding: 10px; margin-bottom: 20px;")

        self.mqtt_port_le = QLineEdit()
        self.mqtt_port_le.setPlaceholderText('Port')
        self.mqtt_port_le.setStyleSheet("padding: 10px; margin-bottom: 20px;")

        # Create buttons for SET_MQTT and DISCONNECT_MQTT
        self.btn_set_mqtt = QPushButton('SET_MQTT')
        self.btn_set_mqtt.clicked.connect(self.set_mqtt)
        self.btn_set_mqtt.setStyleSheet("padding: 12px; margin-bottom: 10px;")
        
        self.btn_disconn_mqtt = QPushButton('DISCONNECT_MQTT')
        self.btn_disconn_mqtt.clicked.connect(lambda: self.send_cmd('DISCONNECT_MQTT'))
        self.btn_disconn_mqtt.setStyleSheet("padding: 12px; margin-bottom: 20px;")

        # Add widgets to the layout
        mqtt_layout.addWidget(self.mqtt_srv_le)
        mqtt_layout.addWidget(self.mqtt_port_le)
        mqtt_layout.addWidget(self.btn_set_mqtt)
        #mqtt_layout.addWidget(self.btn_disconn_mqtt)

        layout.addLayout(mqtt_layout)
        layout.addStretch(1)  # Add stretch to center the layout in the remaining space

        return tab

    def set_mqtt(self):
        mqtt_server = self.mqtt_srv_le.text()
        mqtt_port = self.mqtt_port_le.text()

        # Check if either of the inputs is empty
        if not mqtt_server or not mqtt_port:
            # Show a warning message box
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle("Input Error")
            msg.setText("Please fill in both MQTT Server and Port.")
            msg.exec_()
        else:
            # Proceed with sending the command
            self.send_cmd(f'SET_MQTT:{mqtt_server};{mqtt_port}')



    def _create_rs485_tab(self):
        tab = QWidget(); layout = QVBoxLayout(tab)
        h = QHBoxLayout()
        
        self.bus_baud     = QLineEdit('115200'); self.bus_bits = QLineEdit('8')
        self.bus_parity   = QComboBox(); self.bus_parity.addItems(['N','E','O'])
        self.bus_stop     = QLineEdit('1'); self.bus_flow = QComboBox(); self.bus_flow.addItems(['None','RTS/CTS'])
        self.btn_apply_bus= QPushButton('Apply Bus'); self.btn_apply_bus.clicked.connect(self.apply_bus)
        self.btn_refresh_bus= QPushButton('Refresh Bus'); self.btn_refresh_bus.clicked.connect(lambda: self.send_cmd('GET_BUS_CFG'))
        for w in [QLabel('Baud:'), self.bus_baud, QLabel('Bits:'), self.bus_bits,
                  QLabel('Parity:'), self.bus_parity, QLabel('Stop:'), self.bus_stop,
                  QLabel('Flow:'), self.bus_flow, self.btn_apply_bus, self.btn_refresh_bus]: h.addWidget(w)
        h.addStretch(); layout.addLayout(h)
        self.slave_table = QTableWidget(0,6)
        self.slave_table.setHorizontalHeaderLabels(['Addr','Func','Start','Count','Topic','Status'])
        self.slave_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.slave_table)
        h2 = QHBoxLayout()
        btn_add = QPushButton('+ Add'); btn_add.clicked.connect(self.add_slave)
        btn_rem = QPushButton('- Remove'); btn_rem.clicked.connect(self.remove_slave)
        btn_refresh = QPushButton('Refresh Slaves'); btn_refresh.clicked.connect(lambda: self.send_cmd('GET_SLAVE_LIST'))
        for b in [btn_add, btn_rem, btn_refresh]: h2.addWidget(b)
        h2.addStretch(); layout.addLayout(h2)

        # Add a new button to trigger CHECK_RS485
        btn_check_rs485 = QPushButton("Check RS485 Status")
        btn_check_rs485.setObjectName("checkRS485Btn")
        btn_check_rs485.clicked.connect(self.check_rs485)
        layout.addWidget(btn_check_rs485)
        return tab
    

    # --- Serial & Commands ---
    def scan_ports(self):
        self.combo_port.clear()
        for p in serial.tools.list_ports.comports(): self.combo_port.addItem(p.device)
        self.log_line.emit(f'Scanned: {[p.device for p in serial.tools.list_ports.comports()]}')

    def connect_serial(self):
        port = self.combo_port.currentText(); baud = int(self.combo_baud.currentText())
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            return self.log_line.emit(f'Error opening {port}@{baud}: {e}')
        self.log_line.emit(f'Opened {port}@{baud}')
        Thread(target=self._read_loop, daemon=True).start()
        for cmd in ['GET_BUS_CFG','GET_STATUS','GET_SLAVE_LIST','CHECK_RS485']: 
            self.send_cmd(cmd)
            #time.sleep(1)

    def disconnect_serial(self):
        if self.serial_conn: self.serial_conn.close(); self.serial_conn=None; self.log_line.emit('Serial closed')

    def send_cmd(self, cmd):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write((cmd+'\n').encode()); self.log_line.emit(f'➡️ {cmd}')
        else:
            self.log_line.emit('Serial not open')

    def _read_loop(self):
        temp = []
        while self.serial_conn and self.serial_conn.is_open:
            line = self.serial_conn.readline().decode(errors='ignore').strip()
            if line:
                self.log_line.emit(line)
                if line.startswith('WIFI:'):
                    self.wifi_update.emit('Connected' in line)
                elif line.startswith('MQTT:'):
                    self.mqtt_update.emit('Connected' in line)
                elif line.startswith('CFG:BUS:'):
                    _,_,vals = line.partition(':BUS:'); b,db,par,st,fl = vals.split(',')
                    self.bus_config_received.emit({'baud':int(b),'dataBits':int(db),'parity':par,'stopBits':st,'flow':fl})
                elif line.startswith('CFG:SLAVE:'):
                    _,_,rest = line.partition(':SLAVE:'); a,func,s,c,topic = rest.split(',')
                    temp.append({'addr':int(a),'func':int(func),'start':int(s),'count':int(c),'topic':topic})
                elif line=='END_SLAVE_LIST':
                    self._populating=True; self.slave_list_received.emit(temp.copy()); temp.clear(); self._populating=False
                elif line.startswith('STATUS:SLAVE:'):
                    _,_,rest = line.partition(':SLAVE:'); a,status = rest.split(','); self._update_slave_status(int(a),status)

    # --- UI Handlers ---
    def _on_wifi_update(self, ok):
        self.wifi_dot.setStyleSheet(f"color:{'#0f0' if ok else '#f00'}"); self.wifi_lbl.setText('WiFi:'+('Connected' if ok else 'Disconnected'))

    def _on_mqtt_update(self, ok):
        self.mqtt_dot.setStyleSheet(f"color:{'#0f0' if ok else '#f00'}"); self.mqtt_lbl.setText('MQTT:'+('Connected' if ok else 'Disconnected'))

    def _on_bus_config(self, cfg):
        self.bus_baud.setText(str(cfg['baud'])); self.bus_bits.setText(str(cfg['dataBits']))
        idx = self.bus_parity.findText(cfg['parity']); self.bus_parity.setCurrentIndex(idx if idx>=0 else 0)
        self.bus_stop.setText(cfg['stopBits']); idx=self.bus_flow.findText(cfg['flow']); self.bus_flow.setCurrentIndex(idx if idx>=0 else 0)

    def _on_slave_list(self, lst):
        self.slave_table.setRowCount(0)
        for s in lst:
            r = self.slave_table.rowCount(); self.slave_table.insertRow(r)
            self.slave_table.setItem(r,0,QTableWidgetItem(str(s['addr'])))
            self.slave_table.setItem(r,1,QTableWidgetItem(str(s['func'])))
            self.slave_table.setItem(r,2,QTableWidgetItem(str(s['start'])))
            self.slave_table.setItem(r,3,QTableWidgetItem(str(s['count'])))
            self.slave_table.setItem(r,4,QTableWidgetItem(s['topic']))
            self.slave_table.setItem(r,5,QTableWidgetItem('Unknown'))

    def _update_slave_status(self, addr, status):
        for r in range(self.slave_table.rowCount()):
            if int(self.slave_table.item(r,0).text())==addr:
                self.slave_table.setItem(r,5,QTableWidgetItem(status)); return

    # --- Table Edit Handler ---
    def _on_slave_item_changed(self, item):
        if self._populating: return
        row, col = item.row(), item.column()
        addr_item = self.slave_table.item(row,0)
        if not addr_item: return
        try:
            addr = int(addr_item.text())
        except:
            return
        if col in (1,2,3):  # func, start, count
            try:
                func  = int(self.slave_table.item(row,1).text())
                start = int(self.slave_table.item(row,2).text())
                cnt   = int(self.slave_table.item(row,3).text())
                self.send_cmd(f'SET_SLAVE_CFG:{addr};{func};{start};{cnt}')
            except:
                pass
        elif col==4:  # topic
            topic = self.slave_table.item(row,4).text()
            self.send_cmd(f'SET_SLAVE_TOPIC:{addr};{topic}')

    # --- Actions ---
    def apply_bus(self):
        cmd = f"SET_BUS_CFG:{self.bus_baud.text()};{self.bus_bits.text()};{self.bus_parity.currentText()};{self.bus_stop.text()};{self.bus_flow.currentText()}"
        self.send_cmd(cmd)

    def add_slave(self):
        addr,ok = QInputDialog.getInt(self,'Add Slave','Address:',1,1,247)
        if not ok: return
        self.send_cmd(f'ADD_SLAVE:{addr}')
        self.send_cmd('GET_SLAVE_LIST')

    def remove_slave(self):
        sel = self.slave_table.selectedItems()
        if not sel:
            return
        row = sel[0].row()
        addr = int(self.slave_table.item(row, 0).text())

        # Check if there's only one slave address left
        if self.slave_table.rowCount() <= 1:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle("Cannot Remove Slave")
            msg.setText("You cannot delete the last slave address. Please ensure at least one slave is present.")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return

        self.send_cmd(f'REMOVE_SLAVE:{addr}')
        self.send_cmd('GET_SLAVE_LIST')
    
    def check_rs485(self):
        self.send_cmd("CHECK_RS485")

    def open_help(self):
        dlg = HelpDialog(self)
        dlg.exec_()
# MARKDOWN HELP CONTENT
serial_md = """
# Serial Tab Help

- **Scan**: click “Scan” to list ports  
- **Connect**: select port/baud then hit “Connect”  
- **Log**: all traffic shows here
## use Scan Look for port FIRST, Select Baud rate (use 115200) then Connect Before using another tab!
### If first connect cause watchdog (esp reset) it is recommend to disconnect and connect again   
"""

wifi_md = """
# Wi-Fi Tab Help

- Wifi SSID: 
- Wifi password:
- Test wifi:
- Test wifi password:
"""

mqtt_md = """
# MQTT Tab Help
Topic can be edit and sent in rs485 table named topic, Default port 1883
"""

rs485_md = """
# RS-485 Tab Help

### Selectable Options:
- **Baud Rate**: Choose from common values (e.g., 9600,19200,38400,57600,115200).
- **Data Bits**: Typically 7,8 bits.
- **Stop Bits**: Choose either 1 or 2 stop bits.
- **Parity**: Options include None (N), Even (E), Odd (O).

### RS485 Status:
- **Online/Offline Status**: If the status is unknown, use the "CHECK_RS485" command to check the RS485 health.

### Common Issues:
- When OFFLINE appear please at least check 2 times by using RS485 check button then open the serial tab.
- When no data being parse (No value in the table) use refresh slaves button again
"""
class HelpDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Help")
        self.resize(900, 700)


        tabs = QTabWidget(self)
        tabs.addTab(self._make_page("Serial", serial_md),   "Serial")
        tabs.addTab(self._make_page("Wi-Fi", wifi_md),      "Wi-Fi")
        tabs.addTab(self._make_page("MQTT", mqtt_md),       "MQTT")
        tabs.addTab(self._make_page("RS-485", rs485_md),     "RS-485")

        layout = QVBoxLayout(self)
        layout.addWidget(tabs)

    def _make_page(self, title, md_text):
        browser = QTextBrowser()
        css = """
        <style>
          body { font-family: Consolas, monospace; color: #D4D4D4; background: #1E1E1E; }
          h1,h2,h3 { color: #569CD6; }
          code { background: #252526; padding: 2px 4px; border-radius: 2px; }
          pre { background: #252526; padding: 8px; border-radius: 4px; overflow: auto; }
          ul { margin-left: 20px; }
        </style>
        """
        
        html = css + markdown.markdown(md_text, extensions=['fenced_code','tables'])
        browser.setHtml(html)
        return browser
        
    


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = SensorDataGUI()
    gui.show()
    sys.exit(app.exec_())
