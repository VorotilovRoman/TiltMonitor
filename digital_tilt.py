import os
import sys
import threading
import time
import re
from collections import deque, defaultdict

import serial
import serial.tools.list_ports
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QSpinBox, QPushButton, QTextEdit,
    QComboBox, QCheckBox, QSplitter, QDoubleSpinBox
)
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot, Qt, QTimer
import pyqtgraph as pg
# ---- 3D импорты ----
from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt5.QtOpenGL import QGLWidget



# ----------------------------------------------------------------------
# Работа с последовательным портом (COM-устройство)
# ----------------------------------------------------------------------
class SerialWorker(QObject):
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    error = pyqtSignal(str)
    data_received = pyqtSignal(str)       # сырая строка
    coordinates = pyqtSignal(float, float, float)   # X, Y, Z

    def __init__(self):
        super().__init__()
        self.serial_conn = None
        self.running = False
        self.read_thread = None
        self.lock = threading.RLock()

    def connect_to_device(self, port_name: str, baudrate: int, ending: str):
        with self.lock:
            if self.serial_conn and self.serial_conn.is_open:
                self.disconnect()
            try:
                print(f"[DEBUG] Подключение к {port_name} на скорости {baudrate}")
                self.serial_conn = serial.Serial(
                    port=port_name,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.5
                )
                self.running = True
                self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
                self.read_thread.start()
                self.connected.emit()
                # Отправить пустую команду после подключения
                self.send_command("READ", ending)
            except Exception as e:
                self.error.emit(f"Ошибка подключения: {str(e)}")
                self.serial_conn = None

    def disconnect(self):
        with self.lock:
            self.running = False
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.close()
                except:
                    pass
                self.serial_conn = None
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join(0.1)
            self.disconnected.emit()

    def send_command(self, cmd: str, ending: str):
        with self.lock:
            if not self.serial_conn or not self.serial_conn.is_open:
                self.error.emit("Нет активного соединения")
                return
            full_cmd = cmd + ending
            print(f"[DEBUG] Отправка: {repr(full_cmd)}")
            try:
                self.serial_conn.write(full_cmd.encode('utf-8'))
            except Exception as e:
                self.error.emit(f"Ошибка отправки: {str(e)}")
                self.disconnect()

    def _read_loop(self):
        buffer = bytearray()
        while self.running:
            if not self.serial_conn or not self.serial_conn.is_open:
                break
            try:
                data = self.serial_conn.read(4096)
                if data:
                    buffer.extend(data)
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        line_str = line.decode('utf-8', errors='replace').strip()
                        if line_str:
                            self.data_received.emit(line_str)
                            coords = self._parse_coordinates(line_str)
                            if coords:
                                x, y, z = coords
                                self.coordinates.emit(x, y, z)
            except Exception as e:
                if self.running:
                    self.error.emit(f"Ошибка чтения: {str(e)}")
                    self.disconnect()
                break
        print("[DEBUG] Поток чтения завершён")

    @staticmethod
    def _parse_coordinates(line: str):
        pattern = r"X:\s*([+-]?\d*\.?\d+)\s*Y:\s*([+-]?\d*\.?\d+)\s*Z:\s*([+-]?\d*\.?\d+)"
        match = re.search(pattern, line)
        if match:
            try:
                x = float(match.group(1))
                y = float(match.group(2))
                z = float(match.group(3))
                return x, y, z
            except ValueError:
                return None
        return None


# ----------------------------------------------------------------------
# 3D виджет для отображения вращающегося объекта (куб)
# ----------------------------------------------------------------------


class Rotation3DWidget(QGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.model_scale = 0.05
        self.material_groups = []  # (color_rgb, flat_vertices_list)

        # Загружаем модель и материалы
        if not self._load_obj('Tilt_Switch_Digital.obj', 'Tilt_Switch_Digital.mtl'):
            self.model_scale = 1
            print("Не удалось загрузить модель, будет использован тестовый куб")

    def _load_mtl(self, mtl_path):
        """Читает MTL файл, возвращает словарь {имя_материала: (r,g,b)}"""
        colors = {}
        if not os.path.exists(mtl_path):
            return colors
        current_material = None
        with open(mtl_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if parts[0] == 'newmtl':
                    current_material = parts[1]
                elif parts[0] == 'Kd' and current_material:
                    try:
                        r = float(parts[1])
                        g = float(parts[2])
                        b = float(parts[3])
                        colors[current_material] = (r, g, b)
                    except ValueError:
                        pass
        print(f"Загружено материалов из MTL: {len(colors)}")
        return colors

    def _load_obj(self, obj_path, mtl_path):
        """Загружает OBJ, группирует треугольники по usemtl, центрирует модель."""
        mtl_colors = self._load_mtl(mtl_path)
        vertices = []           # список вершин (x,y,z)
        groups = defaultdict(list)  # имя_материала -> список треугольников (каждый треугольник - [v1,v2,v3])
        current_material = None

        try:
            with open(obj_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split()
                    if parts[0] == 'v':
                        # вершина: v x y z
                        x = float(parts[1])
                        y = float(parts[2])
                        z = float(parts[3])
                        vertices.append((x, y, z))
                    elif parts[0] == 'usemtl':
                        current_material = parts[1]
                    elif parts[0] == 'f':
                        # грань: f v1/vt1/vn1 v2/vt2/vn2 ...
                        # нас интересуют только индексы вершин
                        face_vertices = []
                        for part in parts[1:]:
                            idx = part.split('/')[0]
                            if idx:
                                face_vertices.append(int(idx) - 1)  # OBJ индексация с 1
                        # Преобразуем в треугольники (если грань квадратная или многоугольник)
                        if len(face_vertices) == 3:
                            tri = [face_vertices[0], face_vertices[1], face_vertices[2]]
                            groups[current_material].append(tri)
                        elif len(face_vertices) == 4:
                            groups[current_material].append([face_vertices[0], face_vertices[1], face_vertices[2]])
                            groups[current_material].append([face_vertices[0], face_vertices[2], face_vertices[3]])
                        else:
                            # веер для произвольного многоугольника
                            for i in range(1, len(face_vertices)-1):
                                groups[current_material].append([face_vertices[0], face_vertices[i], face_vertices[i+1]])
        except Exception as e:
            print(f"Ошибка загрузки OBJ: {e}")
            return False

        if not vertices or not groups:
            print("Нет вершин или граней")
            return False

        # Центрирование модели
        xs = [v[0] for v in vertices]
        ys = [v[1] for v in vertices]
        zs = [v[2] for v in vertices]
        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0
        cz = (min(zs) + max(zs)) / 2.0
        print(f"Центр модели: ({cx:.3f}, {cy:.3f}, {cz:.3f})")

        default_color = (0.8, 0.6, 0.4)   # цвет, если материал не найден

        self.material_groups.clear()
        for mat_name, triangles in groups.items():
            color = mtl_colors.get(mat_name, default_color)
            flat = []
            for tri in triangles:
                for vi in tri:
                    vx, vy, vz = vertices[vi]
                    flat.extend([vx - cx, vy - cy, vz - cz])
            if flat:
                self.material_groups.append((color, flat))

        total_tri = sum(len(flat)//3 for _, flat in self.material_groups)
        print(f"Извлечено групп: {len(self.material_groups)}, всего треугольников: {total_tri}")
        return True

    def initializeGL(self):
        glClearColor(0.2, 0.2, 0.2, 1.0)
        glEnable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glEnable(GL_COLOR_MATERIAL)
        glShadeModel(GL_SMOOTH)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h if h else 1, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluLookAt(3, 2, 5, 0, 0, 0, 0, 1, 0)

        # Рисуем оси и сетку (в мировых координатах)
        self._draw_axes()
        self._draw_grid()

        glPushMatrix()
        glScalef(self.model_scale, self.model_scale, self.model_scale)
        glRotatef(self.roll, 1, 0, 0)
        glRotatef(self.pitch, 0, 1, 0)
        glRotatef(self.yaw, 0, 0, 1)

        if self.material_groups:
            for color, flat_verts in self.material_groups:
                glColor3f(*color)
                glBegin(GL_TRIANGLES)
                for i in range(0, len(flat_verts), 3):
                    glVertex3f(flat_verts[i], flat_verts[i+1], flat_verts[i+2])
                glEnd()
        else:
            self.draw_cube()

        glPopMatrix()


    def _draw_axes(self):
        """Рисует цветные оси координат X, Y, Z длиной 2.0."""
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glBegin(GL_LINES)
        # Ось X (красная)
        glColor3f(1, 0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(2, 0, 0)
        # Ось Y (зелёная)
        glColor3f(0, 1, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 2, 0)
        # Ось Z (синяя)
        glColor3f(0, 0, 1)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 2)
        glEnd()
        glLineWidth(1.0)

    def _draw_grid(self):
        """Рисует сетку на плоскости XZ (пол) в диапазоне [-2, 2] с шагом 0.5."""
        glColor3f(0.5, 0.5, 0.5)  # серый цвет
        glLineWidth(1.0)
        glBegin(GL_LINES)
        step = 0.5
        limit = 2.0
        x = -limit
        while x <= limit + 0.01:
            glVertex3f(x, 0, -limit)
            glVertex3f(x, 0, limit)
            x += step
        z = -limit
        while z <= limit + 0.01:
            glVertex3f(-limit, 0, z)
            glVertex3f(limit, 0, z)
            z += step
        glEnd()

    def draw_cube(self):
        """Тестовый куб на случай отсутствия модели"""
        glBegin(GL_QUADS)
        glColor3f(1,0,0); glVertex3f(-0.5,-0.5,0.5); glVertex3f(0.5,-0.5,0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(-0.5,0.5,0.5)
        glColor3f(0,1,0); glVertex3f(-0.5,-0.5,-0.5); glVertex3f(-0.5,0.5,-0.5); glVertex3f(0.5,0.5,-0.5); glVertex3f(0.5,-0.5,-0.5)
        glColor3f(0,0,1); glVertex3f(-0.5,-0.5,-0.5); glVertex3f(-0.5,-0.5,0.5); glVertex3f(-0.5,0.5,0.5); glVertex3f(-0.5,0.5,-0.5)
        glColor3f(1,1,0); glVertex3f(0.5,-0.5,-0.5); glVertex3f(0.5,0.5,-0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(0.5,-0.5,0.5)
        glColor3f(1,0,1); glVertex3f(-0.5,0.5,-0.5); glVertex3f(-0.5,0.5,0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(0.5,0.5,-0.5)
        glColor3f(0,1,1); glVertex3f(-0.5,-0.5,-0.5); glVertex3f(0.5,-0.5,-0.5); glVertex3f(0.5,-0.5,0.5); glVertex3f(-0.5,-0.5,0.5)
        glEnd()

    def set_angles(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.update()

# ----------------------------------------------------------------------
# Главное окно приложения
# ----------------------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Монитор углов наклона (COM-устройство)")
        self.setGeometry(100, 100, 1400, 900)

        # --- Рабочий объект для COM-порта ---
        self.worker = SerialWorker()
        self.worker_thread = QThread()
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.start()

        self.worker.connected.connect(self.on_connected)
        self.worker.disconnected.connect(self.on_disconnected)
        self.worker.error.connect(self.on_error)
        self.worker.coordinates.connect(self.on_coordinates)

        # --- Таймер для периодического опроса ---
        self.poll_timer = QTimer()
        self.poll_timer.timeout.connect(self.send_poll_command)
        self.polling_enabled = False

        # --- Хранение данных для графика ---
        self.all_x = []
        self.all_y = []
        self.all_z = []
        self.last_n_x = deque(maxlen=100)
        self.last_n_y = deque(maxlen=100)
        self.last_n_z = deque(maxlen=100)
        self.use_last_n = False
        self.current_x = self.all_x
        self.current_y = self.all_y
        self.current_z = self.all_z

        # --- UI ---
        self.init_ui()
        self._set_controls_enabled(False)

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # ---------- Группа подключения ----------
        com_group = QGroupBox("Подключение к устройству")
        com_layout = QVBoxLayout(com_group)
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("COM-порт:"))
        self.com_port_combo = QComboBox()
        self.refresh_ports()
        refresh_btn = QPushButton("Обновить")
        refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(self.com_port_combo)
        port_layout.addWidget(refresh_btn)
        port_layout.addStretch()
        com_layout.addLayout(port_layout)

        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Скорость (бод):"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        baud_layout.addWidget(self.baud_combo)
        baud_layout.addStretch()
        com_layout.addLayout(baud_layout)

        ending_layout = QHBoxLayout()
        ending_layout.addWidget(QLabel("Окончание строки:"))
        self.ending_combo = QComboBox()
        self.ending_combo.addItems(["\\n (LF)", "\\r\\n (CR+LF)", "\\r (CR)"])
        self.ending_combo.setCurrentText("\\n (LF)")
        ending_layout.addWidget(self.ending_combo)
        ending_layout.addStretch()
        com_layout.addLayout(ending_layout)

        self.connect_btn = QPushButton("Подключиться")
        self.connect_btn.clicked.connect(self.toggle_connection)
        com_layout.addWidget(self.connect_btn)
        com_layout.addStretch()
        left_layout.addWidget(com_group)

        # ---------- Группа опроса ----------
        poll_group = QGroupBox("Опрос")
        poll_layout = QVBoxLayout(poll_group)
        freq_layout = QHBoxLayout()
        freq_layout.addWidget(QLabel("Интервал опроса (мс):"))
        self.poll_interval_spin = QSpinBox()
        self.poll_interval_spin.setRange(10, 10000)
        self.poll_interval_spin.setValue(50)
        self.poll_interval_spin.setSuffix(" мс")
        self.poll_interval_spin.valueChanged.connect(self.update_poll_interval)
        freq_layout.addWidget(self.poll_interval_spin)
        freq_layout.addStretch()
        poll_layout.addLayout(freq_layout)

        self.poll_btn = QPushButton("Старт опроса")
        self.poll_btn.clicked.connect(self.toggle_polling)
        self.poll_btn.setEnabled(False)
        poll_layout.addWidget(self.poll_btn)
        poll_layout.addStretch()
        left_layout.addWidget(poll_group)

        # ---------- Группа сопоставления осей и начального положения ----------
        axis_group = QGroupBox("Сопоставление осей и начальное положение")
        axis_layout = QVBoxLayout(axis_group)

        axis_layout.addWidget(QLabel("Какой угол датчика вращает какую ось модели?"))

        # Ось X модели (roll)
        hx = QHBoxLayout()
        hx.addWidget(QLabel("Вращение оси X модели:"))
        self.map_roll = QComboBox()
        self.map_roll.addItems(["X датчика", "Y датчика", "Z датчика"])
        self.map_roll.setCurrentIndex(1)
        self.inv_roll = QCheckBox("Инвертировать")
        self.inv_roll.setChecked(True)
        self.disable_roll = QCheckBox("Отключить ось")      # <-- добавлено
        self.disable_roll.setChecked(False)
        hx.addWidget(self.map_roll)
        hx.addWidget(self.inv_roll)
        hx.addWidget(self.disable_roll)                    # <-- добавлено
        axis_layout.addLayout(hx)

        # Ось Y модели (pitch)
        hy = QHBoxLayout()
        hy.addWidget(QLabel("Вращение оси Y модели:"))
        self.map_pitch = QComboBox()
        self.map_pitch.addItems(["X датчика", "Y датчика", "Z датчика"])
        self.map_pitch.setCurrentIndex(0)
        self.inv_pitch = QCheckBox("Инвертировать")
        self.inv_pitch.setChecked(False)
        self.disable_pitch = QCheckBox("Отключить ось")    # <-- добавлено
        self.disable_pitch.setChecked(False)
        hy.addWidget(self.map_pitch)
        hy.addWidget(self.inv_pitch)
        hy.addWidget(self.disable_pitch)                  # <-- добавлено
        axis_layout.addLayout(hy)

        # Ось Z модели (yaw)
        hz = QHBoxLayout()
        hz.addWidget(QLabel("Вращение оси Z модели:"))
        self.map_yaw = QComboBox()
        self.map_yaw.addItems(["X датчика", "Y датчика", "Z датчика"])
        self.map_yaw.setCurrentIndex(2)
        self.inv_yaw = QCheckBox("Инвертировать")
        self.inv_yaw.setChecked(False)
        self.disable_yaw = QCheckBox("Отключить ось")      # <-- добавлено
        self.disable_yaw.setChecked(True)                  # по умолчанию ось Z модели отключена
        hz.addWidget(self.map_yaw)
        hz.addWidget(self.inv_yaw)
        hz.addWidget(self.disable_yaw)                    # <-- добавлено
        axis_layout.addLayout(hz)

        # Разделитель
        axis_layout.addWidget(QLabel("Начальное смещение (градусы) при (0,0,0) датчика"))

        # Смещение для оси X модели
        off_x_layout = QHBoxLayout()
        off_x_layout.addWidget(QLabel("Смещение X модели:"))
        self.offset_roll = QDoubleSpinBox()
        self.offset_roll.setRange(-360.0, 360.0)
        self.offset_roll.setValue(0.0)
        self.offset_roll.setSuffix("°")
        self.offset_roll.setSingleStep(1.0)
        off_x_layout.addWidget(self.offset_roll)
        off_x_layout.addStretch()
        axis_layout.addLayout(off_x_layout)

        # Смещение для оси Y модели
        off_y_layout = QHBoxLayout()
        off_y_layout.addWidget(QLabel("Смещение Y модели:"))
        self.offset_pitch = QDoubleSpinBox()
        self.offset_pitch.setRange(-360.0, 360.0)
        self.offset_pitch.setValue(0.0)
        self.offset_pitch.setSuffix("°")
        self.offset_pitch.setSingleStep(1.0)
        off_y_layout.addWidget(self.offset_pitch)
        off_y_layout.addStretch()
        axis_layout.addLayout(off_y_layout)

        # Смещение для оси Z модели
        off_z_layout = QHBoxLayout()
        off_z_layout.addWidget(QLabel("Смещение Z модели:"))
        self.offset_yaw = QDoubleSpinBox()
        self.offset_yaw.setRange(-360.0, 360.0)
        self.offset_yaw.setValue(0.0)
        self.offset_yaw.setSuffix("°")
        self.offset_yaw.setSingleStep(1.0)
        off_z_layout.addWidget(self.offset_yaw)
        off_z_layout.addStretch()
        axis_layout.addLayout(off_z_layout)

        # Кнопка сброса смещений
        reset_offset_btn = QPushButton("Сбросить смещения в ноль")
        reset_offset_btn.clicked.connect(self.reset_offsets)
        axis_layout.addWidget(reset_offset_btn)

        axis_layout.addStretch()
        left_layout.addWidget(axis_group)

        # ---------- Группа лога ----------
        log_group = QGroupBox("Лог событий")
        log_layout = QVBoxLayout(log_group)
        self.log_checkbox = QCheckBox("Писать лог")
        self.log_checkbox.setChecked(True)
        log_layout.addWidget(self.log_checkbox)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        left_layout.addWidget(log_group)

        # ---------- Группа графика ----------
        graph_group = QGroupBox("График углов")
        graph_layout = QVBoxLayout(graph_group)
        axis_panel = QHBoxLayout()
        self.cb_x = QCheckBox("Показать X (крен)")
        self.cb_x.setChecked(True)
        self.cb_y = QCheckBox("Показать Y (тангаж)")
        self.cb_y.setChecked(True)
        self.cb_z = QCheckBox("Показать Z (рыскание)")
        self.cb_z.setChecked(False)
        for cb in (self.cb_x, self.cb_y, self.cb_z):
            cb.stateChanged.connect(self.update_graph_visibility)
            axis_panel.addWidget(cb)
        axis_panel.addStretch()
        graph_layout.addLayout(axis_panel)

        self.last_n_check = QCheckBox("Отображать только последние 100 точек")
        self.last_n_check.stateChanged.connect(self.toggle_last_n_mode)
        graph_layout.addWidget(self.last_n_check)

        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setLabel('left', 'Угол (градусы)')
        self.graph_widget.setLabel('bottom', 'Номер измерения')
        self.graph_widget.addLegend()
        self.curve_x = self.graph_widget.plot(pen='r', name='X (крен)', visible=True)
        self.curve_y = self.graph_widget.plot(pen='g', name='Y (тангаж)', visible=True)
        self.curve_z = self.graph_widget.plot(pen='b', name='Z (рыскание)', visible=True)
        graph_layout.addWidget(self.graph_widget)

        left_layout.addWidget(graph_group, stretch=1)

        # ---------- Правая панель (3D) ----------
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        self.toggle_3d_btn = QPushButton("Скрыть 3D сцену")
        self.toggle_3d_btn.setCheckable(True)
        self.toggle_3d_btn.setChecked(True)
        self.toggle_3d_btn.clicked.connect(self.toggle_3d_visibility)
        right_layout.addWidget(self.toggle_3d_btn)

        self.gl_widget = Rotation3DWidget()
        self.gl_widget.setMinimumHeight(300)
        right_layout.addWidget(self.gl_widget)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([700, 600])
        main_layout.addWidget(splitter)
        self.update_graph_visibility()

    # --------------------------------------------------------------
    # Прочие методы (COM-порт, опрос, лог и т.д.) без изменений
    # --------------------------------------------------------------
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        current = self.com_port_combo.currentText()
        self.com_port_combo.clear()
        for port in ports:
            self.com_port_combo.addItem(port.device)
        if current and self.com_port_combo.findText(current) >= 0:
            self.com_port_combo.setCurrentText(current)
        elif self.com_port_combo.count() > 0:
            self.com_port_combo.setCurrentIndex(0)

    def _get_ending(self) -> str:
        text = self.ending_combo.currentText()
        if "CR+LF" in text:
            return "\r\n"
        elif "CR" in text:
            return "\r"
        else:
            return "\n"

    def toggle_connection(self):
        if self.connect_btn.text() == "Подключиться":
            port = self.com_port_combo.currentText().strip()
            if not port:
                self.append_log("Ошибка: не выбран COM-порт")
                return
            baud = int(self.baud_combo.currentText())
            self.append_log(f"Подключение к {port} на скорости {baud}...")
            self.worker.connect_to_device(port, baud, self._get_ending())
        else:
            self.worker.disconnect()
            self.append_log("Отключение от устройства")
            self.connect_btn.setText("Подключиться")
            self._set_controls_enabled(False)

    def _set_controls_enabled(self, enabled: bool):
        self.poll_btn.setEnabled(enabled)
        if not enabled:
            self.stop_polling()

    @pyqtSlot()
    def on_connected(self):
        self.append_log("Устройство подключено")
        self.connect_btn.setText("Отключиться")
        self._set_controls_enabled(True)

    @pyqtSlot()
    def on_disconnected(self):
        self.append_log("Соединение разорвано")
        self.connect_btn.setText("Подключиться")
        self._set_controls_enabled(False)

    @pyqtSlot(str)
    def on_error(self, err_msg):
        self.append_log(f"ОШИБКА: {err_msg}")

    def append_log(self, message: str):
        if not self.log_checkbox.isChecked():
            return
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def update_poll_interval(self):
        self.poll_timer.setInterval(self.poll_interval_spin.value())

    def toggle_polling(self):
        if not self.polling_enabled:
            self.start_polling()
        else:
            self.stop_polling()

    def start_polling(self):
        if not self.worker.serial_conn or not self.worker.serial_conn.is_open:
            self.append_log("Не удаётся начать опрос: устройство не подключено")
            return
        self.polling_enabled = True
        self.poll_timer.start(self.poll_interval_spin.value())
        self.poll_btn.setText("Стоп опроса")
        self.append_log(f"Опрос запущен (интервал {self.poll_interval_spin.value()} мс)")

    def stop_polling(self):
        self.poll_timer.stop()
        self.polling_enabled = False
        self.poll_btn.setText("Старт опроса")
        self.append_log("Опрос остановлен")

    def send_poll_command(self):
        if not self.worker.serial_conn or not self.worker.serial_conn.is_open:
            self.stop_polling()
            return
        self.worker.send_command("READ", self._get_ending())

    # --------------------------------------------------------------
    # Преобразование углов с учётом сопоставления осей, смещений и отключения осей модели
    # --------------------------------------------------------------
    def transform_angles(self, x_sensor, y_sensor, z_sensor):
        # Ограничиваем входные значения диапазоном [-90, 90] (характеристика акселерометра)
        x_sensor = max(-90.0, min(90.0, x_sensor))
        y_sensor = max(-90.0, min(90.0, y_sensor))
        z_sensor = max(-90.0, min(90.0, z_sensor))

        sensor = {'X': x_sensor, 'Y': y_sensor, 'Z': z_sensor}

        def get_value(combo, inv_check):
            src = combo.currentText()
            key = src[0]
            val = sensor[key]
            if inv_check.isChecked():
                val = -val
            return val

        # Вычисляем углы из датчика (с учётом инверсии)
        roll_from_sensor = get_value(self.map_roll, self.inv_roll)
        pitch_from_sensor = get_value(self.map_pitch, self.inv_pitch)
        yaw_from_sensor = get_value(self.map_yaw, self.inv_yaw)

        # Если ось модели отключена, используем только смещение (начальное положение)
        if self.disable_roll.isChecked():
            roll_final = self.offset_roll.value()
        else:
            roll_final = roll_from_sensor + self.offset_roll.value()

        if self.disable_pitch.isChecked():
            pitch_final = self.offset_pitch.value()
        else:
            pitch_final = pitch_from_sensor + self.offset_pitch.value()

        if self.disable_yaw.isChecked():
            yaw_final = self.offset_yaw.value()
        else:
            yaw_final = yaw_from_sensor + self.offset_yaw.value()

        return roll_final, pitch_final, yaw_final

    def reset_offsets(self):
        """Сбросить начальные смещения в 0"""
        self.offset_roll.setValue(0.0)
        self.offset_pitch.setValue(0.0)
        self.offset_yaw.setValue(0.0)
        self.append_log("Начальные смещения сброшены в ноль")

    # --------------------------------------------------------------
    # Обработка координат
    # --------------------------------------------------------------
    @pyqtSlot(float, float, float)
    def on_coordinates(self, x, y, z):
        # Ограничиваем значения датчика диапазоном [-90, 90] (акселерометр)
        x = max(-90.0, min(90.0, x))
        y = max(-90.0, min(90.0, y))
        z = max(-90.0, min(90.0, z))

        self.append_log(f"X: {x:.3f}  Y: {y:.3f}  Z: {z:.3f}")

        # Преобразование с учётом сопоставления, смещений и отключения осей
        roll, pitch, yaw = self.transform_angles(x, y, z)
        self.gl_widget.set_angles(roll, pitch, yaw)

        # Для графика сохраняем исходные углы датчика (уже ограниченные)
        self.current_x.append(x)
        self.current_y.append(y)
        self.current_z.append(z)

        indices = np.arange(len(self.current_x))
        self.curve_x.setData(indices, list(self.current_x) if isinstance(self.current_x, deque) else self.current_x)
        self.curve_y.setData(indices, list(self.current_y) if isinstance(self.current_y, deque) else self.current_y)
        self.curve_z.setData(indices, list(self.current_z) if isinstance(self.current_z, deque) else self.current_z)

    def update_graph_visibility(self):
        self.curve_x.setVisible(self.cb_x.isChecked())
        self.curve_y.setVisible(self.cb_y.isChecked())
        self.curve_z.setVisible(self.cb_z.isChecked())

    def toggle_last_n_mode(self, state):
        self.use_last_n = (state == Qt.Checked)
        self.all_x.clear()
        self.all_y.clear()
        self.all_z.clear()
        self.last_n_x.clear()
        self.last_n_y.clear()
        self.last_n_z.clear()

        if self.use_last_n:
            self.current_x = self.last_n_x
            self.current_y = self.last_n_y
            self.current_z = self.last_n_z
        else:
            self.current_x = self.all_x
            self.current_y = self.all_y
            self.current_z = self.all_z

        self.curve_x.clear()
        self.curve_y.clear()
        self.curve_z.clear()

    def toggle_3d_visibility(self):
        if self.toggle_3d_btn.isChecked():
            self.gl_widget.show()
            self.toggle_3d_btn.setText("Скрыть 3D сцену")
        else:
            self.gl_widget.hide()
            self.toggle_3d_btn.setText("Показать 3D сцену")

    def closeEvent(self, event):
        self.stop_polling()
        self.worker.disconnect()
        self.worker_thread.quit()
        self.worker_thread.wait(1000)
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())