import socket
from ursina import *
from ursina.shaders import lit_with_shadows_shader
from collections import deque
import threading
import queue
import time

# --- Настройки UDP ---
UDP_IP_PC = "192.168.4.2"    # IP вашего ПК
UDP_PORT_PC = 12345          # Порт для получения данных
UDP_IP_ESP32 = "192.168.4.1" # IP ESP32
UDP_PORT_ESP32 = 12346       # Порт для команд

# Очереди для потокобезопасной передачи данных
data_queue = queue.Queue()
cmd_queue = queue.Queue()

# --- UDP обработчик в отдельном потоке ---
def udp_handler():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP_PC, UDP_PORT_PC))
    sock.settimeout(0.001)
    
    # Отправка начального пакета
    sock.sendto(b'hello', (UDP_IP_ESP32, UDP_PORT_ESP32))
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            line = data.decode('utf-8').strip()
            data_queue.put((line, addr))
        except socket.timeout:
            continue
        except Exception as e:
            print(f"Ошибка UDP: {e}")
            time.sleep(0.1)

# --- Графика Ursina ---
app = Ursina()

# 3D модель
cube = Entity(
    model='cube',
    color=color.white,
    texture='white_cube',
    scale=(2, 1, 2),
    shader=lit_with_shadows_shader
)
EditorCamera()

# Буферы для сглаживания
rolls = deque(maxlen=5)
pitches = deque(maxlen=5)
yaws = deque(maxlen=5)

# Состояние
calibrating = False
calibration_type = ""
precise_mode = False

# UI элементы
angle_text = Text(
    text='',
    position=window.top_left,
    origin=(-0.5, 0.5),
    scale=1.5,
    color=color.azure
)


# --- Отправка команд ---
def send_command(cmd):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(cmd.encode(), (UDP_IP_ESP32, UDP_PORT_ESP32))
        print(f"Отправлена команда: {cmd}")
        return True
    except Exception as e:
        print(f"Ошибка отправки команды {cmd}: {e}")
        return False

# --- Обработчики кнопок ---
def calibrate_gyro():
    if send_command('calibrate_gyro'):
        gyro_btn.color = color.red

def calibrate_mag():
    if send_command('calibrate_mag'):
        mag_btn.color = color.red

def toggle_precise_mode():
    global precise_mode
    cmd = 'precise_on' if not precise_mode else 'precise_off'
    if send_command(cmd):
        precise_mode = not precise_mode
        toggle_btn.text = (
            'Переключиться в плавный режим' if precise_mode else 'Переключиться в точный режим'
        )
        print(f"Отправлена команда переключения режима: {cmd}")


# Кнопки с начальным цветом
gyro_btn = Button(
    text='Калибровка гироскопа', 
    scale=(0.5, 0.05), 
    y=0.45, 
    on_click=calibrate_gyro,
    color=color.azure
)
mag_btn = Button(
    text='Калибровка магнитометра', 
    scale=(0.5, 0.05), 
    y=0.38, 
    on_click=calibrate_mag,
    color=color.azure
)
toggle_btn = Button(
    text='Переключить точный режим', 
    scale=(0.5, 0.05), 
    y=0.31
)

# --- Обработка данных ---
def process_data(line, addr):
    global calibrating, calibration_type, precise_mode
    
    if line == "start_calibrate_gyro":
        print("Начало калибровки гироскопа")
        calibrating = True
        calibration_type = "gyro"
        gyro_btn.color = color.red
        return
        
    elif line == "end_calibrate_gyro":
        print("Конец калибровки гироскопа")
        calibrating = False
        calibration_type = ""
        gyro_btn.color = color.azure
        return
        
    elif line == "start_calibrate_mag":
        print("Начало калибровки магнитометра")
        calibrating = True
        calibration_type = "mag"
        mag_btn.color = color.red
        return
        
    elif line == "end_calibrate_mag":
        print("Конец калибровки магнитометра")
        calibrating = False
        calibration_type = ""
        mag_btn.color = color.azure
        return
    
    elif line.startswith("mode_changed"):
        value = line.split(":")[1]
        precise_mode = (value == '1')
        print(f"Режим точности {'включён' if precise_mode else 'выключен'}")
        return

    if not calibrating:
        try:
            roll, pitch, yaw = map(float, line.split(','))
            rolls.append(roll)
            pitches.append(pitch)
            yaws.append(yaw)

            # Сглаживание движений
            cube.rotation_x = sum(rolls) / len(rolls)
            cube.rotation_y = sum(yaws) / len(yaws)
            cube.rotation_z = sum(pitches) / len(pitches)

            angle_text.text = (
                f'Roll : {cube.rotation_x:.1f}°\n'
                f'Pitch: {cube.rotation_z:.1f}°\n'
                f'Yaw  : {cube.rotation_y:.1f}°\n'
            )
        except ValueError as e:
            print(f"Ошибка разбора данных: {e}")


# Привязка обработчиков к кнопкам
gyro_btn.on_click = calibrate_gyro
mag_btn.on_click = calibrate_mag
toggle_btn.on_click = toggle_precise_mode

# --- Основной цикл обновления ---
def update():
    try:
        while not data_queue.empty():
            line, addr = data_queue.get_nowait()
            process_data(line, addr)
    except queue.Empty:
        pass
    except Exception as e:
        print(f"Ошибка обработки данных: {e}")

# Запуск UDP обработчика в отдельном потоке
udp_thread = threading.Thread(target=udp_handler, daemon=True)
udp_thread.start()

# Настройка FPS счетчика
window.fps_counter.scale = 1.5

# Запуск приложения
app.run()
