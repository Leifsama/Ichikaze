#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import time
import pygame
import math
import serial

import warnings

from time import sleep
from gpiozero import DistanceSensor, Button # <-- Přidáno Button
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero.exc import DistanceSensorNoEcho

# Globální proměnné pro senzory, inicializujeme na None
sensor1 = None
sensor2 = None
button = None # <-- Přidána proměnná pro tlačítko
factory = None

#log print
Print_auto_logic_control = False
Print_read_sensors = False

# Nové globální proměnné pro uložení stavu
global_distance1_status = "N/A"
global_distance2_status = "N/A"
global_tlacitko_stav = "N/A"

distance_cm_1 = 0
distance_cm_2 = 0

global_angle_servo_1 = 0.0
global_angle_servo_2 = 0.0
global_target_angle = 0.0
global_target_rpm = 0

# --- Autonomní konstanty pro Ackermanovo řízení podle senzoru ---
AUTONOMOUS_SPEED = 1000   # Výchozí rychlost serva pro řízení
DISTANCE_MIN_CM = 20.0   # Vzdálenost pro plné levé řízení (-45°)
DISTANCE_MAX_CM = 100.0   # Vzdálenost pro plné pravé řízení (+45°)
ANGLE_LEFT_DEG = -40.0   # Cílový úhel pro DISTANCE_MIN_CM
ANGLE_RIGHT_DEG = 40.0   # Cílový úhel pro DISTANCE_MAX_CM

SPEED_MIN_CM = 20.0       # Vzdálenost pro minimální rychlost
SPEED_MAX_CM = 40.0       # Vzdálenost pro maximální rychlost
SPEED_CUTOFF_CM = 60.0    # Vzdálenost, kde rychlost opět klesá na minimum

RPM_MIN_PERCENT = 20.0    # Minimální rychlost v % (např. 10%)
RPM_MAX_PERCENT = 100.0   # Maximální rychlost v % (100%)
MAX_RPM_VALUE = 30 # Doplňte hodnotu podle vašich motorů (např. 1023)
MAX_RPM_STEP = 1

# --- Autonomní Konstanty pro Vyhýbání ---
Auto_sub_mode = "Auto"
CELNI_MIN = 40.0             # Minimální vzdálenost pro spuštění vyhýbání (v cm)
CELNI_TRIGGER_COUNT = 3   # Počet po sobě jdoucích měření pro spuštění (zvyšuje spolehlivost)
REVERSE_DURATION_S = 15.0     # Doba couvání v sekundách
REVERSE_SPEED_RPM = 10     # Rychlost couvání (smer je definovan pomoci direction)
STEER_DELAY_S = 2  # Čas v sekundách, po který čekáme na natočení kol
celni_avoidance_step = "N/A" # Stav v rámci vyhýbacího manévru: "N/A", "Stop_and_Steer", "Reverse_Drive"
celni_steer_start_time = 0.0 # Časovač pro natočení kol (spustí se jen při vstupu do Stop_and_Steer)

# --- Proměnné pro stav Vyhýbání ---
celni_hit_counter = 0        # Počítadlo po sobě jdoucích překročení min. vzdálenosti
celni_avoidance_start_time = 0.0 # Čas, kdy manévr začal

# --- Servo Constants ---
SW_MIN_POS = 800
SW_MAX_POS = 3295
SERVO_MIN_POS_FULL = 0
SERVO_MAX_POauto_coS_FULL = 4095
SERVO_CENTER_POS = 2048
SERVO_MIN_SPEED = 0
SERVO_MAX_SPEED = 1000

# --- Ackerman Geometry Constants ---
L = 360 # Vzdálenost mezi přední a zadní osou [mm]
T = 285 # Vzdálenost mezi středy kol na nápravě [mm]
POS_DEG_RATIO = 1024 / 90

ack_group_options = ["Předek", "Zadek", "Všechna kola stejně", "Auto"]
ack_group_idx = 3 # Nastavit výchozí index na "Auto" (index 3)
ack_selected_group = ack_group_options[ack_group_idx] # Výchozí hodnota pro skupinu

try:
    # Použijeme Pigpio pro přesnější měření času.
    # Důležité: Služba 'pigpiod' musí běžet (sudo systemctl start pigpiod).
    factory = PiGPIOFactory()

    # Dočasně potlačíme systémové varování "DistanceSensorNoEcho", aby nám nezahlcovalo konzoli.
    stderr_orig = sys.stderr
    sys.stderr = open('/dev/null', 'w')
    warnings.filterwarnings("ignore", category=UserWarning)

    #Senzor 1
    sensor1 = DistanceSensor(echo=23, trigger=18, pin_factory=factory)

    #Senzor 2
    sensor2 = DistanceSensor(echo=16, trigger=17, pin_factory=factory)
    
    #Tlačítko: GPIO 5 (Pin 29), aktivujeme interní pull-up rezistor
    button = Button(5, pull_up=True)

    # Vrátíme standardní výstup
    sys.stderr = stderr_orig
    print("--- Start měření vzdálenosti a stavu tlačítka ---")
    print(f"Tlačítko na GPIO 5, Senzory na GPIO 23/18 a 16/17.")
    print("Pro ukončení stiskněte Ctrl+C")
    print("-" * 30)
    # Krátké čekání pro stabilizaci
    sleep(0.5)
    

except Exception as e:
    # SELHÁNÍ STARTU PIGPIO
    # Pokud se toto stane, program sice nespadne hned, ale senzory nefungují!
    print("--- CHYBA: NELZE SE PŘIPOJIT K PIGPIOD (Senzory nebudou fungovat) ---")
    print(f"Detail chyby: {e}")
    # Ujistěte se, že proměnné zůstanou na None (což už jsou, ale zde je dobré místo pro hlášku)
finally:
    # Ujistěte se, že se standardní výstup vrátí, i když dojde k chybě
    if 'stderr_orig' in locals():
        sys.stderr = stderr_orig

# FUNKCE PRO KONTINUÁLNÍ ČTENÍ DAT
def read_sensors():
    global global_distance1_status, global_distance2_status, global_tlacitko_stav, distance_cm_1, distance_cm_2, button
    global sensor1 # Ujistěte se, že sensor1 je globální
    global sensor2
    global Print_read_sensors

   # Důležitá kontrola: Pokud se Pigpio zhroutilo, sensor1 je None!
    if sensor1 is None:
        global_distance1_status = "CHYBA Pigpio"
        global_distance2_status = "CHYBA Pigpio"
        return # Okamžitě ukončíme funkci, abychom se vyhnuli chybám

    # Pokud jsme tady, Pigpio je aktivní (i když nestabilní)
    try:
        # Čtení Senzoru 1 (musí být chráněno, pokud vrací chybu)
        distance_cm_1 = sensor1.distance * 100
        global_distance1_status = f"{distance_cm_1:.1f} cm"

        # Čtení Senzoru 2 (kontrola, jestli existuje, je zde již volitelná,
        # protože už víme, že sensor1 existuje a chyba Pigpio je vyřešena)
        if sensor2 is not None:
            distance_cm_2 = sensor2.distance * 100
            global_distance2_status = f"{distance_cm_2:.1f} cm"
        else:
            global_distance2_status = "NEPŘIPOJENO" # Nastavíme pro případ, že sensor2 nebyl inicializován, ale sensor1 ano
            
    except Exception as e:
        # Pokud čtení senzoru selže (i při aktivním Pigpio), vrátíme N/A
        global_distance1_status = "N/A"
        global_distance2_status = "N/A"
        print(f"DEBUG: Chyba při čtení senzoru: {e}")
    
    
    if button is not None:
        if button.is_pressed:
            global_tlacitko_stav = "STISKNUTO"
        else:
            global_tlacitko_stav = "UVOLNĚNO"
    else:
        global_tlacitko_stav = "CHYBA"
       

       
    if Print_read_sensors:
        print(f"Vzdál sensor1 (pravý): {distance_cm_1}") 
        print(f"Vzdál sensor2 (uprostred): {distance_cm_2}") 
# --- Import STservo_sdk ---
try:
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
    from STservo_sdk import *
except ImportError as e:
    print(f"CHYBA: Nepodařilo se importovat STservo_sdk. Ujistěte se, že STservo_sdk.py (nebo balíček STservo_sdk) je v nadřazeném adresáři, nebo je cesta nastavena správně. Chyba: {e}")
    sys.exit(1)

# === CRC8 MAXIM pro motory DDSM210 ===
def crc8_maxim(data):
    """Vypočítá 8bitový CRC checksum pro data."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc & 0xFF

def send_cmd_motor(ser_motor, cmd):
    """Odešle příkaz motoru s přidaným CRC."""
    full = bytearray(cmd + [crc8_maxim(cmd)])
    ser_motor.write(full)



# --- Pygame Initialization ---
pygame.init()
pygame.font.init()

SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 900
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Pygame Servo Controller")

font_title = pygame.font.SysFont('Arial', 40, bold=True)
font_label = pygame.font.SysFont('Arial', 24)
font_input = pygame.font.SysFont('Arial', 24)
font_button = pygame.font.SysFont('Arial', 28)
font_highlight = pygame.font.SysFont('Arial', 28, bold=True)

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
LIGHT_GRAY = (220, 220, 220)
DARK_GRAY = (100, 100, 100)
BLUE = (0, 128, 255)
GREEN = (0, 200, 0)
RED = (200, 0, 0)
ORANGE = (255, 165, 0)
LIGHT_BLUE = (173, 216, 230)
YELLOW = (255, 255, 0)
ACCENT_COLOR = (0, 200, 100)
REVERSE_COLOR = (255, 100, 100)
ERROR_COLOR = (255, 50, 50)
BUTTON_COLOR = (70, 74, 82)
BUTTON_HOVER_COLOR = (90, 94, 102)
TEXT_COLOR = (230, 230, 230)
BACKGROUND_COLOR = (240, 240, 240)

# --- UI Element Classes ---
class InputBox:
    def __init__(self, x, y, w, h, text=''):
        self.rect = pygame.Rect(x, y, w, h)
        self.color = GRAY
        self.text = text
        self.active = False
        self.text_surface = font_input.render(self.text, True, DARK_GRAY)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.active = not self.active
            else:
                self.active = False
        self.color = LIGHT_BLUE if self.active else GRAY
        if event.type == pygame.KEYDOWN:
            if self.active:
                if event.key == pygame.K_BACKSPACE:
                    self.text = self.text[:-1]
                elif event.unicode.isnumeric():
                    self.text += event.unicode
                self.text_surface = font_input.render(self.text, True, DARK_GRAY)

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, self.rect, border_radius=5)
        text_rect = self.text_surface.get_rect(center=self.rect.center)
        screen.blit(self.text_surface, text_rect)
        if self.active:
            cursor_pos = text_rect.right
            pygame.draw.line(screen, DARK_GRAY, (cursor_pos, self.rect.top + 5), (cursor_pos, self.rect.bottom - 5), 2)

class Button:
    def __init__(self, x, y, w, h, text, on_click, color=ACCENT_COLOR):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.on_click = on_click
        self.base_color = color
        self.is_enabled = True
        self.text_surface = font_button.render(self.text, True, WHITE)

    def handle_event(self, event):
        if self.is_enabled and event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.on_click()

    def draw(self, screen):
        current_color = self.base_color if self.is_enabled else DARK_GRAY
        pygame.draw.rect(screen, current_color, self.rect, border_radius=10)
        
        text_rect = self.text_surface.get_rect(center=self.rect.center)
        screen.blit(self.text_surface, text_rect)

# --- Servo State and Control Functions ---
SERVO_IDS = [1, 2, 3, 4]
FRONT_SERVOS = [1, 2]
REAR_SERVOS = [3, 4]
ALL_SERVOS = [1, 2, 3, 4]

try:
    portHandler = PortHandler("/dev/ttyACM0")
    if not portHandler.openPort():
        raise IOError("Could not open serial port for servos.")
    if not portHandler.setBaudRate(1000000):
        raise IOError("Could not set baudrate for servos.")
    sts_servo = sts(portHandler)
    print("Servo connection established.")
except Exception as e:
    print(f"CHYBA: Selhalo navázání komunikace se servy. {e}")
    sys.exit(1)
    
servo_states = {
    1: {'id': 1, 'online': False, 'position': -1},
    2: {'id': 2, 'online': False, 'position': -1},
    3: {'id': 3, 'online': False, 'position': -1},
    4: {'id': 4, 'online': False, 'position': -1}
}
last_requested_positions = {servo_id: SERVO_CENTER_POS for servo_id in SERVO_IDS}
limit_45_deg_enabled = False
MAX_45_DEG_POS = SERVO_CENTER_POS + (POS_DEG_RATIO * 45)
MIN_45_DEG_POS = SERVO_CENTER_POS - (POS_DEG_RATIO * 45)

def move_to_position(servo_id, target_pos, speed):
    if not servo_states[servo_id]['online']:
        return
    clamped_pos = max(SW_MIN_POS, min(SW_MAX_POS, target_pos))
    if limit_45_deg_enabled:
        clamped_pos = max(MIN_45_DEG_POS, min(MAX_45_DEG_POS, clamped_pos))
    last_requested_positions[servo_id] = clamped_pos
    comm_result, error = sts_servo.WritePosEx(servo_id, clamped_pos, speed, 0)
    if comm_result != COMM_SUCCESS or error != 0:
        print(f"CHYBA: Nepodařilo se odeslat příkaz servu {servo_id}. {sts_servo.getTxRxResult(comm_result)} / {sts_servo.getRxPacketError(error)}")

def stop_servo(servo_id):
    if not servo_states[servo_id]['online']:
        return
    current_pos_for_stop = last_requested_positions[servo_id]
    comm_result, error = sts_servo.WritePosEx(servo_id, current_pos_for_stop, 0, 0)
    if comm_result != COMM_SUCCESS or error != 0:
        print(f"CHYBA: Nepodařilo se zastavit servo {servo_id}. {sts_servo.getTxRxResult(comm_result)} / {sts_servo.getRxPacketError(error)}")

def stop_all_servos():
    for servo_id in servo_states.keys():
        if servo_states[servo_id]['online']:
            stop_servo(servo_id)

def refresh_servo_status():
    found_any = False
    for servo_id in SERVO_IDS:
        pos, comm_result, error = sts_servo.ReadPos(servo_id)
        time.sleep(0.01)
        if comm_result == COMM_SUCCESS and error == 0:
            servo_states[servo_id]['online'] = True
            servo_states[servo_id]['position'] = pos
            found_any = True
        else:
            servo_states[servo_id]['online'] = False
            servo_states[servo_id]['position'] = -1

    if not found_any:
        for servo_id in SERVO_IDS:
             pos, comm_result, error = sts_servo.ReadPos(servo_id)
             if comm_result == COMM_SUCCESS and error == 0:
                 servo_states[servo_id]['online'] = True
                 servo_states[servo_id]['position'] = pos

def calculate_ackerman_slave_pos(master_pos, master_id, track_width=T, wheelbase=L, pos_deg_ratio=POS_DEG_RATIO):
    angle_master_deg = (master_pos - SERVO_CENTER_POS) / pos_deg_ratio
    
    if abs(angle_master_deg) < 0.1:
        return SERVO_CENTER_POS
        
    angle_master_rad = math.radians(angle_master_deg)
    is_right_turn = angle_master_deg > 0
  
    if is_right_turn:
        tan_inner_angle = math.tan(angle_master_rad)
        if tan_inner_angle == 0:
            tan_inner_angle = 0.0001
        
        cot_outer_angle = 1 / tan_inner_angle + track_width / wheelbase
        angle_slave_rad = math.atan(1 / cot_outer_angle)
        
    else:
        tan_outer_angle = math.tan(abs(angle_master_rad))
        if tan_outer_angle == 0:
            tan_outer_angle = 0.0001
          
        cot_inner_angle = 1 / tan_outer_angle - track_width / wheelbase
        angle_slave_rad = math.atan(1 / cot_inner_angle)
        angle_slave_rad = -angle_slave_rad
    
    slave_pos = SERVO_CENTER_POS + math.degrees(angle_slave_rad) * pos_deg_ratio
    
    return int(slave_pos)
    
# --- NOVÉ FUNKCE PRO PŘEVOD ÚHLU a RYCHLOSTI ---

def pos_from_angle(angle_deg):
    """Převádí úhel ve stupních na pozici v tickách (0-4095)."""
    # Omezení úhlu pro bezpečný provoz
    angle_deg = max(-90, min(90, angle_deg))
    return int(SERVO_CENTER_POS + angle_deg * POS_DEG_RATIO)

def pos_from_angle(angle_deg):
    """Převádí úhel ve stupních na pozici v tickách (0-4095)."""
    # Omezení úhlu pro bezpečný provoz
    angle_deg = max(-90, min(90, angle_deg))
    return int(SERVO_CENTER_POS + angle_deg * POS_DEG_RATIO)

def calculate_ackerman_rpm_XX(base_rpm, angle_servo_in_deg, angle_servo_out_deg, track_width_T, wheelbase_L):
    """
    Vypočítá Ackermannův diferenciální RPM pro pojezdové motory.
    base_rpm je celková rychlost (již snížená o faktor zatáčení a vzdálenosti).
    Vrací pole rychlostí [LP, PP, LZ, PZ].
    """
    
    # Faktor pro zrychlení/zpomalení maximálně o 20%
    MAX_DIFF_FACTOR = 0.20 
    MAX_STEER_ANGLE = 50.0 # ANGLE_RIGHT_DEG / abs(ANGLE_LEFT_DEG)
    
    # Úhel poměru je od 0.0 do 1.0 (jak moc je natočeno)
    # Použijeme angle_servo_out_deg pro určení poměru
    angle_ratio = abs(angle_servo_out_deg) / MAX_STEER_ANGLE
    
    # Faktor pro vnější kola (zrychlení) a vnitřní kola (zpomalení)
    factor_outer = 1.0 + (MAX_DIFF_FACTOR * angle_ratio) 
    factor_inner = 1.0 - (MAX_DIFF_FACTOR * angle_ratio) 

    # Určení, která kola jsou vnitřní a vnější, se určí podle úhlu Servo 1 (nebo global_target_angle)
    # POUŽIJEME global_target_angle (je dostupný jako global_angle_servo_out_deg v tomto kontextu)
    # Vypočítáme RPM
    if angle_servo_out_deg < 0: # Zatáčení DOLEVA
        rpm_lp = base_rpm * factor_inner  # LP: Vnitřní 
        rpm_pp = base_rpm * factor_outer  # PP: Vnější 
        rpm_lz = base_rpm * factor_inner  # LZ: Vnitřní 
        rpm_pz = base_rpm * factor_outer  # PZ: Vnější 
    else: # Zatáčení DOPRAVA
        rpm_lp = base_rpm * factor_outer  # LP: Vnější 
        rpm_pp = base_rpm * factor_inner  # PP: Vnitřní 
        rpm_lz = base_rpm * factor_outer  # LZ: Vnější 
        rpm_pz = base_rpm * factor_inner  # PZ: Vnitřní 

    # POZOR: Použijeme round() pro zaokrouhlení, NE int()!
    
    # Vrátíme pole rychlostí (v RPM) zaokrouhlené na celé číslo:
    return [round(rpm_lp), round(rpm_pp), round(rpm_lz), round(rpm_pz)]
    
def calculate_ackerman_rpm(base_rpm, angle_target_deg, track_width_T, wheelbase_L):
    """
    Vypočítá Ackermannův diferenciální RPM pro pojezdové motory pomocí
    geometrického vzorce, kde base_rpm je rychlost středu zadní nápravy.
    
    Používá angle_target_deg (cílový úhel řízení) k určení poloměru zatáčení.
    """
    # Úhel musí být v radiánech
    angle_rad = math.radians(angle_target_deg)

    # Pokud je úhel velmi malý, neaplikujeme diferenciaci
    if abs(angle_rad) < 0.05: # Ponecháme malý mrtvý bod
        return [base_rpm, base_rpm, base_rpm, base_rpm]

    # Geometrický poloměr otáčení (R) k zadní nápravě
    # R = L / tan(angle_rad)
    # POZOR: Pro zatáčení vodorovné se uvažuje průměrná rychlost,
    # poloměr otáčení R musí být k virtuálnímu středu.
    # Použijeme zjednodušený vzorec RPM_diff:
    
    # Faktory pro diferenciální rychlost:
    # Faktor = (T / 2) * tan(angle_rad) / L
    # Upravujeme rychlost zadní nápravy (blíže ke středu)
    
    # Použijeme jednoduchou diferenciaci podle poloměrů k bodu otáčení.
    # Předpokládáme, že base_rpm je průměrná rychlost.
    
    # Geometrická diference, odvozená z poměru poloměrů:
    # diff_factor = (T / 2R) = (T * tan(angle_rad)) / (2 * L)
    diff_factor = (track_width_T * math.tan(angle_rad)) / (2 * wheelbase_L)
    
    factor_outer = 1.0 + abs(diff_factor)
    factor_inner = 1.0 - abs(diff_factor)
    
    # Určení, která kola jsou vnitřní a vnější
    if angle_target_deg < 0: # Zatáčení DOLEVA
        rpm_lp = base_rpm * factor_inner  # LP: Vnitřní
        rpm_pp = base_rpm * factor_outer  # PP: Vnější
        # U 4WD s Ackermannem se obvykle diferenciace aplikuje na všechna kola
        rpm_lz = base_rpm * factor_inner  # LZ: Vnitřní
        rpm_pz = base_rpm * factor_outer  # PZ: Vnější
    else: # Zatáčení DOPRAVA
        rpm_lp = base_rpm * factor_outer  # LP: Vnější
        rpm_pp = base_rpm * factor_inner  # PP: Vnitřní
        rpm_lz = base_rpm * factor_outer  # LZ: Vnější
        rpm_pz = base_rpm * factor_inner  # PZ: Vnitřní

    # Vrátíme pole rychlostí (v RPM) zaokrouhlené na celé číslo
    return [round(rpm_lp), round(rpm_pp), round(rpm_lz), round(rpm_pz)]

# === Nová funkce pro řízení jednoho motoru ===
def control_drive_motors_diff(motor_id_current, rpm_magnitude, direction):
    """
    Odešle příkaz pro rychlost POUZE jednomu motoru s daným ID
    s aplikací ackermann diference a směru.

    :param motor_id_current: ID motoru (0x01, 0x02, 0x03, 0x04)
    :param rpm_magnitude: Absolutní hodnota RPM pro tento konkrétní motor
    :param direction: Globální směr (1 pro vpřed, -1 pro vzad)
    """
    if not ser_motor:
        return
    
    # Původní kód násobí RPM * 10
    base_rpm_val = rpm_magnitude * 10 * direction
    
    # Aplikujeme reverzaci pro pravou stranu (motory 0x02 a 0x04),
    # která je nutná kvůli zapojení motorů DDSM210.
    effective_rpm_val = base_rpm_val
    if motor_id_current in [0x02, 0x04]:
        effective_rpm_val *= -1
        
    # Sestavíme a odešleme příkaz
    rpm_bytes = int(effective_rpm_val).to_bytes(2, 'big', signed=True)
    cmd_velocity = [
        motor_id_current, 0x64,
        rpm_bytes[0], rpm_bytes[1],
        0x00, 0x00,
        0x50, 0x00, 0x00
    ]
    send_cmd_motor(ser_motor, cmd_velocity)

def set_all_drive_motor_speed(rpm):
    """Nastaví rychlost pro všechny pojezdové motory (vpřed, direction=1)."""
    # Původní logika control_drive_motors obslouží všechny motory.
    # Používáme direction=1 pro jízdu vpřed.
    control_drive_motors(rpm, 1)


# === Motor Control Setup ===
motor_ids = [0x01, 0x02, 0x03, 0x04]
motor_data = {
    id: {"speed_rpm": None, "position_deg": None, "mileage_laps": None, "error_code": None}
    for id in motor_ids
}
current_motor_rpm = 0
motor_direction = 0

# --- NOVÉ GLOBÁLNÍ STAVY PRO POD-REŽIM ---
# Pod-režim pro scénu full_ackerman: "Manual" nebo "Auto"
full_ackerman_sub_mode = "Manual" 
# Globální proměnné pro zobrazení cílových hodnot v Auto režimu
global_target_angle = 0.0
global_target_rpm = 0


try:
    ser_motor = serial.Serial("/dev/ttyACM1", baudrate=115200, timeout=0.3)
    print("Motor serial connection established.")
    for motor_id_init in motor_ids:
        cmd_init = [motor_id_init, 0xA0, 0x02] + [0x00] * 6
        send_cmd_motor(ser_motor, cmd_init)
        time.sleep(0.01)
except serial.SerialException as e:
    print(f"Chyba při inicializaci sériového portu pro motory: {e}")
    print("Zkontrolujte, zda je zařízení /dev/ttyACM1 připojeno a dostupné.")
    ser_motor = None

# === Nová funkce pro řízení motorů ===
def control_drive_motors(rpm, direction):
    if not ser_motor:
        return
    base_rpm_val = rpm * 10 * direction
    for motor_id_current in motor_ids:
        effective_rpm_val = base_rpm_val
        if motor_id_current in [0x02, 0x04]:
            effective_rpm_val *= -1
        rpm_bytes = effective_rpm_val.to_bytes(2, 'big', signed=True)
        cmd_velocity = [
            motor_id_current, 0x64,
            rpm_bytes[0], rpm_bytes[1],
            0x00, 0x00,
            0x50, 0x00, 0x00
        ]
        send_cmd_motor(ser_motor, cmd_velocity)
        #time.sleep(0.005)
        
def stop_all_motors():
    if not ser_motor:
        return
    for motor_id_stop in motor_ids:
        stop_rpm_bytes = (0).to_bytes(2, 'big', signed=True)
        cmd_stop = [motor_id_stop, 0x64, stop_rpm_bytes[0], stop_rpm_bytes[1], 0x00, 0x00, 0x50, 0x00, 0x00]
        send_cmd_motor(ser_motor, cmd_stop)
        time.sleep(0.01)
#   linearni funkce pro zataceni
def linear_map(value, input_min, input_max, output_min, output_max):
    """Mapuje hodnotu z jednoho rozsahu na druhý pomocí lineární interpolace."""
    # Oříznutí hodnoty, aby nepřesáhla vstupní rozsah
    value = max(input_min, min(value, input_max))
    # Lineární interpolace
    input_range = input_max - input_min
    output_range = output_max - output_min
    
    # Vyhnutí se dělení nulou
    if input_range == 0:
        return output_min
        
    scaled_value = float(value - input_min) / input_range
    return output_min + (scaled_value * output_range)
    
    
    
# --- Scene Management ---
current_scene = "main_menu"
def set_scene(scene_name):
    global current_scene
    current_scene = scene_name
    print(f"Přepínám na scénu: {current_scene}")
    stop_all_servos()
    if ser_motor:
        stop_all_motors()
    for servo_id in SERVO_IDS:
        last_requested_positions[servo_id] = SERVO_CENTER_POS

# --- UI Elements for scenes ---
main_menu_buttons = {
    "manual_control": Button(SCREEN_WIDTH // 2 - 200, 300, 400, 70, "Manuální ovládání", lambda: set_scene("manual_control")),
    "auto_control": Button(SCREEN_WIDTH // 2 - 200, 400, 400, 70, "Automatické ovládání", lambda: set_scene("auto_control")),
    "ackerman_control": Button(SCREEN_WIDTH // 2 - 200, 500, 400, 70, "Ackermanovo řízení", lambda: set_scene("ackerman_control")),
    "full_ackerman": Button(SCREEN_WIDTH // 2 - 200, 600, 400, 70, "Full Ackerman", lambda: set_scene("full_ackerman"))
}

global_speed_input = InputBox(100, 150, 150, 40, '1000')
manual_stop_button = Button(SCREEN_WIDTH // 2 - 100, SCREEN_HEIGHT - 100, 200, 60, "Zastavit vše", stop_all_servos, color=RED)
manual_back_button = Button(50, SCREEN_HEIGHT - 60, 150, 40, "Zpět do menu", lambda: set_scene("main_menu"), color=ORANGE)
target_position_inputs = {servo_id: InputBox(250, 250 + i * 130 + 50, 150, 40, '2048') for i, servo_id in enumerate(SERVO_IDS)}
per_servo_move_buttons = {servo_id: Button(500, 250 + i * 130 + 50, 150, 40, "Pohyb", lambda s_id=servo_id: move_to_position(s_id, int(target_position_inputs[s_id].text), int(global_speed_input.text))) for i, servo_id in enumerate(SERVO_IDS)}

auto_speed_input = InputBox(450, 250, 150, 40, '1000')
auto_step_input = InputBox(450, 300, 150, 40, '10')
auto_back_button = Button(50, SCREEN_HEIGHT - 60, 150, 40, "Zpět do menu", lambda: set_scene("main_menu"), color=ORANGE)
auto_stop_button = Button(SCREEN_WIDTH // 2 - 100, SCREEN_HEIGHT - 100, 200, 60, "Zastavit vše", stop_all_servos, color=RED)
selected_group = "Předek"
group_options = ["Předek", "Zadek", "Vše", "Auto"]
group_idx = 0

ack_speed_input = InputBox(250, 300, 150, 40, '1000')
ack_step_input = InputBox(250, 350, 150, 40, '10')
ack_back_button = Button(50, SCREEN_HEIGHT - 60, 150, 40, "Zpět do menu", lambda: set_scene("main_menu"), color=ORANGE)
ack_selected_group = "Předek"
ack_group_options = ["Předek", "Zadek", "Všechna kola stejně", "Auto"]
ack_group_idx = 0
# Nové UI pole pro max rychlost motoru
max_motor_rpm_input = InputBox(550, 350, 150, 40, '100')

def toggle_45_deg_limit():
    global limit_45_deg_enabled
    limit_45_deg_enabled = not limit_45_deg_enabled
    print(f"Limit 45 stupňů je nyní: {'Zapnut' if limit_45_deg_enabled else 'Vypnut'}")

toggle_45_deg_limit_button = Button(50, 200, 200, 50, "Limit 45°: Vypnut", toggle_45_deg_limit, color=ORANGE)

# --- Drawing and Event Handling for each Scene ---
def draw_current_mode_label():
    mode_text = ""
    if current_scene == "manual_control":
        mode_text = "Režim: Manuální ovládání"
    elif current_scene == "auto_control":
        mode_text = "Režim: Automatické ovládání"
    elif current_scene == "ackerman_control":
        mode_text = "Režim: Ackermanovo řízení"
    elif current_scene == "full_ackerman":
        mode_text = "Režim: Full Ackerman"
    
    if mode_text:
        mode_surface = font_label.render(mode_text, True, DARK_GRAY)
        screen.blit(mode_surface, (10, 10))

def draw_main_menu():
    screen.fill(LIGHT_GRAY)
    title_text = font_title.render("Výběr režimu ovládání", True, DARK_GRAY)
    title_rect = title_text.get_rect(center=(SCREEN_WIDTH / 2, 100))
    screen.blit(title_text, title_rect)
    for button in main_menu_buttons.values():
        button.draw(screen)
    pygame.display.flip()

def handle_main_menu_events(event):
    for button in main_menu_buttons.values():
        button.handle_event(event)

def draw_manual_control_scene():
    screen.fill(LIGHT_GRAY)
    draw_current_mode_label()
    title_text = font_title.render("Manuální ovládání (textové)", True, DARK_GRAY)
    title_rect = title_text.get_rect(center=(SCREEN_WIDTH / 2, 60))
    screen.blit(title_text, title_rect)
    
    label_speed = font_label.render("Rychlost (0-1000):", True, DARK_GRAY)
    screen.blit(label_speed, (100, 120))
    global_speed_input.draw(screen)

    label_pos = font_label.render("Cílová pozice (0-4095):", True, DARK_GRAY)
    screen.blit(label_pos, (250, 120))
    for i, servo_id in enumerate(SERVO_IDS):
        label_text = font_label.render(f"Servo {servo_id}:", True, DARK_GRAY)
        screen.blit(label_text, (100, 250 + i * 130 + 50))
        target_position_inputs[servo_id].draw(screen)
        per_servo_move_buttons[servo_id].draw(screen)

    pos_center_button = Button(SCREEN_WIDTH // 2 + 100, SCREEN_HEIGHT - 100, 200, 60, "Vycentrovat vše", lambda: move_to_position(0, SERVO_CENTER_POS, int(global_speed_input.text)))
    pos_center_button.draw(screen)
    manual_stop_button.draw(screen)
    manual_back_button.draw(screen)

    refresh_servo_status()
    y_offset = 250
    for servo_id in SERVO_IDS:
        state = servo_states[servo_id]
        status_text = f"Servo {servo_id}: Stav: {'Online' if state['online'] else 'Offline'}, Pozice: {state['position'] if state['online'] else 'N/A'}"
        color = GREEN if state['online'] else RED
        status_surface = font_label.render(status_text, True, color)
        screen.blit(status_surface, (650, y_offset + (servo_id - 1) * 130))

    pygame.display.flip()

def handle_manual_control_events(event):
    global_speed_input.handle_event(event)
    for input_box in target_position_inputs.values():
        input_box.handle_event(event)
    for button in per_servo_move_buttons.values():
        button.handle_event(event)
    manual_stop_button.handle_event(event)
    manual_back_button.handle_event(event)

def draw_auto_control_scene():
    global selected_group
    screen.fill(LIGHT_GRAY)
    draw_current_mode_label()
    title_text = font_title.render("Automatické ovládání (klávesnicí)", True, DARK_GRAY)
    title_rect = title_text.get_rect(center=(SCREEN_WIDTH / 2, 60))
    screen.blit(title_text, title_rect)

    label_speed = font_label.render("Rychlost (0-1000):", True, DARK_GRAY)
    screen.blit(label_speed, (450, 220))
    auto_speed_input.draw(screen)

    label_step = font_label.render("Krok (jednotky):", True, DARK_GRAY)
    screen.blit(label_step, (450, 270))
    auto_step_input.draw(screen)

    instructions = font_label.render("Použijte šipky Nahoru/Dolů pro řízení.", True, DARK_GRAY)
    instructions_rect = instructions.get_rect(center=(SCREEN_WIDTH/2, 450))
    screen.blit(instructions, instructions_rect)

    # Vizuální zvýraznění vybrané skupiny
    group_label_y = 500
    for i, group_name in enumerate(group_options):
        text_surface = font_label.render(group_name, True, DARK_GRAY)
        if group_name == selected_group:
            text_surface = font_highlight.render(group_name, True, BLUE)
        text_rect = text_surface.get_rect(center=(SCREEN_WIDTH/2, group_label_y + i * 40))
        screen.blit(text_surface, text_rect)
    
    group_inst = font_label.render("Použijte šipky Vlevo/Vpravo pro změnu skupiny.", True, DARK_GRAY)
    group_inst_rect = group_inst.get_rect(center=(SCREEN_WIDTH/2, 650))
    screen.blit(group_inst, group_inst_rect)
    
    auto_back_button.draw(screen)
    auto_stop_button.draw(screen)

    refresh_servo_status()
    y_offset = 250
    for servo_id in SERVO_IDS:
        state = servo_states[servo_id]
        status_text = f"Servo {servo_id}: Stav: {'Online' if state['online'] else 'Offline'}, Pozice: {state['position'] if state['online'] else 'N/A'}"
        color = GREEN if state['online'] else RED
        status_surface = font_label.render(status_text, True, color)
        screen.blit(status_surface, (50, y_offset + (servo_id - 1) * 30))

    pygame.display.flip()

def handle_auto_control_events(event):
    global selected_group, group_idx
    auto_speed_input.handle_event(event)
    auto_step_input.handle_event(event)
    auto_back_button.handle_event(event)
    auto_stop_button.handle_event(event)
    
    if event.type == pygame.KEYDOWN:
        if event.key == pygame.K_SPACE:
            group_idx = (group_idx - 1) % len(group_options)
            selected_group = group_options[group_idx]
#        elif event.key == pygame.K_SPACE:
#            group_idx = (group_idx + 1) % len(group_options)
#            selected_group = group_options[group_idx]

## Úhly pro Ackermannovo řízení. (Konstanta pro rozdíl úhlů kol)
ACKERMAN_ANGLE_FRONT = 20 


def auto_control_logic():
    global distance_cm_1, distance_cm_2, global_target_angle, global_angle_servo_1, global_angle_servo_2, global_target_rpm, MAX_RPM_VALUE, button, MAX_RPM_STEP
    global global_target_rpm
    global global_target_angle
    global global_angle_servo_1
    global global_angle_servo_2
    
    global Print_auto_logic_control
    
    global Auto_sub_mode
    global celni_hit_counter
    global celni_avoidance_start_time
    global REVERSE_DURATION_S
    global REVERSE_SPEED_RPM
    global celni_avoidance_step
    global celni_steer_start_time
    
    if Auto_sub_mode == "Auto":
        # 1. Získání rychlosti serva
        # Zkusíme získat rychlost z UI, jinak použijeme výchozí konstantu
        try:
            # Použijeme vstupní pole pro rychlost z auto_control_scene (auto_speed_input)
            #steering_speed = int(auto_speed_input.text)
            steering_speed = 100
        except (ValueError, AttributeError):
            steering_speed = AUTONOMOUS_SPEED 
            
        # Oříznutí rychlosti
        steering_speed = max(0, min(1000, steering_speed))

        # 2. Zjistíme aktuální vzdálenost ze senzoru (distance_cm_1 je globální a aktualizuje se v read_sensors)
        current_distance = distance_cm_1
       
     
        # 3. Lineární mapování vzdálenosti (20-60 cm) na cílový úhel (-45° až +45°)
        # Využíváme stávající funkci linear_map
        # distance=20cm -> target_angle_deg = -45.0 (Vlevo)
        # distance=60cm -> target_angle_deg = 45.0 (Vpravo)
        target_angle_deg = linear_map(
            current_distance, 
            DISTANCE_MIN_CM, 
            DISTANCE_MAX_CM, 
            ANGLE_LEFT_DEG, 
            ANGLE_RIGHT_DEG
        )

        # Uložíme cílový úhel do globální proměnné pro zobrazení v UI (nepovinné, ale dobré pro debug)
        global_target_angle = target_angle_deg
        
        # 4. Převod cílového úhlu na pozici pro HLAVNÍ servo (Servo 1 - vnější kolo při zatáčení vpravo)
        # Využíváme stávající funkci pos_from_angle
        master_pos = pos_from_angle(target_angle_deg)
        
        # 5. Výpočet Ackermannovy pozice pro VEDLEJŠÍ servo (Servo 2)
        # Využíváme stávající Ackermannovu geometrii
        slave_pos = calculate_ackerman_slave_pos(master_pos, 1) # 1 značí, že Servo 1 je Master

        # 6. Odeslání příkazu servům
        # Využíváme stávající funkci move_to_position
        move_to_position(1, master_pos, steering_speed)
        move_to_position(2, slave_pos, steering_speed)
        
        # 7. Aktualizace globálních úhlů pro vizualizaci (nepovinné)
        global_angle_servo_1 = target_angle_deg
        # Převod slave pozice na úhel pro zobrazení
        global_angle_servo_2 = (slave_pos - SERVO_CENTER_POS) / POS_DEG_RATIO
        
            #NOVÁ LOGIKA POHONU (DRIVE)
        # ==========================================================
        
        # Předpokládaná výchozí rychlost (pokud je mimo rozsah 20-60 cm)
        target_rpm_percent = RPM_MIN_PERCENT 
        
        
        
        
            
        if current_distance < SPEED_MIN_CM or current_distance > SPEED_CUTOFF_CM:
            # Vzdálenost je < 20cm nebo > 60cm, rychlost je minimální (10%)
            target_rpm_percent = RPM_MIN_PERCENT
        
        elif current_distance >= SPEED_MIN_CM and current_distance <= SPEED_MAX_CM:
            # FÁZE 1: Zrychlení (od 20 cm do 40 cm, 10% až 100%)
            target_rpm_percent = linear_map(
                current_distance, 
                SPEED_MIN_CM, 
                SPEED_MAX_CM, 
                RPM_MIN_PERCENT, 
                RPM_MAX_PERCENT
            )
            
        elif current_distance > SPEED_MAX_CM and current_distance <= SPEED_CUTOFF_CM:
            # FÁZE 2: Zpomalení (od 40 cm do 60 cm, 100% zpět na 10%)
            # Použijeme obrácené mapování: 
            # Vstup 40cm -> 100%, Vstup 60cm -> 10%
            target_rpm_percent = linear_map(
                current_distance, 
                SPEED_MAX_CM,          # Vstup 40 cm
                SPEED_CUTOFF_CM,       # Vstup 60 cm
                RPM_MAX_PERCENT,       # Výstup 100%
                RPM_MIN_PERCENT        # Výstup 10%
            )
        
        # Určíme směr jízdy (závisí na tom, jestli je global_target_rpm kladné nebo záporné)
        if global_target_rpm >= 0:
            direction = 1 # Vpřed
        else:
            direction = -1 # Vzad
            
            # --- NOVÁ LOGIKA PRO DIFERENCIÁLNÍ RPM ---
       #NOVÁ LOGIKA POHONU (DRIVE)
        # ==========================================================

        # 7. Výpočet základní rychlosti a aplikace korekce zatáčení.
        
        # 7a. Výpočet základní rychlosti (závislé na vzdálenosti)
        # MAX_RPM_VALUE je 30[cite: 104], což je maximální hodnota (před násobením * 10 v control_drive_motors_diff).
        #base_rpm_magnitude = (target_rpm_percent / 100.0) * MAX_RPM_VALUE
        base_rpm_magnitude = MAX_RPM_VALUE
        actual_steer_angle = (global_angle_servo_1 + global_angle_servo_2) / 2.0
        
       # Krok 7b (NOVÝ): Výpočet faktoru zpomalení pomocí lineární interpolace
        # Cíl: 
        #   - Uhel 0.0 -> Faktor RPM_MAX_PERCENT (1.0)
        #   - Uhel MAX_STEER_ANGLE (50.0) -> Faktor RPM_MIN_PERCENT (0.5)

        # 1. Normalizovaný absolutní úhel (0.0 až 1.0)
        # Používáme aktuální úhel serva pro plynulé zpomalení
        abs_angle_for_factor = abs(actual_steer_angle) 
        # ANGLE_RIGHT_DEG je v kódu 50.0 (MAX_STEER_ANGLE)
        normalized_angle = min(abs_angle_for_factor / ANGLE_RIGHT_DEG, 1.0)


        # 2. Převedeme procentuální konstanty (20.0, 100.0) na faktory (0.2, 1.0)
        # TOTO JE KLÍČOVÁ OPRAVA: Dělíme 100.0
        factor_max = RPM_MAX_PERCENT / 100.0  # Bude 1.0
        factor_min = RPM_MIN_PERCENT / 100.0  # Bude 0.2

        # 3. Lineární interpolace faktoru zpomalení (nyní s faktory 0.2 a 1.0)
        speed_reduction_factor = (factor_max - factor_min) * (1.0 - normalized_angle) + factor_min
        # 4. Nastavení globální cílové RPM
        global_target_rpm = round(base_rpm_magnitude * speed_reduction_factor)
        # Určíme směr jízdy (závisí na tom, jestli je global_target_rpm kladné nebo záporné)
        if global_target_rpm >= 0:
            direction = 1 # Vpřed
        else:
            direction = -1 # Vzad
            
        # --- NOVÁ LOGIKA PRO DIFERENCIÁLNÍ RPM --- 
        if button.is_pressed: 
            control_drive_motors(0,0) 
            stop_all_servos() 
        elif global_target_rpm != 0: 
            # 1. Vypočítáme RPM pro jednotlivá kola (volá se nová/opravená funkce)
            rpm_list = calculate_ackerman_rpm(global_target_rpm, global_target_angle, T, L)

            # ID motorů v pořadí: [0x01 (LP), 0x02 (PP), 0x03 (LZ), 0x04 (PZ)]
            motor_ids = [0x01, 0x02, 0x03, 0x04] 
            
            # 2. Voláme funkci pro každý motor zvlášť s jeho vypočítaným RPM
            for i, motor_id in enumerate(motor_ids): 
                # Původní kód volal control_drive_motors_single.
                # Předpokládám, že jste zamýšlel použít existující control_drive_motors_diff[cite: 121].
                control_drive_motors_diff(motor_id, rpm_list[i], direction) # Opraveno volání
                
            # --- Upravený VÍCE ŘÁDKOVÝ DIAGNOSTICKÝ VÝPIS 🚀 ---
            # =========================================================================
            # ID motorů v pořadí: [0x01 (LP), 0x02 (PP), 0x03 (LZ), 0x04 (PZ)]
            motor_labels = ["LP", "PP", "LZ", "PZ"]
            motor_ids = [0x01, 0x02, 0x03, 0x04]
            if Print_auto_logic_control:
                print("-" * 30)
                print(f"DIAGNOSTIKA POHONU:")
                # ZACHOVÁNÍ CÍLOVÉHO ÚHLU
                print(f"Cílový Úhel Řízení (Ackerman): {global_target_angle:.1f}°")
                # NOVÝ ŘÁDEK PRO AKTUÁLNÍ ÚHEL
                print(f"Aktuální Úhel Řízení (Průměr Servo): {actual_steer_angle:.1f}°")
                # Opravíme zobrazení faktoru zpomalení (převod na procenta)
                print(f"Faktor zpomalení (za úhel): {speed_reduction_factor:.2f}x")
                print(f"Základní RPM (po korekci úhlu): {global_target_rpm:.1f} RPM | Směr: {'Vpřed' if direction == 1 else 'Vzad'}")

            # 2. Voláme funkci pro každý motor zvlášť s jeho vypočítaným RPM
            for i, motor_id in enumerate(motor_ids):
                motor_rpm = rpm_list[i]
                # Tisk individuální rychlosti
                if Print_auto_logic_control:
                    print(f"  -> {motor_labels[i]} (Motor ID 0x0{i+1}): {motor_rpm:.1f} RPM")
                    control_drive_motors_diff(motor_id, motor_rpm, direction)

                    print("-" * 30)
                    # DOPLNĚNÍ O NOVÝ ŘÁDEK (JAK V PŘEDCHOZÍCH LOGÁCH)
                    print(f"Aktuální základní/cílová rychlost: {global_target_rpm:.1f} RPM")
            
        else: 
            control_drive_motors(0, 0) 
            
        #print(f"Aktuální rychlost: {global_target_rpm:.1f} RPM")
        if Print_auto_logic_control:
            print(f"Aktuální základní/cílová rychlost: {global_target_rpm:.1f} RPM")
            
        # Logika detekce překážky
        if distance_cm_2 != 0 and distance_cm_2 < CELNI_MIN:
            celni_hit_counter += 1
            print(f"counter +1 {distance_cm_2}")
        else:
            celni_hit_counter = 0 # Vzdálenost je OK, resetujeme
            
            #print(f"counter 0 sen1 {distance_cm_1}")
         # PŘECHOD: Spuštění VYHÝBÁNÍ (Avoidance)
        if celni_hit_counter >= CELNI_TRIGGER_COUNT:
            print("KOLIZE DETEKOVÁNA: Spuštění vyhýbacího manévru.")
            celni_avoidance_start_time = time.time()
 
            # 1. PŘEPNUTÍ DO MANÉVRU
            Auto_sub_mode = "Avoidance"
            
            # 2. INICIALIZACE SEKVENČNÍHO STAVU (Začínáme zastavením a řízením)
            celni_avoidance_step = "Stop_and_Steer"
            celni_steer_start_time = time.time() # Spustíme časovač pro natočení kol
            
            # 3. Zastavíme motory okamžitě
            stop_all_motors()
            
            # 4. Nastavíme cíl řízení, který se použije v 'Stop_and_Steer'
            global_target_angle = global_target_angle # Např. -40.0°
            global_target_rpm = 0 # RPM je pro tuto fázi 0
    
            return # Ukončíme, aby se neaplikovala normální logika řízení   
            
        #pass
        #print(f"celni counter {celni_hit_counter}")
        #print(f"celni distance {distance_cm_2}")
    elif Auto_sub_mode == "Avoidance":
        
        # ====================================================
        # Fáze A: Zastavení a natočení kol ("Stop_and_Steer")
        # ====================================================
        if celni_avoidance_step == "Stop_and_Steer":
            
            # 1. Zastavíme motory (bezpečné opakování)
            stop_all_motors()
            
            # 2. Natočíme kola (Váš kód pro nastavení Serv)
            #actual_steer_angle = control_steer_servos(global_target_angle, T, L) # Předpoklad: tato fce řídí serva

            actual_steer_angle = -global_target_angle
            steering_speed = 100
            
            target_pos_master = pos_from_angle(40)
            target_pos_slave = calculate_ackerman_slave_pos(target_pos_master, 1) #
            
            #master_pos = pos_from_angle(target_angle_deg)
            #slave_pos = calculate_ackerman_slave_pos(master_pos, 1) # 1 značí, že Servo 1 je Master

            move_to_position(1, target_pos_master, steering_speed) 
            move_to_position(2, target_pos_slave, steering_speed)

            print("-" * 30)
            print("DIAGNOSTIKA VYHÝBÁNÍ (Krok 1: Natočení kol)")
            print(f"Cílový Úhel Řízení (Ackerman): {global_target_angle:.1f}°")
            print(f"Aktuální Úhel Řízení (Průměr Servo): {actual_steer_angle:.1f}°")
            
            # 3. Kontrola, zda uplynul čas na natočení kol
            if time.time() - celni_steer_start_time >= STEER_DELAY_S:
                print(f"Kola natočena za {STEER_DELAY_S}s. Přechod na couvání.")
                
                # Přejdeme k fázi couvání
                celni_avoidance_step = "Reverse_Drive"
                
                # ********** Spustíme ČASOVAČ PRO COUVÁNÍ ZDE **********
                celni_avoidance_start_time = time.time() 
                # ******************************************************
            
            return # Pokračujeme v tomto kroku v další iteraci
        
        elif celni_avoidance_step == "Reverse_Drive":
            
            # 1. Vypočítáme Ackermannovy RPM (pro couvání)
            # Použijeme ABSOLUTNÍ HODNOTU REVERSE_SPEED_RPM
            if button.is_pressed: 
                control_drive_motors(0,0) 
                stop_all_servos()
            else:
                rpm_list = calculate_ackerman_rpm(abs(REVERSE_SPEED_RPM), global_target_angle, T, L)

                motor_ids = [0x01, 0x02, 0x03, 0x04] 
                
                # 2. Voláme funkci pro každý motor (směr -1 pro zpátečku)
                for i, motor_id in enumerate(motor_ids): 
                    control_drive_motors_diff(motor_id, abs(rpm_list[i]), -1)
                
                # Diagnostika
                print("-" * 30)
                print(f"DIAGNOSTIKA POHONU (Krok 2: Couvání):")
                print(f"Couváme po dobu: {time.time() - celni_avoidance_start_time:.2f}s")
                
            # 3. Kontrola konce doby couvání
            if time.time() - celni_avoidance_start_time >= REVERSE_DURATION_S:
                print("Konec vyhýbání. Návrat do automatického režimu.")
                
                # Reset
                global_target_rpm = 0 
                global_target_angle = 0.0 
                
                # Zastavení motorů a vycentrování řízení
                #stop_all_motors() 
                #move_to_position(0, SERVO_CENTER_POS, 100) # Použijeme 0 pro obě serva
                
                # Návrat do hlavního Auto režimu
                Auto_sub_mode = "Auto"
                celni_hit_counter = 0
                celni_avoidance_step = "N/A" # Reset stavu
                return # Manévr dokončen
                
        # Pokud se stav dostane do neznámého kroku
        else:
            print(f"Chyba: Neznámý krok {celni_avoidance_step}. Návrat do Auto.")
            Auto_sub_mode = "Auto"
            celni_hit_counter = 0
            celni_avoidance_step = "N/A"
            return
    
            
        # Kontrola konce doby couvání
        if time.time() - celni_avoidance_start_time >= REVERSE_DURATION_S:
            print("Konec vyhýbání. Návrat do automatického režimu.")
            
            # Nastavíme RPM a úhel na nulu pro bezpečný návrat
            global_target_rpm = 0 
            global_target_angle = 0.0 
            

            # Návrat do hlavního Auto režimu
            Auto_sub_mode = "Auto"
            celni_hit_counter = 0
            return # Manévr dokončen

        # Dokud couváme, RPM a úhel zůstanou nastavené z přechodu (Couvání -200, Obrácený úhel)
        pass # Nic dalšího neděláme
            

def draw_ackerman_control_scene():
    global ack_selected_group, limit_45_deg_enabled
    screen.fill(LIGHT_GRAY)
    draw_current_mode_label()
    title_text = font_title.render("Ackermanovo řízení (pouze serva)", True, DARK_GRAY)
    title_rect = title_text.get_rect(center=(SCREEN_WIDTH / 2, 60))
    screen.blit(title_text, title_rect)

    label_speed = font_label.render("Rychlost (0-1000):", True, DARK_GRAY)
    screen.blit(label_speed, (250, 270))
    ack_speed_input.draw(screen)
    
    label_step = font_label.render("Krok (jednotky):", True, DARK_GRAY)
    screen.blit(label_step, (250, 320))
    ack_step_input.draw(screen)

    ack_instructions = font_label.render("Použijte šipky VLEVO/VPRAVO pro řízení.", True, DARK_GRAY)
    ack_inst_rect = ack_instructions.get_rect(center=(SCREEN_WIDTH/2, 450))
    screen.blit(ack_instructions, ack_inst_rect)
    
    # Vizuální zvýraznění vybrané skupiny
    group_label_y = 500
    for i, group_name in enumerate(ack_group_options):
        text_surface = font_label.render(group_name, True, DARK_GRAY)
        if group_name == ack_selected_group:
            text_surface = font_highlight.render(group_name, True, BLUE)
        text_rect = text_surface.get_rect(center=(SCREEN_WIDTH/2, group_label_y + i * 40))
        screen.blit(text_surface, text_rect)
    
    ack_group_inst = font_label.render("Použijte šipky NAHORU/DOLŮ pro změnu skupiny.", True, DARK_GRAY)
    ack_group_inst_rect = ack_group_inst.get_rect(center=(SCREEN_WIDTH/2, 650))
    screen.blit(ack_group_inst, ack_group_inst_rect)
    
    current_limit_text = "Zapnut" if limit_45_deg_enabled else "Vypnut"
    toggle_45_deg_limit_button.text = f"Limit 45°: {current_limit_text}"
    toggle_45_deg_limit_button.draw(screen)
    
    ack_back_button.draw(screen)
    
    draw_ackerman_simulation(screen, last_requested_positions[1], last_requested_positions[2], ack_selected_group)

    refresh_servo_status()
    y_offset = 250
    for servo_id in SERVO_IDS:
        state = servo_states[servo_id]
        status_text = f"Servo {servo_id}: Stav: {'Online' if state['online'] else 'Offline'}, Pozice: {state['position'] if state['online'] else 'N/A'}"
        color = GREEN if state['online'] else RED
        status_surface = font_label.render(status_text, True, color)
        screen.blit(status_surface, (50, y_offset + (servo_id - 1) * 30))

    motor_status_text = font_label.render("Status motorů: Není aktivní", True, DARK_GRAY)
    screen.blit(motor_status_text, (50, 250 + 4 * 30))

    pygame.display.flip()

def handle_ackerman_control_events(event):
    global ack_selected_group, ack_group_idx
    ack_speed_input.handle_event(event)
    ack_step_input.handle_event(event)
    ack_back_button.handle_event(event)
    toggle_45_deg_limit_button.handle_event(event)
    
    if event.type == pygame.KEYDOWN:
        if event.key == pygame.K_SPACE:
            ack_group_idx = (ack_group_idx - 1) % len(ack_group_options)
            ack_selected_group = ack_group_options[ack_group_idx]
            print(f"Skupina '{ack_selected_group}' vybrána.")
        elif event.key == pygame.K_SPACE:    #elif event.key == pygame.K_DOWN:  
            ack_group_idx = (ack_group_idx + 1) % len(ack_group_options)
            ack_selected_group = ack_group_options[ack_group_idx]
            print(f"Skupina '{ack_selected_group}' vybrána.")

def update_servos_for_ackerman():
    global last_requested_positions
    try:
        ack_speed_val = int(ack_speed_input.text)
        ack_step_val = int(ack_step_input.text)
    except ValueError:
        return

    keys = pygame.key.get_pressed()
    
    if keys[pygame.K_LEFT] or keys[pygame.K_RIGHT]:
        # Centrujeme ne-aktivní serva
        if ack_selected_group == "Předek":
            move_to_position(3, SERVO_CENTER_POS, ack_speed_val)
            move_to_position(4, SERVO_CENTER_POS, ack_speed_val)
        elif ack_selected_group == "Zadek":
            move_to_position(1, SERVO_CENTER_POS, ack_speed_val)
            move_to_position(2, SERVO_CENTER_POS, ack_speed_val)
            
        # Ovládáme aktivní skupinu
        if keys[pygame.K_LEFT]:
            if ack_selected_group in ["Předek", "Všechna kola stejně"]:
                last_requested_positions[1] -= ack_step_val
            if ack_selected_group == "Zadek":
                last_requested_positions[3] -= ack_step_val
        elif keys[pygame.K_RIGHT]:
            if ack_selected_group in ["Předek", "Všechna kola stejně"]:
                last_requested_positions[1] += ack_step_val
            if ack_selected_group == "Zadek":
                last_requested_positions[3] += ack_step_val

        # Ochrana proti přetečení
        last_requested_positions[1] = max(SW_MIN_POS, min(SW_MAX_POS, last_requested_positions[1]))
        last_requested_positions[3] = max(SW_MIN_POS, min(SW_MAX_POS, last_requested_positions[3]))
        
        if ack_selected_group == "Předek":
            move_to_position(1, last_requested_positions[1], ack_speed_val)
            slave_pos_ack = calculate_ackerman_slave_pos(last_requested_positions[1], 1)
            move_to_position(2, slave_pos_ack, ack_speed_val)
        elif ack_selected_group == "Zadek":
            move_to_position(3, last_requested_positions[3], ack_speed_val)
            slave_pos_ack = calculate_ackerman_slave_pos(last_requested_positions[3], 3)
            move_to_position(4, slave_pos_ack, ack_speed_val)
        elif ack_selected_group == "Všechna kola stejně":
            move_to_position(1, last_requested_positions[1], ack_speed_val)
            move_to_position(2, last_requested_positions[1], ack_speed_val)
            move_to_position(3, last_requested_positions[1], ack_speed_val)
            move_to_position(4, last_requested_positions[1], ack_speed_val)
    else:
        stop_all_servos()

def draw_full_ackerman_scene():
    global last_requested_positions, ack_selected_group, current_motor_rpm, motor_data
    screen.fill(BACKGROUND_COLOR)
    draw_current_mode_label()
    title_text = font_title.render("Full Ackerman", True, ACCENT_COLOR)
    title_rect = title_text.get_rect(center=(SCREEN_WIDTH / 2, 60))
    screen.blit(title_text, title_rect)

    instructions_text = font_label.render("Šipky VLEVO/VPRAVO pro řízení, Šipky NAHORU/DOLŮ pro pojezd", True, DARK_GRAY)
    inst_rect = instructions_text.get_rect(center=(SCREEN_WIDTH / 2, 120))
    screen.blit(instructions_text, inst_rect)
    
    # Nové prvky pro nastavení motorů
    label_max_rpm = font_label.render("Max. rychlost motorů (1-100):", True, DARK_GRAY)
    screen.blit(label_max_rpm, (250, 360))
    max_motor_rpm_input.draw(screen)

    # Vizuální zvýraznění vybrané skupiny
    group_label_y = 150
    for i, group_name in enumerate(ack_group_options):
        text_surface = font_label.render(group_name, True, DARK_GRAY)
        if group_name == ack_selected_group:
            text_surface = font_highlight.render(group_name, True, BLUE)
        text_rect = text_surface.get_rect(center=(SCREEN_WIDTH/2, group_label_y + i * 40))
        screen.blit(text_surface, text_rect)

    # Kreslení stavu motorů
    y_offset_start = 300
    for i, motor_id_val in enumerate(motor_ids):
        data = motor_data[motor_id_val]
        speed_val = data.get('speed_rpm')
        speed_display = f"{speed_val:.1f}" if speed_val is not None else '?'
        speed_text = font_label.render(f"Motor {motor_id_val}: {speed_display} rpm", True, DARK_GRAY)
        screen.blit(speed_text, (50, y_offset_start + i * 40))

    # Kreslení stavu serv
    y_offset_start = 500
    for i, servo_id in enumerate(SERVO_IDS):
        data = servo_states[servo_id]
        pos_text = font_label.render(f"Servo {servo_id}: {data['position'] if data['online'] else 'Offline'}", True, GREEN if data['online'] else RED)
        screen.blit(pos_text, (50, y_offset_start + i * 40))
        
    draw_ackerman_simulation(screen, last_requested_positions[1], last_requested_positions[2], ack_selected_group)
    
    # --- NOVÁ SEKCE PRO ZOBRAZENÍ STAVU SENZORŮ A TLAČÍTKA ---
    sensor_y_start = 700
    
    # Stav Senzoru 1
    sensor1_text = font_highlight.render(f"Senzor 1: {global_distance1_status}", True, DARK_GRAY)
    screen.blit(sensor1_text, (50, sensor_y_start))
    
    # Stav Senzoru 2
    sensor2_text = font_highlight.render(f"Senzor 2: {global_distance2_status}", True, DARK_GRAY)
    screen.blit(sensor2_text, (50, sensor_y_start + 40))

    # Stav Tlačítka
    button_color = GREEN if "UVOLNĚNO" in global_tlacitko_stav else RED
    button_text = font_highlight.render(f"Tlačítko: {global_tlacitko_stav}", True, button_color)
    screen.blit(button_text, (50, sensor_y_start + 80))
    # --- KONEC NOVÉ SEKCIE ---

    ack_back_button.draw(screen)
    pygame.display.flip()

def handle_full_ackerman_events(event):
    global ack_selected_group, ack_group_idx
    ack_back_button.handle_event(event)
    max_motor_rpm_input.handle_event(event)

    if event.type == pygame.KEYDOWN:
        if event.key == pygame.K_SPACE:
            ack_group_idx = (ack_group_idx - 1) % len(ack_group_options)
            ack_selected_group = ack_group_options[ack_group_idx]
            print(f"Skupina '{ack_selected_group}' vybrána.")
        elif event.key == pygame.K_SPACE:
            ack_group_idx = (ack_group_idx + 1) % len(ack_group_options)
            ack_selected_group = ack_group_options[ack_group_idx]
            print(f"Skupina '{ack_selected_group}' vybrána.")

def update_full_ackerman():
    global last_requested_positions, current_motor_rpm, motor_direction
    
    keys = pygame.key.get_pressed()
    
    # Ovládání pojezdu (motorů)
    try:
        max_rpm_val = int(max_motor_rpm_input.text)
    except ValueError:
        max_rpm_val = 100  # Výchozí hodnota

    new_motor_direction = 0
    if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
        new_motor_direction = 1
    elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
        new_motor_direction = -1
    
    if new_motor_direction != motor_direction:
        motor_direction = new_motor_direction
        current_motor_rpm = 0
    
    if motor_direction != 0:
        current_motor_rpm += 5
        current_motor_rpm = min(max_rpm_val, current_motor_rpm)
        control_drive_motors(current_motor_rpm, motor_direction)
    else:
        current_motor_rpm = 0
        control_drive_motors(current_motor_rpm, motor_direction)
   

    # Ovládání Ackermanova řízení
    step_size = 10
    if keys[pygame.K_LEFT] or keys[pygame.K_RIGHT]:
        # Centrujeme ne-aktivní serva
        if ack_selected_group == "Předek":
            move_to_position(3, SERVO_CENTER_POS, 1000)
            move_to_position(4, SERVO_CENTER_POS, 1000)
        elif ack_selected_group == "Zadek":
            move_to_position(1, SERVO_CENTER_POS, 1000)
            move_to_position(2, SERVO_CENTER_POS, 1000)
        
        # Ovládáme aktivní skupinu
        if keys[pygame.K_LEFT]:
            if ack_selected_group in ["Předek", "Všechna kola stejně"]:
                last_requested_positions[1] -= step_size
            if ack_selected_group == "Zadek":
                last_requested_positions[3] -= step_size
        elif keys[pygame.K_RIGHT]:
            if ack_selected_group in ["Předek", "Všechna kola stejně"]:
                last_requested_positions[1] += step_size
            if ack_selected_group == "Zadek":
                last_requested_positions[3] += step_size

        # Ochrana proti přetečení
        last_requested_positions[1] = max(SW_MIN_POS, min(SW_MAX_POS, last_requested_positions[1]))
        last_requested_positions[3] = max(SW_MIN_POS, min(SW_MAX_POS, last_requested_positions[3]))
        
        if ack_selected_group == "Předek":
            move_to_position(1, last_requested_positions[1], 1000)
            slave_pos_ack = calculate_ackerman_slave_pos(last_requested_positions[1], 1)
            move_to_position(2, slave_pos_ack, 1000)
        elif ack_selected_group == "Zadek":
            move_to_position(3, last_requested_positions[3], 1000)
            slave_pos_ack = calculate_ackerman_slave_pos(last_requested_positions[3], 3)
            move_to_position(4, slave_pos_ack, 1000)
        elif ack_selected_group == "Všechna kola stejně":
            move_to_position(1, last_requested_positions[1], 1000)
            move_to_position(2, last_requested_positions[1], 1000)
            move_to_position(3, last_requested_positions[1], 1000)
            move_to_position(4, last_requested_positions[1], 1000)
    elif ack_selected_group == "Auto":   #plný automat :-)
        auto_control_logic()
    else:
        stop_all_servos()


# --- Drawing and Event Handling for all Scenes ---
def draw_ackerman_simulation(screen, front_left_pos, front_right_pos, ack_selected_group):
    SIM_X = 750
    SIM_Y = 350
    CAR_WIDTH = 150
    CAR_HEIGHT = 300
    WHEEL_WIDTH = 20
    WHEEL_HEIGHT = 40
    
    fl_angle = (last_requested_positions[1] - SERVO_CENTER_POS) / POS_DEG_RATIO
    fr_angle = (last_requested_positions[2] - SERVO_CENTER_POS) / POS_DEG_RATIO
    rl_angle = 0
    rr_angle = 0
    
    if ack_selected_group == "Zadek":
        rl_angle = (last_requested_positions[3] - SERVO_CENTER_POS) / POS_DEG_RATIO
        rr_angle = (last_requested_positions[4] - SERVO_CENTER_POS) / POS_DEG_RATIO
    elif ack_selected_group == "Všechna kola stejně":
        rl_angle = (last_requested_positions[1] - SERVO_CENTER_POS) / POS_DEG_RATIO
        rr_angle = (last_requested_positions[1] - SERVO_CENTER_POS) / POS_DEG_RATIO
        
    car_rect = pygame.Rect(SIM_X - CAR_WIDTH/2, SIM_Y - CAR_HEIGHT/2, CAR_WIDTH, CAR_HEIGHT)
    pygame.draw.rect(screen, BLUE, car_rect, border_radius=10)
    
    fl_pos = (SIM_X - CAR_WIDTH/2, SIM_Y - CAR_HEIGHT/2)
    fr_pos = (SIM_X + CAR_WIDTH/2 - WHEEL_WIDTH, SIM_Y - CAR_HEIGHT/2)
    rl_pos = (SIM_X - CAR_WIDTH/2, SIM_Y + CAR_HEIGHT/2 - WHEEL_HEIGHT)
    rr_pos = (SIM_X + CAR_WIDTH/2 - WHEEL_WIDTH, SIM_Y + CAR_HEIGHT/2 - WHEEL_HEIGHT)
    
    def draw_rotated_wheel(pos, angle):
        wheel_surface = pygame.Surface((WHEEL_WIDTH, WHEEL_HEIGHT), pygame.SRCALPHA)
        pygame.draw.rect(wheel_surface, DARK_GRAY, (0, 0, WHEEL_WIDTH, WHEEL_HEIGHT), border_radius=5)
        rotated_wheel = pygame.transform.rotate(wheel_surface, -angle)
        rotated_rect = rotated_wheel.get_rect(center=(pos[0] + WHEEL_WIDTH/2, pos[1] + WHEEL_HEIGHT/2))
        screen.blit(rotated_wheel, rotated_rect)

    draw_rotated_wheel(fl_pos, fl_angle)
    draw_rotated_wheel(fr_pos, fr_angle)
    draw_rotated_wheel(rl_pos, rl_angle)
    draw_rotated_wheel(rr_pos, rr_angle)

# --- Hlavní smyčka programu ---
running = True
last_status_refresh = time.time()
while running:
    read_sensors() 
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if current_scene == "main_menu":
            handle_main_menu_events(event)
        elif current_scene == "manual_control":
            handle_manual_control_events(event)
        elif current_scene == "auto_control":
            handle_auto_control_events(event)
        elif current_scene == "ackerman_control":
            handle_ackerman_control_events(event)
        elif current_scene == "full_ackerman":
            handle_full_ackerman_events(event)
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            running = False
    
    if current_scene == "auto_control":
        update_servos_for_auto_control()
    elif current_scene == "ackerman_control":
        update_servos_for_ackerman()
    elif current_scene == "full_ackerman":
        update_full_ackerman()
    if time.time() - last_status_refresh > 1.0:
        refresh_servo_status()
        last_status_refresh = time.time()

    # --- Vykreslování ---
    if current_scene == "main_menu":
        draw_main_menu()
    elif current_scene == "manual_control":
        draw_manual_control_scene()
    elif current_scene == "auto_control":
        draw_auto_control_scene()
    elif current_scene == "ackerman_control":
        draw_ackerman_control_scene()
    elif current_scene == "full_ackerman":
        draw_full_ackerman_scene()
        
            # Stav Autonomního režimu
    if full_ackerman_sub_mode == "Auto":
        # Zobrazíme cílový úhel řízení (ne Ackermannův)
        target_info = f"Cíl Úhel: {global_target_angle:.1f}° | Cíl RPM: {global_target_rpm}"
        draw_text(screen, target_info, 10, 50, (0, 0, 255))
        
        # NOVÉ: Zobrazíme vypočtené Ackermannovy úhly
        servo_info = (f"Servo 1: {global_angle_servo_1:.1f}° | "
                      f"Servo 2: {global_angle_servo_2:.1f}°")
        draw_text(screen, servo_info, 10, 70, (0, 150, 0)) # Nový řádek

    pygame.display.flip()
    pygame.time.Clock().tick(30)

# === Ukončení programu ===
print("Zastavuji motory a serva, uklízím...")
stop_all_servos()
if ser_motor:
    stop_all_motors()
    ser_motor.close()
portHandler.closePort()
pygame.quit()
print("✅ Motory a serva zastaveny, konec programu.")
