#!/usr/bin/env python3
# test_auto.py - Scope Program v1.16
version = "1.17"

import time
import threading
import json
import math
import os
import pygame
import cv2
import argparse
import RPi.GPIO as GPIO
from pygame.locals import *

# -------------------- Configuration --------------------
STEP_DELAY_FAST = 0.0035
STEP_DELAY_SLOW = 0.01
BACKOFF_STEPS = 100
MAX_INIT_STEPS = 12000
DEBOUNCE_COUNT_INIT = 6
SENSOR_DEBOUNCE_SECS = 0.08

# Manual tab-step tolerance fallback (used for delta-based inference)
OPT2_STEP_THRESHOLD = 50

# Pins
LIMIT_ZOOM = 17
LIMIT_FOCUS = 5
OPTICAL_1 = 26
OPTICAL_2 = 19

motors = {
    "TRAY":  {"STEP": 22, "DIR": 27, "BTN_CW": 12, "BTN_CCW": 25},
    "FOCUS": {"STEP": 21, "DIR": 20, "BTN_CW": 18, "BTN_CCW": 4},
    "ZOOM":  {"STEP": 13, "DIR": 6,  "BTN_CW": 23, "BTN_CCW": 24}
}
# Each specimen maps to a tuple of two ranges (start, end)
specimen_ranges = {
    "Specimen 1": [(13809, 14168), (5575, 5940)],
    "Specimen 2": [(14630, 14987), (6399, 6765)],
    "Specimen 3": [(15454, 15819), (7223, 7585)],
    "Specimen 4": [(16284, 16641), (8060, 8400)],
    "Specimen 5": [(17103, 17462), (8900, 9219)],
    "Specimen 6": [(9600, 10100), (9600, 10100)],
    "Specimen 7": [(10495, 10856), (2292, 2651)],
    "Specimen 8": [(11314, 11678), (3133, 3466)],
    "Specimen 9": [(12150, 12506), (3937, 4297)],
    "Specimen 10": [(12977, 13348), (4758, 5114)]
}

NUM_SPECIMENS = len(specimen_ranges)

# Steps per revolution (used to compute degrees & revolutions for absolute rotation)
STEPS_PER_REV = max(
    end
    for ranges in specimen_ranges.values()
    for (_, end) in ranges
) + 500  # heuristic buffer


# Tolerance fraction for matching tray_steps to a specimen range (10%)
RANGE_TOLERANCE_FRACTION = 0.10

# Safety limits
MAX_ZOOM_STEPS = 10000
MAX_FOCUS_STEPS = 10000

# Autonomous timing
AUTONOMY_IDLE_TIME = 20.0  # seconds to display per specimen (idle time)

# UI
SCREEN_WIDTH = 1920
SCREEN_HEIGHT = 1080
FPS = 60
PANEL_WIDTH = 500
PANEL_PADDING = 20
PANEL_BG_COLOR = (0, 0, 0, 180)
PANEL_HEIGHT = SCREEN_HEIGHT // 2.5

# -------------------- State & Locks --------------------
motor_locks = {name: threading.Lock() for name in motors}
tray_counter_lock = threading.Lock()
zoom_lock = threading.Lock()
focus_lock = threading.Lock()

running = True

# debug overlay (show yellow step/rotation text only when True)
show_debug_overlay = False
initialization_in_progress = False
initialized = False
error_state = False
abort_autonomous = False

limit_state = {LIMIT_ZOOM: False, LIMIT_FOCUS: False}
optical_state = {OPTICAL_1: False, OPTICAL_2: False}

# Steps / offsets
zoom_steps = 0
focus_steps = 0
tray_steps = 10000
rotation_offset_steps = 0  # relative offset from last OPTICAL_2 rising edge

last_opt2_tray_steps = 0
expected_center_steps = None

tray_direction = 1
last_specimen_index = None

# zoom mapping used by UI
zoom_min = 0
zoom_max = -4395

# user/autonomy timestamps
last_user_action_time = time.time() - (AUTONOMY_IDLE_TIME + 1.0)
last_auto_advance_time = time.time() - (AUTONOMY_IDLE_TIME + 1.0)
autonomous_mode = True

# current specimen tab/state
current_tab_index = None     # 1-based tab index (1=wide tab)
current_specimen_index = 6   # start by showing specimen 6 by default

# OPTICAL_2 fall->rise manual detection state (kept as fallback)
opt2_saw_fall = False
opt2_fall_tray_steps = 0

# pygame init
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Microscope Viewer with Autonomous Overlay")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 36)
smaller_font = pygame.font.SysFont(None, 24)

# track threads for clean shutdown
threads = []
def spawn_thread(target, args=(), daemon=True):
    t = threading.Thread(target=target, args=args)
    t.daemon = daemon
    t.start()
    threads.append(t)
    return t

# -------------------- Helpers --------------------
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def safe_input(pin):
    try:
        return GPIO.input(pin)
    except Exception as e:
        print(f"[GPIO ERROR] safe_input pin={pin} err={e}")
        return GPIO.LOW

def _safe_gpio_output(pin, value):
    global error_state
    try:
        GPIO.output(pin, value)
        return True
    except Exception as e:
        print(f"[GPIO ERROR] pin {pin} write failed: {e}")
        error_state = True
        return False

# -------------------- Specimen utilities --------------------
def specimen_for_tab(tab_index):
    """Map a tab index to the specimen that is across from that tab."""
    # tab 1 -> specimen 6 mapping as per your design (circular)
    return ((tab_index + (NUM_SPECIMENS // 2) - 1) % NUM_SPECIMENS) + 1

def specimen_index_from_tray_steps(step_count, tolerance_fraction=0.05):
    """
    Determine the specimen index from absolute tray steps using specimen_ranges.
    Returns specimen index (1-based) if within any range or within tolerance of center, otherwise None.
    """
    best_idx = None
    best_dist = None

    for idx, (specimen_name, ranges) in enumerate(specimen_ranges.items()):
        for start, end in ranges:
            # direct in-range match
            if start <= step_count <= end:
                return idx + 1  # 1-based index

            # tolerance based on center
            center = (start + end) // 2
            width = max(1, end - start + 1)
            tol = max(50, int(width * tolerance_fraction))
            dist = abs(step_count - center)

            if dist <= tol:
                return idx + 1

            # track closest center for potential fallback
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_idx = idx + 1

    # If nothing within tolerance, return None
    return None

# -------------------- Specimen JSON / UI --------------------
def load_specimen(index):
    """Load specimen JSON safely; return dict with defaults."""
    fname = f"specimen_{index}.json"
    try:
        with open(fname, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        data = {}
    result = {
        "name": data.get("name", f"Specimen {index}"),
        "location": data.get("location", "Unknown"),
        "collector": data.get("collector", "Unknown"),
        "chem": data.get("chem", "Unknown"),
        "map_image": data.get("map_image", "placeholder.png"),
        "eds_image": data.get("eds_image", "placeholder.png"),
        "qr_code_image": data.get("qr_code_image", "placeholder.png"),
        "default_zoom": int(data.get("default_zoom", 0) or 0),
        "default_focus": int(data.get("default_focus", 0) or 0),
        "default_rotation_offset": int(data.get("default_rotation_offset", 0) or 0)
    }
    print(f"[SPECIMEN] Loaded specimen_{index}.json -> zoom={result['default_zoom']} focus={result['default_focus']} offset={result['default_rotation_offset']}")
    return result

# UI image placeholders
map_image = None
eds_image = None
qr_image = None
text_lines = []

# Preload specimen names once
all_specimen_names = []
for i in range(1, 11):  # 1 to 10
    s = load_specimen(i)
    all_specimen_names.append(s["name"])

def update_specimen(index):
    global specimen, map_image, eds_image, qr_image, text_lines, current_specimen_index
    specimen = load_specimen(index)
    image_w = PANEL_WIDTH - 2 * PANEL_PADDING
    image_h = max(8, int((PANEL_HEIGHT - 2 * PANEL_PADDING - 3 * 10 - 3 * 36) / 3))
    map_image = load_image(specimen["map_image"], image_w, image_h)
    eds_image = load_image(specimen["eds_image"], image_w, image_h)
    qr_image = load_image(specimen["qr_code_image"], image_w, image_h)
    text_lines = [
        font.render(f"Name: {specimen['name']}", True, (255, 255, 255)),
        smaller_font.render(f"Chem Formula: {specimen['chem']}", True, (255, 255, 255)),
        smaller_font.render(f"Location: {specimen['location']}", True, (255, 255, 255)),
        smaller_font.render(f"Collector: {specimen['collector']}", True, (255, 255, 255))
    ]
    current_specimen_index = index
    print(f"[UI] update_specimen -> index={index} name={specimen['name']}")

def load_image(path, max_w, max_h):
    try:
        img = pygame.image.load(path).convert_alpha()
        img = pygame.transform.smoothscale(img, (max_w, max_h))
        return img
    except Exception:
        return None

# Panel animation state
panel_x = SCREEN_WIDTH
panel_target_x = SCREEN_WIDTH - PANEL_WIDTH
PANEL_ANIM_SPEED = 30
panel_open_state = False
    
def draw_panel(x_pos):
    panel_surf = pygame.Surface((PANEL_WIDTH, PANEL_HEIGHT), pygame.SRCALPHA)
    panel_surf.fill(PANEL_BG_COLOR)
    y = PANEL_PADDING

    # Draw images
    for img in [map_image, eds_image, qr_image]:
        if img:
            panel_surf.blit(img, (PANEL_PADDING, y))
            y += image_height + 10

    # Draw specimen metadata
    for line in text_lines:
        panel_surf.blit(line, (PANEL_PADDING, y))
        y += 36

    # Draw the specimen list
    list_font = pygame.font.SysFont(None, 24)
    y += 20  # spacing before list
    for i, name in enumerate(all_specimen_names, start=1):
        if i == current_specimen_index:
            text = list_font.render(f"→ {i}. {name}", True, (255, 255, 255))  # white
        else:
            text = list_font.render(f"  {i}. {name}", True, (150, 150, 150))  # gray
        panel_surf.blit(text, (PANEL_PADDING, y))
        y += 26  # line spacing

    screen.blit(panel_surf, (x_pos, 0))

def trigger_panel():
    global panel_x
    panel_x = SCREEN_WIDTH

def update_panel():
    global panel_x, panel_open_state
    # Show panel only when OPTICAL_2 is HIGH (blocked)
    should_open = optical_state.get(OPTICAL_2, False)
    if should_open and not panel_open_state:
        print("[UI] Panel OPEN (OPTICAL_2 HIGH)")
        panel_open_state = True
    if not should_open and panel_open_state:
        print("[UI] Panel CLOSED (OPTICAL_2 LOW)")
        panel_open_state = False

    if panel_open_state:
        if panel_x > panel_target_x:
            panel_x -= PANEL_ANIM_SPEED
            if panel_x < panel_target_x:
                panel_x = panel_target_x
    else:
        if panel_x < SCREEN_WIDTH:
            panel_x += PANEL_ANIM_SPEED
            if panel_x > SCREEN_WIDTH:
                panel_x = SCREEN_WIDTH
    if panel_x < SCREEN_WIDTH:
        draw_panel(panel_x)

# -------------------- GPIO setup --------------------
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for motor in motors.values():
        GPIO.setup(motor["STEP"], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(motor["DIR"], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(motor["BTN_CW"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(motor["BTN_CCW"], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.setup(LIMIT_ZOOM, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LIMIT_FOCUS, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(OPTICAL_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(OPTICAL_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# -------------------- Sensor monitoring --------------------
last_calibrated_time = 0.0
last_opt2_rise = 0.0
logged_s1 = False

def monitor_sensors():
    global last_calibrated_time, last_opt2_rise, tray_steps, rotation_offset_steps, last_opt2_tray_steps
    global last_specimen_index, initialized, current_tab_index, current_specimen_index
    global opt2_saw_fall, opt2_fall_tray_steps, logged_s1

    print("[SENSORS] monitor_sensors thread started")
    prev_s1 = bool(safe_input(OPTICAL_1))
    prev_s2 = bool(safe_input(OPTICAL_2))
    optical_state[OPTICAL_1] = prev_s1
    optical_state[OPTICAL_2] = prev_s2
    while running:
        # limit switches
        for pin, name in [(LIMIT_ZOOM, "ZOOM LIMIT"), (LIMIT_FOCUS, "FOCUS LIMIT")]:
            state = safe_input(pin)
            if state == GPIO.HIGH and not limit_state[pin]:
                print(f"[LIMIT] {name} REACHED (pin {pin})")
                limit_state[pin] = True
                pygame.mixer.init()
                pygame.mixer.music.load("alert.mp3")
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy():
                    time.sleep(0.1)
            elif state == GPIO.LOW and limit_state[pin]:
                limit_state[pin] = False

        s1 = bool(safe_input(OPTICAL_1))
        s2 = bool(safe_input(OPTICAL_2))
        now = time.time()

        # detect falling edge OPTICAL_2: HIGH -> LOW
        if prev_s2 and not s2:
            # record the current tray_steps as the 'leave' point
            with tray_counter_lock:
                opt2_fall_tray_steps = tray_steps
            opt2_saw_fall = True
            print(f"[SENSOR] OPTICAL_2 FALL detected -> saved_tray_steps={opt2_fall_tray_steps}")

        # detect rising edge OPTICAL_2: LOW -> HIGH
        if not prev_s2 and s2:
            #last_opt2_tray_steps = cur
            # debounce
            if (now - last_opt2_rise) > SENSOR_DEBOUNCE_SECS:
                last_opt2_rise = now
                with tray_counter_lock:
                    cur = tray_steps

                    
                # Prefer absolute mapping via specimen_ranges
                matched_specimen = specimen_index_from_tray_steps(cur)
                if matched_specimen is not None:
                    # Update UI showing specimen matched to absolute ranges
                    current_specimen_index = matched_specimen
                    # compute tab corresponding to that specimen
                    # find a tab that maps to this specimen (inverse of specimen_for_tab)
                    # We can iterate tabs 1..NUM_SPECIMENS and pick the one where specimen_for_tab(tab) == matched_specimen
                    for t in range(1, NUM_SPECIMENS+1):
                        if specimen_for_tab(t) == matched_specimen:
                            current_tab_index = t
                            break
                    update_specimen(matched_specimen)
                    trigger_panel()
                    last_specimen_index = matched_specimen
                    last_opt2_tray_steps = cur
                    print(f"[SENSOR] Updating last_opt2_tray_steps={last_opt2_tray_steps}")
                    print(f"[SENSOR] OPTICAL_2 RISING -> matched specimen by absolute ranges: specimen={matched_specimen} tab={current_tab_index} tray_steps={cur}")
                else:
                    # fallback: if we previously saw a fall, infer the direction by delta
                    if opt2_saw_fall:
                        delta = cur - opt2_fall_tray_steps
                        if delta > OPT2_STEP_THRESHOLD:
                            dir_flag = +1
                        elif delta < -OPT2_STEP_THRESHOLD:
                            dir_flag = -1
                        else:
                            dir_flag = 0
                        # If current_tab_index unknown derive from tray_steps via absolute mapping fallback
                        if current_tab_index is None:
                            t = None
                            # attempt to find any tab whose specimen center is near cur
                            t_candidate = None
                            si = specimen_index_from_tray_steps(cur)
                            if si:
                                # find tab that maps to this specimen
                                for ttt in range(1, NUM_SPECIMENS+1):
                                    if specimen_for_tab(ttt) == si:
                                        t = ttt
                                        break
                            if t is None:
                                t = 1
                            current_tab_index = t

                        if dir_flag == 0:
                            new_tab = current_tab_index
                        else:
                            new_tab = ((current_tab_index - 1 + dir_flag) % NUM_SPECIMENS) + 1

                        specimen_index = specimen_for_tab(new_tab)
                        update_specimen(specimen_index)
                        trigger_panel()
                        current_tab_index = new_tab
                        current_specimen_index = specimen_index
                        last_specimen_index = specimen_index
                        print(f"[SENSOR] OPTICAL_2 RISING fallback -> delta={delta} dir={dir_flag} new_tab={new_tab} specimen={specimen_index} tray_steps={cur}")
                    else:
                        # No fall recorded, try to best-effort map by absolute centers (closest)
                        t = specimen_index_from_tray_steps(cur)
                        if t:
                            current_specimen_index = t
                            update_specimen(t)
                            trigger_panel()
                            print(f"[SENSOR] OPTICAL_2 RISING (no prior fall) -> specimen={t} tray_steps={cur}")
                        else:
                            print(f"[SENSOR] OPTICAL_2 RISING but could not map to specimen (tray_steps={cur})")
                # clear fall-state
                opt2_saw_fall = False

        # log OPTICAL_1 rising (brief)
        if s1 and not prev_s1:
            if not logged_s1:
                print("[SENSOR] OPTICAL_1 RISING")
                logged_s1 = True
        if not s1:
            logged_s1 = False

        # Both sensors -> wide tab (tab1). When not init, reset counters and absolute origin.
        if s1 and s2:
            if (now - last_calibrated_time) > SENSOR_DEBOUNCE_SECS:
                last_calibrated_time = now
                if initialization_in_progress:
                    print("[SENSOR] Wide tab detected DURING INIT (monitor will NOT reset counters).")
                else:
                    if not initialized:
                        print(f"[INIT] Calibrated on tab 1 (wide tab) detected by sensors")
                    with tray_counter_lock:
                        tray_steps = 10000
                        rotation_offset_steps = 0
                        print("[RESET] Tray step counter reset to 0 (Tab 1) and rotation offset zeroed.")
                    last_specimen_index = None
                    current_tab_index = 1
                    current_specimen_index = specimen_for_tab(1)
                    update_specimen(current_specimen_index)
                    trigger_panel()
                    print(f"[TAB->SPECIMEN] Tab 1 -> Specimen {current_specimen_index}")

        # update prevs and optical_state
        prev_s1 = s1
        prev_s2 = s2
        optical_state[OPTICAL_1] = s1
        optical_state[OPTICAL_2] = s2

        time.sleep(0.02)

# -------------------- Low-level motor helpers --------------------
def move_motor_relative(motor_name, steps, step_delay=STEP_DELAY_FAST, timeout=None):
    global zoom_steps, focus_steps, tray_steps, rotation_offset_steps, error_state, running, abort_autonomous, initialization_in_progress

    if error_state:
        print(f"[MOVE] Not moving {motor_name} (error_state={error_state})")
        return False

    motor = motors[motor_name]
    step_pin = motor["STEP"]
    dir_pin = motor["DIR"]

    lock = motor_locks[motor_name]
    acquired = lock.acquire(timeout=2.0)
    if not acquired:
        print(f"[MOVE] Could not acquire lock for {motor_name}")
        return False

    print(f"[MOVE] Acquired motor lock for {motor_name}")
    try:
        if steps == 0:
            return True
        direction_high = GPIO.HIGH if steps > 0 else GPIO.LOW
        if not _safe_gpio_output(dir_pin, direction_high):
            print(f"[MOVE] failed to set dir for {motor_name}")
            return False

        n = abs(int(steps))
        start = time.time()
        for i in range(n):
            if not running:
                print(f"[MOVE] Aborting {motor_name} because running=False")
                return False
            if error_state:
                print(f"[MOVE] Aborting {motor_name} because error_state")
                return False
            if abort_autonomous and not initialization_in_progress:
                print(f"[MOVE] Aborting {motor_name} because abort_autonomous set (and not init)")
                return False
            if timeout is not None and (time.time() - start) > timeout:
                print(f"[MOVE] Timeout moving {motor_name} after {time.time()-start:.1f}s (i={i}/{n})")
                return False

            if not _safe_gpio_output(step_pin, GPIO.HIGH):
                return False
            time.sleep(step_delay)
            if not _safe_gpio_output(step_pin, GPIO.LOW):
                return False
            time.sleep(step_delay)

            if motor_name == "TRAY":
                with tray_counter_lock:
                    delta = 1 if direction_high == GPIO.HIGH else -1
                    tray_steps += delta
                    rotation_offset_steps += delta
            elif motor_name == "ZOOM":
                with zoom_lock:
                    zoom_steps += 1 if direction_high == GPIO.HIGH else -1
                    if abs(zoom_steps) > MAX_ZOOM_STEPS:
                        print("[ERROR] Zoom safety exceeded")
                        error_state = True
                        return False
                    if safe_input(LIMIT_ZOOM) == GPIO.HIGH:
                        print("[LIMIT] Zoom limit hit during move, backing off")
                        back_dir = GPIO.LOW if direction_high == GPIO.HIGH else GPIO.HIGH
                        if not _safe_gpio_output(dir_pin, back_dir):
                            return False
                        for _ in range(BACKOFF_STEPS):
                            if not running:
                                return False
                            if not _safe_gpio_output(step_pin, GPIO.HIGH): return False
                            time.sleep(step_delay)
                            if not _safe_gpio_output(step_pin, GPIO.LOW): return False
                            time.sleep(step_delay)
                            zoom_steps += 1 if back_dir == GPIO.HIGH else -1
                        if not _safe_gpio_output(dir_pin, direction_high):
                            return False
            elif motor_name == "FOCUS":
                with focus_lock:
                    focus_steps += 1 if direction_high == GPIO.HIGH else -1
                    if abs(focus_steps) > MAX_FOCUS_STEPS:
                        print("[ERROR] Focus safety exceeded")
                        error_state = True
                        return False
                    if safe_input(LIMIT_FOCUS) == GPIO.HIGH:
                        print("[LIMIT] Focus limit hit during move, backing off")
                        back_dir = GPIO.LOW if direction_high == GPIO.HIGH else GPIO.HIGH
                        if not _safe_gpio_output(dir_pin, back_dir):
                            return False
                        for _ in range(BACKOFF_STEPS):
                            if not running:
                                return False
                            if not _safe_gpio_output(step_pin, GPIO.HIGH): return False
                            time.sleep(step_delay)
                            if not _safe_gpio_output(step_pin, GPIO.LOW): return False
                            time.sleep(step_delay)
                            focus_steps += 1 if back_dir == GPIO.HIGH else -1
                        if not _safe_gpio_output(dir_pin, direction_high):
                            return False

        return True
    finally:
        lock.release()
        print(f"[MOVE] Released motor lock for {motor_name}")

def move_motor_to_absolute(motor_name, target_steps, step_delay=STEP_DELAY_FAST):
    if motor_name == "TRAY":
        with tray_counter_lock:
            cur = tray_steps
    elif motor_name == "ZOOM":
        with zoom_lock:
            cur = zoom_steps
    elif motor_name == "FOCUS":
        with focus_lock:
            cur = focus_steps
    else:
        return False
    delta = target_steps - cur
    estimated_time = abs(delta) * (2 * step_delay)
    timeout = max(15.0, estimated_time * 2.0 + 5.0)
    print(f"[MOVE-ABS] motor={motor_name} current={cur} target={target_steps} delta={delta} est_time={estimated_time:.2f}s timeout={timeout:.1f}s")
    ok = move_motor_relative(motor_name, delta, step_delay=step_delay, timeout=timeout)
    print(f"[MOVE-ABS] motor={motor_name} completed -> ok={ok}")
    return ok

# -------------------- TRAY next-tab + dro helper (used by init and auto) --------------------
def move_tray_to_next_tab_and_apply_offset(dro, step_delay_fast=STEP_DELAY_FAST, max_steps=MAX_INIT_STEPS):
    """
    Move forward until the next OPTICAL_2 rising (tab), then apply dro (relative).
    Returns True/False.
    """
    global tray_steps, rotation_offset_steps, abort_autonomous, running

    motor = motors["TRAY"]
    step_pin = motor["STEP"]; dir_pin = motor["DIR"]
    lock = motor_locks["TRAY"]
    acquired = lock.acquire(timeout=5.0)
    if not acquired:
        print("[MOVE-TRAY] Could not acquire TRAY motor lock")
        return False
    print("[MOVE-TRAY] Acquired TRAY lock for next-tab movement")

    try:
        if not _safe_gpio_output(dir_pin, GPIO.HIGH):
            print("[MOVE-TRAY] Failed to set TRAY dir HIGH")
            return False

        saw_s2_false = False
        consec_rise = 0
        steps_taken = 0

        while steps_taken < max_steps and running:
            if abort_autonomous and not initialization_in_progress:
                print("[MOVE-TRAY] Aborting because abort_autonomous set")
                return False

            if not _safe_gpio_output(step_pin, GPIO.HIGH):
                return False
            time.sleep(step_delay_fast)
            if not _safe_gpio_output(step_pin, GPIO.LOW):
                return False
            time.sleep(step_delay_fast)

            with tray_counter_lock:
                tray_steps += 1
                rotation_offset_steps += 1
            steps_taken += 1

            s2 = safe_input(OPTICAL_2)
            if not saw_s2_false:
                if not s2:
                    saw_s2_false = True
            else:
                if s2:
                    consec_rise += 1
                    if consec_rise >= 2:
                        print(f"[MOVE-TRAY] OPTICAL_2 rising detected after {steps_taken} steps")
                        break
                else:
                    consec_rise = 0

            if steps_taken % 500 == 0:
                print(f"[MOVE-TRAY] scanning steps={steps_taken} s2={s2}")

        if steps_taken >= max_steps:
            print("[MOVE-TRAY] Timed out searching for next tab")
            return False

        # Apply dro (if present)
        if dro == 0:
            print("[MOVE-TRAY] No dro to apply")
            return True

        dir_high_for_dro = GPIO.HIGH if dro > 0 else GPIO.LOW
        if not _safe_gpio_output(dir_pin, dir_high_for_dro):
            return False
        dro_steps = abs(int(dro))
        print(f"[MOVE-TRAY] Applying dro={dro} ({dro_steps} steps, dir={'FORWARD' if dir_high_for_dro==GPIO.HIGH else 'REVERSE'})")

        for i in range(dro_steps):
            if abort_autonomous and not initialization_in_progress:
                print("[MOVE-TRAY] Aborting during dro because abort_autonomous set")
                return False
            if not _safe_gpio_output(step_pin, GPIO.HIGH):
                return False
            time.sleep(step_delay_fast)
            if not _safe_gpio_output(step_pin, GPIO.LOW):
                return False
            time.sleep(step_delay_fast)
            with tray_counter_lock:
                tray_steps += 1 if dir_high_for_dro == GPIO.HIGH else -1
                rotation_offset_steps += 1 if dir_high_for_dro == GPIO.HIGH else -1

        print(f"[MOVE-TRAY] Applied dro; new tray_steps={tray_steps}")
        return True
    finally:
        lock.release()
        print("[MOVE-TRAY] Released TRAY lock")

# -------------------- Initialization (zoom, focus, tray) --------------------
def initialize_zoom():
    global zoom_steps
    print("[INIT] Initializing ZOOM axis...")
    motor = motors["ZOOM"]
    step_pin = motor["STEP"]; dir_pin = motor["DIR"]
    lock = motor_locks["ZOOM"]
    lock.acquire()
    try:
        found = False
        if not _safe_gpio_output(dir_pin, GPIO.HIGH):
            print("[INIT] failed to set zoom dir HIGH")
            return False
        for i in range(MAX_INIT_STEPS):
            if not running:
                return False
            if not _safe_gpio_output(step_pin, GPIO.HIGH): return False
            time.sleep(STEP_DELAY_FAST)
            if not _safe_gpio_output(step_pin, GPIO.LOW): return False
            time.sleep(STEP_DELAY_FAST)
            if safe_input(LIMIT_ZOOM) == GPIO.HIGH:
                found = True
                print("[INIT] Zoom limit detected (dir HIGH).")
                break
            if i % 1500 == 0:
                print(f"[INIT-ZOOM] i={i} limit={safe_input(LIMIT_ZOOM)}")
        if not found:
            if not _safe_gpio_output(dir_pin, GPIO.LOW):
                return False
            for i in range(MAX_INIT_STEPS):
                if not running: return False
                if not _safe_gpio_output(step_pin, GPIO.HIGH): return False
                time.sleep(STEP_DELAY_FAST)
                if not _safe_gpio_output(step_pin, GPIO.LOW): return False
                time.sleep(STEP_DELAY_FAST)
                if safe_input(LIMIT_ZOOM) == GPIO.HIGH:
                    found = True
                    print("[INIT] Zoom limit detected (dir LOW).")
                    break
                if i % 1500 == 0:
                    print(f"[INIT-ZOOM] rev i={i} limit={safe_input(LIMIT_ZOOM)}")
        if not found:
            print("[WARN] Zoom limit not found during init.")
            return False
        try:
            curdir = safe_input(dir_pin)
        except Exception:
            curdir = GPIO.HIGH
        back_dir = GPIO.LOW if curdir == GPIO.HIGH else GPIO.HIGH
        if not _safe_gpio_output(dir_pin, back_dir): return False
        for _ in range(BACKOFF_STEPS):
            if not running: return False
            if not _safe_gpio_output(step_pin, GPIO.HIGH): return False
            time.sleep(STEP_DELAY_FAST)
            if not _safe_gpio_output(step_pin, GPIO.LOW): return False
            time.sleep(STEP_DELAY_FAST)
        with zoom_lock:
            zoom_steps = 0
        print("[INIT] Zoom initialized and zeroed.")
        return True
    finally:
        lock.release()

def initialize_focus():
    global focus_steps
    print("[INIT] Initializing FOCUS axis...")
    motor = motors["FOCUS"]
    step_pin = motor["STEP"]; dir_pin = motor["DIR"]
    lock = motor_locks["FOCUS"]
    lock.acquire()
    try:
        found = False
        if not _safe_gpio_output(dir_pin, GPIO.HIGH):
            return False
        for i in range(MAX_INIT_STEPS):
            if not running: return False
            if not _safe_gpio_output(step_pin, GPIO.HIGH): return False
            time.sleep(STEP_DELAY_FAST)
            if not _safe_gpio_output(step_pin, GPIO.LOW): return False
            time.sleep(STEP_DELAY_FAST)
            if safe_input(LIMIT_FOCUS) == GPIO.HIGH:
                found = True
                print("[INIT] Focus limit detected (dir HIGH).")
                break
            if i % 1500 == 0:
                print(f"[INIT-FOCUS] i={i} limit={safe_input(LIMIT_FOCUS)}")
        if not found:
            if not _safe_gpio_output(dir_pin, GPIO.LOW):
                return False
            for i in range(MAX_INIT_STEPS):
                if not running: return False
                if not _safe_gpio_output(step_pin, GPIO.HIGH): return False
                time.sleep(STEP_DELAY_FAST)
                if not _safe_gpio_output(step_pin, GPIO.LOW): return False
                time.sleep(STEP_DELAY_FAST)
                if safe_input(LIMIT_FOCUS) == GPIO.HIGH:
                    found = True
                    print("[INIT] Focus limit detected (dir LOW).")
                    break
                if i % 1500 == 0:
                    print(f"[INIT-FOCUS] rev i={i} limit={safe_input(LIMIT_FOCUS)}")
        if not found:
            print("[WARN] Focus limit not found during init.")
            return False
        try:
            curdir = safe_input(dir_pin)
        except Exception:
            curdir = GPIO.HIGH
        back_dir = GPIO.LOW if curdir == GPIO.HIGH else GPIO.HIGH
        if not _safe_gpio_output(dir_pin, back_dir):
            return False
        for _ in range(BACKOFF_STEPS):
            if not running: return False
            if not _safe_gpio_output(step_pin, GPIO.HIGH): return False
            time.sleep(STEP_DELAY_FAST)
            if not _safe_gpio_output(step_pin, GPIO.LOW): return False
            time.sleep(STEP_DELAY_FAST)
        with focus_lock:
            focus_steps = 0
        print("[INIT] Focus initialized and zeroed.")
        return True
    finally:
        lock.release()

def initialize_tray_and_advance():
    """
    Carefully initialize tray:
      - rotate until wide tab -> set absolute origin
      - advance to next tab (fast) and apply specimen JSON offset (dro)
      - apply zoom/focus from that specimen JSON
      - set initialized state
    """
    global tray_steps, rotation_offset_steps, initialized, initialization_in_progress, last_user_action_time, last_specimen_index, last_auto_advance_time, current_tab_index, current_specimen_index

    print("[INIT] Initializing TRAY axis (rotate until wide tab detected)...")
    motor = motors["TRAY"]
    step_pin = motor["STEP"]; dir_pin = motor["DIR"]
    lock = motor_locks["TRAY"]

    acquired = lock.acquire(timeout=5.0)
    if not acquired:
        print("[INIT] Could not acquire TRAY motor lock for scanning.")
        return False
    print("[INIT] TRAY motor lock acquired by init (for manual scanning).")

    try:
        if not _safe_gpio_output(dir_pin, GPIO.HIGH):
            print("[INIT] Could not set TRAY dir for init")
            return False

        steps_taken = 0
        consecutive = 0
        print("[INIT] Searching for wide tab (s1 & s2 true)...")
        while steps_taken < MAX_INIT_STEPS and running:
            s1 = safe_input(OPTICAL_1)
            s2 = safe_input(OPTICAL_2)
            if s1 and s2:
                consecutive += 1
                if consecutive >= DEBOUNCE_COUNT_INIT:
                    with tray_counter_lock:
                        tray_steps = 10000
                        rotation_offset_steps = 0
                    print("[INIT] Wide tab detected (treated as tab 1). tray_steps and rotation_offset zeroed.")
                    current_tab_index = 1
                    break
            else:
                consecutive = 0

            # slow step while scanning
            if not _safe_gpio_output(step_pin, GPIO.HIGH):
                return False
            time.sleep(STEP_DELAY_SLOW)
            if not _safe_gpio_output(step_pin, GPIO.LOW):
                return False
            time.sleep(STEP_DELAY_SLOW)
            with tray_counter_lock:
                tray_steps += 1
                rotation_offset_steps += 1
            steps_taken += 1

            if steps_taken % 500 == 0:
                print(f"[INIT-TRAY] scanning steps={steps_taken} s1={s1} s2={s2}")

        if steps_taken >= MAX_INIT_STEPS:
            print("[INIT] TRAY init timed out finding wide tab.")
            return False

    finally:
        try:
            lock.release()
        except Exception:
            pass
        print("[INIT] Released TRAY motor lock after scanning")

    # Now advance to next tab and use specimen across mapping
    next_tab = 2
    specimen_shown = specimen_for_tab(next_tab)
    s = load_specimen(specimen_shown)
    dz = s.get("default_zoom", 0)
    df = s.get("default_focus", 0)
    dro = s.get("default_rotation_offset", 0)

    print(f"[INIT] Advancing to next tab (tab {next_tab}). Will apply dro={dro} for specimen {specimen_shown}")
    ok = move_tray_to_next_tab_and_apply_offset(dro)
    if not ok:
        print("[INIT] Failed during move to next tab + offset")
        return False

    print(f"[INIT] Applying ZOOM={dz} and FOCUS={df}")
    _ = move_motor_to_absolute("ZOOM", dz, step_delay=STEP_DELAY_FAST)
    _ = move_motor_to_absolute("FOCUS", df, step_delay=STEP_DELAY_FAST)

    update_specimen(specimen_shown)
    trigger_panel()
    last_specimen_index = specimen_shown
    current_tab_index = next_tab
    current_specimen_index = specimen_shown

    print("[INIT] Tray initialization and auto-advance completed; handing back to init controller.")
    return True

# -------------------- Button handling --------------------
def set_user_activity():
    global last_user_action_time, autonomous_mode, abort_autonomous
    last_user_action_time = time.time()
    autonomous_mode = False
    abort_autonomous = True
    print("[USER] User interaction detected: switching to manual mode (abort_autonomous=True).")

def spin_motor(motor_name, step_pin, dir_pin, direction, button_pin):
    global tray_direction, zoom_steps, focus_steps, tray_steps, rotation_offset_steps, error_state, running

    lock = motor_locks[motor_name]
    if not lock.acquire(blocking=False):
        print(f"[BUTTON] Could not acquire lock for {motor_name}, skipping spin (another controller active).")
        return
    try:
        if not _safe_gpio_output(dir_pin, direction):
            return
        if motor_name == "TRAY":
            tray_direction = 1 if direction == GPIO.HIGH else -1

        print(f"[BUTTON] {motor_name} manual control TAKEN (holding button)")

        while running and safe_input(button_pin) == GPIO.LOW and not error_state:
            if motor_name == "TRAY":
                delay = STEP_DELAY_SLOW if optical_state.get(OPTICAL_2, False) else STEP_DELAY_FAST
            else:
                delay = STEP_DELAY_FAST

            if not _safe_gpio_output(step_pin, GPIO.HIGH):
                break
            time.sleep(delay)
            if not _safe_gpio_output(step_pin, GPIO.LOW):
                break
            time.sleep(delay)

            # update counters (no per-step printing)
            if motor_name == "TRAY":
                with tray_counter_lock:
                    delta = tray_direction
                    tray_steps += delta
                    rotation_offset_steps += delta
            elif motor_name == "ZOOM":
                with zoom_lock:
                    zoom_steps += 1 if direction == GPIO.HIGH else -1
                    if abs(zoom_steps) > MAX_ZOOM_STEPS:
                        print("[ERROR] Zoom safety exceeded (user)")
                        error_state = True
                        break
                    if safe_input(LIMIT_ZOOM) == GPIO.HIGH:
                        print("[LIMIT] Zoom limit hit during user move, backing off (user)")
                        back_dir = GPIO.LOW if direction == GPIO.HIGH else GPIO.HIGH
                        if not _safe_gpio_output(dir_pin, back_dir):
                            break
                        for _ in range(BACKOFF_STEPS):
                            if not running: break
                            if not _safe_gpio_output(step_pin, GPIO.HIGH): break
                            time.sleep(delay)
                            if not _safe_gpio_output(step_pin, GPIO.LOW): break
                            time.sleep(delay)
                            zoom_steps += 1 if back_dir == GPIO.HIGH else -1
                        if not _safe_gpio_output(dir_pin, direction): break
            elif motor_name == "FOCUS":
                with focus_lock:
                    focus_steps += 1 if direction == GPIO.HIGH else -1
                    if abs(focus_steps) > MAX_FOCUS_STEPS:
                        print("[ERROR] Focus safety exceeded (user)")
                        error_state = True
                        break
                    if safe_input(LIMIT_FOCUS) == GPIO.HIGH:
                        print("[LIMIT] Focus limit hit during user move, backing off (user)")
                        back_dir = GPIO.LOW if direction == GPIO.HIGH else GPIO.HIGH
                        if not _safe_gpio_output(dir_pin, back_dir):
                            break
                        for _ in range(BACKOFF_STEPS):
                            if not running: break
                            if not _safe_gpio_output(step_pin, GPIO.HIGH): break
                            time.sleep(delay)
                            if not _safe_gpio_output(step_pin, GPIO.LOW): break
                            time.sleep(delay)
                            focus_steps += 1 if back_dir == GPIO.HIGH else -1
                        if not _safe_gpio_output(dir_pin, direction): break
    finally:
        try:
            lock.release()
        except Exception:
            pass
        print(f"[BUTTON] {motor_name} manual control RELEASED")

def debounce_and_spin(motor_name, step_pin, dir_pin, button_pin, direction):
    time.sleep(0.02)
    if running and safe_input(button_pin) == GPIO.LOW:
        set_user_activity()
        spin_motor(motor_name, step_pin, dir_pin, direction, button_pin)

def monitor_button(motor_name, step_pin, dir_pin, button_pin, direction):
    print(f"[BUTTON MON] start monitoring motor={motor_name}, button_pin={button_pin}")
    last_state = safe_input(button_pin)
    while running:
        current_state = safe_input(button_pin)
        if current_state != last_state:
            if current_state == GPIO.LOW:
                if initialization_in_progress:
                    print(f"[BUTTON MON] Ignoring press on pin {button_pin} during init (initialization_in_progress={initialization_in_progress})")
                else:
                    print(f"[BUTTON MON] Press detected motor={motor_name} pin={button_pin}")
                    spawn_thread(debounce_and_spin, args=(motor_name, step_pin, dir_pin, button_pin, direction))
        last_state = current_state
        time.sleep(0.01)

# -------------------- Autonomous --------------------
def autonomous_loop():
    global autonomous_mode, abort_autonomous, last_specimen_index, last_auto_advance_time, last_user_action_time
    global current_tab_index, current_specimen_index, opt2_saw_fall, opt2_fall_tray_steps

    print("[AUTO] autonomous_loop thread started")
    autonomous_mode = True
    abort_autonomous = False

    while running:
        if error_state:
            time.sleep(0.5)
            continue

        # Wait until initialization completes
        if initialization_in_progress or not initialized:
            time.sleep(0.2)
            continue

        # Respect recent user activity
        if time.time() - last_user_action_time < AUTONOMY_IDLE_TIME:
            if autonomous_mode:
                print("[AUTO] User activity detected - suppressing auto")
            autonomous_mode = False
            abort_autonomous = True
            time.sleep(0.5)
            continue
        else:
            if not autonomous_mode:
                print("[AUTO] Idle timeout reached - resuming autonomous")
            autonomous_mode = True
            abort_autonomous = False

        # Reconcile a pending OPTICAL_2 fall (user left beam but no rise)
        if opt2_saw_fall:
            with tray_counter_lock:
                cur = tray_steps
            # primary: try to map absolute tray_steps to specimen
            mapped = specimen_index_from_tray_steps(cur)
            if mapped is not None:
                current_specimen_index = mapped
                # find tab that maps to the specimen
                for t in range(1, NUM_SPECIMENS+1):
                    if specimen_for_tab(t) == mapped:
                        current_tab_index = t
                        break
                update_specimen(mapped)
                trigger_panel()
                print(f"[AUTO] Reconciled pending fall -> mapped by absolute ranges to specimen {mapped} (tray_steps={cur})")
            else:
                # fallback to delta inference
                delta = cur - opt2_fall_tray_steps
                if delta > OPT2_STEP_THRESHOLD:
                    # user moved forward enough to assume next tab
                    # compute next specimen from current_specimen_index (if available) else default 1
                    if current_specimen_index is None:
                        current_specimen_index = 1
                    current_specimen_index = (current_specimen_index % NUM_SPECIMENS) + 1
                    print(f"[AUTO] Reconciled pending fall -> delta={delta} => assumed moved forward to specimen {current_specimen_index}")
                elif delta < -OPT2_STEP_THRESHOLD:
                    if current_specimen_index is None:
                        current_specimen_index = 1
                    current_specimen_index = ((current_specimen_index - 2) % NUM_SPECIMENS) + 1
                    print(f"[AUTO] Reconciled pending fall -> delta={delta} => assumed moved backward to specimen {current_specimen_index}")
                else:
                    print(f"[AUTO] Reconciled pending fall -> delta={delta} => assumed still on specimen {current_specimen_index}")
                update_specimen(current_specimen_index)
            # clear the fall-state
            opt2_saw_fall = False

        # enforce display hold after previous auto-advance (do not immediately sequence)
        if time.time() - last_auto_advance_time < AUTONOMY_IDLE_TIME:
            time.sleep(0.2)
            continue

        # Determine current specimen if unknown by absolute tray_steps
        if current_specimen_index is None:
            with tray_counter_lock:
                t = specimen_index_from_tray_steps(tray_steps)
            if t is not None:
                current_specimen_index = t
                print(f"[AUTO] Determined current_specimen_index from tray_steps => {current_specimen_index}")
            else:
                current_specimen_index = 1
                print("[AUTO] Could not determine current specimen from tray_steps; defaulting to 1")

        # Compute the next specimen (always advance forward)
        next_specimen = (current_specimen_index % NUM_SPECIMENS) + 1
        s = load_specimen(next_specimen)
        dz = s.get("default_zoom", 0)
        df = s.get("default_focus", 0)
        dro = s.get("default_rotation_offset", 0)

        print(f"[AUTO] Advancing to specimen {next_specimen} (will detect next tab then apply dro={dro})")

        ok = move_tray_to_next_tab_and_apply_offset(dro)
        if not ok:
            if abort_autonomous:
                print("[AUTO] Move aborted by user")
                time.sleep(0.1)
                continue
            if error_state:
                print("[AUTO] Move failed due to error")
                time.sleep(0.1)
                continue

        # after successful move, set current specimen
        current_specimen_index = next_specimen

        # apply zoom/focus
        ok = move_motor_to_absolute("ZOOM", dz, step_delay=STEP_DELAY_FAST)
        if not ok:
            if abort_autonomous: continue
            if error_state: continue

        ok = move_motor_to_absolute("FOCUS", df, step_delay=STEP_DELAY_FAST)
        if not ok:
            if abort_autonomous: continue
            if error_state: continue

        update_specimen(next_specimen)
        trigger_panel()
        last_specimen_index = next_specimen

        with tray_counter_lock:
            tab_hit = specimen_index_from_tray_steps(tray_steps)
        print(f"[AUTO] Move complete. tray_steps={tray_steps} => specimen_mapped={tab_hit}")
        print(f"[AUTO] Now displaying specimen {next_specimen} for {AUTONOMY_IDLE_TIME}s (or until user interrupt)")

        last_auto_advance_time = time.time()
        display_start = time.time()
        while time.time() - display_start < AUTONOMY_IDLE_TIME and running:
            if abort_autonomous or not autonomous_mode or error_state:
                break
            time.sleep(0.2)

# -------------------- Threads start / cleanup --------------------
def start_motor_threads():
    # spawn button monitors and autonomous loop (monitor_sensors should already be running)
    for name, pins in motors.items():
        spawn_thread(monitor_button, args=(name, pins["STEP"], pins["DIR"], pins["BTN_CW"], GPIO.HIGH))
        spawn_thread(monitor_button, args=(name, pins["STEP"], pins["DIR"], pins["BTN_CCW"], GPIO.LOW))
    spawn_thread(autonomous_loop)

def run_initialization():
    global initialization_in_progress, initialized, last_auto_advance_time
    initialization_in_progress = True
    print("[INIT_THREAD] Starting initialization sequence (zoom -> focus -> tray)")
    try:
        zoom_ok = initialize_zoom()
        print(f"[INIT_THREAD] zoom init result: {zoom_ok}")
        focus_ok = initialize_focus()
        print(f"[INIT_THREAD] focus init result: {focus_ok}")

        tray_ok = initialize_tray_and_advance()
        print(f"[INIT_THREAD] tray init result: {tray_ok}")

        if tray_ok:
            initialized = True
            last_auto_advance_time = time.time()
            print("[INIT_THREAD] Initialization succeeded; initialized=True and holding first specimen")
        else:
            print("[INIT_THREAD] Initialization failed or was aborted.")
    finally:
        initialization_in_progress = False
        print(f"[INIT_THREAD] Initialization finished. initialized={initialized}, initialization_in_progress={initialization_in_progress}")

def cleanup():
    global running
    print("[CLEANUP] Shutting down...")
    running = False
    time.sleep(0.05)
    for t in threads:
        try:
            if t.is_alive():
                t.join(timeout=1.0)
        except Exception:
            pass
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f"[CLEANUP] GPIO cleanup error: {e}")

# -------------------- Camera/Main Loop --------------------
def main_loop():
    global error_state, autonomous_mode, abort_autonomous, last_user_action_time, last_auto_advance_time, initialized

    gst_str = (
        "v4l2src device=/dev/video0 ! "
        "video/x-raw, width=1920, height=1080, framerate=50/1, colorimetry=bt601 ! "
        "videoconvert ! appsink"
    )
    print("[VIDEO] Opening video stream...")
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("[VIDEO] GStreamer open failed; falling back to /dev/video0")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[VIDEO] Failed to open camera. Video disabled (will continue headless).")
            cap = None
        else:
            print("[VIDEO] Fallback /dev/video0 opened.")
    else:
        print("[VIDEO] Video stream opened (GStreamer).")

    # ensure UI images before panel draws
    update_specimen(current_specimen_index)

    # start sensor thread early so it can see optical pins during init
    # NOTE: main will call setup_gpio before this
    # we started monitor_sensors earlier in main(), so don't start again here

    # spawn button monitors and autonomous
    start_motor_threads()

    # start initialization routine
    spawn_thread(run_initialization)

    while running:
        if cap:
            ret, frame = cap.read()
            if ret:
                try:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame_surface = pygame.image.frombuffer(frame_rgb.tobytes(), frame_rgb.shape[1::-1], "RGB")
                    screen.blit(frame_surface, (0, 0))
                except Exception as e:
                    print(f"[VIDEO] Frame draw error: {e}")
                    screen.fill((0, 0, 0))
            else:
                time.sleep(0.01)
                screen.fill((0, 0, 0))
        else:
            screen.fill((0, 0, 0))

        # compute zoom_scale for ruler
        with zoom_lock:
            denom = abs((zoom_max - zoom_min)) if abs((zoom_max - zoom_min)) != 0 else 1
            zoom_fraction = (abs(zoom_steps) - zoom_min) / denom
            zoom_fraction = clamp(zoom_fraction, 0.0, 1.0)
            zoom_scale = 1.0 + 2.3 * zoom_fraction

        # Top-left debug text (optional)
        if show_debug_overlay:
            zoom_text = font.render(f"Zoom step: {zoom_steps}", True, (255, 255, 0))
            focus_text = font.render(f"Focus step: {focus_steps}", True, (255, 255, 0))
            # read tray counters under the lock to avoid tearing
            with tray_counter_lock:
                abs_steps = tray_steps
                local_last_opt2 = last_opt2_tray_steps
            rel_rot = abs_steps - local_last_opt2
            # Debug
            last_text = font.render(f"last_opt2_tray_steps: {local_last_opt2}", True, (255, 255, 0))
            rotation_rel_text = font.render(f"Rotation offset (rel): {rel_rot}", True, (255, 255, 0))
        else:
            # still need abs_steps for absolute rotation math below
            with tray_counter_lock:
                abs_steps = tray_steps


        if show_debug_overlay:
            screen.blit(zoom_text, (20, 20))
            screen.blit(focus_text, (20, 60))
            screen.blit(rotation_rel_text, (20, 100))
            #screen.blit(rotation_abs_text, (20, 140))
            # Debug
            screen.blit(last_text, (20, 180))
        
        # Show initialization status only while in progress
        if initialization_in_progress:
            big_font = pygame.font.SysFont(None, 48)  # larger font
            small_font = pygame.font.SysFont(None, 36)

            line1 = big_font.render("Initialization In Progress", True, (200, 200, 255))
            line2 = small_font.render("A Project of MinDatNH tom@mindatnh.org", True, (200, 200, 255))

            # Center horizontally
            line1_rect = line1.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 - 20))
            line2_rect = line2.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 + 30))

            screen.blit(line1, line1_rect)
            screen.blit(line2, line2_rect)
        
        if error_state:
            err = font.render("ERROR: Motor limit exceeded or GPIO error. Motors disabled.", True, (255, 0, 0))
            screen.blit(err, (SCREEN_WIDTH//2 - 300, SCREEN_HEIGHT//2))
            pygame.display.flip()
            for event in pygame.event.get():
                if event.type == QUIT:
                    return
                elif event.type == KEYDOWN and event.key == K_ESCAPE:
                    return
            clock.tick(FPS)
            continue

        # draw ruler (unchanged)
        ruler_y = SCREEN_HEIGHT - 50
        ruler_width = int(0.8 * SCREEN_WIDTH)
        ruler_x_start = (SCREEN_WIDTH - ruler_width) // 2
        mm_per_px = 0.05 / zoom_scale
        ideal_tick_spacing_px = 160
        ideal_tick_mm = ideal_tick_spacing_px * mm_per_px
        try:
            exponent = math.floor(math.log10(ideal_tick_mm))
        except Exception:
            exponent = 0
        base = ideal_tick_mm / (10 ** exponent) if exponent != 0 else ideal_tick_mm
        if base < 1.5:
            tick_mm = 1 * (10 ** exponent)
        elif base < 3.5:
            tick_mm = 2 * (10 ** exponent)
        elif base < 7.5:
            tick_mm = 5 * (10 ** exponent)
        else:
            tick_mm = 10 * (10 ** exponent)
        tick_px = int(tick_mm / mm_per_px) if mm_per_px != 0 else 1
        num_ticks = max(1, ruler_width // max(1, tick_px))
        pygame.draw.line(screen, (0, 255, 0), (ruler_x_start, ruler_y), (ruler_x_start + ruler_width, ruler_y), 2)
        for i in range(num_ticks + 1):
            x = ruler_x_start + i * tick_px
            pygame.draw.line(screen, (0, 255, 0), (x, ruler_y - 10), (x, ruler_y + 10), 2)
            raw_mm = (x - ruler_x_start) * mm_per_px
            label_str = f"{raw_mm:.1f} mm"
            label = font.render(label_str, True, (255, 255, 255))
            screen.blit(label, (x - 20, ruler_y - 35))

        update_panel()
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == QUIT:
                return
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                return

        # resume auto if idle long enough
        if not autonomous_mode and (time.time() - last_user_action_time) > AUTONOMY_IDLE_TIME:
            print("[AUTO] Idle timeout reached; resuming autonomous mode.")
            abort_autonomous = False
            autonomous_mode = True

        clock.tick(FPS)

    if cap:
        cap.release()

# -------------------- Main --------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", action="store_true", help="Show yellow debug overlay (zoom/focus/rotation text)")
    args = parser.parse_args()
    show_debug_overlay = args.debug
    if show_debug_overlay:
        print("[CONFIG] Debug overlay enabled (yellow debug texts will be shown)")
    try:
        print(f"Scope Program v{version} starting.")
        print(f"[START] zoom_min={zoom_min}, zoom_max={zoom_max}, AUTONOMY_IDLE_TIME={AUTONOMY_IDLE_TIME}")
        print(f"[CONFIG] OPT2_STEP_THRESHOLD={OPT2_STEP_THRESHOLD} RANGE_TOL={RANGE_TOLERANCE_FRACTION*100:.0f}% STEPS_PER_REV={STEPS_PER_REV}")
        setup_gpio()
        # start monitor_sensors so sensors will be read during init
        spawn_thread(monitor_sensors)
        # run main loop (which will spawn init, button monitors, and autonomous)
        main_loop()
    except KeyboardInterrupt:
        print("\nExiting (KeyboardInterrupt)...")
    finally:
        cleanup()
        pygame.quit()
import argparse
