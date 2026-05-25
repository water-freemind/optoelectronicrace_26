import time, os, gc, sys, math
from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA, Pin, UART
import cv_lite
import image

# ======================== 1. 参数调优区 ========================

# --- 边缘检测 (影响找框的灵敏度) ---
CANNY_1 = 36
CANNY_2 = 235
APPROX_EPS = 0.02
MAX_COS = 0.98
MIN_AREA = 2000

RATIO_LOW = 0.4
RATIO_HIGH = 1.8

RECT_WIDTH_CM = 27.6
RECT_HEIGHT_CM = 19.1
CIRCLE_RADIUS_CM = 6.0
TARGET_POINT = (164, 106)

MAX_LOST_FRAMES = 15
MAX_CENTER_JUMP = 150
SWITCH_LIMIT = 10

INSET_RATIO = 0.90  # 【画矩形专用】矩形内缩比例

# --- 滤波参数 (新增) ---
# EMA_ALPHA 决定新数据的权重。范围 0.0 ~ 1.0。
# 值越小 (如 0.2)：滤波越强，数据越丝滑，但跟随会有微小延迟。
# 值越大 (如 0.8)：跟随越快，但抗抖动能力变弱。
EMA_ALPHA = 0.55

# --- 曝光自适应 ---
TARGET_BRIGHTNESS = 128
GAIN_MIN = 0.5
GAIN_MAX = 2.0
BINARY_THRESH = 128
# ==============================================================

# 硬件引脚
KEY_PIN_NUM = 53
RGB_R_PIN = 62
RGB_G_PIN = 20
RGB_B_PIN = 63

LED_ON = 0
LED_OFF = 1

# 全局系统状态
MODE_CIRCLE = 0
MODE_RECT = 1
current_mode = MODE_CIRCLE  # 默认上机画圆

DISPLAY_MODE_NORMAL = 0
DISPLAY_MODE_EDGE   = 1
DISPLAY_MODE_BINARY = 2
display_mode = DISPLAY_MODE_NORMAL

current_base_point_index = 0
base_point_counter = 0

prev_min_corners = None
lost_counter = 0

# EMA 历史值缓存
last_dx_c, last_dy_c = 0.0, 0.0
last_dx_b, last_dy_b = 0.0, 0.0
prev_center = None
key_press_time = 0
key_long_handled = False

# ================= 几何算法 =================
def get_line_intersection(line1, line2):
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2
    A1, B1 = y2 - y1, x1 - x2
    C1 = A1 * x1 + B1 * y1
    A2, B2 = y4 - y3, x3 - x4
    C2 = A2 * x3 + B2 * y3
    det = A1 * B2 - A2 * B1
    if det == 0: return ((x1 + x3) / 2, (y1 + y3) / 2)
    return ((B2 * C1 - B1 * C2) / det, (A1 * C2 - A2 * C1) / det)

def calculate_perspective_circle(center, corners, radius_cm):
    width_px = math.sqrt((corners[1][0] - corners[0][0])**2 + (corners[1][1] - corners[0][1])**2)
    height_px = math.sqrt((corners[2][0] - corners[1][0])**2 + (corners[2][1] - corners[1][1])**2)
    px_per_cm_x = width_px / RECT_WIDTH_CM
    px_per_cm_y = height_px / RECT_HEIGHT_CM
    rx, ry = radius_cm * px_per_cm_x, radius_cm * px_per_cm_y
    pts = []
    for i in range(16):
        angle = 2 * math.pi * i / 16
        pts.append((int(center[0] + rx * math.cos(angle)), int(center[1] + ry * math.sin(angle))))
    return pts

def calculate_perspective_rectangle(center, corners, inset_ratio):
    pts = []
    inner_corners = []
    for corner in corners:
        nx = center[0] + (corner[0] - center[0]) * inset_ratio
        ny = center[1] + (corner[1] - center[1]) * inset_ratio
        inner_corners.append((nx, ny))

    for i in range(4):
        p_start = inner_corners[i]
        p_end = inner_corners[(i + 1) % 4]
        for j in range(4):
            ratio = j / 4.0
            x = p_start[0] + (p_end[0] - p_start[0]) * ratio
            y = p_start[1] + (p_end[1] - p_start[1]) * ratio
            pts.append((int(x), int(y)))
    return pts

def sort_corners(corners, center):
    sc = sorted(corners, key=lambda p: math.atan2(p[1]-center[1], p[0]-center[0]))
    if len(sc) == 4: return [sc[2], sc[3], sc[0], sc[1]]
    return sc

def get_adaptive_canny(lost_count):
    if lost_count == 0:
        return (36, 235, 0.98)
    elif lost_count <= 3:
        return (28, 240, 0.99)
    elif lost_count <= 8:
        return (22, 245, 0.995)
    else:
        return (18, 250, 0.999)

# ================= 串口协议 =================
uart2 = None
def sending_data(flag, sx, dx, sy, dy, idx, sbx, dbx, sby, dby):
    global uart2
    if uart2 is None: return
    packet = [0xAA, 0x0D, flag, sx, dx, sy, dy, idx, sbx, dbx, sby, dby]
    checksum = sum(packet) & 0xFF
    try: uart2.write(bytes(packet) + bytes([checksum]))
    except: pass

# ================= RGB 控制 =================
def set_rgb(r_on, g_on, b_on):
    try:
        pin_r.value(LED_ON if r_on else LED_OFF)
        pin_g.value(LED_ON if g_on else LED_OFF)
        pin_b.value(LED_ON if b_on else LED_OFF)
    except:
        pass

def update_rgb_by_mode():
    if current_mode == MODE_CIRCLE:
        set_rgb(False, True, False)  # 绿色代表画圆
    else:
        set_rgb(False, False, True)  # 蓝色代表画矩形

# ================= 主程序 =================
sensor = None
try:
    fpioa = FPIOA()
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    uart2 = UART(UART.UART2, 115200)

    fpioa.set_function(KEY_PIN_NUM, FPIOA.GPIO53)
    key_pin = Pin(KEY_PIN_NUM, Pin.IN, Pin.PULL_DOWN)

    fpioa.set_function(RGB_R_PIN, FPIOA.GPIO62)
    fpioa.set_function(RGB_G_PIN, FPIOA.GPIO20)
    fpioa.set_function(RGB_B_PIN, FPIOA.GPIO63)
    pin_r = Pin(RGB_R_PIN, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
    pin_g = Pin(RGB_G_PIN, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
    pin_b = Pin(RGB_B_PIN, Pin.OUT, pull=Pin.PULL_NONE, drive=7)

    update_rgb_by_mode()

    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(Sensor.QVGA)
    sensor.set_pixformat(Sensor.RGB888)

    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    MediaManager.init()
    sensor.reset()
    time.sleep_ms(100)
    sensor.run()
    clock = time.clock()

    image_shape = [240, 320]
    last_key_state = 0

    while True:
        clock.tick()
        os.exitpoint()

        # --- 1. 按键切换模式 ---
        current_key_state = key_pin.value()

        if current_key_state == 1 and last_key_state == 0:
            key_press_time = time.ticks_ms()
            key_long_handled = False
        elif current_key_state == 1:
            if not key_long_handled:
                elapsed = time.ticks_diff(time.ticks_ms(), key_press_time)
                if elapsed >= 800:
                    current_mode = 1 - current_mode
                    current_base_point_index = 0
                    base_point_counter = 0
                    update_rgb_by_mode()
                    print("Mode Switched:", "Rectangle" if current_mode == MODE_RECT else "Circle")
                    key_long_handled = True
        elif current_key_state == 0 and last_key_state == 1:
            if not key_long_handled:
                display_mode = (display_mode + 1) % 3
                names = ["Normal", "Edge", "Binary"]
                print("Display Mode:", names[display_mode])

        last_key_state = current_key_state


        # --- 2. 视觉检测 ---
        img = sensor.snapshot()
        img_np = img.to_numpy_ref()

        h, w = image_shape
        total, cnt = 0, 0
        step = 8
        for y in range(0, h, step):
            row = img_np[y]
            for x in range(0, w, step):
                p = row[x]
                total += p[0] + p[1] + p[2]
                cnt += 3
        avg_brightness = total / cnt if cnt > 0 else 128.0
        if avg_brightness > 1.0:
            gain = TARGET_BRIGHTNESS / avg_brightness
            if gain < GAIN_MIN:
                gain = GAIN_MIN
            elif gain > GAIN_MAX:
                gain = GAIN_MAX
        else:
            gain = 1.0
        if abs(gain - 1.0) > 0.05:
            img_np = cv_lite.rgb888_adjust_exposure_fast(image_shape, img_np, gain)

        c1, c2, mc = get_adaptive_canny(lost_counter)

        rects_list = cv_lite.rgb888_find_rectangles_with_corners(
            image_shape, img_np, c1, c2,
            APPROX_EPS, 0.005, mc, 3
        )

        current_corners = None
        max_area = 0

        if rects_list:
            for rect_data in rects_list:
                if len(rect_data) < 12: continue

                rx, ry, rw, rh = rect_data[0:4]
                area = rw * rh
                aspect_ratio = rw / rh if rh > 0 else 0

                if area > MIN_AREA and (RATIO_LOW < aspect_ratio < RATIO_HIGH):
                    if area > max_area:
                        max_area = area
                        current_corners = [
                            (rect_data[4], rect_data[5]), (rect_data[6], rect_data[7]),
                            (rect_data[8], rect_data[9]), (rect_data[10], rect_data[11])
                        ]

        # --- 2.5 空间一致性校验 ---
        if current_corners and prev_center is not None:
            test_center = get_line_intersection(
                [current_corners[0], current_corners[2]],
                [current_corners[1], current_corners[3]]
            )
            jump_dist = math.sqrt(
                (test_center[0] - prev_center[0])**2 +
                (test_center[1] - prev_center[1])**2
            )
            if jump_dist > MAX_CENTER_JUMP:
                current_corners = None

        # --- 3. 防闪烁逻辑 ---
        if current_corners:
            prev_center = get_line_intersection(
                [current_corners[0], current_corners[2]],
                [current_corners[1], current_corners[3]]
            )
            prev_min_corners = current_corners
            lost_counter = 0
            is_locked = True
        else:
            lost_counter += 1
            if prev_min_corners and lost_counter < MAX_LOST_FRAMES:
                current_corners = prev_min_corners
                is_locked = True
            else:
                prev_min_corners = None
                prev_center = None
                is_locked = False

        flag_byte = 0xBB if is_locked else 0xCC
        s_dx_c, v_dx_c, s_dy_c, v_dy_c = 0, 0, 0, 0
        s_dx_b, v_dx_b, s_dy_b, v_dy_b = 0, 0, 0, 0

        # --- 4. 业务解算与滤波 ---
        if is_locked and current_corners:
            for i in range(4):
                img.draw_line(int(current_corners[i][0]), int(current_corners[i][1]),
                              int(current_corners[(i+1)%4][0]), int(current_corners[(i+1)%4][1]), color=(0,255,0), thickness=2)

            center = get_line_intersection([current_corners[0], current_corners[2]], [current_corners[1], current_corners[3]])
            cx, cy = int(center[0]), int(center[1])

            img.draw_cross(cx, cy, size=3, color=(255, 0, 255), thickness=2)

            sorted_c = sort_corners(current_corners, (cx, cy))

            if current_mode == MODE_CIRCLE:
                target_points = calculate_perspective_circle((cx, cy), sorted_c, CIRCLE_RADIUS_CM)
            else:
                target_points = calculate_perspective_rectangle((cx, cy), sorted_c, INSET_RATIO)

            if target_points:
                for idx, p in enumerate(target_points):
                    color = (255, 0, 0) if idx == current_base_point_index else (255, 255, 0)
                    img.draw_circle(p[0], p[1], 3, color=color, thickness=1)

                cp = target_points[current_base_point_index]

                # 切换目标点时，为防止滤波导致突变拖影，瞬间重置滤波值
                if abs(TARGET_POINT[0]-cp[0]) < SWITCH_LIMIT and abs(TARGET_POINT[1]-cp[1]) < SWITCH_LIMIT:
                    base_point_counter += 1
                    if base_point_counter >= 2:
                        current_base_point_index = (current_base_point_index + 1) % 16
                        base_point_counter = 0
                        # 切换点位时清除历史缓存，防止向新点走时产生拖拽感
                        last_dx_b = float(TARGET_POINT[0] - target_points[current_base_point_index][0])
                        last_dy_b = float(TARGET_POINT[1] - target_points[current_base_point_index][1])

                # 原始误差计算
                raw_dx_c = TARGET_POINT[0] - cx
                raw_dy_c = TARGET_POINT[1] - cy
                raw_dx_b = TARGET_POINT[0] - cp[0]
                raw_dy_b = TARGET_POINT[1] - cp[1]

                # ========= 【核心新增：EMA 低通滤波】 =========
                last_dx_c = (EMA_ALPHA * raw_dx_c) + ((1 - EMA_ALPHA) * last_dx_c)
                last_dy_c = (EMA_ALPHA * raw_dy_c) + ((1 - EMA_ALPHA) * last_dy_c)
                last_dx_b = (EMA_ALPHA * raw_dx_b) + ((1 - EMA_ALPHA) * last_dx_b)
                last_dy_b = (EMA_ALPHA * raw_dy_b) + ((1 - EMA_ALPHA) * last_dy_b)

                # 将平滑后的浮点数转为整型发给单片机
                filtered_dx_c = int(last_dx_c)
                filtered_dy_c = int(last_dy_c)
                filtered_dx_b = int(last_dx_b)
                filtered_dy_b = int(last_dy_b)
                # ===============================================

                # 处理中心点符号和数值
                s_dx_c, v_dx_c = (1, abs(filtered_dx_c)) if filtered_dx_c < 0 else (0, filtered_dx_c)
                s_dy_c, v_dy_c = (1, abs(filtered_dy_c)) if filtered_dy_c < 0 else (0, filtered_dy_c)

                # 处理基准点符号和数值
                s_dx_b, v_dx_b = (1, abs(filtered_dx_b)) if filtered_dx_b < 0 else (0, filtered_dx_b)
                s_dy_b, v_dy_b = (1, abs(filtered_dy_b)) if filtered_dy_b < 0 else (0, filtered_dy_b)

        sending_data(flag_byte, s_dx_c, int(v_dx_c), s_dy_c, int(v_dy_c), current_base_point_index, s_dx_b, int(v_dx_b), s_dy_b, int(v_dy_b))

        if display_mode == DISPLAY_MODE_EDGE:
            edge_np = cv_lite.rgb888_find_edges(image_shape, img_np, 50, 150)
            disp = image.Image(w, h, image.GRAYSCALE, alloc=image.ALLOC_REF, data=edge_np)
            disp.draw_string_advanced(5, 5, 15, "FPS: {:.1f}".format(clock.fps()), color=(255,255,255))
            disp.draw_string_advanced(5, 20, 15, "EDGE MODE", color=(0,255,0))
            disp.compressed_for_ide()
            Display.show_image(disp, x=(800-320)//2, y=(480-240)//2)
        elif display_mode == DISPLAY_MODE_BINARY:
            binary_np = cv_lite.rgb888_threshold_binary(image_shape, img_np, BINARY_THRESH, 255)
            disp = image.Image(w, h, image.GRAYSCALE, alloc=image.ALLOC_REF, data=binary_np)
            disp.draw_string_advanced(5, 5, 15, "FPS: {:.1f}".format(clock.fps()), color=(255,255,255))
            disp.draw_string_advanced(5, 20, 15, "BINARY MODE", color=(255,255,0))
            disp.compressed_for_ide()
            Display.show_image(disp, x=(800-320)//2, y=(480-240)//2)
        else:
            img.draw_circle(TARGET_POINT[0], TARGET_POINT[1], 1, color=(255,0,0), thickness=1)
            img.draw_string_advanced(5, 5, 15, "FPS: {:.1f}".format(clock.fps()), color=(255,255,255))
            mode_text = "Mode: Circle (G)" if current_mode == MODE_CIRCLE else "Mode: Rect (B)"
            mode_color = (0, 255, 0) if current_mode == MODE_CIRCLE else (0, 0, 255)
            img.draw_string_advanced(5, 20, 15, mode_text, color=mode_color)
            img.compressed_for_ide()
            Display.show_image(img, x=(800-320)//2, y=(480-240)//2)

        del img_np
        gc.collect()

finally:
    if sensor: sensor.stop()
    set_rgb(False, False, False)
    Display.deinit()
    MediaManager.deinit()
