import time, os, gc, sys, math
from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA, Pin, UART
import cv_lite
import image

# ======================== 1. 参数调优区 ========================

# --- 边缘检测 (影响找框的灵敏度) ---
CANNY_1 = 63
CANNY_2 = 131
APPROX_EPS = 0.02
MAX_COS = 0.98

# 保留你原来的面积阈值设置
MIN_AREA = 800
MAX_AREA = 13000

RATIO_LOW = 0.4
RATIO_HIGH = 1.8

RECT_WIDTH_CM = 27.6
RECT_HEIGHT_CM = 19.1
CIRCLE_RADIUS_CM = 6.0
TARGET_POINT = (160, 120)

MAX_LOST_FRAMES = 15
MAX_CENTER_JUMP = 150
SWITCH_LIMIT = 10

INSET_RATIO = 0.90  # 【画矩形专用】矩形内缩比例

# --- 滤波参数 (新增) ---
EMA_ALPHA = 0.55

# --- 曝光自适应 ---
TARGET_BRIGHTNESS = 128
GAIN_MIN = 0.5
GAIN_MAX = 2.0
BINARY_THRESH = 128

# --- 激光标定 ---
LASER_THRESHOLD = [0, 60, 0, 80, 150, 255]
LASER_CALIBRATED = True
LASER_X = 165
LASER_E = -872.0
LASER_F = 125.83
FOCAL_PIXELS = 140#166.8
calib_distances = [50, 100, 150]
calib_step = -1
calib_records = []
calib_record_flag = False
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
current_mode = MODE_RECT  # 默认上机画矩形

DISPLAY_MODE_NORMAL = 0
DISPLAY_MODE_BINARY = 1
display_mode = DISPLAY_MODE_NORMAL

current_base_point_index = 0
base_point_counter = 0

prev_min_corners = None
prev_rect_area = 0
lost_counter = 0

# EMA 历史值缓存
last_dx_c, last_dy_c = 0.0, 0.0
last_dx_b, last_dy_b = 0.0, 0.0
prev_center = None
key_press_time = 0
key_long_handled = False
last_click_time = 0
DOUBLE_CLICK_WINDOW = 500

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

def rect_pixel_width(sorted_corners):
    w1 = math.sqrt((sorted_corners[1][0]-sorted_corners[0][0])**2 + (sorted_corners[1][1]-sorted_corners[0][1])**2)
    w2 = math.sqrt((sorted_corners[2][0]-sorted_corners[3][0])**2 + (sorted_corners[2][1]-sorted_corners[3][1])**2)
    return (w1 + w2) / 2.0

def get_adaptive_canny(lost_count):
    if lost_count == 0:
        return (36, 235, 0.98)
    elif lost_count <= 3:
        return (28, 240, 0.99)
    elif lost_count <= 8:
        return (22, 245, 0.995)
    else:
        return (18, 250, 0.999)

def find_laser(img_np):
    blobs = cv_lite.rgb888_find_blobs(image_shape, img_np, LASER_THRESHOLD, 2, 1)
    if blobs and len(blobs) >= 4:
        best_area = 0
        best = None
        for i in range(0, len(blobs), 4):
            x, y, bw, bh = blobs[i], blobs[i+1], blobs[i+2], blobs[i+3]
            a = bw * bh
            if 1 < a < 100 and a > best_area:
                best_area = a
                best = (x + bw // 2, y + bh // 2)
        return best
    return None

def estimate_distance(pixel_width):
    if FOCAL_PIXELS <= 0 or pixel_width <= 0:
        return 0.0
    return RECT_WIDTH_CM * FOCAL_PIXELS / pixel_width

# (去除了原代码中不稳定的 find_white_paper 和多余重复定义的 rect_pixel_width 函数)

def fit_laser_params(records):
    n = len(records)
    if n < 2:
        return (0.0, 0.0, 0.0, 0.0)
    x = [1.0 / r[0] for r in records]
    ox = [r[1] for r in records]
    oy = [r[2] for r in records]
    def lsq(xs, ys):
        n2 = len(xs)
        sx = sum(xs)
        sy = sum(ys)
        sxx = sum(xi*xi for xi in xs)
        sxy = sum(xs[i]*ys[i] for i in range(n2))
        denom = n2 * sxx - sx * sx
        if abs(denom) < 1e-9:
            return 0.0, sy / n2
        a = (n2 * sxy - sx * sy) / denom
        b = (sy - a * sx) / n2
        return a, b
    A, B = lsq(x, ox)
    C, D = lsq(x, oy)
    return A, B, C, D

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
                display_mode = (display_mode + 1) % 2
                names = ["Normal", "Binary"]
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
            APPROX_EPS, 0.005, mc, 5
        )

        current_corners = None
        current_area = 0
        best_score = -1.0

        # ========= 同心框过滤：有更小邻居的矩形 → 外框 → 惩罚 =========
        if rects_list:
            valid_rects = []
            for rect_data in rects_list:
                if len(rect_data) < 12: continue
                rx, ry, rw, rh = rect_data[0:4]
                area = rw * rh
                aspect_ratio = rw / rh if rh > 0 else 0
                if MIN_AREA < area < MAX_AREA and (RATIO_LOW < aspect_ratio < RATIO_HIGH):
                    cx_r = (rect_data[4] + rect_data[6] + rect_data[8] + rect_data[10]) / 4.0
                    cy_r = (rect_data[5] + rect_data[7] + rect_data[9] + rect_data[11]) / 4.0
                    corners = [
                        (rect_data[4], rect_data[5]), (rect_data[6], rect_data[7]),
                        (rect_data[8], rect_data[9]), (rect_data[10], rect_data[11])
                    ]
                    valid_rects.append({'area': area, 'cx': cx_r, 'cy': cy_r, 'corners': corners})

            for i in range(len(valid_rects)):
                r1 = valid_rects[i]
                score = MAX_AREA / r1['area']
                if prev_min_corners is not None and prev_rect_area > 0:
                    r = min(r1['area'], prev_rect_area) / max(r1['area'], prev_rect_area)
                    score *= r
                for j in range(len(valid_rects)):
                    if i == j: continue
                    r2 = valid_rects[j]
                    dist = math.sqrt((r1['cx'] - r2['cx'])**2 + (r1['cy'] - r2['cy'])**2)
                    if dist < 20 and r2['area'] < r1['area']:
                        score *= 0.01
                        break
                if score > best_score:
                    best_score = score
                    current_area = r1['area']
                    current_corners = r1['corners']

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
            prev_rect_area = current_area
            lost_counter = 0
            is_locked = True
        else:
            lost_counter += 1
            if prev_min_corners and lost_counter < MAX_LOST_FRAMES:
                current_corners = prev_min_corners
                is_locked = True
            else:
                prev_min_corners = None
                prev_rect_area = 0
                prev_center = None
                is_locked = False

        flag_byte = 0xBB if is_locked else 0xCC
        tx, ty = TARGET_POINT
        s_dx_c, v_dx_c, s_dy_c, v_dy_c = 0, 0, 0, 0
        s_dx_b, v_dx_b, s_dy_b, v_dy_b = 0, 0, 0, 0

        # --- 4. 业务解算与滤波 ---
        if is_locked and current_corners:
            for i in range(4):
                img.draw_line(int(current_corners[i][0]), int(current_corners[i][1]),
                              int(current_corners[(i+1)%4][0]), int(current_corners[(i+1)%4][1]), color=(0,255,0), thickness=2)

            # 用原角点估算中心方向用于向内收敛
            raw_center = get_line_intersection([current_corners[0], current_corners[2]], [current_corners[1], current_corners[3]])

            refined_corners = []
            for (kx, ky) in current_corners:
                ix, iy = int(kx), int(ky)
                if 0 <= ix < w and 0 <= iy < h:
                    p = img_np[iy, ix]
                    if p[0] > 80 and p[1] > 80 and p[2] > 80:
                        refined_corners.append((ix, iy))
                    else:
                        dx = raw_center[0] - ix
                        dy = raw_center[1] - iy
                        dist = math.sqrt(dx*dx + dy*dy)
                        if dist > 0:
                            step_x, step_y = dx / dist, dy / dist
                            found = False
                            for s in range(1, int(dist)):
                                sx = int(ix + step_x * s)
                                sy = int(iy + step_y * s)
                                if 0 <= sx < w and 0 <= sy < h:
                                    p2 = img_np[sy, sx]
                                    if p2[0] > 80 and p2[1] > 80 and p2[2] > 80:
                                        bx = int(ix + step_x * max(1, s - 1))
                                        by = int(iy + step_y * max(1, s - 1))
                                        refined_corners.append((bx, by))
                                        found = True
                                        break
                            if not found:
                                refined_corners.append((ix, iy))
                else:
                    refined_corners.append((ix, iy))
            current_corners = refined_corners
            center = get_line_intersection([current_corners[0], current_corners[2]], [current_corners[1], current_corners[3]])
            cx, cy = int(center[0]), int(center[1])

            img.draw_cross(cx, cy, size=3, color=(255, 0, 255), thickness=2)

            sorted_c = sort_corners(current_corners, (cx, cy))

            pw = rect_pixel_width(sorted_c)

            tx = TARGET_POINT[0]
            ty = TARGET_POINT[1]
            if LASER_CALIBRATED and pw > 0:
                dist = estimate_distance(pw)
                if dist > 0:
                    tx = LASER_X
                    ty = int(LASER_E / dist + LASER_F)

            if current_mode == MODE_CIRCLE:
                target_points = calculate_perspective_circle((cx, cy), sorted_c, CIRCLE_RADIUS_CM)
            else:
                target_points = calculate_perspective_rectangle((cx, cy), sorted_c, INSET_RATIO)

            if target_points:
                for idx, p in enumerate(target_points):
                    color = (255, 0, 0) if idx == current_base_point_index else (255, 255, 0)
                    img.draw_circle(p[0], p[1], 3, color=color, thickness=1)

                cp = target_points[current_base_point_index]

                if abs(tx-cp[0]) < SWITCH_LIMIT and abs(ty-cp[1]) < SWITCH_LIMIT:
                    base_point_counter += 1
                    if base_point_counter >= 2:
                        current_base_point_index = (current_base_point_index + 1) % 16
                        base_point_counter = 0
                        last_dx_b = float(tx - target_points[current_base_point_index][0])
                        last_dy_b = float(ty - target_points[current_base_point_index][1])

                raw_dx_c = tx - cx
                raw_dy_c = ty - cy
                raw_dx_b = tx - cp[0]
                raw_dy_b = ty - cp[1]

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

        if display_mode == DISPLAY_MODE_BINARY:
            binary_np = cv_lite.rgb888_threshold_binary(image_shape, img_np, BINARY_THRESH, 255)
            disp = image.Image(w, h, image.GRAYSCALE, alloc=image.ALLOC_REF, data=binary_np)
            disp.draw_string_advanced(5, 5, 15, "FPS: {:.1f}".format(clock.fps()), color=(255,255,255))
            disp.draw_string_advanced(5, 20, 15, "BINARY MODE", color=(255,255,0))
            disp.compressed_for_ide()
            Display.show_image(disp, x=(800-320)//2, y=(480-240)//2)
        else:
            img.draw_circle(tx, ty, 1, color=(255,0,0), thickness=1)
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
