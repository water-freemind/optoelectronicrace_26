import time, os, gc, sys, math
from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA, Pin, UART
import cv_lite

# ======================== 1. 参数调优区 ========================

# --- 边缘检测 (影响找框的灵敏度) ---
CANNY_1 = 25
CANNY_2 = 227

# --- 几何特征 (影响矩形的锁定) ---
APPROX_EPS = 0.02
MAX_COS = 0.92
MIN_AREA = 2000

# --- 目标形状比例 (A4纸专项过滤) ---
RATIO_LOW = 0.4
RATIO_HIGH = 1.3

# --- 物理尺寸与坐标 ---
RECT_WIDTH_CM = 27.6
RECT_HEIGHT_CM = 19.1
CIRCLE_RADIUS_CM = 6.0
TARGET_POINT = (159, 94)

# --- 状态过滤 (防闪烁) ---
MAX_LOST_FRAMES = 5 # 允许最大丢失帧数
SWITCH_LIMIT = 15
# ==============================================================

current_base_point_index = 0
base_point_counter = 0

# 防闪烁缓存变量
prev_min_corners = None
lost_counter = 0

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

def sort_corners(corners, center):
    sc = sorted(corners, key=lambda p: math.atan2(p[1]-center[1], p[0]-center[0]))
    if len(sc) == 4: return [sc[2], sc[3], sc[0], sc[1]]
    return sc

uart2 = None
def sending_data(flag, sx, dx, sy, dy, idx, sbx, dbx, sby, dby):
    global uart2
    if uart2 is None: return
    packet = [0xAA, 0x0D, flag, sx, dx, sy, dy, idx, sbx, dbx, sby, dby]
    checksum = sum(packet) & 0xFF
    try: uart2.write(bytes(packet) + bytes([checksum]))
    except: pass

# ================= 主程序 =================
sensor = None
try:
    fpioa = FPIOA()
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    uart2 = UART(UART.UART2, 115200)

    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(Sensor.QVGA)
    sensor.set_pixformat(Sensor.RGB888)

    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    image_shape = [240, 320]

    while True:
        clock.tick()
        os.exitpoint()

        img = sensor.snapshot()
        img_np = img.to_numpy_ref()

        rects_list = cv_lite.rgb888_find_rectangles_with_corners(
            image_shape, img_np, CANNY_1, CANNY_2,
            APPROX_EPS, 0.005, MAX_COS, 3
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

        # --- 状态保持逻辑 (防闪烁) ---
        if current_corners:
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
                is_locked = False

        flag_byte = 0xBB if is_locked else 0xCC
        s_dx_c, v_dx_c, s_dy_c, v_dy_c = 0, 0, 0, 0
        s_dx_b, v_dx_b, s_dy_b, v_dy_b = 0, 0, 0, 0

        if is_locked and current_corners:
            for i in range(4):
                img.draw_line(int(current_corners[i][0]), int(current_corners[i][1]),
                              int(current_corners[(i+1)%4][0]), int(current_corners[(i+1)%4][1]), color=(0,255,0), thickness=2)

            center = get_line_intersection([current_corners[0], current_corners[2]], [current_corners[1], current_corners[3]])
            cx, cy = int(center[0]), int(center[1])

            img.draw_cross(cx, cy, size=3, color=(255, 0, 255), thickness=2)

            sorted_c = sort_corners(current_corners, (cx, cy))
            circle_points = calculate_perspective_circle((cx, cy), sorted_c, CIRCLE_RADIUS_CM)

            if circle_points:
                for idx, p in enumerate(circle_points):
                    color = (255, 0, 0) if idx == current_base_point_index else (255, 255, 0)
                    img.draw_circle(p[0], p[1], 3, color=color, thickness=1)

                cp = circle_points[current_base_point_index]
                if abs(TARGET_POINT[0]-cp[0]) < SWITCH_LIMIT and abs(TARGET_POINT[1]-cp[1]) < SWITCH_LIMIT:
                    base_point_counter += 1
                    if base_point_counter >= 2:
                        current_base_point_index = (current_base_point_index + 1) % 16
                        base_point_counter = 0

                dx_c, dy_c = TARGET_POINT[0] - cx, TARGET_POINT[1] - cy
                dx_b, dy_b = TARGET_POINT[0] - cp[0], TARGET_POINT[1] - cp[1]
                s_dx_c, v_dx_c = (1, abs(dx_c)) if dx_c < 0 else (0, dx_c)
                s_dy_c, v_dy_c = (1, abs(dy_c)) if dy_c < 0 else (0, dy_c)
                s_dx_b, v_dx_b = (1, abs(dx_b)) if dx_b < 0 else (0, dx_b)
                s_dy_b, v_dy_b = (1, abs(dy_b)) if dy_b < 0 else (0, dy_b)

        img.draw_circle(TARGET_POINT[0], TARGET_POINT[1], 1, color=(255,0,0), thickness=1)

        # 状态显示
        img.draw_string_advanced(5, 5, 18, "FPS: {:.1f}".format(clock.fps()), color=(255,255,255))

        sending_data(flag_byte, s_dx_c, int(v_dx_c), s_dy_c, int(v_dy_c), current_base_point_index, s_dx_b, int(v_dx_b), s_dy_b, int(v_dy_b))

        img.compressed_for_ide()
        Display.show_image(img, x=(800-320)//2, y=(480-240)//2)

        del img_np
        gc.collect()

finally:
    if sensor: sensor.stop()
    Display.deinit()
    MediaManager.deinit()
