# K230 寻找最大矩形框示例
# 功能：检测图像中的矩形，找到最大的矩形框，并打印四个角点和中心点坐标

import time, os, gc, sys, image
from media.sensor import *
from media.display import *
from media.media import *

# ==================== 修改点 1 ====================
# 注释掉报错的自定义串口库，因为IDE测试视觉不需要发数据给单片机
# from ybUtils.YbUart import YbUart
# ==================================================

import cv_lite  # 既然固件支持了，原封不动保留原作者的C++加速库！

# 图像检测尺寸配置
DETECT_WIDTH = 320  # 宽度对齐到16的倍数
DETECT_HEIGHT = 240               # 高度
IMAGE_CENTER_X = DETECT_WIDTH // 2  # 图像中线x坐标

# ==================== 修改点 2 ====================
# 显示模式选择 Display mode selection
# 强制设为 "VIRT" (虚拟屏幕)。如果在没插物理屏幕的情况下强行初始化 "LCD" 模式，K230底层会报错死机。
# 如果你之后插了屏幕，把它改回 "LCD" 即可。
DISPLAY_MODE = "VIRT"
# ==================================================

# 根据显示模式设置分辨率 Set resolution based on display mode
if DISPLAY_MODE == "VIRT":
    DISPLAY_WIDTH = DETECT_WIDTH
    DISPLAY_HEIGHT = DETECT_HEIGHT
elif DISPLAY_MODE == "LCD":
    DISPLAY_WIDTH = 640
    DISPLAY_HEIGHT = 480
else:
    raise ValueError("Unknown DISPLAY_MODE, please select 'VIRT', 'LCD'")

sensor = None
uart1 = None  # UART1实例

import math

# 卡尔曼滤波器类
class KalmanFilter:
    """二维卡尔曼滤波器，用于单个坐标点的跟踪"""
    def __init__(self, process_noise=1e-2, measurement_noise=1e-1):
        self.state = [0.0, 0.0, 0.0, 0.0]
        self.P = [[1000.0, 0.0, 0.0, 0.0],
                  [0.0, 1000.0, 0.0, 0.0],
                  [0.0, 0.0, 1000.0, 0.0],
                  [0.0, 0.0, 0.0, 1000.0]]
        q = process_noise
        self.Q = [[q, 0.0, 0.0, 0.0],
                  [0.0, q, 0.0, 0.0],
                  [0.0, 0.0, q, 0.0],
                  [0.0, 0.0, 0.0, q]]
        r = measurement_noise
        self.R = [[r, 0.0],
                  [0.0, r]]
        self.H = [[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0]]
        self.initialized = False

    def predict(self, dt=1.0):
        if not self.initialized: return
        F = [[1.0, 0.0, dt, 0.0],
             [0.0, 1.0, 0.0, dt],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]]
        new_state = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            for j in range(4):
                new_state[i] += F[i][j] * self.state[j]
        self.state = new_state
        FP = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    FP[i][j] += F[i][k] * self.P[k][j]
        FPFT = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    FPFT[i][j] += FP[i][k] * F[j][k]
        for i in range(4):
            for j in range(4):
                self.P[i][j] = FPFT[i][j] + self.Q[i][j]

    def update(self, measurement):
        x, y = measurement
        if not self.initialized:
            self.state = [x, y, 0.0, 0.0]
            self.initialized = True
            return
        predicted_measurement = [self.state[0], self.state[1]]
        residual = [x - predicted_measurement[0], y - predicted_measurement[1]]
        HP = [[0.0] * 4 for _ in range(2)]
        for i in range(2):
            for j in range(4):
                for k in range(4):
                    HP[i][j] += self.H[i][k] * self.P[k][j]
        HPHT = [[0.0] * 2 for _ in range(2)]
        for i in range(2):
            for j in range(2):
                for k in range(4):
                    HPHT[i][j] += HP[i][k] * self.H[j][k]
        S = [[HPHT[i][j] + self.R[i][j] for j in range(2)] for i in range(2)]
        det_S = S[0][0] * S[1][1] - S[0][1] * S[1][0]
        if abs(det_S) < 1e-10: return
        S_inv = [[S[1][1] / det_S, -S[0][1] / det_S],
                 [-S[1][0] / det_S, S[0][0] / det_S]]
        PHT = [[0.0] * 2 for _ in range(4)]
        for i in range(4):
            for j in range(2):
                for k in range(2):
                    PHT[i][j] += self.P[i][k] * self.H[j][k]
        K = [[0.0] * 2 for _ in range(4)]
        for i in range(4):
            for j in range(2):
                for k in range(2):
                    K[i][j] += PHT[i][k] * S_inv[k][j]
        for i in range(4):
            for j in range(2):
                self.state[i] += K[i][j] * residual[j]
        KH = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(2):
                    KH[i][j] += K[i][k] * self.H[k][j]
        I_KH = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                I_KH[i][j] = (1.0 if i == j else 0.0) - KH[i][j]
        new_P = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    new_P[i][j] += I_KH[i][k] * self.P[k][j]
        self.P = new_P

    def get_position(self):
        if not self.initialized: return None
        return (int(self.state[0]), int(self.state[1]))

    def reset(self):
        self.initialized = False
        self.state = [0.0, 0.0, 0.0, 0.0]
        self.P = [[1000.0, 0.0, 0.0, 0.0],
                  [0.0, 1000.0, 0.0, 0.0],
                  [0.0, 0.0, 1000.0, 0.0],
                  [0.0, 0.0, 0.0, 1000.0]]

class CoordinateFilter:
    """坐标滤波器，使用卡尔曼滤波平滑矩形坐标变化"""
    def __init__(self, process_noise=1e-2, measurement_noise=1e-1):
        self.corner_filters = [KalmanFilter(process_noise, measurement_noise) for _ in range(4)]
        self.center_filter = KalmanFilter(process_noise, measurement_noise)
        self.dt = 1.0
        self.data_count = 0

    def add_corners(self, corners):
        self.data_count += 1
        for i, corner in enumerate(corners):
            self.corner_filters[i].predict(self.dt)
            self.corner_filters[i].update(corner)

    def add_center(self, center):
        self.center_filter.predict(self.dt)
        self.center_filter.update(center)

    def get_filtered_corners(self):
        if self.data_count < 2: return None
        filtered_corners = []
        for corner_filter in self.corner_filters:
            pos = corner_filter.get_position()
            if pos is None: return None
            filtered_corners.append(pos)
        return filtered_corners

    def get_filtered_center(self):
        if self.data_count < 2: return None
        return self.center_filter.get_position()

    def reset(self):
        for corner_filter in self.corner_filters:
            corner_filter.reset()
        self.center_filter.reset()
        self.data_count = 0

coord_filter = CoordinateFilter(process_noise=1e-2, measurement_noise=1e-1)


# ==================== 修改点 3 ====================
def uart_init():
    """初始化UART串口"""
    global uart1
    # 剔除报错点，保持函数为空，不影响后续代码结构
    pass
# ==================================================

def camera_init():
    """初始化摄像头"""
    global sensor
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    if DISPLAY_MODE == "VIRT":
        Display.init(Display.VIRT, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, fps=100, to_ide=True)
    elif DISPLAY_MODE == "LCD":
        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)

    MediaManager.init()
    sensor.run()

# ==================== 修改点 4 ====================
def uart_deinit():
    """释放UART串口资源"""
    global uart1
    # 剔除报错点
    pass
# ==================================================

def camera_deinit():
    """释放摄像头资源"""
    global sensor
    sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()

def set_display_mode(mode):
    global DISPLAY_MODE, DISPLAY_WIDTH, DISPLAY_HEIGHT
    if mode in ["VIRT", "LCD"]:
        DISPLAY_MODE = mode
        if DISPLAY_MODE == "VIRT":
            DISPLAY_WIDTH = DETECT_WIDTH
            DISPLAY_HEIGHT = DETECT_HEIGHT
        elif DISPLAY_MODE == "LCD":
            DISPLAY_WIDTH = 640
            DISPLAY_HEIGHT = 480

def calculate_center(corners):
    if len(corners) != 4:
        return None
    x1, y1 = corners[0]
    x2, y2 = corners[1]
    x3, y3 = corners[2]
    x4, y4 = corners[3]
    try:
        dx1 = x3 - x1
        dy1 = y3 - y1
        dx2 = x4 - x2
        dy2 = y4 - y2
        det = dx1 * (-dy2) - dy1 * (-dx2)
        if abs(det) < 1e-10:
            cx = sum(corner[0] for corner in corners) / 4
            cy = sum(corner[1] for corner in corners) / 4
        else:
            t1 = ((x2 - x1) * (-dy2) - (y2 - y1) * (-dx2)) / det
            cx = x1 + t1 * dx1
            cy = y1 + t1 * dy1
    except (ZeroDivisionError, ValueError):
        cx = sum(corner[0] for corner in corners) / 4
        cy = sum(corner[1] for corner in corners) / 4
    return (int(cx), int(cy))

def capture_picture():
    """捕获图像并进行矩形检测"""
    fps = time.clock()

    max_rect = None
    x_error = 0.0
    pid_output = 0.0
    pid_abs = 0.0
    pid_scaled = 0.0
    filtered_corners = None
    filtered_center = None

    while True:
        fps.tick()
        try:
            os.exitpoint()
            global sensor
            img = sensor.snapshot()

            img_gray = img.to_grayscale()

            hist = img_gray.get_histogram()
            otsu_threshold_obj = hist.get_threshold()
            threshold_value = (otsu_threshold_obj.value(), 255)
            img_binary = img_gray.binary([threshold_value])

            image_shape = [DETECT_HEIGHT, DETECT_WIDTH]
            img_gray_np = img_gray.to_numpy_ref()

            # 保持原版的cv_lite高帧率算法
            canny_thresh1 = 50
            canny_thresh2 = 150
            approx_epsilon = 0.04
            area_min_ratio = 0.01
            max_angle_cos = 0.3
            gaussian_blur_size = 5

            rects_data = cv_lite.grayscale_find_rectangles(
                image_shape, img_gray_np,
                canny_thresh1, canny_thresh2,
                approx_epsilon,
                area_min_ratio,
                max_angle_cos,
                gaussian_blur_size
            )

            if rects_data and len(rects_data) >= 4:
                filtered_rects = []
                for i in range(0, len(rects_data), 4):
                    if i + 3 < len(rects_data):
                        x, y, w, h = rects_data[i], rects_data[i+1], rects_data[i+2], rects_data[i+3]
                        area = w * h
                        aspect_ratio = w / h if h > 0 else 0
                        img.draw_string_advanced(5, 45, 16, f"Area: {aspect_ratio:.2f}", color=(0, 255, 255, 0))

                        if area > 1500 and 0.6 < aspect_ratio < 1.6:
                            rect_info = {
                                'x': x, 'y': y, 'w': w, 'h': h,
                                'area': area, 'aspect_ratio': aspect_ratio
                            }
                            filtered_rects.append(rect_info)

                if filtered_rects:
                    max_rect = max(filtered_rects, key=lambda r: r['area'])
                else:
                    max_rect = None

                if max_rect:
                    x, y, w, h = max_rect['x'], max_rect['y'], max_rect['w'], max_rect['h']
                    corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]

                    center = calculate_center(corners)
                    x_error = center[0] - IMAGE_CENTER_X

                    if x_error > 100:
                        x_error = 100
                    elif x_error < -100:
                        x_error = -100

                    global coord_filter
                    coord_filter.add_corners(corners)
                    coord_filter.add_center(center)

                    # 串口发送代码块：因为前面把uart1初始化删除了，这里uart1为None，不会报错也不会执行
                    if uart1:
                        try:
                            error_int = int(x_error)
                            if error_int < 0:
                                error_bytes = (error_int + 65536).to_bytes(2, 'little')
                            else:
                                error_bytes = error_int.to_bytes(2, 'little')
                            frame_data = bytes([0x66, 0x66]) + error_bytes + bytes([0xf6, 0xf6])
                            uart1.write(frame_data)
                        except Exception as e:
                            pass

                    filtered_corners = coord_filter.get_filtered_corners()
                    filtered_center = coord_filter.get_filtered_center()
                else:
                    filtered_corners = coord_filter.get_filtered_corners()
                    filtered_center = coord_filter.get_filtered_center()

                if filtered_corners and filtered_center:
                    min_x = min(corner[0] for corner in filtered_corners)
                    max_x = max(corner[0] for corner in filtered_corners)
                    min_y = min(corner[1] for corner in filtered_corners)
                    max_y = max(corner[1] for corner in filtered_corners)

                    img.draw_rectangle([min_x, min_y, max_x - min_x, max_y - min_y], color=(0, 255, 0), thickness=2)

                    for i, corner in enumerate(filtered_corners):
                        img.draw_circle(corner[0], corner[1], 5, color=(255, 0, 0), thickness=2)

                    img.draw_circle(filtered_center[0], filtered_center[1], 8, color=(0, 0, 255), thickness=2)
                else:
                    if max_rect:
                        x, y, w, h = max_rect['x'], max_rect['y'], max_rect['w'], max_rect['h']
                        img.draw_rectangle([x, y, w, h], color=(0, 255, 0), thickness=2)
                        for i, corner in enumerate(corners):
                            img.draw_circle(corner[0], corner[1], 5, color=(255, 0, 0), thickness=2)
                        if center:
                            img.draw_circle(center[0], center[1], 8, color=(0, 0, 255), thickness=2)
            else:
                coord_filter.reset()

            img.draw_circle(IMAGE_CENTER_X, DETECT_HEIGHT // 2, 3, color=(255, 255, 0), thickness=2, fill=False)

            fps_text = f"FPS: {fps.fps():.1f}"
            mode_text = f"Display: {DISPLAY_MODE}"
            img.draw_string_advanced(5, 5, 16, fps_text, color=(255, 255, 255, 0))
            img.draw_string_advanced(5, 25, 16, mode_text, color=(255, 255, 255, 0))

            if 'x_error' in locals():
                error_text = f"Error: {x_error:.1f}"
                img.draw_string_advanced(DETECT_WIDTH - 150, 5, 16, error_text, color=(0, 255, 255, 0))

            if DISPLAY_MODE == "LCD":
                x = int((DISPLAY_WIDTH - DETECT_WIDTH) / 2)
                y = int((DISPLAY_HEIGHT - DETECT_HEIGHT) / 2)
                Display.show_image(img, x=x, y=y)
            else:
                Display.show_image(img)
            img = None

            gc.collect()

        except KeyboardInterrupt as e:
            break
        except BaseException as e:
            break

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_is_init = False
    uart_is_init = False

    try:
        uart_init()
        uart_is_init = True

        camera_init()
        camera_is_init = True

        capture_picture()

    except Exception as e:
        pass
    finally:
        if camera_is_init:
            camera_deinit()
        if uart_is_init:
            uart_deinit()

if __name__ == "__main__":
    main()
