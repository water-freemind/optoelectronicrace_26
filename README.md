# optoelectronicrace_26

基于 TI MSPM0G3507 的光电竞赛车控制工程，使用 Code Composer Studio（CCS）、TI ARM Clang 和 MSPM0 SDK。项目在循迹底盘基础上集成 K230 视觉通信、二维云台、激光器、目标瞄准和矩形绘制等功能。

本目录对应独立远程仓库：[`water-freemind/optoelectronicrace_26`](https://github.com/water-freemind/optoelectronicrace_26)。它与同级的 `26ticar` 是两个不同的 CCS 项目和两个不同的 Git 仓库，必须分别打开、构建和提交。

## 功能概览

### 底盘与循迹

固件初始化 OLED、旋钮菜单、电机、编码器、灰度传感器、JY62 和板载 UART。主循环持续刷新菜单、姿态和灰度传感器；打开循迹开关后，`APP/Trackline.c` 根据传感器加权误差执行 PID 差速控制。

底盘模块包括八路灰度/循迹采集、电机 PWM 与方向控制、A/B 编码器、里程计、OLED 显示、蜂鸣器和运行时参数调节。

### K230 视觉目标处理

`APP/k230_track.c` 通过 `UART0` 接收 K230 的 13 字节数据帧，帧头为 `0xAA 0x0D`，末字节为前 12 字节累加和。数据结构包含：目标锁定/丢失状态、目标中心相对偏差、目标索引以及圆点偏差。

K230 相关应用逻辑包括：

- 目标中心两阶段瞄准：粗调、稳定等待、精调和最大尝试次数保护
- 目标丢失后的分段扫描搜索
- 目标锁定后控制云台 X/Y 轴，并在满足条件后锁存激光
- 根据视觉矩形/圆点信息进行绘制流程

### 云台与激光

`APP/gimbal.c` 通过 UART 向两个张大头 57 电机发送位置控制指令，X/Y 地址分别为 `0x01` / `0x02`；位置模式使用 32 细分，标称 6400 pulse/rev。`APP/Laser.c` 控制激光器输出，激活电平由 `LASER_ACTIVE_LOW` 配置。

## 硬件与接口配置

| 功能 | 配置 |
| --- | --- |
| MCU | MSPM0G3507，Cortex-M0+，LQFP-48（PT） |
| K230 视觉 UART | UART0，115200 baud，RX PA31 / TX PA28 |
| 云台/上位机 UART | UART2，115200 baud，RX PA22 / TX PA21 |
| JY62 UART | UART1，RX PA9 / TX PA8 |
| OLED | GPIO 软件 I²C，具体引脚以 SysConfig 为准 |
| 激光控制 | `GPIO_Laser`，具体引脚以 SysConfig 为准 |
| 电机 | `GPIO_MOTOR` + `PWM_0` |
| 编码器 | `QEI_0` 与 `GPIO_ENCODER_B` |
| 循迹采集 | ADC + DMA，模块名 `ADC_line_detector` |
| 系统节拍 | SysTick，配置周期值 32000 |

权威配置文件为 `optoelectronicrace_26.syscfg`。修改外设、引脚、中断或 DMA 时，只编辑该文件并使用 CCS/SysConfig 重新生成 `Debug/ti_msp_dl_config.c` 与 `.h`。

## K230 协议

MSPM0 接收固定 13 字节帧：

```text
AA 0D flag s_dx dx s_dy dy idx s_dx_b dx_b s_dy_b dy_b checksum
```

- `flag=0xBB`：目标锁定；`flag=0xCC`：目标丢失
- `s_dx/dx`：中心 X 偏差的符号和绝对值
- `s_dy/dy`：中心 Y 偏差的符号和绝对值
- `idx`：圆点索引
- `s_dx_b/dx_b`、`s_dy_b/dy_b`：圆点偏差
- `checksum`：前 12 个字节的 8 位累加和

默认像素到脉冲换算为 `X=5 pulse/px`、`Y=4 pulse/px`；实际相机视场角、镜头距离和机械安装偏差需要现场标定。

## 关键参数

K230 视觉和云台参数位于 `APP/k230_track.h`：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `PULSE_PER_PX_X` / `Y` | 5 / 4 | 像素到云台脉冲换算 |
| `AIM_DEADBAND_X` / `Y` | 20 / 15 | 瞄准死区（脉冲） |
| `SCAN_STEP_PULSES` | 890 | 分段扫描步长 |
| `COARSE_SPEED` / `FINE_SPEED` | 720 / 360 | 粗调/精调速度 |
| `FINE_MAX_ATTEMPTS` | 3 | 精调最大次数 |
| `Y_MIN_PULSE` / `Y_MAX_PULSE` | -190 / 380 | Y 轴软件限位 |

涉及电机安全的速度、加速度、限位和激光时序参数，修改后必须先脱离负载验证，并确认激光器具备独立的安全防护。

## 构建环境

- Code Composer Studio，TI ARM Clang `TICLANG_4.0.4.LTS`
- MSPM0 SDK `2.10.00.04`
- 目标配置：`targetConfigs/MSPM0G3507.ccxml`
- SysConfig：`optoelectronicrace_26.syscfg`
- 构建输出：`Debug/optoelectronicrace_26.out`

建议在 CCS 中导入本目录作为现有工程并执行 **Build Project**。命令行构建需使用本机实际 CCS/SDK 路径，例如：

```powershell
$env:COM_TI_MSPM0_SDK_INSTALL_DIR = "D:/MSP-SDK/mspm0_sdk_2_10_00_04"
$env:CG_TOOL_ROOT = "D:/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS"
$env:PROJECT_ROOT = "D:/MSP-SDK/CCS_project/optoelectronicrace_26"
$env:PROJECT_BUILD_DIR = "$env:PROJECT_ROOT/Debug"
& "D:\CCS\ccs\utils\bin\gmake" -C $env:PROJECT_BUILD_DIR -j4 all
```

不要手工编辑 `Debug/` 下的 makefile、目标文件、链接信息或 SysConfig 生成文件。

## 烧录与调试建议

1. 在 CCS 中导入本目录工程并确认目标配置。
2. Build Project 生成 `Debug/optoelectronicrace_26.out`。
3. 首次上电时断开电机负载，先验证 OLED、串口接收和云台限位。
4. 再单独验证循迹、K230 帧解析、云台移动和激光开关。
5. 视觉目标丢失、UART 帧校验失败或关闭菜单开关时，应确认执行路径会停止相关执行器。

## 目录说明

| 路径 | 内容 |
| --- | --- |
| `APP/` | 循迹、JY62、激光、K230 视觉和云台应用逻辑 |
| `BSP/` | 延时和 UART 板级支持 |
| `Drive/` | 电机、编码器、灰度传感器、OLED、蜂鸣器和旋钮 |
| `Menu/` | OLED 菜单框架和用户界面 |
| `K230visual/` | K230 侧视觉算法脚本版本 |
| `targetConfigs/` | CCS 调试目标配置 |
| `Debug/` | CCS 生成的构建产物，不应手工修改 |

## 已知限制

- K230 帧协议依赖固定长度和累加校验，视觉端帧率或串口噪声变化时需要重新评估接收缓存策略。
- 云台脉冲换算、方向极性和 Y 轴限位依赖具体机械装配，不能直接套用于其他机构。
- 当前工程是 CCS 专用嵌入式项目，构建依赖 TI 工具链和 MSPM0 SDK，通用 GCC 环境不能替代 CCS 验证。
- `Debug/` 和本地分析目录属于生成/环境文件，不应作为功能源码提交。
