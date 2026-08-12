<div align="center">

# 🕷️ TarantulaBot

**A Bio-Inspired Quadrupedal Wall-Climbing Robot with Electromagnetic Adhesion and a Soft Manipulator**

*仿生四足爬壁机器人 · 电磁吸附 · 软体机械手*

<!-- 图1：整机照片（机器人贴在钢面上工作）。放这里当 hero 图，第一眼抓人。 -->
<!-- IMAGE 1: Hero shot — the robot adhered to the steel wall. This is the first thing visitors see. -->
![TarantulaBot](./imageshero.jpg)

[English](#english) · [中文](#中文)

</div>

---

<a name="english"></a>
## English

### Overview

TarantulaBot is a bio-inspired quadrupedal wall-climbing robot designed to inspect and interact with ferromagnetic vertical structures such as steel towers, tanks, and I-beams. Unlike most wall-climbing platforms that are limited to passive inspection, TarantulaBot combines **legged climbing** with an **onboard soft manipulator**, so it can both traverse a surface and grasp small objects on it.

The platform integrates four subsystems on a two-tier control architecture: **the Orange Pi 5 decides, the ESP32 executes.**

<!-- 图2：系统架构框图（Orange Pi / ESP32 两层）。放在 Overview 下面，帮读者建立整体认知。 -->
<!-- IMAGE 2: System architecture diagram (two-tier Orange Pi / ESP32). Anchors the reader's mental model. -->
![System Architecture](docs/images/architecture.png)

### Key Features

- **12-DOF quadruped** — 4 legs × 3-DOF, driven by Feetech STS3215 bus servos
- **Flat-foot constrained gait** — ankle held vertical so the flat foot stays flush; each step is a HIP lift + YAW arc sweep
- **IMU attitude stabilization** — MPU6050 + complementary filter + PD, distributing a body-tilt error into per-leg height offsets (recovers a ~10° disturbance to level in ~0.40 s)
- **Per-foot electromagnetic adhesion** — MOSFET-switched, staggered activation, flyback-protected; handoff synchronized to the swing-to-stance transition
- **ArUco bearing-based visual servoing** — track a marker with no camera calibration
- **Soft vacuum-membrane manipulator** — grasps small irregular objects (keys, screws)
- **Flask web interface** — dark-theme control panel, bilingual toggle, per-leg magnet test, auto-reconnecting serial

### Demo

<!-- 图3：斜面爬行照片（标注角度 30–45°）。展示核心能力。 -->
<!-- IMAGE 3: Robot climbing the inclined steel plate (annotate 30–45°). Shows the core capability. -->
![Climbing demo](docs/images/climbing.jpg)

<!-- 图4：软夹爪抓取特写（钥匙/螺丝）。展示操作能力。 -->
<!-- IMAGE 4: Soft gripper holding a key / screw. Shows manipulation. -->
![Grasping demo](docs/images/grasping.jpg)

> 📹 A demo video is available here: `docs/demo.mp4` *(or link to your video)*

### How It Works

**Flat-foot constrained gait.** The feet are flat pads, not point contacts — a flat pad only adheres when its face is parallel to the wall. The ankle is therefore held vertical, and locomotion is produced by lifting the leg (HIP) and sweeping it in a YAW arc to the next footfall, rather than translating the foot in a straight line. Steering is differential: the left and right legs sweep by different amounts.

**Attitude control.** An MPU6050 is fused with a complementary filter (α ≈ 0.98) to estimate roll and pitch. A per-axis PD controller converts the tilt error into per-leg vertical offsets — the low side extends and the high side retracts, levelling the body since it cannot rotate itself.

<!-- 图5：IMU 姿态恢复曲线（roll/pitch 随时间，标 <1s）。有实测数据的图最有说服力。 -->
<!-- IMAGE 5: Measured PD attitude-recovery curve. Real data is the most convincing figure in the repo. -->
![Attitude recovery](docs/images/recovery.png)

**Visual servoing.** ArUco marker tracking uses the marker's horizontal pixel offset from image center as a heading error and drives it to zero — no camera intrinsics needed, because only alignment matters, not distance.

### Repository Structure

```
TarantulaBot/
├── firmware/          # ESP32 firmware (Arduino C++): servo bus + magnet switching
├── orangepi/          # High-level control (Python): IMU PD, gait, ArUco, Flask
│   ├── pid_ctrl.py    # IMU attitude PD controller
│   ├── app.py         # Flask web interface
│   └── ...
├── cad/               # 3D models (SolidWorks / STL) for legs and body plate
├── docs/              # Report, images, diagrams
└── README.md
```
> 按你的实际目录调整；上面是建议结构。

### Hardware

| Component | Detail |
|---|---|
| Compute (high-level) | Orange Pi 5 (RK3588S, ROS2 Humble) |
| Controller (low-level) | ESP32 (WROOM-32) |
| Servos | 12 × Feetech STS3215 (1 Mbaud TTL bus) |
| Adhesion | 4 × 12 V electromagnets, N-MOSFET switching, flyback diodes |
| IMU | MPU6050 (I²C) |
| Manipulator | Soft vacuum-membrane gripper + miniature pump |
| Structure | 3D-printed (PLA); SolidWorks source in `cad/` |

### Getting Started

```bash
# On the Orange Pi 5
git clone https://github.com/<your-username>/TarantulaBot.git
cd TarantulaBot/orangepi
pip install -r requirements.txt

# Flash firmware/ to the ESP32 via Arduino IDE (SCServo library required)

# Run the web control interface
python3 app.py
# then open http://<orangepi-ip>:5000 in a browser
```
> 具体命令按你的实际代码补全。

### Status & Limitations

TarantulaBot has been validated on flat ground and inclined steel up to **30–45°**, with reliable adhesion, attitude recovery, and soft-gripper grasping. **Full vertical (90°) wall-climbing was attempted but not yet achieved** — currently limited by adhesion-handoff timing, dynamic load transfer during leg swing, and structural rigidity under shear. This is the primary target for the next iteration.

### Team

**Team XD** — Xu Chenfei · Xu Pandeng
Singapore University of Technology and Design (SUTD) · Robotics and Automation (MTD)
Mentor: Prof. Pablo Valdivia y Alvarado

### License

Released under the MIT License — see [LICENSE](LICENSE). *（如果你想用别的许可，告诉我改）*

---

<a name="中文"></a>
## 中文

### 项目简介

TarantulaBot 是一款仿生四足爬壁机器人,面向钢塔、储罐、工字梁等**铁磁性垂直结构**的巡检与作业。不同于大多数只能被动巡检的爬壁平台,TarantulaBot 把**足式攀爬**与**机载软体机械手**结合在一起——既能在表面移动,又能抓取表面上的小物件。

整机由四个子系统组成,采用两层控制架构:**Orange Pi 5 负责决策,ESP32 负责执行。**

<!-- 系统架构框图同上（图2） -->

### 核心特性

- **12 自由度四足** —— 4 腿 × 3 自由度,Feetech STS3215 总线舵机驱动
- **平足约束步态** —— 踝关节保持垂直,让平底足始终平贴表面;每步为 HIP 抬升 + YAW 弧形扫动
- **IMU 姿态稳定** —— MPU6050 + 互补滤波 + PD,将本体倾斜误差分配为每条腿的高度偏移(约 10° 扰动可在 ~0.40s 内回稳)
- **足端电磁吸附** —— MOSFET 开关、交错激活、flyback 保护;吸附时序严格同步到摆动相→支撑相切换
- **ArUco 方位视觉伺服** —— 无需相机标定即可跟踪标记
- **软体真空膜机械手** —— 抓取钥匙、螺丝等小型不规则物体
- **Flask 网页控制界面** —— 深色主题、中英切换、每条腿电磁铁独立测试、串口自动重连

### 演示

<!-- 斜面爬行图（图3）、软夹爪抓取图（图4）同上 -->

> 📹 演示视频:`docs/demo.mp4`（或替换成你的视频链接）

### 工作原理

**平足约束步态。** 足底是平垫而非点接触——平垫只有在与墙面平行时才能吸附。因此踝关节保持垂直,迈步靠抬腿(HIP)+ YAW 弧扫到下一落点实现,而不是让足直线平移。转向采用左右腿差速扫动。

**姿态控制。** MPU6050 经互补滤波(α ≈ 0.98)估计 roll/pitch,每轴 PD 控制器把倾斜误差转成每条腿的垂直偏移——低的一侧伸长、高的一侧缩短,靠不等高把本体撑平(本体自身无法旋转)。

<!-- IMU 姿态恢复曲线（图5）同上 -->

**视觉伺服。** ArUco 标记跟踪以标记中心相对图像中心的水平像素偏移作为朝向误差并将其归零——因为只关心对准、不关心距离,所以不需要相机内参。

### 目录结构

```
TarantulaBot/
├── firmware/          # ESP32 固件（Arduino C++）：舵机总线 + 电磁铁开关
├── orangepi/          # 上位机控制（Python）：IMU PD、步态、ArUco、Flask
│   ├── pid_ctrl.py    # IMU 姿态 PD 控制器
│   ├── app.py         # Flask 网页界面
│   └── ...
├── cad/               # 三维模型（SolidWorks / STL）：腿部与机身板
├── docs/              # 报告、图片、示意图
└── README.md
```
> 请按你的实际目录调整。

### 硬件清单

| 部件 | 说明 |
|---|---|
| 主控（高层） | Orange Pi 5（RK3588S，ROS2 Humble） |
| 下位机（底层） | ESP32（WROOM-32） |
| 舵机 | 12 × Feetech STS3215（1 Mbaud TTL 总线） |
| 吸附 | 4 × 12 V 电磁铁，N-MOSFET 开关，flyback 二极管 |
| IMU | MPU6050（I²C） |
| 机械手 | 软体真空膜夹爪 + 微型气泵 |
| 结构 | 3D 打印（PLA）；SolidWorks 源文件见 `cad/` |

### 快速开始

```bash
# 在 Orange Pi 5 上
git clone https://github.com/<你的用户名>/TarantulaBot.git
cd TarantulaBot/orangepi
pip install -r requirements.txt

# 用 Arduino IDE 将 firmware/ 烧录到 ESP32（需 SCServo 库）

# 运行网页控制界面
python3 app.py
# 浏览器打开 http://<orangepi-ip>:5000
```
> 具体命令按你的实际代码补全。

### 现状与局限

TarantulaBot 已在平地和倾斜钢面(**30–45°**)上验证,吸附可靠,姿态可恢复,软夹爪抓取成功。**90° 垂直爬墙已尝试但尚未完全实现**——当前受限于吸附时序、摆动相动态载荷转移、以及剪切载荷下的结构刚性,这是下一代的首要目标。

### 团队

**Team XD** —— 徐晨飞 · 徐磐登
新加坡科技设计大学(SUTD)· 机器人与自动化(MTD)
导师:Prof. Pablo Valdivia y Alvarado

### 许可

采用 MIT 许可证,见 [LICENSE](LICENSE)。

</div>
