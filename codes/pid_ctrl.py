"""
TarantulaBot 姿态PID控制器 (Orange Pi端)
IMU(roll/pitch) -> PID -> 四腿z偏移 -> 串口 "ADJ z0 z1 z2 z3" -> ESP32

两种用法:
  1) 单独运行(自己开串口, 站立自平衡测试):  python3 pid_ctrl.py
  2) 被app.py调用(共享app的串口):
       from pid_ctrl import AttitudeController
       ctrl = AttitudeController(send_func)   # send_func(str)负责写串口
       ctrl.start() / ctrl.stop()

安装假定: MPU6050在机头, X轴朝机头方向, 元件面朝上
  若实测方向相反, 改 ROLL_SIGN / PITCH_SIGN
"""
import time
import threading
from imu import IMU

# ===== 配置 =====
LOOP_HZ    = 50
ROLL_SIGN  = +1     # 实测: 机身左倾时imu.py的roll为正 -> +1; 为负 -> -1
PITCH_SIGN = +1     # 实测: 机头下压时pitch为正 -> +1; 为负 -> -1
DEADZONE   = 1.0    # 度, 死区(从1.5收紧, 提升灵敏度)
LP_ALPHA   = 0.75   # 姿态低通(从0.85降低, 减少相位滞后导致的慢速摆动)
ZMAX       = 18.0   # mm, 单腿最大修正(略小于固件ZOFF_MAX)

# 腿的位置系数: z修正 = roll项*侧向系数 + pitch项*纵向系数
#   顺序 LF LB RF RB;  左腿侧向+1/右腿-1;  前腿纵向+1/后腿-1
SIDE  = [+1, +1, -1, -1]
FRONT = [+1, -1, +1, -1]

class PID:
    def __init__(self, kp, ki, kd, imax=10.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.imax = imax
        self.i = 0.0
        self.last_e = None

    def reset(self):
        self.i = 0.0
        self.last_e = None

    def update(self, e, dt):
        self.i += e * dt
        self.i = max(-self.imax, min(self.imax, self.i))   # 抗饱和
        d = 0.0 if self.last_e is None else (e - self.last_e) / dt
        self.last_e = e
        return self.kp*e + self.ki*self.i + self.kd*d


class AttitudeController:
    def __init__(self, send_func, kp=0.8, ki=0.0, kd=0.08):
        """send_func: 函数, 接收一条字符串指令并写入ESP32串口
           增益单位: mm每度. 调参: 先Kp(0.5起), 稳后加Kd, 有稳态误差再开一点Ki"""
        self.send = send_func
        self.pid_roll  = PID(kp, ki, kd)
        self.pid_pitch = PID(kp, ki, kd)
        self.imu = IMU()
        self.running = False
        self.thread = None
        self.f_roll = 0.0
        self.f_pitch = 0.0
        self.status = {"roll":0.0, "pitch":0.0, "z":[0,0,0,0]}

    def start(self):
        if self.running:
            return
        self.imu.calibrate(1.5)
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        self.send("ADJ 0 0 0 0")   # 清零偏移

    def _loop(self):
        dt_target = 1.0 / LOOP_HZ
        last = time.time()
        while self.running:
            roll, pitch = self.imu.update()
            roll  *= ROLL_SIGN
            pitch *= PITCH_SIGN

            # 低通
            self.f_roll  = LP_ALPHA*self.f_roll  + (1-LP_ALPHA)*roll
            self.f_pitch = LP_ALPHA*self.f_pitch + (1-LP_ALPHA)*pitch

            now = time.time()
            dt = max(now - last, 1e-3)
            last = now

            # 死区 + PID (目标姿态=水平, 误差=-当前角)
            er = -self.f_roll  if abs(self.f_roll)  > DEADZONE else 0.0
            ep = -self.f_pitch if abs(self.f_pitch) > DEADZONE else 0.0
            ur = self.pid_roll.update(er, dt)    # mm, 横滚修正量
            up = self.pid_pitch.update(ep, dt)   # mm, 俯仰修正量

            # 分配到四腿: 哪边低哪边腿伸长(z更低=顶高)
            z = []
            for i in range(4):
                zi = -(ur*SIDE[i] + up*FRONT[i])
                zi = max(-ZMAX, min(ZMAX, zi))
                z.append(zi)

            self.send(f"ADJ {z[0]:.1f} {z[1]:.1f} {z[2]:.1f} {z[3]:.1f}")
            self.status = {"roll":round(self.f_roll,1),
                           "pitch":round(self.f_pitch,1),
                           "z":[round(v,1) for v in z]}

            sleep = dt_target - (time.time() - now)
            if sleep > 0:
                time.sleep(sleep)


# ===== 单独运行: 站立自平衡测试 =====
if __name__ == '__main__':
    import serial
    esp = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    time.sleep(2)

    def send(s):
        esp.write((s + '\n').encode())

    ctrl = AttitudeController(send, kp=1.4, ki=0.0, kd=0.08)
    print("站立自平衡测试: 机器人保持STAND, 倾斜机身/垫东西观察腿部补偿")
    print("Ctrl+C退出")
    send("STAND")
    time.sleep(1)
    ctrl.start()
    try:
        while True:
            s = ctrl.status
            print(f"\rroll={s['roll']:+5.1f} pitch={s['pitch']:+5.1f} "
                  f"z={s['z']}", end='', flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.stop()
        print("\n已停止, 偏移清零")
