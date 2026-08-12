/*
 * TarantulaBot 固件 - 平脚步态 + 姿态偏移 + 四路电磁铁爬墙版 + 原地转/平移
 *
 * ADJ z0 z1 z2 z3  (mm, 顺序LF LB RF RB, ±20) 由Orange Pi PID下发, 失联500ms清零
 * 四路电磁铁: LF=16 LB=14 RF=13 RB=12
 * CLIMB模式: wave时序 + 磁铁配合(摆动腿释放, 其余吸附, 恒>=3磁铁吸墙)
 *
 * 运动模式(moveMode, 用于FWD/BACK/LEFT/RIGHT, 不影响CLIMB):
 *   0 前进/后退: 四腿同向摆动
 *   1 原地转:   左侧(LF,LB) vs 右侧(RF,RB) 反向摆动 -> 纯自转,不前进 (已实测修正方向)
 *   (曾尝试横向平移模式, 实测为原地踏步, 单自由度YAW摆动架构无法合成横向分量, 已移除;
 *    需要转向移动时用"原地转+前进"组合实现)
 *
 * ID: LF 1/2/3  LB 4/5/6  RF 7/8/9  RB 10/11/12 (YAW, HIP, ANKLE)
 * 平脚约束: knee = -90 - hip
 */
#include <Arduino.h>
#include <SCServo.h>
#include <math.h>

SMS_STS st1;
SMS_STS st2;

// 四路电磁铁 (按腿索引 LF LB RF RB)
#define MAG_LF    16
#define MAG_LB    14
#define MAG_RF    13
#define MAG_RB    12
const int MAG_PIN[4] = {MAG_LF, MAG_LB, MAG_RF, MAG_RB};

#define PUMP_PIN  25
#define BOOT_PIN  0
#define ACT_IN1   32
#define ACT_IN2   33

const float L1 = 76.0, L2 = 72.0, L3 = 61.0;
const float CNT_PER_DEG = 4096.0/360.0;
const float RAD2DEG = 57.29578;
const float DEG2RAD = 0.0174533;

struct Calib { int cnt0; float ang0; int sign; };
struct Leg {
  const char* name; SMS_STS* bus;
  int idYaw, idHip, idKnee;
  float hipy;
  Calib yaw, hip, knee;
};

// 标定 (用户最新实测版)
Leg legs[4] = {
  {"LF", &st1,  1, 2, 3,  +1, {1999,0,-1}, {1861,-45,+1}, {2153,-41,+1}},
  {"LB", &st1,  4, 5, 6,  -1, {1955,0,+1}, {1251,-45,+1}, {1687,-41,+1}},
  {"RF", &st2,  7, 8, 9,  -1, {1975,0,-1}, {1002,-45,+1}, {1880,-41,+1}},
  {"RB", &st2, 10,11,12,  +1, {2058,0,+1}, {1202,-45,+1}, {1752,-41,+1}},
};

// ===== 步态参数 =====
float STAND_HIP  = -45.0;
float LEG_TRIM[4]  = {4.0, 4.0, 4.0, 4.0};   // 度, 四腿独立踝竖直补偿 (LF,LB,RF,RB)
float LEG_REACH[4] = {22.0, 26.0, 24.0, 25.0};   // mm, 四腿独立伸展量 (LF,LB,RF,RB) 实测标定
float STRIDE_DEG = 16.0;
float STEP_H     = 30.0;   // mm, 抬腿高度(trot用)
float WAVE_STEP_H = 50.0;  // mm, wave专用抬腿高度(比trot抬更高, 应对斜坡/爬墙场景)
float WAVE_SWING = 0.20;
float TROT_DUTY  = 0.35;
float TROT_SAG   = 8.0;
float WAVE_RATE  = 0.028;
float TROT_RATE  = 0.020;
int   mode = 0;            // 0=站立 1=trot 2=wave 3=climb 4=strafe(左右平移,支持wave/trot两种时序)
bool  lastPressed = false;

int   dirSign = +1;      // 仅CLIMB沿用
float turnBias = 0.0;    // 仅CLIMB沿用

// FWD/BACK/LEFT/RIGHT 用的独立方向系统 (原地转)
int   moveMode = 0;      // 0=前进后退 1=原地转
int   moveSign = +1;     // 该模式下的方向

// 全向步态(mode=5)专用: 基于实测LEG_DIR_DEG, 一个公式覆盖FWD/BACK/STRAFE
float moveDirDeg = 0.0;  // 目标移动方向(clock角度, 0=前进 180=后退 90=右移 270=左移)
int   omniGait   = 2;    // 1=trot 2=wave

// ===== 姿态偏移 (Orange Pi PID下发) =====
float zOff[4] = {0,0,0,0};
unsigned long lastAdjMs = 0;
const unsigned long ADJ_TIMEOUT = 500;
const float ZOFF_MAX = 20.0;

// 错峰控制: 多个磁铁不同时通断, 每个间隔MAG_STAGGER_MS, 摊平峰值电流
// 改为非阻塞版: 原来用delay()会卡住主循环480ms, 期间舵机收不到新指令表现为"卡顿变慢"
// 现在用millis()记录状态, loop()正常跑, gaitStep/omniStep不受影响
const int MAG_STAGGER_MS = 120;
bool magSeqActive = false;
int  magSeqTarget = HIGH;      // 本次序列是要全开(HIGH)还是全关(LOW)
int  magSeqIdx = 0;
unsigned long magSeqLastMs = 0;

void magAllOn()  { magSeqActive=true; magSeqTarget=HIGH; magSeqIdx=0; magSeqLastMs=millis(); }
void magAllOff() { magSeqActive=true; magSeqTarget=LOW;  magSeqIdx=0; magSeqLastMs=millis(); }

void magSeqUpdate() {
  if (!magSeqActive) return;
  if (magSeqIdx == 0 || millis() - magSeqLastMs >= (unsigned long)MAG_STAGGER_MS) {
    if (magSeqIdx < 4) {
      digitalWrite(MAG_PIN[magSeqIdx], magSeqTarget);
      magSeqIdx++;
      magSeqLastMs = millis();
    } else {
      magSeqActive = false;
    }
  }
}

int angToCnt(Calib &c, float ang_deg) {
  return (int)round(c.cnt0 + c.sign*(ang_deg - c.ang0)*CNT_PER_DEG);
}

void flatFootMove(int i, float yawDeg, float z, int speed) {
  Leg &lg = legs[i];
  z -= LEG_REACH[i];   // 正值=该腿多往下/往外伸展(跨面时脚不够长用)
  float sinHip = (z + L3) / L2;
  sinHip = constrain(sinHip, -1.0, 1.0);
  float hipDeg  = asin(sinHip) * RAD2DEG;
  float kneeDeg = -90.0 - hipDeg + LEG_TRIM[i];   // 四腿独立踝补偿, 不再分组
  lg.bus->WritePosEx(lg.idYaw,  angToCnt(lg.yaw,  yawDeg),  speed, 50);
  lg.bus->WritePosEx(lg.idHip,  angToCnt(lg.hip,  hipDeg),  speed, 50);
  lg.bus->WritePosEx(lg.idKnee, angToCnt(lg.knee, kneeDeg), speed, 50);
}

float standZ() { return L2 * sin(STAND_HIP*DEG2RAD) - L3; }

void standAll(int speed=800) {
  for (int i=0;i<4;i++) flatFootMove(i, 0, standZ() + zOff[i], speed);
}

int swingDir(int i) { return (legs[i].hipy > 0) ? -1 : +1; }

void legTarget(int i, float phase, int gait, float &yawDeg, float &z) {
  bool sw; float s;
  if (gait == 1) {
    float grp = (i==0 || i==3) ? 0.0 : 0.5;
    float lp = fmod(phase + grp, 1.0);
    sw = lp < TROT_DUTY;
    s = sw ? lp/TROT_DUTY : (lp-TROT_DUTY)/(1.0-TROT_DUTY);
  } else {
    float seq;
    if (i==1) seq=0;
    else if (i==0) seq=1;
    else if (i==3) seq=2;
    else seq=3;
    float lp = fmod(phase - seq/4.0 + 1.0, 1.0);
    sw = lp < WAVE_SWING;
    s = sw ? lp/WAVE_SWING : (lp-WAVE_SWING)/(1.0-WAVE_SWING);
  }

  // 两种运动模式的振幅分配:
  //   0 前进/后退: 四腿同向摆动
  //   1 原地转:   左侧(LF,LB) vs 右侧(RF,RB) 反向摆动 -> 纯自转
  // 注: 曾尝试moveMode=2(前后腿反向=平移), 实测为原地踏步(单自由度YAW摆动架构
  //     无法独立合成横向分量), 已移除。需要平移时用"原地转+前进"组合实现转向。
  float amp;
  if (moveMode == 1) {
    float side = (i==0 || i==1) ? -1.0 : +1.0;   // LF,LB=左侧  RF,RB=右侧 (已按实测翻转)
    amp = STRIDE_DEG * moveSign * side;
  } else {
    amp = STRIDE_DEG * moveSign;                  // 前进/后退
  }

  float q;
  z = standZ() + zOff[i];
  float stepH = (gait == 1) ? STEP_H : WAVE_STEP_H;
  if (sw) {
    q = (s - sin(2*M_PI*s)/(2*M_PI)) - 0.5;
    z += stepH * (1-cos(2*M_PI*s)) / 2;
  } else {
    q = 0.5 - s;
    if (gait == 1) z -= TROT_SAG;
  }
  yawDeg = swingDir(i) * amp * q * 2.0;
}

void gaitStep(int gait) {
  static float phase = 0;
  phase = fmod(phase + ((gait==1) ? TROT_RATE : WAVE_RATE), 1.0);
  int speed = 3400;
  for (int i=0;i<4;i++){
    float yawDeg, z;
    legTarget(i, phase, gait, yawDeg, z);
    flatFootMove(i, yawDeg, z, speed);
  }
  delay(40);
}

// ===== 爬墙步态: wave时序 + 磁铁配合 (未改动, 仍用dirSign/turnBias) =====
void climbStep() {
  static float phase = 0;
  phase = fmod(phase + WAVE_RATE, 1.0);
  int speed = 3400;

  for (int i=0;i<4;i++){
    float seq;
    if (i==1) seq=0;
    else if (i==0) seq=1;
    else if (i==3) seq=2;
    else seq=3;
    float lp = fmod(phase - seq/4.0 + 1.0, 1.0);
    bool sw = lp < WAVE_SWING;
    float s = sw ? lp/WAVE_SWING : (lp-WAVE_SWING)/(1.0-WAVE_SWING);

    if (sw) {
      if (s < 0.15) digitalWrite(MAG_PIN[i], LOW);   // 起摆释放, 时机正确(脚刚离墙)
      // 原来这里有 else if (s>0.85) 提前通电 —— 此时脚离墙面仍有约20%*STEP_H的气隙
      // (气隙对磁吸力极敏感, 提前通电基本吸不住), 这正是"交接打滑"的根源, 已删除
      // 改为完全依赖下面stance分支: 只有真正进入支撑相(脚已贴墙, 气隙=0)才通电
    } else {
      digitalWrite(MAG_PIN[i], HIGH);   // 支撑相开始瞬间=脚已贴墙, 此时通电才有效
    }

    float amp = STRIDE_DEG * dirSign;
    if (i==0 || i==1) amp *= (1.0 - turnBias);
    else              amp *= (1.0 + turnBias);

    float q, z = standZ() + zOff[i];
    if (sw) {
      q = (s - sin(2*M_PI*s)/(2*M_PI)) - 0.5;
      z += WAVE_STEP_H * (1-cos(2*M_PI*s)) / 2;
    } else {
      q = 0.5 - s;
    }
    float yawDeg = swingDir(i) * amp * q * 2.0;
    flatFootMove(i, yawDeg, z, speed);
  }
  delay(40);
}

// ===== 全向步态 (基于实测方向标定, 一个公式覆盖前进/后退/左移/右移) =====
// 每条腿的真实响应方向(clock角度制, 12点=0°=机头, 顺时针为正) - 2026-08实测标定:
//   LF=-15° LB=-165° RF=+45° RB=+135° (左右两侧明显不对称, 之前所有±1差动猜测
//   必然失败的根源就在这里 —— 几何本身不对称, 不存在简单的对称差动组合)
// 任意移动方向φ(clock角度), 每条腿振幅权重 = cos(该腿角度 - φ), 不再需要swingDir
const float LEG_DIR_DEG[4] = { -15.0, -165.0, 45.0, 135.0 };  // LF LB RF RB

void omniStep(float moveDirDeg, int gait) {
  static float phase = 0;
  phase = fmod(phase + ((gait==1) ? TROT_RATE : WAVE_RATE), 1.0);
  int speed = 3400;
  float phi = moveDirDeg * DEG2RAD;

  for (int i=0;i<4;i++){
    bool sw; float s;
    if (gait == 1) {
      float grp = (i==0 || i==3) ? 0.0 : 0.5;
      float lp = fmod(phase + grp, 1.0);
      sw = lp < TROT_DUTY;
      s = sw ? lp/TROT_DUTY : (lp-TROT_DUTY)/(1.0-TROT_DUTY);
    } else {
      float seq;
      if (i==1) seq=0;
      else if (i==0) seq=1;
      else if (i==3) seq=2;
      else seq=3;
      float lp = fmod(phase - seq/4.0 + 1.0, 1.0);
      sw = lp < WAVE_SWING;
      s = sw ? lp/WAVE_SWING : (lp-WAVE_SWING)/(1.0-WAVE_SWING);
    }

    float theta = LEG_DIR_DEG[i] * DEG2RAD;
    float weight = cos(theta - phi);      // 该腿对目标方向的贡献权重, 替代swingDir
    float amp = STRIDE_DEG * weight;

    float q, z = standZ() + zOff[i];
    float stepH = (gait == 1) ? STEP_H : WAVE_STEP_H;
    if (sw) {
      q = (s - sin(2*M_PI*s)/(2*M_PI)) - 0.5;
      z += stepH * (1-cos(2*M_PI*s)) / 2;
    } else {
      q = 0.5 - s;
      if (gait == 1) z -= TROT_SAG;
    }
    float yawDeg = amp * q * 2.0;          // 不再乘swingDir, weight已含正确符号
    flatFootMove(i, yawDeg, z, speed);
  }
  delay(40);
}

void actuatorStop(){ digitalWrite(ACT_IN1,LOW); digitalWrite(ACT_IN2,LOW); }

// ===== 单腿方向标定测试 (方案B第一步: 精确测量每条腿足端真实响应方向) =====
// 其余三腿保持站姿不动, 被测腿从yawDeg=-SWEEP摆到+SWEEP再摆回, 悬空往返
// 观察足端摆动方向对应的钟点(以机头=12点), 记录后用于计算完整2D步态
void testLegDir(int i) {
  const float SWEEP = 25.0;  // 度, 摆动幅度(适当放大方便观察)
  int speed = 800;
  float z = standZ();
  Serial.printf(">>> 测试 %s: 从-%.0f度摆到+%.0f度, 观察足端摆动方向(以机头=12点)\n",
                legs[i].name, SWEEP, SWEEP);
  flatFootMove(i, -SWEEP, z, speed); delay(1000);
  flatFootMove(i, +SWEEP, z, speed); delay(1200);
  flatFootMove(i, -SWEEP, z, speed); delay(1200);
  flatFootMove(i, 0, z, speed);      delay(800);
  Serial.printf("<<< %s 完成, 记录方向\n", legs[i].name);
}

bool parseAdj(String &cmd) {
  if (!cmd.startsWith("ADJ")) return false;
  float v[4];
  int idx = 3, n = 0;
  while (n < 4) {
    while (idx < (int)cmd.length() && cmd[idx]==' ') idx++;
    if (idx >= (int)cmd.length()) return false;
    int sp = cmd.indexOf(' ', idx);
    String tok = (sp<0) ? cmd.substring(idx) : cmd.substring(idx, sp);
    v[n++] = tok.toFloat();
    if (sp<0) break;
    idx = sp;
  }
  if (n < 4) return false;
  for (int i=0;i<4;i++) zOff[i] = constrain(v[i], -ZOFF_MAX, ZOFF_MAX);
  lastAdjMs = millis();
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, 19, 21);
  Serial2.begin(1000000, SERIAL_8N1, 22, 23);
  st1.pSerial=&Serial1; st2.pSerial=&Serial2;

  for(int i=0;i<4;i++){ pinMode(MAG_PIN[i],OUTPUT); digitalWrite(MAG_PIN[i],LOW); }
  pinMode(PUMP_PIN,OUTPUT);
  pinMode(BOOT_PIN,INPUT_PULLUP);
  pinMode(ACT_IN1,OUTPUT); pinMode(ACT_IN2,OUTPUT);
  digitalWrite(PUMP_PIN,LOW); actuatorStop();

  delay(1000);
  Serial.println("=== TarantulaBot 平脚步态 + 四路磁铁爬墙固件 (含原地转/平移) ===");
  standAll();
  delay(1000);
  magAllOn();
  Serial.println("站立完成, 四磁铁错峰吸附");
}

void loop() {
  magSeqUpdate();   // 非阻塞磁铁错峰序列推进, 不影响舵机指令时序

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (parseAdj(cmd)) {
      if (mode == 0) standAll(1500);
    }
    else if (cmd=="FWD"||cmd=="WALK"||cmd=="WAVE"){mode=5; moveDirDeg=0.0;   omniGait=2; Serial.println("前进(全向,wave)");}
    else if (cmd=="BACK")         {mode=5; moveDirDeg=180.0; omniGait=2; Serial.println("后退(全向,wave)");}
    else if (cmd=="LEFT")         {mode=2; moveMode=1; moveSign=+1; Serial.println("原地左转(wave)");}
    else if (cmd=="RIGHT")        {mode=2; moveMode=1; moveSign=-1; Serial.println("原地右转(wave)");}
    else if (cmd=="TROT")  {mode=1; moveMode=0; moveSign=+1; Serial.println("Trot前进");}
    else if (cmd=="TBACK") {mode=1; moveMode=0; moveSign=-1; Serial.println("Trot后退");}
    else if (cmd=="TLEFT") {mode=1; moveMode=1; moveSign=+1; Serial.println("原地左转(trot)");}
    else if (cmd=="TRIGHT"){mode=1; moveMode=1; moveSign=-1; Serial.println("原地右转(trot)");}
    else if (cmd=="CLIMB") {mode=3; dirSign=+1; turnBias=0;   Serial.println("爬墙(wave+磁铁时序)");}
    else if (cmd=="TESTDIR_LF") {mode=0; standAll(800); delay(500); testLegDir(0);}
    else if (cmd=="TESTDIR_LB") {mode=0; standAll(800); delay(500); testLegDir(1);}
    else if (cmd=="TESTDIR_RF") {mode=0; standAll(800); delay(500); testLegDir(2);}
    else if (cmd=="TESTDIR_RB") {mode=0; standAll(800); delay(500); testLegDir(3);}
    else if (cmd=="STRAFE_LEFT")   {mode=5; moveDirDeg=90.0;  omniGait=2; Serial.println("左移(全向,wave)");}
    else if (cmd=="STRAFE_RIGHT")  {mode=5; moveDirDeg=270.0; omniGait=2; Serial.println("右移(全向,wave)");}
    else if (cmd=="TSTRAFE_LEFT")  {mode=5; moveDirDeg=90.0;  omniGait=1; Serial.println("左移(全向,trot)");}
    else if (cmd=="TSTRAFE_RIGHT") {mode=5; moveDirDeg=270.0; omniGait=1; Serial.println("右移(全向,trot)");}
    else if (cmd=="SLOW")  {WAVE_RATE=0.003; TROT_RATE=0.003; Serial.println("慢放");}
    else if (cmd=="FAST")  {WAVE_RATE=0.028; TROT_RATE=0.020; Serial.println("正常速度");}
    else if (cmd=="STAND"||cmd=="STOP"){mode=0; standAll(); Serial.println("站立");}
    // 四腿独立踝补偿: TRIM_LF/LB/RF/RB <值>
    else if (cmd.startsWith("TRIM_LF ")) { LEG_TRIM[0]=constrain(cmd.substring(8).toFloat(),-30,30); if(mode==0)standAll(600); }
    else if (cmd.startsWith("TRIM_LB ")) { LEG_TRIM[1]=constrain(cmd.substring(8).toFloat(),-30,30); if(mode==0)standAll(600); }
    else if (cmd.startsWith("TRIM_RF ")) { LEG_TRIM[2]=constrain(cmd.substring(8).toFloat(),-30,30); if(mode==0)standAll(600); }
    else if (cmd.startsWith("TRIM_RB ")) { LEG_TRIM[3]=constrain(cmd.substring(8).toFloat(),-30,30); if(mode==0)standAll(600); }
    else if (cmd.startsWith("TRIM "))    {   // 兼容: 四腿同时设为同一值
      float v = constrain(cmd.substring(5).toFloat(), -30.0, 30.0);
      for(int k=0;k<4;k++) LEG_TRIM[k]=v;
      if (mode == 0) standAll(600);
    }
    // 四腿独立伸展量(mm): REACH_LF/LB/RF/RB <值>, 跨面/够不到时("踩空")用
    else if (cmd.startsWith("REACH_LF ")) { LEG_REACH[0]=constrain(cmd.substring(9).toFloat(),-25,25); if(mode==0)standAll(600); }
    else if (cmd.startsWith("REACH_LB ")) { LEG_REACH[1]=constrain(cmd.substring(9).toFloat(),-25,25); if(mode==0)standAll(600); }
    else if (cmd.startsWith("REACH_RF ")) { LEG_REACH[2]=constrain(cmd.substring(9).toFloat(),-25,25); if(mode==0)standAll(600); }
    else if (cmd.startsWith("REACH_RB ")) { LEG_REACH[3]=constrain(cmd.substring(9).toFloat(),-25,25); if(mode==0)standAll(600); }
    else if (cmd=="MAGNET_ON")  {magAllOn();  Serial.println("四磁铁错峰吸附");}
    else if (cmd=="MAGNET_OFF") {magAllOff(); Serial.println("四磁铁错峰释放");}
    else if (cmd=="MAG_LF_ON") {digitalWrite(MAG_LF,HIGH);} else if (cmd=="MAG_LF_OFF"){digitalWrite(MAG_LF,LOW);}
    else if (cmd=="MAG_LB_ON") {digitalWrite(MAG_LB,HIGH);} else if (cmd=="MAG_LB_OFF"){digitalWrite(MAG_LB,LOW);}
    else if (cmd=="MAG_RF_ON") {digitalWrite(MAG_RF,HIGH);} else if (cmd=="MAG_RF_OFF"){digitalWrite(MAG_RF,LOW);}
    else if (cmd=="MAG_RB_ON") {digitalWrite(MAG_RB,HIGH);} else if (cmd=="MAG_RB_OFF"){digitalWrite(MAG_RB,LOW);}
    else if (cmd=="PUMP_ON")    {digitalWrite(PUMP_PIN,HIGH);Serial.println("气泵开");}
    else if (cmd=="PUMP_OFF")   {digitalWrite(PUMP_PIN,LOW); Serial.println("气泵关");}
    else if (cmd=="ACT_EXTEND") {digitalWrite(ACT_IN1,HIGH);digitalWrite(ACT_IN2,LOW); Serial.println("推杆伸");}
    else if (cmd=="ACT_RETRACT"){digitalWrite(ACT_IN1,LOW); digitalWrite(ACT_IN2,HIGH);Serial.println("推杆缩");}
    else if (cmd=="ACT_STOP")   {actuatorStop(); Serial.println("推杆停");}
  }

  if (lastAdjMs && millis() - lastAdjMs > ADJ_TIMEOUT) {
    bool wasActive = false;
    for (int i=0;i<4;i++){ if (zOff[i]!=0) wasActive=true; zOff[i]=0; }
    lastAdjMs = 0;
    if (wasActive && mode==0) standAll(800);
  }

  bool pressed = (digitalRead(BOOT_PIN)==LOW);
  static unsigned long lastMs = 0;
  static int bootIdx = 0;   // 0=站立 1=trot 2=wave 3=左移测试 4=右移测试 (跳过CLIMB,避免误触发磁铁)
  if (pressed && !lastPressed && (millis()-lastMs>300)) {
    lastMs = millis();
    bootIdx = (bootIdx+1) % 5;
    dirSign=+1; turnBias=0;
    moveMode=0; moveSign=+1;
    if (bootIdx==0){ mode=0; standAll(); Serial.println("[BOOT] 站立"); }
    else if (bootIdx==1){ mode=1; Serial.println("[BOOT] Trot前进"); }
    else if (bootIdx==2){ mode=2; Serial.println("[BOOT] Wave前进"); }
    else if (bootIdx==3){ mode=5; moveDirDeg=90.0;  omniGait=2; Serial.println("[BOOT] 左移测试(全向)"); }
    else               { mode=5; moveDirDeg=270.0; omniGait=2; Serial.println("[BOOT] 右移测试(全向)"); }
  }
  lastPressed = pressed;

  if (mode==1) gaitStep(1);
  else if (mode==2) gaitStep(2);
  else if (mode==3) climbStep();
  else if (mode==5) omniStep(moveDirDeg, omniGait);
}