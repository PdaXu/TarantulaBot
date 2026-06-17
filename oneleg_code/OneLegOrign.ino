#include <Arduino.h>
#include <SCServo.h>

SMS_STS st;

#define DO_PIN 16
#define BOOT_PIN 0

bool magnetOn = false;
bool lastPressed = false;

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, 19, 21);
  st.pSerial = &Serial1;
  
  pinMode(DO_PIN, OUTPUT);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  digitalWrite(DO_PIN, LOW);
  
  delay(1000);
  Serial.println("启动");
  
  magnetOn = true;
  digitalWrite(DO_PIN, HIGH);
  Serial.println("电磁铁开");
}

void doStep() {
  // 电磁铁断电
  magnetOn = false;
  digitalWrite(DO_PIN, LOW);
  Serial.println("电磁铁关，舵机启动");
  delay(500);
  
  // 抬腿
  st.WritePosEx(1, 2043, 1500, 50);
  st.WritePosEx(2, 2400, 1500, 50);
  st.WritePosEx(3, 1702, 1500, 50);
  Serial.println("抬腿");
  delay(1000);
  
  // 平转
  st.WritePosEx(1, 2626, 1500, 50);
  st.WritePosEx(2, 2400, 1500, 50);
  st.WritePosEx(3, 1702, 1500, 50);
  Serial.println("平转");
  delay(1000);
  
  // 落腿
  st.WritePosEx(1, 2626, 1500, 50);
  st.WritePosEx(2, 2068, 1500, 50);
  st.WritePosEx(3, 2021, 1500, 50);
  Serial.println("落腿");
  delay(1000);

   // 电磁铁开
  magnetOn = true;
  digitalWrite(DO_PIN, HIGH);
  Serial.println("落地，电磁铁开");
  
  // 复位
  st.WritePosEx(1, 2043, 1500, 50);
  st.WritePosEx(2, 2068, 1500, 50);
  st.WritePosEx(3, 2021, 1500, 50);
  Serial.println("复位");
  delay(1000);
  
  // 电磁铁开
  /*magnetOn = true;
  digitalWrite(DO_PIN, HIGH);
  Serial.println("落地，电磁铁开");
  */
}

void loop() {
  // BOOT键触发
  bool pressed = (digitalRead(BOOT_PIN) == LOW);
  if (pressed && !lastPressed) {
    doStep();
    delay(50);
  }
  lastPressed = pressed;
  
  // 串口指令触发
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "STEP") {
      doStep();
    } else if (cmd == "MAGNET_ON") {
      magnetOn = true;
      digitalWrite(DO_PIN, HIGH);
      Serial.println("电磁铁开");
    } else if (cmd == "MAGNET_OFF") {
      magnetOn = false;
      digitalWrite(DO_PIN, LOW);
      Serial.println("电磁铁关");
    }
  }
}