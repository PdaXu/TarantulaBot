#include <SCServo.h>

SMS_STS st;
#define DO_PIN 16
#define BOOT_PIN 0  // BOOT键对应GPIO0

//舵机测试
/*

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, RX, TX);
  st.pSerial = &Serial1;
  pinMode(BOOT_PIN, INPUT_PULLUP);
  delay(1000);
  Serial.println("准备好了，按BOOT键启动舵机");
}

void loop() {
  if (digitalRead(BOOT_PIN) == LOW) {
    // 抬腿
    st.WritePosEx(1, 1500, 1500, 50);
    st.WritePosEx(2, 1500, 1500, 50);
    st.WritePosEx(3, 1500, 1500, 50);
    delay(2000);
    
    // 落腿
    st.WritePosEx(1, 2500, 1500, 50);
    st.WritePosEx(2, 2500, 1500, 50);
    st.WritePosEx(3, 2500, 1500, 50);
    delay(2000);
  }
}

*/

//电磁铁模块


bool magnetOn = false;
bool lastPressed = false;

void setup() {
  pinMode(DO_PIN, OUTPUT);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  digitalWrite(DO_PIN, LOW);
  Serial.begin(115200);
  Serial.println("准备好了，按BOOT键切换电磁铁");
}

void loop() {
  bool pressed = (digitalRead(BOOT_PIN) == LOW);
  
  if (pressed && !lastPressed) {
    magnetOn = !magnetOn;
    digitalWrite(DO_PIN, magnetOn ? HIGH : LOW);
    Serial.println(magnetOn ? "电磁铁开" : "电磁铁关");
    delay(50);  // 防抖
  }
  
  lastPressed = pressed;
}
