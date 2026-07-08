const uint8_t L_ENC_A = 2, L_ENC_B = 4;
const uint8_t R_ENC_A = 3, R_ENC_B = 5;
const uint8_t L_MOTOR_PWM = 9,  L_MOTOR_DIR = 7;
const uint8_t R_MOTOR_PWM = 10, R_MOTOR_DIR = 8;

volatile long left_count = 0;
volatile long right_count = 0;

void leftEncoderISR() {
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) left_count++;
  else left_count--;
}
void rightEncoderISR() {
  // 取り付け向きで符号が反転することがあるので実機で確認して調整
  if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) right_count--;
  else right_count++;
}

String inputLine = "";
unsigned long lastSendMs = 0;
unsigned long lastCommandMs = 0;
const unsigned long SEND_INTERVAL_MS = 20;     // 50Hz
const unsigned long COMMAND_TIMEOUT_MS = 500;  // 0.5秒来なければ停止

void setMotor(uint8_t pwmPin, uint8_t dirPin, int value) {
  value = constrain(value, -255, 255);
  digitalWrite(dirPin, value >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(value));
}

void parseCommand(const String &line) {
  if (line.length() < 3 || line[0] != 'M') return;
  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  if (c1 < 0 || c2 < 0) return;

  int leftVal  = line.substring(c1 + 1, c2).toInt();
  int rightVal = line.substring(c2 + 1).toInt();
  setMotor(L_MOTOR_PWM, L_MOTOR_DIR, leftVal);
  setMotor(R_MOTOR_PWM, R_MOTOR_DIR, rightVal);
  lastCommandMs = millis();
}

void setup() {
  Serial.begin(115200);
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  pinMode(L_MOTOR_PWM, OUTPUT); pinMode(L_MOTOR_DIR, OUTPUT);
  pinMode(R_MOTOR_PWM, OUTPUT); pinMode(R_MOTOR_DIR, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, CHANGE);

  Serial.println("READY");
  lastCommandMs = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { parseCommand(inputLine); inputLine = ""; }
    else inputLine += c;
  }

  // フェイルセーフ: コマンド途絶時は停止
  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    setMotor(L_MOTOR_PWM, L_MOTOR_DIR, 0);
    setMotor(R_MOTOR_PWM, R_MOTOR_DIR, 0);
  }

  unsigned long now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    long lc, rc;
    noInterrupts(); lc = left_count; rc = right_count; interrupts();
    Serial.print("E,"); Serial.print(lc); Serial.print(",");
    Serial.print(rc); Serial.print(","); Serial.println(now);
  }
}
