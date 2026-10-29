// prop_drive.ino - Arduino Uno + L298N
// Reads "left,right\n" (PWM -255..255) at 115200 baud from the PC.
// Stops both motors if no command arrives for 400 ms (failsafe).
const int ENA = 5,  IN1 = 6, IN2 = 7;   // motor A (left value)
const int ENB = 10, IN3 = 8, IN4 = 9;   // motor B (right value)
const bool INVERT_A = false;
const bool INVERT_B = true;
const unsigned long FAILSAFE_MS = 400;

char buf[32];
byte idx = 0;
unsigned long lastCmd = 0;

void drive(int en, int a, int b, int pwm, bool invert) {
  if (invert) pwm = -pwm;
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0)      { digitalWrite(a, HIGH); digitalWrite(b, LOW);  }
  else if (pwm < 0) { digitalWrite(a, LOW);  digitalWrite(b, HIGH); }
  else              { digitalWrite(a, LOW);  digitalWrite(b, LOW);  }
  analogWrite(en, abs(pwm));
}

void stopAll() {
  drive(ENA, IN1, IN2, 0, false);
  drive(ENB, IN3, IN4, 0, false);
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopAll();
  Serial.begin(115200);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[idx] = '\0';
      char *comma = strchr(buf, ',');
      if (comma) {
        *comma = '\0';
        int l = atoi(buf);
        int r = atoi(comma + 1);
        drive(ENA, IN1, IN2, l, INVERT_A);
        drive(ENB, IN3, IN4, r, INVERT_B);
        lastCmd = millis();
      }
      idx = 0;
    } else if (c != '\r') {
      if (idx < sizeof(buf) - 1) buf[idx++] = c;
      else idx = 0;               // line too long: drop it
    }
  }
  if (millis() - lastCmd > FAILSAFE_MS) stopAll();
}
