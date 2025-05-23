int motorPins[3][3] = {
  {3, 4, 5},   // Motor1: EN, IN1, IN2
  {6, 7, 8},   // Motor2
  {9, 10, 11}  // Motor3
};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
    pinMode(motorPins[i][0], OUTPUT); // PWM
    pinMode(motorPins[i][1], OUTPUT); // IN1
    pinMode(motorPins[i][2], OUTPUT); // IN2
  }
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // "120,-100,50\n"
    input.trim();

    int speeds[3];
    int index = 0;
    char *ptr = strtok((char*)input.c_str(), ",");
    while (ptr != NULL && index < 3) {
      speeds[index++] = atoi(ptr);
      ptr = strtok(NULL, ",");
    }

    for (int i = 0; i < 3; i++) {
      int pwm = abs(speeds[i]);
      pwm = constrain(pwm, 0, 255);
      if (speeds[i] >= 0) {
        digitalWrite(motorPins[i][1], HIGH);
        digitalWrite(motorPins[i][2], LOW);
      } else {
        digitalWrite(motorPins[i][1], LOW);
        digitalWrite(motorPins[i][2], HIGH);
      }
      analogWrite(motorPins[i][0], pwm);
    }
  }
}
