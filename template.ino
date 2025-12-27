#include "project.h"

void setup() {
  Serial.begin(BAUD);
}

void loop() {
  char b = Serial.read();

  // echo!
  if (b != -1)
    Serial.print(b);
}
