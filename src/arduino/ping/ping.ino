void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() >= 8)
  {
    uint8_t timestamp_buffer[8];
    Serial.readBytes(timestamp_buffer, 8);
    Serial.write(timestamp_buffer, 8);
  }
}
