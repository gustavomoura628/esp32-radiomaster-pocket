#include <Arduino.h>
#include "crsf_reader.hpp"

CRSFReader crsf;

enum Channels
{
  THROTTLE = 3,
  ROLL = 1,
  PITCH = 2,
  YAW = 4,
  ARMED = 5
};

uint16_t channels_min[16];
uint16_t channels_max[16];

void setup() {
  Serial.begin(115200);
  crsf.begin();
  Serial.println(F("CRSFReader ready"));

  for (int i = 0; i < 16; i++) channels_min[i] = UINT16_MAX;
  for (int i = 0; i < 16; i++) channels_max[i] = 0;
}

// Scale value to 0.0-1.0
float axis(int channel) {
  uint16_t value = crsf.get_channel(channel);
  channels_min[channel] = min(value, channels_min[channel]);
  channels_max[channel] = max(value, channels_max[channel]);
  return (value - channels_min[channel]) / static_cast<float>(channels_max[channel] - channels_min[channel]);
}

void loop() {
  if (crsf.update()) {
    float throttle = axis(THROTTLE);
    float roll     = axis(ROLL);
    float pitch    = axis(PITCH);
    float yaw      = axis(YAW);
    bool armed      = crsf.get_channel(ARMED) > 1500; // 1500 is the threshold for arming
    Serial.printf("Throttle: %.2f, Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Armed: %s\n",
                  throttle, roll, pitch, yaw, armed ? "Yes" : "No");
  }
}
