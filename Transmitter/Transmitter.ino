// ESP32 ESP-NOW transmitter for the drone.
// Two analog joysticks (Mode 2) + an arm switch.
//
//   Left stick:  Y = throttle, X = yaw
//   Right stick: Y = pitch,    X = roll
//   Arm switch:  GPIO -> GND when ON (internal pull-up)
//
// Sticks must be CENTRED (roll/pitch/yaw) at power-on: centres are calibrated then.
// Spring-centred joysticks make throttle awkward (it rests at half). A throttle
// stick with the centring spring removed, or a slide pot, is much better.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---- must match Drone/src/config.h + radio.h ----
#define RADIO_WIFI_CHANNEL 1
#define RADIO_MAGIC        0xD5A1
struct __attribute__((packed)) RcPacket {
  uint16_t magic;
  uint16_t seq;
  int16_t  throttle;  // 0..1000
  int16_t  roll;      // -500..500
  int16_t  pitch;     // -500..500, + = stick forward
  int16_t  yaw;       // -500..500
  uint8_t  flags;     // bit0 = arm
};
#define RC_FLAG_ARM 0x01

// ---- pins (ADC1 only! ADC2 does not work with Wi-Fi) ----
#define PIN_THROTTLE 34
#define PIN_YAW      35
#define PIN_PITCH    32
#define PIN_ROLL     33
#define PIN_ARM      25
#define PIN_LED      2

// Flip any axis that moves the wrong way.
#define INVERT_THROTTLE false
#define INVERT_YAW      false
#define INVERT_PITCH    false
#define INVERT_ROLL     false

// Throttle raw ADC range — print raw values ('r' on serial) and adjust.
#define THR_RAW_MIN 200
#define THR_RAW_MAX 3900

#define SEND_HZ 50

// Replace with the drone's MAC (printed at its boot) to stop talking to
// every ESP-NOW device nearby. Broadcast is fine for first tests.
uint8_t peerMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

int centreYaw, centrePitch, centreRoll;
uint16_t seq = 0;
bool printRaw = false;

int readAvg(int pin) {
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(pin);
  return sum / 4;
}

int16_t axis(int pin, int centre, bool invert) {
  int v = readAvg(pin) - centre;
  int span = v >= 0 ? (4095 - centre) : centre;
  if (span < 1) span = 1;
  long out = (long)v * 500 / span;
  out = constrain(out, -500, 500);
  return invert ? -out : out;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ARM, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  analogReadResolution(12);
  delay(300);

  long y = 0, p = 0, r = 0;
  for (int i = 0; i < 50; i++) {
    y += analogRead(PIN_YAW); p += analogRead(PIN_PITCH); r += analogRead(PIN_ROLL);
    delay(5);
  }
  centreYaw = y / 50; centrePitch = p / 50; centreRoll = r / 50;
  Serial.printf("Centres: yaw %d pitch %d roll %d\n", centreYaw, centrePitch, centreRoll);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(RADIO_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init failed");
    for (;;) delay(1000);
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = 0;  // use current channel
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peer) != ESP_OK) Serial.println("add_peer failed");

  Serial.printf("TX ready, MAC %s. Send 'r' to toggle raw ADC print.\n",
                WiFi.macAddress().c_str());
  if (digitalRead(PIN_ARM) == LOW)
    Serial.println("WARNING: arm switch is ON — the drone will refuse until you cycle it.");
}

void loop() {
  static uint32_t lastSend = 0, lastPrint = 0;
  uint32_t now = millis();

  if (Serial.available() && Serial.read() == 'r') printRaw = !printRaw;

  if (now - lastSend >= 1000 / SEND_HZ) {
    lastSend = now;
    RcPacket pkt;
    pkt.magic = RADIO_MAGIC;
    pkt.seq = seq++;

    int thrRaw = readAvg(PIN_THROTTLE);
    long thr = (long)(thrRaw - THR_RAW_MIN) * 1000 / (THR_RAW_MAX - THR_RAW_MIN);
    thr = constrain(thr, 0, 1000);
    if (INVERT_THROTTLE) thr = 1000 - thr;
    pkt.throttle = thr;

    pkt.yaw   = axis(PIN_YAW,   centreYaw,   INVERT_YAW);
    pkt.pitch = axis(PIN_PITCH, centrePitch, INVERT_PITCH);
    pkt.roll  = axis(PIN_ROLL,  centreRoll,  INVERT_ROLL);
    bool arm = digitalRead(PIN_ARM) == LOW;
    pkt.flags = arm ? RC_FLAG_ARM : 0;

    esp_now_send(peerMac, (uint8_t *)&pkt, sizeof(pkt));
    digitalWrite(PIN_LED, arm ? HIGH : LOW);

    if (now - lastPrint >= 200) {
      lastPrint = now;
      if (printRaw)
        Serial.printf("raw thr %d yaw %d pitch %d roll %d\n", thrRaw,
                      analogRead(PIN_YAW), analogRead(PIN_PITCH), analogRead(PIN_ROLL));
      else
        Serial.printf("thr %4d yaw %4d pitch %4d roll %4d arm %d\n",
                      pkt.throttle, pkt.yaw, pkt.pitch, pkt.roll, arm);
    }
  }
}
