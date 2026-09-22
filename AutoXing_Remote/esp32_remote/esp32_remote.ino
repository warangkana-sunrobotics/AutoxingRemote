#include <WiFi.h>
#include <HTTPClient.h>

// =========================
// Wi-Fi
// =========================
const char* WIFI_SSID = "Sunrobot_5G
const char* WIFI_PASSWORD = "Sunrobot12345";

// PC/server running remote_server.py.
// Example: http://192.168.1.100:5000/button
const char* SERVER_URL = "http://192.168.1.100:5000/button";

// =========================
// Buttons
// =========================
const uint8_t BUTTON_1_PIN = 18;
const uint8_t BUTTON_2_PIN = 19;
const uint8_t BUTTON_3_PIN = 21;

const unsigned long DEBOUNCE_MS = 60;
const unsigned long COMMAND_COOLDOWN_MS = 800;

struct ButtonState {
  uint8_t pin;
  int number;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeMs;
};

ButtonState buttons[] = {
  {BUTTON_1_PIN, 1, HIGH, HIGH, 0},
  {BUTTON_2_PIN, 2, HIGH, HIGH, 0},
  {BUTTON_3_PIN, 3, HIGH, HIGH, 0}
};

unsigned long lastCommandMs = 0;

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Connecting Wi-Fi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
    Serial.print(".");
  }

  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Wi-Fi connected. ESP32 IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi connection failed.");
  }
}

bool sendButton(int buttonNumber) {
  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  http.setTimeout(5000);

  if (!http.begin(SERVER_URL)) {
    Serial.println("HTTP begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  String body = String("{\"button\":") + buttonNumber + "}";

  Serial.print("Sending button ");
  Serial.println(buttonNumber);

  int code = http.POST(body);
  String response = http.getString();

  Serial.print("HTTP code: ");
  Serial.println(code);
  Serial.print("Server: ");
  Serial.println(response);

  http.end();
  return code >= 200 && code < 300;
}

void checkButton(ButtonState& b) {
  bool reading = digitalRead(b.pin);

  if (reading != b.lastReading) {
    b.lastChangeMs = millis();
    b.lastReading = reading;
  }

  if (millis() - b.lastChangeMs >= DEBOUNCE_MS && reading != b.stableState) {
    b.stableState = reading;

    // INPUT_PULLUP: LOW means pressed.
    if (b.stableState == LOW) {
      if (millis() - lastCommandMs >= COMMAND_COOLDOWN_MS) {
        sendButton(b.number);
        lastCommandMs = millis();
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  for (auto& b : buttons) {
    pinMode(b.pin, INPUT_PULLUP);
    b.lastReading = digitalRead(b.pin);
    b.stableState = b.lastReading;
  }

  connectWiFi();
}

void loop() {
  connectWiFi();

  for (auto& b : buttons) {
    checkButton(b);
  }

  delay(5);
}
