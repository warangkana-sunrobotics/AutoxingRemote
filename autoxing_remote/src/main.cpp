#include <Arduino.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <IRremote.hpp>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <string.h>
#include <vector>

// --- ตั้งค่า Wi-Fi ---
const char* ssid = "Sunrobot_5G";     // SSID Wi-Fi ของหุ่นยนต์
const char* password = "Sunrobot12345";

// --- ตั้งค่า Autoxing Robot ---
const char* robot_ip = "192.168.100.55";       // IP ของหุ่นยนต์ (หรือ 192.168.12.1)
const int robot_port = 8090;
const char* robot_secret = ""; // ใส่ Secret Key จริงถ้าหุ่นกำหนดให้ใช้

WebSocketsClient robotWebSocket;
bool webSocketConnected = false;
bool twistFeedbackReceived = false;
unsigned long lastTwistSendTime = 0;
const unsigned long TWIST_SEND_INTERVAL = 150;

enum ControlMode { MODE_IDLE, MODE_MANUAL, MODE_NAVIGATION };
ControlMode controlMode = MODE_IDLE;
bool navigationActive = false;
float manualSpeed = 0.30f;
const float MIN_MANUAL_SPEED = 0.10f;
const float MAX_MANUAL_SPEED = 1.50f;
const float SPEED_STEP = 0.05f;
const float TURN_SPEED = 0.50f;
const uint8_t CHARGE_RETRY_COUNT = 3;

struct PointTarget {
  String name;
  float x;
  float y;
  float ori;
  const char* moveType;
  bool configured;
};

// Populated from the current map's point overlays at startup.
std::vector<PointTarget> pointTargets;
std::vector<PointTarget> navigationQueue;
const size_t MAX_NAVIGATION_QUEUE = 20;

void startNextQueuedMove();

// --- ตั้งค่า IR Receiver ---
const int IR_RECEIVE_PIN = 15;

// --- รหัสปุ่ม IR Remote ตามรูปภาพ ---
#define CODE_UP       0xE718FF00
#define CODE_DOWN     0xAD52FF00
#define CODE_LEFT     0xF708FF00
#define CODE_RIGHT    0xA55AFF00
#define CODE_OK       0xE31CFF00

#define CODE_DECREASE 0xE916FF00
#define CODE_INCREASE 0xF20DFF00

#define CODE_0        0xE619FF00
#define CODE_1        0xBA45FF00
#define CODE_2        0xB946FF00
#define CODE_3        0xB847FF00
#define CODE_4        0xBB44FF00
#define CODE_5        0xBF40FF00
#define CODE_6        0xBC43FF00
#define CODE_7        0xF807FF00
#define CODE_8        0xEA15FF00
#define CODE_9        0xF609FF00

// --- ขาของบอร์ด AutoXing Remote PCB rev 1.0 (ดู AutoxingRemote_schematic_full.pdf) ---
const int PIN_I2C_SDA = 21;     // LCD SDA ผ่าน level shifter Q1
const int PIN_I2C_SCL = 22;     // LCD SCL ผ่าน level shifter Q2
const int PIN_JOY_X = 34;       // Joystick VRX (ADC1)
const int PIN_JOY_Y = 35;       // Joystick VRY (ADC1)
const int PIN_JOY_SW = 39;      // Joystick SW, active LOW, pull-up R3 บนบอร์ด
const int PIN_LED_GREEN = 19;   // D1
const int PIN_LED_RED = 18;     // D2
const int PIN_VBAT = 36;        // VBAT/2 จาก R1/R2 (ADC1)

// --- Keypad 4x4: R1-R4 = IO32,33,25,26 / C1-C4 = IO27,14,13,4 ---
const byte KEYPAD_ROWS = 4;
const byte KEYPAD_COLS = 4;
char keypadKeys[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'},
};
byte keypadRowPins[KEYPAD_ROWS] = {32, 33, 25, 26};
byte keypadColPins[KEYPAD_COLS] = {27, 14, 13, 4};
Keypad keypad = Keypad(makeKeymap(keypadKeys), keypadRowPins, keypadColPins, KEYPAD_ROWS, KEYPAD_COLS);

// --- LCD 1602 I2C (backpack PCF8574 ที่ 0x27 หรือ 0x3F ตรวจหาเองตอนบูต) ---
LiquidCrystal_I2C* lcd = nullptr;
String lcdLines[2];
String lcdMessage = "";
unsigned long lcdMessageUntil = 0;

// --- Joystick ---
const int JOY_DEADZONE = 350;             // ADC counts (เต็มสเกล 4095)
const bool JOY_INVERT_X = false;          // กลับทิศถ้าโยกขวาแล้วหุ่นเลี้ยวซ้าย
const bool JOY_INVERT_Y = true;           // กลับทิศถ้าดันขึ้นแล้วหุ่นถอยหลัง
int joyCenterX = 2048;
int joyCenterY = 2048;
bool joystickActive = false;              // กำลังขับด้วย joystick อยู่
bool joystickWaitCenter = false;          // หลัง STOP ต้องปล่อยคันโยกกลับกลางก่อนถึงจะขับต่อได้
unsigned long lastJoyModeAttempt = 0;

// --- Battery / LED ---
float batteryVoltage = 0.0f;
const float BATTERY_LOW_VOLTAGE = 3.40f;
unsigned long stopFlashUntil = 0;
String currentTargetName = "";

void showMessage(const String& text, unsigned long durationMs = 2000);
void emergencyStop();

// --- ตัวแปรจัดการสถานะ ---
String inputBuffer = "";                      // บัฟเฟอร์เก็บตัวเลขที่พิมพ์
unsigned long lastOkTime = 0;                 // เวลาที่กด OK ครั้งล่าสุด (ใช้ตรวจเช็ก Double Click)
const unsigned long DOUBLE_CLICK_TIME = 600;  // ระยะเวลากด OK 2 ครั้งติดกัน (มิลลิวินาที)

// --- ตัวแปรสำหรับ Wi-Fi reconnect (เพิ่มใหม่) ---
unsigned long lastWifiRetryTime = 0;
const unsigned long WIFI_RETRY_INTERVAL = 5000; // ถ้า Wi-Fi หลุด ลองต่อใหม่ทุก 5 วินาที

// --- ตัวแปรสำหรับปุ่มทิศทางแบบกดค้าง (เพิ่มใหม่) ---
uint32_t lastDirectionCode = 0;                       // ปุ่มทิศทางที่กำลังถูกกดค้างอยู่ (0 = ไม่มี)
unsigned long lastDirectionSignalTime = 0;            // เวลาที่ได้รับสัญญาณทิศทางล่าสุด (กดแรกหรือ repeat)
const unsigned long DIRECTION_RELEASE_TIMEOUT = 500;  // รองรับช่วงห่างของ repeat frame จากรีโมต

// --- ฟังก์ชันส่งคำสั่ง HTTP ไปยังหุ่นยนต์ ---

bool setRobotControlMode(const char* mode) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  String url = "http://" + String(robot_ip) + ":" + String(robot_port) +
               "/services/wheel_control/set_control_mode";
  if (!http.begin(url)) return false;
  http.addHeader("Content-Type", "application/json");
  if (String(robot_secret) != "") http.addHeader("Secret", robot_secret);
  // `mode` is used by the remote-control guide; `control_mode` is the
  // field name in the Service API schema. Sending both supports either API version.
  String body = String("{\"mode\":\"") + mode + "\",\"control_mode\":\"" + mode + "\"}";
  int response = http.POST(body);
  Serial.printf("[MODE] %s | Response: %d\n", mode, response);
  http.end();
  return response >= 200 && response < 300;
}

void sendManualControl(float linearX, float angularZ);
void sendRobotStopCommand();

bool cancelCurrentNavigation() {
  HTTPClient http;
  String url = "http://" + String(robot_ip) + ":" + String(robot_port) +
               "/chassis/moves/current";
  if (http.begin(url)) {
    http.addHeader("Content-Type", "application/json");
    if (String(robot_secret) != "") http.addHeader("Secret", robot_secret);
    int response = http.sendRequest("PATCH", "{\"state\":\"cancelled\"}");
    Serial.printf("[NAV] Cancel current move | Response: %d\n", response);
    http.end();
    return response >= 200 && response < 300;
  }
  return false;
}

PointTarget* findPointTarget(const String& name) {
  for (PointTarget& target : pointTargets) {
    if (target.name == name) return &target;
  }
  return nullptr;
}

bool loadPointTargetsFromRobot() {
  const String baseUrl = "http://" + String(robot_ip) + ":" + String(robot_port);
  HTTPClient http;
  if (!http.begin(baseUrl + "/chassis/current-map")) return false;
  if (String(robot_secret) != "") http.addHeader("Secret", robot_secret);
  int response = http.GET();
  String currentMapJson = response >= 200 && response < 300 ? http.getString() : "";
  http.end();
  if (currentMapJson.length() == 0) {
    Serial.printf("[POI] GET current map failed: %d\n", response);
    return false;
  }

  DynamicJsonDocument currentMapDoc(2048);
  DeserializationError error = deserializeJson(currentMapDoc, currentMapJson);
  if (error) {
    Serial.printf("[POI] Current map JSON error: %s\n", error.c_str());
    return false;
  }
  int mapId = currentMapDoc["id"] | -1;
  if (mapId < 0) {
    Serial.println("[POI] Current map has no saved map ID");
    return false;
  }

  if (!http.begin(baseUrl + "/maps/" + String(mapId))) return false;
  if (String(robot_secret) != "") http.addHeader("Secret", robot_secret);
  response = http.GET();
  String mapJson = response >= 200 && response < 300 ? http.getString() : "";
  http.end();
  if (mapJson.length() == 0) {
    Serial.printf("[POI] GET map %d failed: %d\n", mapId, response);
    return false;
  }

  // Map responses contain the overlays FeatureCollection as an escaped JSON
  // string, so their ArduinoJson memory use is substantially larger than the
  // small current-map response.
  DynamicJsonDocument mapDoc(12288);
  error = deserializeJson(mapDoc, mapJson);
  if (error) {
    Serial.printf("[POI] Map JSON error: %s (payload %u bytes, doc capacity %u)\n",
                  error.c_str(), (unsigned)mapJson.length(),
                  (unsigned)mapDoc.capacity());
    return false;
  }
  const char* overlaysJson = mapDoc["overlays"] | "";
  if (!overlaysJson[0]) {
    Serial.println("[POI] The map has no overlays");
    return false;
  }

  DynamicJsonDocument overlaysDoc(16384);
  error = deserializeJson(overlaysDoc, overlaysJson);
  if (error) {
    Serial.printf("[POI] Overlays JSON error: %s\n", error.c_str());
    return false;
  }

  int loadedCount = 0;
  const char* mapName = mapDoc["map_name"] | "(unnamed)";
  std::vector<PointTarget> fetchedTargets;
  JsonArray features = overlaysDoc["features"].as<JsonArray>();
  for (JsonObject feature : features) {
    const char* geometryType = feature["geometry"]["type"] | "";
    const char* pointName = feature["properties"]["name"] | "";
    if (strcmp(geometryType, "Point") != 0 || pointName[0] == '\0') continue;
    bool numberedTarget = true;
    for (const char* c = pointName; *c; ++c) {
      if (*c < '0' || *c > '9') {
        numberedTarget = false;
        break;
      }
    }
    bool standbyTarget = strcmp(pointName, "Standby Point") == 0;
    bool chargingTarget = strcmp(pointName, "Charging Station") == 0;
    if (!numberedTarget && !standbyTarget && !chargingTarget) continue;

    JsonArray coordinates = feature["geometry"]["coordinates"].as<JsonArray>();
    if (coordinates.size() < 2) continue;

    PointTarget target;
    target.name = pointName;
    target.x = coordinates[0].as<float>();
    target.y = coordinates[1].as<float>();
    target.moveType = chargingTarget ? "charge" : "standard";
    target.configured = true;
    // Overlay yaw is in degrees; Move API target_ori is in radians.
    JsonVariant yawValue = feature["properties"]["yaw"];
    float yawDegrees = 0.0f;
    if (yawValue.is<const char*>()) yawDegrees = String(yawValue.as<const char*>()).toFloat();
    else if (!yawValue.isNull()) yawDegrees = yawValue.as<float>();
    target.ori = yawDegrees * DEG_TO_RAD;
    fetchedTargets.push_back(target);
    ++loadedCount;
    Serial.printf("[POI] Loaded %s at (%.3f, %.3f)\n", target.name.c_str(), target.x, target.y);
  }

  pointTargets.swap(fetchedTargets);
  Serial.printf("[POI] Loaded %d targets from map %d (%s)\n", loadedCount, mapId, mapName);
  return loadedCount > 0;
}

bool sendRobotMoveCommand(const PointTarget& target) {
  if (!target.configured) {
    Serial.printf("[NAV] Target '%s' was not found in current map overlays\n", target.name.c_str());
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[NAV] Wi-Fi disconnected");
    return false;
  }

  // Explicitly stop remote velocity control before switching to autonomous navigation.
  if (controlMode == MODE_MANUAL && webSocketConnected) {
    sendRobotStopCommand();
    delay(120);
  }
  if (!setRobotControlMode("auto")) return false;

  HTTPClient http;
  String url = "http://" + String(robot_ip) + ":" + String(robot_port) + "/chassis/moves";
  if (!http.begin(url)) return false;
  http.addHeader("Content-Type", "application/json");
  if (String(robot_secret) != "") http.addHeader("Secret", robot_secret);
  String body;
  if (strcmp(target.moveType, "charge") == 0) {
    body = "{\"type\":\"charge\",\"creator\":\"esp32-ir-remote\",\"charge_retry_count\":" +
           String(CHARGE_RETRY_COUNT) + "}";
  } else {
    body = "{\"type\":\"standard\",\"creator\":\"esp32-ir-remote\",\"target_x\":" +
           String(target.x, 3) + ",\"target_y\":" + String(target.y, 3) +
           ",\"target_z\":0,\"target_ori\":" + String(target.ori, 3) + "}";
  }
  int response = http.POST(body);
  String responseBody = http.getString();
  Serial.printf("[NAV] Target: %s (%.2f, %.2f) | Response: %d\n",
                target.name.c_str(), target.x, target.y, response);
  if (responseBody.length() > 0) {
    Serial.printf("[NAV] API response: %s\n", responseBody.c_str());
  }
  if (response >= 200 && response < 300) {
    controlMode = MODE_NAVIGATION;
    navigationActive = true;
    inputBuffer = "";
    currentTargetName = target.name;
  } else {
    showMessage("Move failed " + String(response));
  }
  http.end();
  return response >= 200 && response < 300;
}

void startNextQueuedMove() {
  if (navigationActive || navigationQueue.empty()) return;
  PointTarget nextTarget = navigationQueue.front();
  navigationQueue.erase(navigationQueue.begin());
  Serial.printf("[QUEUE] Starting %s; %u target(s) remain\n",
                nextTarget.name.c_str(), (unsigned)navigationQueue.size());
  if (!sendRobotMoveCommand(nextTarget)) {
    navigationQueue.clear();
    Serial.println("[QUEUE] Target failed to start; remaining queue cleared");
  }
}
// Send zero velocity to stop manual movement.
void sendRobotStopCommand() {
  sendManualControl(0.0, 0.0);
}



void onWebSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    webSocketConnected = true;
    twistFeedbackReceived = true;
    robotWebSocket.sendTXT("{\"enable_topic\":\"/planning_state\"}");
    Serial.println("Robot WebSocket connected");
  } else if (type == WStype_DISCONNECTED) {
    webSocketConnected = false;
    twistFeedbackReceived = false;
    Serial.println("Robot WebSocket disconnected");
  } else if (type == WStype_TEXT && length > 0) {
    // payload is length-delimited; do not assume the WebSocket library null-terminates it.
    String message;
    message.reserve(length);
    for (size_t i = 0; i < length; ++i) message += static_cast<char>(payload[i]);
    if (message.indexOf("twist_feedback") >= 0) {
      twistFeedbackReceived = true;
    }
    if (message.indexOf("/planning_state") >= 0) {
      DynamicJsonDocument stateDoc(1024);
      if (deserializeJson(stateDoc, message) == DeserializationError::Ok &&
          String((const char*)(stateDoc["topic"] | "")) == "/planning_state") {
        const char* moveState = stateDoc["move_state"] | "";
        if (strcmp(moveState, "moving") == 0) {
          navigationActive = true;
        } else if (strcmp(moveState, "idle") == 0 || strcmp(moveState, "succeeded") == 0 ||
                   strcmp(moveState, "failed") == 0 || strcmp(moveState, "cancelled") == 0) {
          if (navigationActive) Serial.printf("[NAV] Finished: %s\n", moveState);
          navigationActive = false;
          if (controlMode == MODE_NAVIGATION) controlMode = MODE_IDLE;
        }
      }
    }
  } else if (type == WStype_ERROR) {
    webSocketConnected = false;
    twistFeedbackReceived = false;
    Serial.println("Robot WebSocket error");
  }
}

bool setRemoteControlMode() { return setRobotControlMode("remote"); }

// AutoXing receives manual velocity commands on the /twist WebSocket topic.
void sendManualControl(float linearX, float angularZ) {
  if (!webSocketConnected) {
    Serial.println("[MANUAL] WebSocket disconnected; command skipped");
    return;
  }
  String payload = "{\"topic\":\"/twist\",\"linear_velocity\":" + String(linearX, 3) +
                   ",\"angular_velocity\":" + String(angularZ, 3) + "}";
  twistFeedbackReceived = false;
  lastTwistSendTime = millis();
  robotWebSocket.sendTXT(payload);
  if (linearX == 0.0f && angularZ == 0.0f) controlMode = MODE_IDLE;
  else controlMode = MODE_MANUAL;
  Serial.printf("[MANUAL] Linear: %.2f, Angular: %.2f\n", linearX, angularZ);
}



// 4. ส่งคำสั่งทิศทางซ้ำตามโค้ดปุ่ม ใช้ตอนกดครั้งแรกและตอนกดค้าง (repeat) (เพิ่มใหม่)

void resendDirectionCommand(uint32_t code) {

  switch (code) {

    case CODE_UP:    sendManualControl(manualSpeed, 0.0);  break; // เดินหน้า

    case CODE_DOWN:  sendManualControl(-manualSpeed, 0.0); break; // ถอยหลัง

    case CODE_LEFT:  sendManualControl(0.0, TURN_SPEED);  break; // เลี้ยวซ้าย

    case CODE_RIGHT: sendManualControl(0.0, -TURN_SPEED); break; // เลี้ยวขวา

    default: break;

  }

}

bool enterManualMode() {
  if (navigationActive || controlMode == MODE_NAVIGATION) {
    Serial.println("[MANUAL] Locked during navigation. Press OK twice to cancel first.");
    return false;
  }
  if (!setRemoteControlMode()) return false;
  controlMode = MODE_MANUAL;
  return true;
}

void changeManualSpeed(float delta) {
  if (navigationActive || controlMode == MODE_NAVIGATION) {
    Serial.println("[SPEED] Speed adjustment is unavailable during navigation");
    return;
  }
  manualSpeed = constrain(manualSpeed + delta, MIN_MANUAL_SPEED, MAX_MANUAL_SPEED);
  Serial.printf("[SPEED] Linear speed: %.2f m/s\n", manualSpeed);
}



// --- เช็ก/ต่อ Wi-Fi ใหม่แบบไม่บล็อกโปรแกรม (เพิ่มใหม่) ---

void checkWifiConnection() {

  if (WiFi.status() != WL_CONNECTED) {

    unsigned long currentTime = millis();

    if (currentTime - lastWifiRetryTime >= WIFI_RETRY_INTERVAL) {

      lastWifiRetryTime = currentTime;

      Serial.println("Wi-Fi หลุด กำลังลองเชื่อมต่อใหม่...");

      WiFi.disconnect();

      WiFi.begin(ssid, password);

    }

  }

}



// --- หยุดฉุกเฉิน: ใช้ร่วมกันระหว่าง OK 2 ครั้ง (IR / ปุ่ม #), ปุ่ม D และปุ่มกด joystick ---

void emergencyStop() {
  Serial.println(">> EMERGENCY STOP: clearing queued moves");
  navigationQueue.clear();
  inputBuffer = "";
  // Prevent the held-direction resend loop and the joystick from issuing another
  // velocity command after the emergency stop request.
  lastDirectionCode = 0;
  lastDirectionSignalTime = 0;
  joystickActive = false;
  joystickWaitCenter = true;
  stopFlashUntil = millis() + 1500;
  if (navigationActive || controlMode == MODE_NAVIGATION) {
    if (cancelCurrentNavigation()) {
      navigationActive = false;
      setRobotControlMode("auto");
      controlMode = MODE_IDLE;
      Serial.println("[NAV] Current move cancelled; queue cleared; manual unlocked");
      showMessage("STOP: nav cancel");
    } else {
      Serial.println("[NAV] Cancel not confirmed; queue cleared, manual remains locked");
      showMessage("STOP: not confirm");
    }
  } else {
    if (controlMode == MODE_MANUAL) sendRobotStopCommand();
    showMessage("STOP");
  }
}



// --- ฟังก์ชันประมวลผลการกดปุ่ม OK ---

void handleOkButton() {

  unsigned long currentTime = millis();



  // เช็กว่าเป็นการกด OK ครั้งที่ 2 ติดต่อกันหรือไม่ (Double Click -> STOP)

  if (lastOkTime != 0 && currentTime - lastOkTime < DOUBLE_CLICK_TIME) {

    Serial.println(">> EMERGENCY STOP: Double OK detected");
    emergencyStop();

    lastOkTime = 0;

    return;

  }



  lastOkTime = currentTime;



  // หากมีการป้อนตัวเลขค้างไว้ก่อนกด OK 1 ครั้ง

  if (inputBuffer.length() > 0) {
    // Refresh the map targets on selection so newly added numbered points are usable
    // without editing code or rebooting the ESP32.
    loadPointTargetsFromRobot();
    String targetName = inputBuffer;
    if (inputBuffer == "0" && findPointTarget("0") == nullptr)
      targetName = "Standby Point";
    else if (inputBuffer == "1234" && findPointTarget("1234") == nullptr)
      targetName = "Charging Station";
    PointTarget* target = findPointTarget(targetName);
    if (target != nullptr) {
      if (navigationQueue.size() >= MAX_NAVIGATION_QUEUE) {
        Serial.println("[QUEUE] Queue full; maximum is 20 destinations");
        showMessage("Queue full (20)");
      } else {
        navigationQueue.push_back(*target);
        Serial.printf("[QUEUE] Added %s; %u destination(s) queued\n",
                      target->name.c_str(), (unsigned)navigationQueue.size());
        showMessage("Queued " + target->name);
        lastOkTime = 0;  // Selecting a target is not the first emergency-stop click.
        if (!navigationActive) startNextQueuedMove();
      }
      // The OK used to select a destination must not count as the first stop click.
      lastOkTime = 0;
    } else {
      Serial.printf("[NAV] Target '%s' was not loaded from the current map\n", targetName.c_str());
      showMessage("No point " + targetName);
    }

    inputBuffer = ""; // ล้างบัฟเฟอร์ตัวเลขหลังจากกด OK เคลื่อนที่แล้ว

  }

}



// --- ป้อนตัวเลขจุดหมาย (ใช้ร่วมกันระหว่างรีโมต IR และ keypad) ---

void prepareForDigitEntry() {
  // Number keys may append destinations while a move is running.
  // They only stop manual mode; they never cancel active navigation.
  if (!navigationActive && controlMode == MODE_MANUAL) sendRobotStopCommand();
  if (!navigationActive && controlMode == MODE_NAVIGATION) {
    setRobotControlMode("auto");
    controlMode = MODE_IDLE;
  }
  lastDirectionCode = 0;
  joystickActive = false;
}

void appendDigit(char digit) {
  if (inputBuffer.length() >= 8) return;  // ชื่อจุดยาวสุด 8 หลัก (พอดีจอ LCD)
  inputBuffer += digit;
  Serial.println("Buffer: " + inputBuffer);
}



// --- Keypad 4x4 ---
// 0-9 = ใส่หมายเลขจุด | # = OK (กด 2 ครั้งเร็ว = STOP) | * = ลบตัวท้าย
// A = เพิ่มความเร็ว | B = ลดความเร็ว | C = ล้างตัวเลข | D = STOP ทันที

void handleKeypad() {
  char key = keypad.getKey();
  if (!key) return;
  Serial.printf("[KEYPAD] %c\n", key);
  if (key >= '0' && key <= '9') {
    prepareForDigitEntry();
    appendDigit(key);
    return;
  }
  switch (key) {
    case '#': handleOkButton(); break;
    case '*':
      if (inputBuffer.length() > 0) inputBuffer.remove(inputBuffer.length() - 1);
      break;
    case 'A': changeManualSpeed(SPEED_STEP); showMessage("Speed " + String(manualSpeed, 2) + " m/s"); break;
    case 'B': changeManualSpeed(-SPEED_STEP); showMessage("Speed " + String(manualSpeed, 2) + " m/s"); break;
    case 'C': inputBuffer = ""; break;
    case 'D': emergencyStop(); break;
    default: break;
  }
}



// --- Joystick: ขับแบบ proportional ด้วย /twist, กดปุ่มที่คันโยก = STOP ---

void calibrateJoystick() {
  long sumX = 0, sumY = 0;
  for (int i = 0; i < 32; ++i) {
    sumX += analogRead(PIN_JOY_X);
    sumY += analogRead(PIN_JOY_Y);
    delay(2);
  }
  joyCenterX = sumX / 32;
  joyCenterY = sumY / 32;
  Serial.printf("[JOY] Center X=%d Y=%d\n", joyCenterX, joyCenterY);
}

// คืนค่า -1..1 จากค่า ADC โดยตัด deadzone รอบจุดกลาง
float joystickAxis(int raw, int center) {
  int delta = raw - center;
  if (abs(delta) < JOY_DEADZONE) return 0.0f;
  float span = delta > 0 ? (4095 - center - JOY_DEADZONE) : (center - JOY_DEADZONE);
  if (span < 1) return 0.0f;
  float value = (delta > 0 ? delta - JOY_DEADZONE : delta + JOY_DEADZONE) / span;
  return constrain(value, -1.0f, 1.0f);
}

void handleJoystick() {
  static unsigned long lastRead = 0;
  static uint8_t lowCount = 0;
  static bool buttonHeld = false;
  if (millis() - lastRead < 50) return;
  lastRead = millis();

  // GPIO39 อาจมี glitch สั้น ๆ ตอน WiFi/ADC ทำงาน จึงต้องอ่าน LOW ติดกัน 2 ครั้งก่อน
  if (digitalRead(PIN_JOY_SW) == LOW) {
    if (lowCount < 2) ++lowCount;
  } else {
    lowCount = 0;
    buttonHeld = false;
  }
  if (lowCount >= 2 && !buttonHeld) {
    buttonHeld = true;
    Serial.println("[JOY] Button -> STOP");
    emergencyStop();
    return;
  }

  float x = joystickAxis(analogRead(PIN_JOY_X), joyCenterX);
  float y = joystickAxis(analogRead(PIN_JOY_Y), joyCenterY);
  if (JOY_INVERT_X) x = -x;
  if (JOY_INVERT_Y) y = -y;
  bool deflected = x != 0.0f || y != 0.0f;

  if (!deflected) {
    joystickWaitCenter = false;
    if (joystickActive) {
      Serial.println("[JOY] Released -> STOP");
      sendRobotStopCommand();
      joystickActive = false;
    }
    return;
  }
  // รีโมต IR กำลังสั่งอยู่ หรือเพิ่งกด STOP แล้วยังไม่ปล่อยคันโยกกลับกลาง
  if (joystickWaitCenter || lastDirectionCode != 0) return;

  if (!joystickActive) {
    if (millis() - lastJoyModeAttempt < 1000) return;  // อย่ายิง HTTP ถี่ ๆ ถ้าล้มเหลว
    lastJoyModeAttempt = millis();
    if (!enterManualMode()) {
      showMessage(navigationActive ? "Nav running" : "Manual failed");
      return;
    }
    joystickActive = true;
  }
  // รอ feedback จากหุ่นก่อนส่งค่าใหม่ (เหมือนปุ่มทิศทาง IR) แต่ไม่รอเกิน 500 ms
  bool ready = twistFeedbackReceived || millis() - lastTwistSendTime > 500;
  if (webSocketConnected && ready && millis() - lastTwistSendTime >= TWIST_SEND_INTERVAL) {
    sendManualControl(y * manualSpeed, -x * TURN_SPEED);
  }
}



// --- แบตเตอรี่: VBAT/2 ที่ IO36 ---

void updateBattery() {
  static unsigned long lastRead = 0;
  if (lastRead != 0 && millis() - lastRead < 1000) return;
  lastRead = millis();
  uint32_t mv = 0;
  for (int i = 0; i < 8; ++i) mv += analogReadMilliVolts(PIN_VBAT);
  float volts = (mv / 8.0f) * 2.0f / 1000.0f;
  batteryVoltage = batteryVoltage == 0.0f ? volts : batteryVoltage * 0.8f + volts * 0.2f;
}

int batteryPercent() {
  return constrain((int)((batteryVoltage - 3.30f) / (4.20f - 3.30f) * 100.0f), 0, 100);
}



// --- LED: เขียว = ต่อหุ่นได้ (กะพริบ = กำลังนำทาง), แดง = ออฟไลน์ / STOP / แบตต่ำ ---

void updateLeds() {
  bool online = WiFi.status() == WL_CONNECTED && webSocketConnected;
  bool fastBlink = (millis() / 150) % 2;
  bool slowBlink = (millis() / 500) % 2;
  bool green = navigationActive ? slowBlink : online;
  bool red = !online || (millis() < stopFlashUntil && fastBlink) ||
             (batteryVoltage > 0.5f && batteryVoltage < BATTERY_LOW_VOLTAGE && slowBlink);
  digitalWrite(PIN_LED_GREEN, green ? HIGH : LOW);
  digitalWrite(PIN_LED_RED, red ? HIGH : LOW);
}



// --- LCD 1602 ---

void initLcd() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  uint8_t address = 0;
  for (uint8_t candidate : {0x27, 0x3F}) {
    Wire.beginTransmission(candidate);
    if (Wire.endTransmission() == 0) {
      address = candidate;
      break;
    }
  }
  if (address == 0) {
    Serial.println("[LCD] Not found at 0x27 / 0x3F");
    return;
  }
  lcd = new LiquidCrystal_I2C(address, 16, 2);
  lcd->init();
  lcd->backlight();
  Serial.printf("[LCD] Found at 0x%02X\n", address);
}

void lcdWriteLine(uint8_t row, String text) {
  if (lcd == nullptr) return;
  if (text.length() > 16) text = text.substring(0, 16);
  while (text.length() < 16) text += ' ';
  if (text == lcdLines[row]) return;  // เขียนเฉพาะเมื่อเปลี่ยน กันจอกระพริบ
  lcdLines[row] = text;
  lcd->setCursor(0, row);
  lcd->print(text);
}

void showMessage(const String& text, unsigned long durationMs) {
  lcdMessage = text;
  lcdMessageUntil = millis() + durationMs;
}

void updateLcd() {
  static unsigned long lastUpdate = 0;
  if (lcd == nullptr || millis() - lastUpdate < 200) return;
  lastUpdate = millis();

  String top;
  if (WiFi.status() != WL_CONNECTED) top = "WiFi connecting";
  else if (!webSocketConnected) top = "Robot offline";
  else if (navigationActive) top = "NAV " + currentTargetName +
                                   (navigationQueue.empty() ? "" : " +" + String(navigationQueue.size()));
  else if (controlMode == MODE_MANUAL) top = "MANUAL " + String(manualSpeed, 2) + "m/s";
  else top = "READY  " + String(manualSpeed, 2) + "m/s";
  lcdWriteLine(0, top);

  String bottom;
  if (millis() < lcdMessageUntil) bottom = lcdMessage;
  else if (inputBuffer.length() > 0) bottom = "Go to: " + inputBuffer + "_";
  else if (batteryVoltage > 0.5f) bottom = "Bat " + String(batteryVoltage, 2) + "V " + String(batteryPercent()) + "%";
  else bottom = "";
  lcdWriteLine(1, bottom);
}



void setup() {

  Serial.begin(9600);

  // --- ฮาร์ดแวร์บนบอร์ด ---
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  digitalWrite(PIN_LED_RED, HIGH);         // แดงติดระหว่างบูต/ต่อ Wi-Fi
  pinMode(PIN_JOY_SW, INPUT);              // GPIO39 ไม่มี pull-up ภายใน (มี R3 บนบอร์ด)
  analogSetPinAttenuation(PIN_JOY_X, ADC_11db);
  analogSetPinAttenuation(PIN_JOY_Y, ADC_11db);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  initLcd();
  lcdWriteLine(0, "AutoXing Remote");
  lcdWriteLine(1, "Starting...");
  calibrateJoystick();                     // ต้องปล่อยคันโยกไว้ตรงกลางตอนเปิดเครื่อง
  updateBattery();



  // เชื่อมต่อ Wi-Fi

  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");
  lcdWriteLine(1, "WiFi " + String(ssid));

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);

    Serial.print(".");

  }

  Serial.println("\nConnected to Wi-Fi!");
  lcdWriteLine(1, "Loading map...");

  if (!loadPointTargetsFromRobot()) showMessage("Map load failed", 3000);
  setRemoteControlMode();
  robotWebSocket.begin(robot_ip, robot_port, "/ws/v2/topics");
  robotWebSocket.onEvent(onWebSocketEvent);
  robotWebSocket.setReconnectInterval(3000);



  // เริ่มต้นตัวรับ IR

  // ปิด LED feedback: LED_BUILTIN ของบาง board definition ตรงกับ IO13 (keypad C3)
  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

  Serial.println("IR Receiver Ready. Waiting for signals...");

}



void loop() {

  robotWebSocket.loop();

  // เช็ก/ต่อ Wi-Fi ใหม่แบบไม่บล็อก (เพิ่มใหม่)

  checkWifiConnection();



  if (IrReceiver.decode()) {

    // สัญญาณ "กดค้าง" (repeat): บางเวอร์ชันไลบรารีจะได้ flag IS_REPEAT,

    // แต่บางกรณี decodedRawData จะเป็น 0x0 (ตามที่คอมเมนต์เดิมสังเกตไว้) — เช็กทั้งสองทางกันพลาด

    bool isRepeat = (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) ||

                     (IrReceiver.decodedIRData.decodedRawData == 0);

    uint32_t rawCode = IrReceiver.decodedIRData.decodedRawData;



    if (isRepeat) {

      // กดค้างปุ่มทิศทางอยู่ -> ส่งคำสั่งเดิมซ้ำ กัน watchdog ฝั่งหุ่นยนต์ตัดการเคลื่อนที่

      if (lastDirectionCode != 0) {

        lastDirectionSignalTime = millis();


      }

    } else if (rawCode != 0) {

      Serial.printf("IR Received Code: 0x%08X\n", rawCode);



      bool isDirectionKey = rawCode == CODE_UP || rawCode == CODE_DOWN ||
                            rawCode == CODE_LEFT || rawCode == CODE_RIGHT;
      bool isSpeedKey = rawCode == CODE_INCREASE || rawCode == CODE_DECREASE;
      bool isNavigationKey = rawCode == CODE_0 || rawCode == CODE_1 || rawCode == CODE_2 ||
                             rawCode == CODE_3 || rawCode == CODE_4 || rawCode == CODE_5 ||
                             rawCode == CODE_6 || rawCode == CODE_7 || rawCode == CODE_8 ||
                             rawCode == CODE_9;
      bool repeatedDirectionFrame = isDirectionKey && rawCode == lastDirectionCode &&
                                    controlMode == MODE_MANUAL;
      if (repeatedDirectionFrame) {
        // Some remotes repeat the full NEC code instead of sending a repeat frame.
        // Refresh the held-key timer, but do not POST the control mode again.
        lastDirectionSignalTime = millis();
      }
      if (isNavigationKey) prepareForDigitEntry();



      if (!repeatedDirectionFrame) switch (rawCode) {

        // --- ปุ่มทิศทาง (กดครั้งแรก) ---

        case CODE_UP:

        case CODE_DOWN:

        case CODE_LEFT:

        case CODE_RIGHT:

          if (!enterManualMode()) {
            Serial.println("[MANUAL] Could not set remote control mode");
            break;
          }

          lastDirectionCode = rawCode;

          lastDirectionSignalTime = millis();

          if (rawCode == CODE_UP) Serial.println("Direction: Forward");

          else if (rawCode == CODE_DOWN) Serial.println("Direction: Backward");

          else if (rawCode == CODE_LEFT) Serial.println("Direction: Turn Left");

          else Serial.println("Direction: Turn Right");

          resendDirectionCommand(rawCode);

          break;



        // --- ปุ่มตัวเลข 0-9 ---

        case CODE_0: appendDigit('0'); break;

        case CODE_1: appendDigit('1'); break;

        case CODE_2: appendDigit('2'); break;

        case CODE_3: appendDigit('3'); break;

        case CODE_4: appendDigit('4'); break;

        case CODE_5: appendDigit('5'); break;

        case CODE_6: appendDigit('6'); break;

        case CODE_7: appendDigit('7'); break;

        case CODE_8: appendDigit('8'); break;

        case CODE_9: appendDigit('9'); break;

        case CODE_INCREASE: changeManualSpeed(SPEED_STEP); break;

        case CODE_DECREASE: changeManualSpeed(-SPEED_STEP); break;



        // --- ปุ่ม OK / STOP ---

        case CODE_OK:

          handleOkButton();

          break;



        default:

          break;

      }

    }



    IrReceiver.resume(); // เตรียมรับสัญญาณถัดไป

  }



  // ปล่อยปุ่มทิศทางแล้ว (ไม่มีสัญญาณซ้ำเข้ามาตามเวลาที่กำหนด) -> สั่งหยุด (เพิ่มใหม่)

  if (lastDirectionCode != 0 && (millis() - lastDirectionSignalTime > DIRECTION_RELEASE_TIMEOUT)) {

    Serial.println("Direction: Released -> STOP");

    sendManualControl(0.0, 0.0);

    lastDirectionCode = 0;

  }

  // Wait for the robot's acknowledgement before the next velocity update.
  if (lastDirectionCode != 0 && webSocketConnected && twistFeedbackReceived &&
      millis() - lastTwistSendTime >= TWIST_SEND_INTERVAL) {
    resendDirectionCommand(lastDirectionCode);
  }

  // The previous move finished; start the next queued destination in order.
  if (!navigationActive && !navigationQueue.empty()) {
    startNextQueuedMove();
  }

  // ฮาร์ดแวร์บนบอร์ด: keypad, joystick, แบตเตอรี่, LED, จอ LCD
  handleKeypad();
  handleJoystick();
  updateBattery();
  updateLeds();
  updateLcd();

}
