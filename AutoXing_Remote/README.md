# AutoXing ESP32 Remote — 3 Buttons

โปรเจกต์นี้ทำให้ปุ่มจริงบน ESP32 เป็นรีโมตสั่งหุ่นยนต์ AutoXing:

- ปุ่ม 1 → Point 1
- ปุ่ม 2 → Point 2
- ปุ่ม 3 → Point 3

ESP32 ทำหน้าที่อ่านปุ่มและส่ง HTTP ไปยัง Python Server เท่านั้น ส่วน APPID / APPSecret / Authorization / Token และ AutoXing API อยู่ฝั่ง Python Server

## Architecture

```text
ESP32 button
    │ Wi-Fi / HTTP
    ▼
Python Remote Server
    │
    ├── AutoXing Token
    ├── Robot List / State
    ├── POI List
    └── Create + Execute Task
    │
    ▼
AutoXing Robot
```

## 1. Server setup

เปิด Terminal ใน `server`:

```bash
python -m venv .venv
```

Windows:

```bash
.venv\\Scripts\\activate
```

ติดตั้ง:

```bash
pip install -r requirements.txt
```

แก้ `config.py`:

```python
config = {
    "APPID": "...",
    "APPSecret": "...",
    "Authorization": "APPCODE ...",
    "URLPrefix": "https://apiglobal.autoxing.com",
    "token": "",
}

ROBOT_ID = "..."

BUTTON_POI = {
    1: "Point 1",
    2: "Point 2",
    3: "Point 3",
}
```

`Point 1/2/3` ต้องตรงกับชื่อ POI ที่ AutoXing API ส่งกลับมา

## 2. ตรวจ POI

รัน:

```bash
python remote_server.py
```

จากเครื่องที่รัน server เปิด:

```text
http://127.0.0.1:5000/pois
```

ถ้า ESP32 อยู่ LAN เดียวกัน ให้ใช้ IP ของ PC เช่น:

```text
http://192.168.1.100:5000/pois
```

ดู `pois` และนำชื่อ POI จริงมาใส่ใน `BUTTON_POI`

## 3. ทดสอบโดยไม่ใช้ ESP32

Windows PowerShell:

```powershell
Invoke-RestMethod -Method Post -Uri http://127.0.0.1:5000/button -ContentType 'application/json' -Body '{"button":1}'
```

ถ้าทำงานสำเร็จจะได้ประมาณ:

```json
{
  "ok": true,
  "button": 1,
  "poi": "Point 1",
  "taskId": "..."
}
```

นี่คือจุดที่ Python สร้าง Task และ Execute Task ให้ AutoXing

## 4. ESP32

เปิด `esp32_remote/esp32_remote.ino` ใน Arduino IDE

แก้:

```cpp
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* SERVER_URL = "http://192.168.1.100:5000/button";
```

เปลี่ยน `192.168.1.100` เป็น IP ของเครื่องที่รัน Python Server

อัปโหลดลง ESP32

### Pin ที่ตั้งค่าไว้

```text
Button 1 → GPIO18
Button 2 → GPIO19
Button 3 → GPIO21
```

ปุ่มใช้ `INPUT_PULLUP` ดังนั้นต่อปุ่มระหว่าง GPIO กับ GND:

```text
GPIO18 ─── Button 1 ─── GND
GPIO19 ─── Button 2 ─── GND
GPIO21 ─── Button 3 ─── GND
```

## 5. การทำงานจริง

```text
กดปุ่ม 1
  ↓
ESP32 POST {"button":1}
  ↓
Python /button
  ↓
ค้นหา Point 1
  ↓
TaskBuilder
  ↓
TaskPoint(Point 1)
  ↓
newTask()
  ↓
executeTask()
  ↓
Robot → Point 1
```

## Security

อย่าใส่ APPSecret / Authorization จริงใน ESP32 และอย่า commit `server/config.py` ที่มี credential จริงขึ้น Git

## หมายเหตุ

โค้ด AutoXing ใน `server/autoxing_sdk/` ถูกจัดจาก Python SDK ใน APIDemo ที่คุณให้มา และยังคงรูปแบบ Task API ของ SDK เดิม: `TaskPoint`, `TaskBuilder`, `newTask()` และ `executeTask()`
