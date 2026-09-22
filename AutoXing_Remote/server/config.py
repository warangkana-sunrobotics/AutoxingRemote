"""AutoXing Remote configuration.
Keep APPSecret and Authorization private. Do not commit real credentials.
"""

config = {
    "APPID": "PUT_YOUR_APPID_HERE",
    "APPSecret": "PUT_YOUR_APPSECRET_HERE",
    "Authorization": "APPCODE PUT_YOUR_AUTHORIZATION_HERE",
    # Thailand/overseas: use the global endpoint.
    "URLPrefix": "https://apiglobal.autoxing.com",
    "token": "",
}

# AutoXing robot to control.
ROBOT_ID = "PUT_ROBOT_ID_HERE"

# Map physical ESP32 buttons to POI names returned by AutoXing.
# The server resolves these names at startup so you do not hard-code coordinates.
BUTTON_POI = {
    1: "Point 1",
    2: "Point 2",
    3: "Point 3",
}

# Server listens on all interfaces so the ESP32 can reach it over LAN.
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 5000
