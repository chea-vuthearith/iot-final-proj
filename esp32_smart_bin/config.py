# WiFi credentials
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"

# MQTT Broker settings
MQTT_BROKER = "192.168.1.100"  # Change to your MQTT broker IP
MQTT_PORT = 1883
MQTT_CLIENT_ID = "smart_trash_bin"
MQTT_USER = ""  # Optional
MQTT_PASS = ""  # Optional

# MQTT Topics
TOPIC_TELEMETRY = "smartbin/telemetry"
TOPIC_STATUS = "smartbin/status"
TOPIC_ALERT = "smartbin/alert"
TOPIC_COMMAND = "smartbin/command"

# Pin Definitions
# Ultrasonic Sensor 1: Entrance (detects items thrown in)
ENTRANCE_TRIG_PIN = 5
ENTRANCE_ECHO_PIN = 18

# Ultrasonic Sensor 2: User Proximity (detects people approaching)
# PROXIMITY_TRIG_PIN = 19
# PROXIMITY_ECHO_PIN = 21

# Ultrasonic Sensor 3: Capacity (measures fill level)
CAPACITY_TRIG_PIN = 22
CAPACITY_ECHO_PIN = 23

# DHT22 Sensor
DHT_PIN = 4

# Servo Motor (lid control)
SERVO_PIN = 13

# I2C LCD
LCD_SDA_PIN = 32
LCD_SCL_PIN = 33
LCD_ADDRESS = 0x27

# Constants
BIN_HEIGHT_CM = 50           # Total bin height in cm
ENTRANCE_THRESHOLD_CM = 10   # Distance to detect item thrown
PROXIMITY_THRESHOLD_CM = 30  # Distance to detect user approach
FULL_THRESHOLD_PERCENT = 80  # Alert threshold for "bin full"

TELEMETRY_INTERVAL_MS = 5000    # Send telemetry every 5 seconds
SENSOR_READ_INTERVAL_MS = 100   # Read sensors every 100ms
LID_OPEN_DURATION_MS = 3000     # Keep lid open for 3 seconds
