"""
Smart IoT-Enabled Trash Bin with Multi-Sensor Monitoring
ESP32 MicroPython Firmware

Features:
- 3x Ultrasonic sensors (entrance, proximity, capacity)
- DHT22 temperature & humidity sensor
- MQTT telemetry
- Servo-controlled lid
- LCD display
"""

import machine
import network
import time
import json
from umqtt.simple import MQTTClient
import dht

from config import *

# ==================== GLOBAL OBJECTS ====================
wifi = None
mqtt_client = None
dht_sensor = None
servo = None
lcd = None

# Ultrasonic sensor pins
entrance_trig = None
entrance_echo = None
# proximity_trig = None
#proximity_echo = None
capacity_trig = None
capacity_echo = None

# ==================== STATE VARIABLES ====================
temperature = 0.0
humidity = 0.0
capacity_percent = 0
trash_count = 0
lid_open = False
user_detected = False
bin_full = False
alert_sent = False
item_detected = False

last_telemetry_time = 0
last_sensor_read_time = 0
lid_open_time = 0


# ==================== LCD CLASS (I2C) ====================
class I2cLcd:
    """Simple I2C LCD driver for 16x2 display"""
    
    LCD_LINE1 = 0x80
    LCD_LINE2 = 0xC0
    
    def __init__(self, i2c, address, cols=16, rows=2):
        self.i2c = i2c
        self.address = address
        self.cols = cols
        self.rows = rows
        self._backlight = 0x08
        self._init_lcd()
    
    def _write_byte(self, data, mode=0):
        high = mode | (data & 0xF0) | self._backlight
        low = mode | ((data << 4) & 0xF0) | self._backlight
        self.i2c.writeto(self.address, bytes([high | 0x04]))
        time.sleep_us(1)
        self.i2c.writeto(self.address, bytes([high]))
        time.sleep_us(50)
        self.i2c.writeto(self.address, bytes([low | 0x04]))
        time.sleep_us(1)
        self.i2c.writeto(self.address, bytes([low]))
        time.sleep_us(50)
    
    def _init_lcd(self):
        time.sleep_ms(50)
        for _ in range(3):
            self._write_byte(0x30)
            time.sleep_ms(5)
        self._write_byte(0x20)
        time.sleep_ms(1)
        self._write_byte(0x28)  # 4-bit, 2 lines, 5x8 font
        self._write_byte(0x0C)  # Display on, cursor off
        self._write_byte(0x06)  # Entry mode
        self.clear()
    
    def clear(self):
        self._write_byte(0x01)
        time.sleep_ms(2)
    
    def set_cursor(self, col, row):
        addr = self.LCD_LINE1 if row == 0 else self.LCD_LINE2
        self._write_byte(addr + col)
    
    def print(self, text):
        for char in text:
            self._write_byte(ord(char), 0x01)
    
    def backlight(self, on=True):
        self._backlight = 0x08 if on else 0x00
        self.i2c.writeto(self.address, bytes([self._backlight]))


# ==================== SETUP FUNCTIONS ====================
def setup_wifi():
    """Connect to WiFi network"""
    global wifi
    print(f"Connecting to WiFi: {WIFI_SSID}")
    
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    wifi.connect(WIFI_SSID, WIFI_PASSWORD)
    
    attempts = 0
    while not wifi.isconnected() and attempts < 30:
        time.sleep_ms(500)
        print(".", end="")
        attempts += 1
    
    if wifi.isconnected():
        print(f"\nWiFi Connected! IP: {wifi.ifconfig()[0]}")
        return True
    else:
        print("\nWiFi Connection Failed!")
        return False


def setup_mqtt():
    """Setup MQTT client and connect to broker"""
    global mqtt_client
    
    mqtt_client = MQTTClient(
        MQTT_CLIENT_ID,
        MQTT_BROKER,
        port=MQTT_PORT,
        user=MQTT_USER if MQTT_USER else None,
        password=MQTT_PASS if MQTT_PASS else None
    )
    mqtt_client.set_callback(mqtt_callback)
    
    try:
        mqtt_client.connect()
        mqtt_client.subscribe(TOPIC_COMMAND)
        print("MQTT Connected!")
        publish_status()
        return True
    except Exception as e:
        print(f"MQTT Connection Failed: {e}")
        return False


def setup_sensors():
    """Initialize all sensors and actuators"""
    global entrance_trig, entrance_echo
    global proximity_trig, proximity_echo
    global capacity_trig, capacity_echo
    global dht_sensor, servo, lcd
    
    # Ultrasonic sensors
    entrance_trig = machine.Pin(ENTRANCE_TRIG_PIN, machine.Pin.OUT)
    entrance_echo = machine.Pin(ENTRANCE_ECHO_PIN, machine.Pin.IN)
    
    proximity_trig = machine.Pin(PROXIMITY_TRIG_PIN, machine.Pin.OUT)
    proximity_echo = machine.Pin(PROXIMITY_ECHO_PIN, machine.Pin.IN)
    
    capacity_trig = machine.Pin(CAPACITY_TRIG_PIN, machine.Pin.OUT)
    capacity_echo = machine.Pin(CAPACITY_ECHO_PIN, machine.Pin.IN)
    
    # DHT22 sensor
    dht_sensor = dht.DHT22(machine.Pin(DHT_PIN))
    
    # Servo motor (PWM)
    servo = machine.PWM(machine.Pin(SERVO_PIN), freq=50)
    close_lid()
    
    # LCD display
    try:
        i2c = machine.I2C(0, sda=machine.Pin(LCD_SDA_PIN), scl=machine.Pin(LCD_SCL_PIN), freq=400000)
        lcd = I2cLcd(i2c, LCD_ADDRESS)
        lcd.backlight(True)
        lcd.clear()
        lcd.set_cursor(0, 0)
        lcd.print("Smart Trash Bin")
        lcd.set_cursor(0, 1)
        lcd.print("Initializing...")
    except Exception as e:
        print(f"LCD init failed: {e}")
        lcd = None
    
    print("Sensors initialized!")


# ==================== SENSOR FUNCTIONS ====================
def read_ultrasonic(trig_pin, echo_pin):
    """Read distance from ultrasonic sensor in cm"""
    trig_pin.value(0)
    time.sleep_us(2)
    trig_pin.value(1)
    time.sleep_us(10)
    trig_pin.value(0)
    
    # Wait for echo to go high (with timeout)
    timeout = time.ticks_us()
    while echo_pin.value() == 0:
        if time.ticks_diff(time.ticks_us(), timeout) > 30000:
            return -1
    pulse_start = time.ticks_us()
    
    # Wait for echo to go low (with timeout)
    while echo_pin.value() == 1:
        if time.ticks_diff(time.ticks_us(), pulse_start) > 30000:
            return -1
    pulse_end = time.ticks_us()
    
    duration = time.ticks_diff(pulse_end, pulse_start)
    distance = (duration * 0.034) / 2
    
    return distance


def read_dht():
    """Read temperature and humidity from DHT22"""
    global temperature, humidity
    
    try:
        dht_sensor.measure()
        temperature = dht_sensor.temperature()
        humidity = dht_sensor.humidity()
    except Exception as e:
        print(f"DHT read error: {e}")


def check_entrance_sensor():
    """Detect items thrown into the bin"""
    global item_detected, trash_count
    
    distance = read_ultrasonic(entrance_trig, entrance_echo)
    
    if 0 < distance < ENTRANCE_THRESHOLD_CM:
        if not item_detected:
            item_detected = True
            trash_count += 1
            print(f"Item thrown! Total count: {trash_count}")
            
            # Publish event
            data = {"event": "trash_thrown", "count": trash_count}
            mqtt_client.publish(TOPIC_STATUS, json.dumps(data))
    else:
        item_detected = False


def check_proximity_sensor():
    """Detect user approaching the bin"""
    global user_detected
    
    distance = read_ultrasonic(proximity_trig, proximity_echo)
    
    if 0 < distance < PROXIMITY_THRESHOLD_CM:
        if not user_detected:
            user_detected = True
            print("User approaching - Opening lid")
            open_lid()
    else:
        user_detected = False


def update_capacity():
    """Calculate bin fill level percentage"""
    global capacity_percent, bin_full
    
    distance = read_ultrasonic(capacity_trig, capacity_echo)
    
    if 0 < distance <= BIN_HEIGHT_CM:
        fill_level = BIN_HEIGHT_CM - int(distance)
        capacity_percent = max(0, min(100, (fill_level * 100) // BIN_HEIGHT_CM))
    
    bin_full = capacity_percent >= FULL_THRESHOLD_PERCENT


# ==================== LID CONTROL ====================
def open_lid():
    """Open the bin lid using servo"""
    global lid_open, lid_open_time
    
    if not lid_open:
        servo.duty(77)  # ~90 degrees (open position)
        lid_open = True
        lid_open_time = time.ticks_ms()
        print("Lid opened")


def close_lid():
    """Close the bin lid using servo"""
    global lid_open
    
    servo.duty(26)  # ~0 degrees (closed position)
    lid_open = False
    print("Lid closed")


# ==================== MQTT FUNCTIONS ====================
def mqtt_callback(topic, msg):
    """Handle incoming MQTT messages"""
    topic = topic.decode()
    message = msg.decode()
    
    print(f"MQTT Message [{topic}]: {message}")
    
    if topic == TOPIC_COMMAND:
        handle_command(message)


def handle_command(message):
    """Process commands received via MQTT"""
    global trash_count, alert_sent
    
    try:
        data = json.loads(message)
        cmd = data.get("cmd", "")
        
        if cmd == "reset":
            trash_count = 0
            alert_sent = False
            print("Trash count reset")
            publish_telemetry()
        elif cmd == "status":
            publish_telemetry()
        elif cmd == "open_lid":
            open_lid()
        elif cmd == "close_lid":
            close_lid()
        else:
            print(f"Unknown command: {cmd}")
    except Exception as e:
        print(f"Command parse error: {e}")


def publish_telemetry():
    """Publish sensor data to MQTT"""
    data = {
        "temperature": temperature,
        "humidity": humidity,
        "capacity": capacity_percent,
        "trash_count": trash_count,
        "lid_open": lid_open,
        "bin_full": bin_full,
        "timestamp": time.ticks_ms()
    }
    
    mqtt_client.publish(TOPIC_TELEMETRY, json.dumps(data))
    print(f"Telemetry: {data}")


def publish_status():
    """Publish device status to MQTT"""
    data = {
        "status": "online",
        "device_id": MQTT_CLIENT_ID,
        "ip": wifi.ifconfig()[0] if wifi else "unknown"
    }
    
    mqtt_client.publish(TOPIC_STATUS, json.dumps(data), retain=True)


def publish_alert(message):
    """Publish alert to MQTT"""
    data = {
        "alert": message,
        "capacity": capacity_percent,
        "timestamp": time.ticks_ms()
    }
    
    mqtt_client.publish(TOPIC_ALERT, json.dumps(data))
    print(f"ALERT: {message}")


# ==================== LCD UPDATE ====================
def update_lcd():
    """Update LCD display with current status"""
    if lcd is None:
        return
    
    lcd.clear()
    
    # Line 1: Capacity and status
    lcd.set_cursor(0, 0)
    status = "FULL!" if bin_full else "OK"
    lcd.print(f"Cap:{capacity_percent}% {status}")
    
    # Line 2: Temperature and humidity
    lcd.set_cursor(0, 1)
    lcd.print(f"T:{int(temperature)}C H:{int(humidity)}%")


# ==================== MAIN ====================
def main():
    """Main program entry point"""
    global last_telemetry_time, last_sensor_read_time, alert_sent
    
    print("\n=== Smart Trash Bin Starting ===\n")
    
    # Initialize
    setup_sensors()
    
    if lcd:
        lcd.clear()
        lcd.set_cursor(0, 0)
        lcd.print("Connecting...")
    
    if not setup_wifi():
        print("WiFi failed - running in offline mode")
    
    if wifi and wifi.isconnected():
        if not setup_mqtt():
            print("MQTT failed - running without telemetry")
    
    if lcd:
        lcd.clear()
        lcd.set_cursor(0, 0)
        lcd.print("System Ready!")
        time.sleep_ms(1000)
    
    print("System Ready!\n")
    
    # Main loop
    while True:
        current_time = time.ticks_ms()
        
        # Check MQTT messages
        if mqtt_client:
            try:
                mqtt_client.check_msg()
            except Exception as e:
                print(f"MQTT check error: {e}")
                try:
                    setup_mqtt()
                except:
                    pass
        
        # Read sensors periodically
        if time.ticks_diff(current_time, last_sensor_read_time) >= SENSOR_READ_INTERVAL_MS:
            last_sensor_read_time = current_time
            read_dht()
            check_entrance_sensor()
            check_proximity_sensor()
            update_capacity()
        
        # Auto-close lid after duration
        if lid_open and time.ticks_diff(current_time, lid_open_time) >= LID_OPEN_DURATION_MS:
            close_lid()
        
        # Publish telemetry periodically
        if time.ticks_diff(current_time, last_telemetry_time) >= TELEMETRY_INTERVAL_MS:
            last_telemetry_time = current_time
            if mqtt_client:
                try:
                    publish_telemetry()
                except Exception as e:
                    print(f"Telemetry publish error: {e}")
            update_lcd()
        
        # Check for bin full alert
        if capacity_percent >= FULL_THRESHOLD_PERCENT and not alert_sent:
            if mqtt_client:
                publish_alert("Bin is almost full!")
            alert_sent = True
        elif capacity_percent < FULL_THRESHOLD_PERCENT:
            alert_sent = False
        
        time.sleep_ms(10)


if __name__ == "__main__":
    main()
