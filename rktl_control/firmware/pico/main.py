# import socket
# import struct
import time
import json

from machine import Pin, Timer, RTC

import network
import upip

# This function uses ntp to set the built in time object
def set_time():
    NTP_DELTA = 2208988800
    NTP_QUERY = bytearray(48)
    NTP_QUERY[0] = 0x1B
    addr = socket.getaddrinfo("pool.ntp.org", 123)[0][-1]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.settimeout(1)
        res = s.sendto(NTP_QUERY, addr)
        msg = s.recv(48)
    finally:
        s.close()
    val = struct.unpack("!I", msg[40:44])[0]
    t = val - NTP_DELTA
    tm = time.gmtime(t)
    RTC().datetime((tm[0], tm[1], tm[2], tm[6] + 1, tm[3], tm[4], tm[5], 0))

# Set up built in LED
led = Pin("LED", Pin.OUT)

# Wifi Setup
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
ssid = "J"
if not sta_if.isconnected():
    sta_if.connect(ssid, "password")
    while not sta_if.isconnected():
        pass
print("Connected to " + ssid)
print("IP Address: " + sta_if.ifconfig()[0])

# use upip to install MQTT libraries
upip.install('micropython-umqtt.simple')
upip.install('micropython-umqtt.robust')
from umqtt.simple import MQTTClient

# Connect to MQTT Server with UUID and Address
client = MQTTClient('19b1c5ab-9add-4117-b6e5-7dbc07feaa64',
                    'test.mosquitto.org')
client.connect()

# Callback function to periodicly blink LED
def led_cb(t: Timer):
    led.value(not led.value())

# Timer to blink LED every 500ms
Timer(-1).init(period=500, mode=Timer.PERIODIC, callback=led_cb)

# Callback and timer to publish encoder value and timestamp to MQTT server
encoder = 0
def encoder_cb(t: Timer):
    global encoder
    client.publish('arc_rl/test/encoder', json.dumps({"timestamp": RTC().datetime(), "encoder": encoder}))
    encoder += 1


Timer(-1).init(period=1000, mode=Timer.PERIODIC, callback=encoder_cb)

# Encoder:
# Set up encoder inputs and initial value to 0
encoder_A = Pin(16, Pin.IN)
encoder_B = Pin(17, Pin.IN)
encoder = 0

# Callback to change encoder value
def encoder_cb(p: Pin):
    global encoder
    if encoder_A.value() == encoder_B.value():
        encoder += 1
    else:
        encoder -= 1

# Trigger interrupt callback on either rising or falling edge of signal
encoder_A.irq(handler=encoder_cb, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)
encoder_B.irq(handler=encoder_cb, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)

# MQTT Subscriber:
# Callback to print value recieved from MQTT
def subscriber_cb(topic, msg):
    print(json.loads(msg.decode("utf-8")))

# Set up client to run callback on message recieved
mqtt_server = 'test.mosquitto.org'
topic = 'arc_rl/test/encoder'
client = MQTTClient('19b1c5ab-9add-4117-b6e5-7dbc07feaa65',
                     'test.mosquitto.org')
client.set_callback(subscriber_cb)
client.connect()
client.subscribe(topic)
print('Connected to %s MQTT broker, subscribed to %s topic' % (mqtt_server, topic))

while True:
    # Periodically check for message
    client.check_msg()
    time.sleep_ms(50)
