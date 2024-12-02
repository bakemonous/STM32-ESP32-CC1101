# boot.py -- run on boot-up
import network, utime, machine

# Replace the following with your WIFI Credentials
SSID = "KB2"#KB2  Additive-LabPAS   Keenetic-4696 odri
SSID_PASSWORD = "Bast2021Ion"#Bast2021Ion  Bast@2023#ION   obCBuo6b 123456789


def do_connect():
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('connecting to network...')
        sta_if.active(False)
        sta_if.active(True)
        sta_if.connect(SSID, SSID_PASSWORD)
        while not sta_if.isconnected():
            print("Attempting to connect....")
            utime.sleep(1)
    print('Connected! Network config:', sta_if.ifconfig())
    
print("Connecting to your wifi...")
do_connect()