# boot.py -- run on boot-up
import network, utime, machine,ubinascii


# Replace the following with your WIFI Credentials
SSID = "odri"#KB2  Additive-LabPAS   Keenetic-4696 odri
SSID_PASSWORD = "123456789"#Bast2021Ion  Bast@2023#ION   obCBuo6b 123456789

def find_network(SSID,SSID_PASSWORD):
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    wifi.disconnect()
    aps = wifi.scan()
    for ap in aps:#ap - кортеж, неизменяемые данные и структура списка
        if (ap[0] == 'KB2'.encode()):#encode()- str to bstr (перевод из обычной строки в байтовую строку)
            SSID_PASSWORD = "Bast2021Ion"
            SSID = "KB2"
            return SSID,SSID_PASSWORD
        if (ap[0] == 'Keenetic-4696'.encode()):
            SSID_PASSWORD = "obCBuo6b"
            SSID = "Keenetic-4696"
            return SSID,SSID_PASSWORD
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
WiFi = find_network(SSID,SSID_PASSWORD)#данные при возвращении нескольких параметров возвращаются в виде кортежа
SSID=WiFi[0]
SSID_PASSWORD=WiFi[1]
print("SSID",SSID)
print("SSID_PASSWORD",SSID_PASSWORD)
do_connect()
