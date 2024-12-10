import time
import os
import gc
import time
import ubinascii
import machine
import json

from cc1101_1 import CC1101
from config_1 import ESP32, code1, code2, SERVER, CLIENT_ID, TOPIC, ESP32_S3 
from umqtt.simple import MQTTClient

ping_interval = 60
buffer =0
rssi_check=-40

def find_temp(buffer):
    n=[" "," "," "]
    p=""
    l1=0
    l2=0
    try:
        for i in range(30):
            try:
                buffer1=buffer[i:i+1].decode('utf-8')#из байтовой строки перевод в обычную
            except:
                return 0
            n[0]=n[1]
            n[1]=n[2]
            n[2]=buffer1
            p=n[0]+n[1]+n[2]
            if (p==code1):#функция обнаружения кодовых слов в сообщении и обозначение их позиции
                l1=i
            if (p==code2):
                l2=i
        print("temp",buffer[0:(l1-3)])#разделение сообщений
        temp_template=buffer[0:(l1-2)].decode('utf-8')
        print("temp",temp_template)
        print("platform slave",buffer[(l1+2):(l2-2)])
        slave_template=buffer[(l1+2):(l2-2)].decode('utf-8')
        platform1=os.uname()[0]
        master_template=platform1
        to_json = {'slave': slave_template, 'master': master_template,'temp':temp_template}
        data_out=json.dumps(to_json)#добавление данных в json

        with open('sw_templates.json', 'w') as f:
            f.write(json.dumps(to_json))
        if (int(temp_template)< 200):
            try:
                main(data_out)# отправка данных на MQTT сервер
                print("next")
            except OSError as e:
                print("Error: " + str(e))
                reset()
    except:
        return 0
    
def reset():
    print("Resetting...")
    time.sleep(5)
    machine.reset()
    
def main(data_output):
    mqttClient = MQTTClient(CLIENT_ID, SERVER, keepalive=60)
    mqttClient.set_last_will(TOPIC,"no signal",0,0)
    mqttClient.connect()
    print(f"Connected to MQTT  Broker :: {SERVER}")
    print(f"Publishing  ::",data_output)
    mqttClient.publish(TOPIC, str(data_output).encode())
    time.sleep(3)
    mqttClient.disconnect()
    
ESP32_SPI=ESP32(ESP32_S3)
print("ESP32_SPI.SPI_ID",ESP32_SPI.SPI_ID)
print("ESP32_SPI.SS_PIN",ESP32_SPI.SS_PIN)
print("ESP32_SPI.GD02_PIN",ESP32_SPI.GD02_PIN)
radio1 = CC1101(ESP32_SPI.SPI_ID, ESP32_SPI.SS_PIN, ESP32_SPI.GD02_PIN)
radio1.check_reg()
while True:
    radio1.write_command(0x36)
    radio1.write_command(0x3A)
    radio1.write_command(0x34)
    radio1.CheckRxFifo(100)
    radio1.CheckCRC()
    if (radio1.get_rssi()> rssi_check):
        buffer=radio1.ReceiveData(buffer)
        print("buffer",buffer)
        platform1=os.uname()[0]
        platform2=0
        find_temp(buffer)
    time.sleep(0.01)