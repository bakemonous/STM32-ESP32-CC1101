# the main executable for esp32 
# should be started/imported in boot.py
# it assumes boot.py initialises wifi
import array
import time
import os
import gc

from array import *

from machine import WDT
from cc1101_1 import CC1101
from config_1 import GD02_PIN, ITHO_REMOTE_ID, ITHO_REMOTE_TYPE, SPI_ID, SS_PIN, BUTTON, sck, mosi
import time
import ubinascii
import machine
from umqtt.simple import MQTTClient
import json
# mqtt
# The callback for when a PUBLISH message is received from the server.
SERVER = "broker.emqx.io"
CLIENT_ID = "dghe3456bdd" #ubinascii.hexlify(machine.unique_id())
TOPIC = b"test-temp-esp32"
last_ping = time.time()
ping_interval = 60
code1= "yi6"
code2= "on7"
def find_temp(buffer):
    n=[" "," "," "]
    p=""
    l1=0
    l2=0
    for i in range(20):
        try:
            buffer1=buffer[i:i+1].decode('utf-8')
        except:
            return 0
        n[0]=n[1]
        n[1]=n[2]
        n[2]=buffer1
        p=n[0]+n[1]+n[2]
        if (p==code1):
            l1=i
        if (p==code2):
            l2=i
    print("temp",buffer[0:(l1-3)])
    temp_template=buffer[0:(l1-2)].decode('utf-8')
    print("platform slave",buffer[(l1+2):(l2-2)])
    slave_template=buffer[(l1+2):(l2-7)].decode('utf-16')
    platform1=os.uname()[0]
    master_template=platform1
    to_json = {'slave': slave_template, 'master': master_template,'temp':temp_template}
    data_out=json.dumps(to_json)

    with open('sw_templates.json', 'w') as f:
        f.write(json.dumps(to_json))
    if __name__ == "__main__":
        try:
            main(data_out)
            print("next")
        except OSError as e:
            print("Error: " + str(e))
            reset()
def bytes_to_int(bytes):
  return int.from_bytes(bytes, byteorder='big', signed=False)
def check_reg():
    print("0x00,0x29=0x00,",hex(radio1.read_register(0x00)))
    print("0x01,0x2E=0x01,",hex(radio1.read_register(0x01)))
    print("0x02,0x06=0x02,",hex(radio1.read_register(0x02)))
    print("0x03,0x47=0x03,",hex(radio1.read_register(0x03)))
    print("0x04,0xD3=0x04,",hex(radio1.read_register(0x04)))
    print("0x05,0x91=0x05,",hex(radio1.read_register(0x05)))
    print("0x06,0xFF=0x06,",hex(radio1.read_register(0x06)))
    print("0x07,0x04=0x07,",hex(radio1.read_register(0x07)))
    print("0x08,0x05=0x08,",hex(radio1.read_register(0x08)))
    print("0x09,0x00=0x09,",hex(radio1.read_register(0x09)))
    print("0x0A,0x00=0x0A,",hex(radio1.read_register(0x0A)))
    print("0x0B,0x08=0x0B,",hex(radio1.read_register(0x0B)))
    print("0x0C,0x00=0x0C,",hex(radio1.read_register(0x0C)))
    print("0x0D,0x10=0x0D,",hex(radio1.read_register(0x0D)))
    print("0x0E,0xB4=0x0E,",hex(radio1.read_register(0x0E)))
    print("0x0F,0x2E=0x0F,",hex(radio1.read_register(0x0F)))
    print("0x10,0xCA=0x10,",hex(radio1.read_register(0x10)))
    print("0x11,0x83=0x11,",hex(radio1.read_register(0x11)))
    print("0x12,0x93=0x12,",hex(radio1.read_register(0x12)))
    print("0x13,0x22=0x13,",hex(radio1.read_register(0x13)))
    print("0x14,0xF8=0x14,",hex(radio1.read_register(0x14)))
    print("0x15,0x34=0x15,",hex(radio1.read_register(0x15)))
    print("0x16,0x07=0x16,",hex(radio1.read_register(0x16)))
    print("0x17,0x30=0x17,",hex(radio1.read_register(0x17)))
    print("0x18,0x18=0x18,",hex(radio1.read_register(0x18)))
    print("0x19,0x16=0x19,",hex(radio1.read_register(0x19)))
    print("0x1A,0x6C=0x1A,",hex(radio1.read_register(0x1A)))
    print("0x1B,0x43=0x1B,",hex(radio1.read_register(0x1B)))
    print("0x1C,0x40=0x1C,",hex(radio1.read_register(0x1C)))
    print("0x1D,0x91=0x1D,",hex(radio1.read_register(0x1D)))
    print("0x1E,0x87=0x1E,",hex(radio1.read_register(0x1E)))
    print("0x1F,0x6B=0x1F,",hex(radio1.read_register(0x1F)))
    print("0x20,0xF8=0x20,",hex(radio1.read_register(0x20)))
    print("0x21,0x56=0x21,",hex(radio1.read_register(0x21)))
    print("0x22,0x10=0x22,",hex(radio1.read_register(0x22)))
    print("0x23,0xE9=0x23,",hex(radio1.read_register(0x23)))
    print("0x24,0x2A=0x24,",hex(radio1.read_register(0x24)))
    print("0x25,0x00=0x25,",hex(radio1.read_register(0x25)))
    print("0x26,0x1F=0x26,",hex(radio1.read_register(0x26)))
    print("0x27,0x41=0x27,",hex(radio1.read_register(0x27)))
    print("0x28,0x00=0x28,",hex(radio1.read_register(0x28)))
    print("0x29,0x59=0x29,",hex(radio1.read_register(0x29)))
    print("0x2A,0x7F=0x2A,",hex(radio1.read_register(0x2A)))
    print("0x2B,0x3F=0x2B,",hex(radio1.read_register(0x2B)))
    print("0x2C,0x81=0x2C,",hex(radio1.read_register(0x2C)))
    print("0x2D,0x35=0x2D,",hex(radio1.read_register(0x2D)))
    print("0x2E,0x09=0x2E,",hex(radio1.read_register(0x2E)))
    print(" ")
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
    time.sleep(3)#3
    mqttClient.disconnect()
    #print("disonnected")
    #time.sleep(2)
def sub_cb(topic, msg):
    print((topic, msg))
def reset():
    print("Resetting...")
    time.sleep(5)
    machine.reset()
def get_bit(num: int, pos: int) -> bool:
    return num >> pos & 1 
def main1():
    mqttClient = MQTTClient(CLIENT_ID, SERVER, keepalive=60)
    mqttClient.set_callback(sub_cb)
    mqttClient.connect()
    mqttClient.subscribe(TOPIC)
    print(f"Connected to MQTT  Broker :: {SERVER}, and waiting for callback function to be called!")
    if False:
        # Blocking wait for message
        mqttClient.wait_msg()
    else:
        # Non-blocking wait for message
        mqttClient.check_msg()
        # Then need to sleep to avoid 100% CPU usage (in a real
        # app other useful actions would be performed instead)
        global last_ping
        if (time.time() - last_ping) >= ping_interval:
            mqttClient.ping()
            last_ping = time.time()
            now = time.localtime()
            print(f"Pinging MQTT Broker, last ping :: {now[0]}/{now[1]}/{now[2]} {now[3]}:{now[4]}:{now[5]}")
        time.sleep(1)
            
    print("Disconnecting...")
    mqttClient.disconnect()
radio1 = CC1101(SPI_ID, SS_PIN, GD02_PIN)
buffer =0
rssi_check=-40
check_reg()
#last_will()
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
'''    
cc1101 = CPC(2,5,4,8000000,8000000) 
'''    
'''    
radio = cc1101(conf.SPI_ID, conf.spibus, conf.spics, conf.speed, conf.gdo0 , conf.gdo2) 

while True:
    #print("Buffer? - ",radio.readBufEsp())
    print("Buffer - ",radio.checkBuffer())  
    time.sleep(0.1)
'''
'''
radio1 = CC1101(SPI_ID, SS_PIN, GD02_PIN)
check_reg()
radio1.write_command(0x36)
radio1.write_command(0x3A)
radio1.write_command(0x34)
#radio1.write_register(CC1101.FSCAL3, 0xE9)
#radio1.write_register(CC1101.FSCAL2, 0x2A)
#radio1.write_register(CC1101.FSCAL1, 0x00)
buffer =0
size=0
temp_template = ['buffer1']
master_template = ['platform1']
slave_template = ['platform2']
to_json = {'slave': slave_template, 'master': master_template,'temp':temp_template}
while True:
    
    #len=chr(radio1.read_register(0x3F))
    #print("packet length=",len)    
    #for (i=0;i<len;i++)
     #   buffer.append(radio1.read_register(0x3F))
    #if (radio1.read_register(0x3F)==8):
    radio1.write_command(0x3A)
    radio1.write_command(0x34)
    status=radio1.read_register(0x3B)
    #print("CRC en",get_bit(radio1.read_register(0x08), 2))
    #print("CRC",get_bit(radio1.read_register(0x38), 7))
    if (not(radio1.CheckRxFifo(100) & 0 )):#status & 0x7f            radio1.CheckCRC()==1
        #print("RXBYTES,0x3B=",hex(radio1.read_register(0x3B)))
        #print("BYTES_IN_RXFIFO,0x7F=",radio1.read_register(0x7F))
        #print("LQI,0x33=",radio1.read_register(0x33))
        #print("BYTES_IN_RXFIFO,0x7F=",radio1.read_register(0x7F))
        #print("CRC",get_bit(radio1.read_register(0x33), 7))
        #print("READ_BURST,0xC0=",hex(radio1.read_register(0xC0)))
        #CheckRxFifo(100)
        if (radio1.CheckCRC()==1):  
            print("CC1101_RXBYTES,0x3B=",status)
            print("RSSI=",radio1.get_rssi())
            print("LQI=",radio1.get_lqi())
            buffer=radio1.ReceiveData(buffer)
            print("buffer",buffer)
            radio1.write_command(0x3A)
            #find_temp(buffer)
            platform1=os.uname()[0]
            platform2=0
            #find_temp(buffer)

        time.sleep(1)
        #time.sleep(0.2)
        #radio1.strobe_REG(0x3A)
        #radio1.strobe_REG(0x34)
        #print("buffer",buffer)
        #print("b size",size)
        while (0==1):#i==0
'''
'''
             k= chr(radio1.read_register(0x3F))
             print ("RXFIFO,0x3F=",k)
             #buffer.append(k)
             v+=1
             s=s+k
             if (v == 8):
                 i=1
                 print("buf",s)
'''
             
'''def on_message(topic, msg):

    command=msg.decode("utf-8").rstrip()
    topic=topic.decode("utf-8").rstrip()
    print(topic+" "+command)
    if command in elero.eleroCmds:
        blind=(topic.split('/')[2])+":"
        first=(command[0]!='P') # for programming commands we want the last remote that knows the blind
        targetBlind,targetRemote = elero.getTarget(blind,first)
        if (targetBlind and conf.enable_cc1101):
            if (command == "Prog"):
                txmsg=elero.construct_msg(targetRemote,targetBlind,'P1')
                for i in range(conf.retrans):
                    radio.transmit(txmsg)
                print("sent P1 to "+blind)
                # any blinds in Async mode will send their address as a 0xD4 type message
                # but for simplicity we'll ignore that and use the address/channel in conf.py
                time.sleep(2.1)
                txmsg=elero.construct_msg(targetRemote,targetBlind,'P2')
                for i in range(conf.retrans):
                    radio.transmit(txmsg)
                print("sent P2 to "+blind)
                time.sleep(0.5)
                txmsg=elero.construct_msg(targetRemote,targetBlind,'P3')
                for i in range(conf.retrans):
                    radio.transmit(txmsg)
                print("sent P3 to "+blind)
            else:
                txmsg=elero.construct_msg(targetRemote,targetBlind,command)
                print("sending: ",''.join('{:02X}:'.format(a) for a in targetRemote),targetBlind[3],''.join('{:02X}:'.format(a) for a in targetBlind[0:3]), command)
                for i in range(conf.retrans):
                    radio.transmit(txmsg)
                    time.sleep(0.1)
                print("sent "+command+" to "+blind)
        else:
             print(blind+" blind not found")
    else:
        print(command+": invalid command")'''
            #with open('sw_templates.json', 'w') as f:
                #f.write(json.dumps(to_json))

           # with open('sw_templates.json') as f:
               # print(f.read())
'''
radio1 = CC1101(SPI_ID, SS_PIN, GD02_PIN)
check_reg()
radio1.write_command(0x36)
radio1.write_command(0x3A)
radio1.write_command(0x34)
#radio1.write_register(CC1101.FSCAL3, 0xE9)
#radio1.write_register(CC1101.FSCAL2, 0x2A)
#radio1.write_register(CC1101.FSCAL1, 0x00)
buffer =0
size=0
temp_template = ['buffer1']
master_template = ['platform1']
slave_template = ['platform2']
to_json = {'slave': slave_template, 'master': master_template,'temp':temp_template}
while True:
    
    #len=chr(radio1.read_register(0x3F))
    #print("packet length=",len)    
    #for (i=0;i<len;i++)
     #   buffer.append(radio1.read_register(0x3F))
    #if (radio1.read_register(0x3F)==8):
    radio1.write_command(0x3A)
    radio1.write_command(0x34)
    status=radio1.read_register(0x3B)
    #print("CRC en",get_bit(radio1.read_register(0x08), 2))
    #print("CRC",get_bit(radio1.read_register(0x38), 7))
    if (not(radio1.CheckRxFifo(100) & 0 )):#status & 0x7f            radio1.CheckCRC()==1
        #print("RXBYTES,0x3B=",hex(radio1.read_register(0x3B)))
        #print("BYTES_IN_RXFIFO,0x7F=",radio1.read_register(0x7F))
        #print("LQI,0x33=",radio1.read_register(0x33))
        #print("BYTES_IN_RXFIFO,0x7F=",radio1.read_register(0x7F))
        #print("CRC",get_bit(radio1.read_register(0x33), 7))
        #print("READ_BURST,0xC0=",hex(radio1.read_register(0xC0)))
        #CheckRxFifo(100)
        if (radio1.CheckCRC()==1):  
            print("CC1101_RXBYTES,0x3B=",status)
            print("RSSI=",radio1.get_rssi())
            print("LQI=",radio1.get_lqi())
            buffer=radio1.ReceiveData(buffer)
            print("buffer",buffer)
            radio1.write_command(0x3A)
            find_temp(buffer)
            platform1=os.uname()[0]
            platform2=0
            find_temp(buffer)

        time.sleep(1)
        #time.sleep(0.2)
        #radio1.strobe_REG(0x3A)
        #radio1.strobe_REG(0x34)
        #print("buffer",buffer)
        #print("b size",size)
        while (0==1):#i==0
             """
             k= chr(radio1.read_register(0x3F))
             print ("RXFIFO,0x3F=",k)
             #buffer.append(k)
             v+=1
             s=s+k
             if (v == 8):
                 i=1
                 print("buf",s)
             """
    #print("RXFIFO,0x3F=",buffer)    
    #data=radio1.read_register(0x3F)
    #for(int i = 0; i<data; i++)
    #    print([i])
    #print("data=",radio.checkBuffer())
#    pinHandler.checkPins()
    #print("data=",radio1.receive_data(10))
    #checkCounter=int(time.time())%conf.checkFreq
    #print("data=",radio1.receive_data(61))
    # garbage collection once every checkFreq seconds'''
