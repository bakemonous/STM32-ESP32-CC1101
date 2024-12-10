# These are the values which are dependent on the microcontroller,
# development board, hardware design and ITHO remote used. This
# module connects cc1101.py and itho.py to your setup. Aside from
# these constants no further configuration is required. Most of the
# values are used as default function arguments, so changing them
# after the program has started has no effect.

BOARD = "Wroom-1 S3 - ESP32S3"  # Reminder which board you have configured. Has no effect.

ESP32_S3=1#в случае если у вас esp32, а не esp32 s3 поставить 0

class ESP32:
    SPI_ID_LIST = [1]  # List with all possible SPI hardware channel ID's for your board
    MISO_PIN_PER_SPI_ID = {"1": 13}  # Pin number of MISO for every SPI channel ID of your board

    SPI_ID = 1  # Hardware SPI channel ID to use for communication with your CC1101
    SS_PIN = 10  # Slave select pin connected to CC1101's CSn. Dependent on your hardware design
    GD02_PIN = 4  # Pin connected to CC101's GD02 pin. Dependent on your hardware design
    sck = 12
    mosi = 11
    def __init__(self, ESP32_flag):
        if (ESP32_flag ==0):
            self.SPI_ID_LIST = [2]  # List with all possible SPI hardware channel ID's for your board
            self.MISO_PIN_PER_SPI_ID = {"2": 19}  # Pin number of MISO for every SPI channel ID of your board

            self.SPI_ID = 2  # Hardware SPI channel ID to use for communication with your CC1101
            self.SS_PIN = 5  # Slave select pin connected to CC1101's CSn. Dependent on your hardware design
            self.sck = 18
            self.mosi = 23

code1= "yi6"# контрольный код, который заранее указан в устройстве slave что бы разделить 2 разныхсообщения, отправленных одной передачей
code2= "on7"
#настройки MQTT
SERVER = "broker.emqx.io"
CLIENT_ID = "dghe3456bdd" #ubinascii.hexlify(machine.unique_id())
TOPIC = b"test-temp-esp32"