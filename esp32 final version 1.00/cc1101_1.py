# MicroPython CC1101 driver
#
# Inspired by the CC1101 drivers written in C from:
# https://github.com/letscontrolit/ESPEasyPluginPlayground/
# https://github.com/arjenhiemstra/IthoEcoFanRFT/blob/master/Master/Itho/CC1101.cpp
# https://github.com/SpaceTeddy/CC1101/blob/master/cc1100_raspi.cpp
# https://github.com/SpaceTeddy/CC1101/blob/master/cc1100_raspi.h
#
# Copyright 2021 (c) Erik de Lange
# Released under MIT license
import array
import time
import gc
import config_1  # hardware dependent configuration

from machine import SPI, Pin
from micropython import const
from config_1 import ESP32, ESP32_S3 

def bitfield(n):
    return [int(digit) for digit in bin(n)[2:]] # [2:] to chop off the "0b" part 
def get_bit(num: int, pos: int) -> bool:
    return num >> pos & 1
class CC1101:
    trxstate=(0)
    FIFO_BUFFER_SIZE = const(64)
    
    # Transfer types
    WRITE_SINGLE_BYTE = const(0x00)
    WRITE_BURST = const(0x40)
    READ_SINGLE_BYTE = const(0x80)
    READ_BURST = const(0xC0)

    # Register types
    CONFIG_REGISTER = const(0x80)
    STATUS_REGISTER = const(0xC0)

    # PATABLE and FIFO address
    PATABLE = const(0x3E)
    TXFIFO = const(0x3F)
    RXFIFO = const(0x3F)
    PA_LowPower = const(0x60)
    PA_LongDistance = (0xC0)

    # FIFO Commands
    TXFIFO_BURST = const(0x7F)  # Burst access to TX FIFO
    TXFIFO_SINGLE_BYTE = const(0x3F)  # Single byte access to TX FIFO
    RXFIFO_BURST = const(0xFF)  # Burst access to RX FIFO
    RXFIFO_SINGLE_BYTE = const(0xBF)  # Single byte access to RX FIFO
    PATABLE_BURST = const(0x7E)  # Power control read/write
    PATABLE_SINGLE_BYTE = const(0xFE)  # Power control read/write

    # Configuration registers
    IOCFG2 = const(0x00)  # GDO2 output pin configuration
    IOCFG1 = const(0x01)  # GDO1 output pin configuration
    IOCFG0 = const(0x02)  # GDO0 output pin configuration
    FIFOTHR = const(0x03)  # RX FIFO and TX FIFO thresholds
    SYNC1 = const(0x04)  # Sync word, high byte
    SYNC0 = const(0x05)  # Sync word, low byte
    PKTLEN = const(0x06)  # Packet length
    PKTCTRL1 = const(0x07)  # Packet automation control
    PKTCTRL0 = const(0x08)  # Packet automation control
    ADDR = const(0x09)  # Device address
    CHANNR = const(0x0A)  # Channel number
    FSCTRL1 = const(0x0B)  # Frequency synthesizer control
    FSCTRL0 = const(0x0C)  # Frequency synthesizer control
    FREQ2 = const(0x0D)  # Frequency control word, high byte
    FREQ1 = const(0x0E)  # Frequency control word, middle byte
    FREQ0 = const(0x0F)  # Frequency control word, low byte
    MDMCFG4 = const(0x10)  # Modem configuration
    MDMCFG3 = const(0x11)  # Modem configuration
    MDMCFG2 = const(0x12)  # Modem configuration
    MDMCFG1 = const(0x13)  # Modem configuration
    MDMCFG0 = const(0x14)  # Modem configuration
    DEVIATN = const(0x15)  # Modem deviation setting
    MCSM2 = const(0x16)  # Main Radio Cntrl State Machine configuration
    MCSM1 = const(0x17)  # Main Radio Cntrl State Machine configuration
    MCSM0 = const(0x18)  # Main Radio Cntrl State Machine configuration
    FOCCFG = const(0x19)  # Frequency Offset Compensation configuration
    BSCFG = const(0x1A)  # Bit Synchronization configuration
    AGCCTRL2 = const(0x1B)  # AGC control
    AGCCTRL1 = const(0x1C)  # AGC control
    AGCCTRL0 = const(0x1D)  # AGC control
    WOREVT1 = const(0x1E)  # High byte Event 0 timeout
    WOREVT0 = const(0x1F)  # Low byte Event 0 timeout
    WORCTRL = const(0x20)  # Wake On Radio control
    FREND1 = const(0x21)  # Front end RX configuration
    FREND0 = const(0x22)  # Front end TX configuration
    FSCAL3 = const(0x23)  # Frequency synthesizer calibration
    FSCAL2 = const(0x24)  # Frequency synthesizer calibration
    FSCAL1 = const(0x25)  # Frequency synthesizer calibration
    FSCAL0 = const(0x26)  # Frequency synthesizer calibration
    RCCTRL1 = const(0x27)  # RC oscillator configuration
    RCCTRL0 = const(0x28)  # RC oscillator configuration
    FSTEST = const(0x29)  # Frequency synthesizer calibration control
    PTEST = const(0x2A)  # Production test
    AGCTEST = const(0x2B)  # AGC test
    TEST2 = const(0x2C)  # Various test settings
    TEST1 = const(0x2D)  # Various test settings
    TEST0 = const(0x2E)  # Various test settings

    # Status registers
    PARTNUM = const(0x30)  # Part number
    VERSION = const(0x31)  # Current version number
    FREQEST = const(0x32)  # Frequency offset estimate
    LQI = const(0x33)  # Demodulator estimate for link quality
    RSSI = const(0x34)  # Received signal strength indication
    MARCSTATE = const(0x35)  # Control state machine state
    WORTIME1 = const(0x36)  # High byte of WOR timer
    WORTIME0 = const(0x37)  # Low byte of WOR timer
    PKTSTATUS = const(0x38)  # Current GDOx status and packet status
    VCO_VC_DAC = const(0x39)  # Current setting from PLL calibration module
    TXBYTES = const(0x3A)  # Underflow and number of bytes in TXFIFO
    RXBYTES = const(0x3B)  # Overflow and number of bytes in RXFIFO
    RCCTRL1_STATUS = const(0x3C)  # Last RC oscillator calibration result
    RCCTRL0_STATUS = const(0xF3)  # Last RC oscillator calibration result

    # Command strobes
    SRES = const(0x30)  # Reset chip
    SFSTXON = const(0x31)  # Enable/calibrate frequency synthesizer
    SXOFF = const(0x32)  # Turn off crystal oscillator
    SCAL = const(0x33)  # Calibrate frequency synthesizer and disable
    SRX = const(0x34)  # Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
    STX = const(0x35)  # Enable TX
    SIDLE = const(0x36)  # Exit RX / TX
    SAFC = const(0x37)  # AFC adjustment of freq synthesizer
    SWOR = const(0x38)  # Start automatic RX polling sequence
    SPWD = const(0x39)  # Enter power down mode when CSn goes high
    SFRX = const(0x3A)  # Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
    SFTX = const(0x3B)  # Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
    SWORRST = const(0x3C)  # Reset real time clock to Event1 value
    SNOP = const(0x3D)  # No operation. May be used to get access to the chip status byte.

    # Bit fields for chip status byte
    STATUS_CHIP_RDYn = const(0x80)  # Should be low when using SPI interface
    STATUS_STATE = const(0x70)
    STATUS_FIFO_BYTES_AVAILABLE = const(0x0F)  # Bytes available in RX FIFO or bytes free in TX FIFO

    # Masks to retrieve status bit
    BITS_TX_FIFO_UNDERFLOW = const(0x80)
    BITS_RX_BYTES_IN_FIFO = const(0x7F)
    BITS_MARCSTATE = const(0x1F)

    # Marc states
    MARCSTATE_SLEEP = const(0x00)
    MARCSTATE_IDLE = const(0x01)
    MARCSTATE_XOFF = const(0x02)
    MARCSTATE_VCOON_MC = const(0x03)
    MARCSTATE_REGON_MC = const(0x04)
    MARCSTATE_MANCAL = const(0x05)
    MARCSTATE_VCOON = const(0x06)
    MARCSTATE_REGON = const(0x07)
    MARCSTATE_STARTCAL = const(0x08)
    MARCSTATE_BWBOOST = const(0x09)
    MARCSTATE_FS_LOCK = const(0x0A)
    MARCSTATE_IFADCON = const(0x0B)
    MARCSTATE_ENDCAL = const(0x0C)
    MARCSTATE_RX = const(0x0D)
    MARCSTATE_RX_END = const(0x0E)
    MARCSTATE_RX_RST = const(0x0F)
    MARCSTATE_TXRX_SWITCH = const(0x10)
    MARCSTATE_RXFIFO_OVERFLOW = const(0x11)
    MARCSTATE_FSTXON = const(0x12)
    MARCSTATE_TX = const(0x13)
    MARCSTATE_TX_END = const(0x14)
    MARCSTATE_RXTX_SWITCH = const(0x15)
    MARCSTATE_TXFIFO_UNDERFLOW = const(0x16)

    # Bit masks for chip status state
    STATE_IDLE = const(0x00)  # IDLE state
    STATE_RX = const(0x10)  # Receive mode
    STATE_TX = const(0x20)  # Transmit mode
    STATE_FSTXON = const(0x30)  # Fast TX ready
    STATE_CALIBRATE = const(0x40)  # Frequency synthesizer calibration is running
    STATE_SETTLING = const(0x50)  # PLL is settling
    STATE_RXFIFO_OVERFLOW = const(0x60)  # RX FIFO has overflowed
    STATE_TXFIFO_UNDERFLOW = const(0x70)  # TX FIFO has underflowed

    def __init__(self, spi_id, ss, gd02):
        ESP32_SPI=ESP32(ESP32_S3)
        """ Create a CC1101 object connected to a microcontroller SPI channel

        This class assumes the usage of SPI hardware channels and the
        corresponding (hardwired) pins. Software SPI is not supported.
        Pin gd02 is only used when receiving messages, not when sending.

        :param int spi_id: microcontroller SPI channel id
        :param int ss: microcontroller pin number used for slave select (SS)
        :param int gd02: microcontroller pin number connected to port GD02 of the CC1101
        """
        if spi_id not in ESP32_SPI.SPI_ID_LIST:
            raise ValueError(f"invalid SPI id {spi_id} for {config.BOARD}")

        self.miso = Pin(ESP32_SPI.MISO_PIN_PER_SPI_ID[str(spi_id)])
        self.ss = Pin(ss, mode=Pin.OUT)
        self.gd02 = Pin(gd02, mode=Pin.IN)
        self.deselect()
        self.spi = SPI(spi_id, baudrate=8000000, polarity=0, phase=0, bits=8,
                       firstbit=SPI.MSB)  # use default pins for mosi, miso and sclk
        self.reset()
        self.write_register(CC1101.IOCFG2, 0x29)
        self.write_register(CC1101.IOCFG1, 0x2E)
        print("0x02,0x06=0x02,",self.read_register(CC1101.IOCFG0))
        self.write_register(CC1101.IOCFG0, 0x06)
        print("0x02,0x06=0x02,",self.read_register(CC1101.IOCFG0))
        self.write_register(CC1101.FIFOTHR, 0x47)
        self.write_register(CC1101.SYNC1, 0xD3)
        self.write_register(CC1101.SYNC0, 0x91)
        self.write_register(CC1101.PKTLEN, 0xFF)
        #self.write_register(CC1101.PKTCTRL1, 0x8C)  # only pkts that pass CRC check
        self.write_register(CC1101.PKTCTRL1, 0x04)
        self.write_register(CC1101.PKTCTRL0, 0x05)
        self.write_register(CC1101.ADDR, 0x00)
        self.write_register(CC1101.CHANNR, 0x00)
        self.write_register(CC1101.FSCTRL1, 0x08)
        self.write_register(CC1101.FSCTRL0, 0x00)
        self.write_register(CC1101.FREQ2, 0x10)
        #self.write_register(CC1101.FREQ1, 0x71) # 71 may need tweaking
        self.write_register(CC1101.FREQ1, 0xB4)
        #self.write_register(CC1101.FREQ0, 0xC0) # C0 or 7A may need tweaking
        self.write_register(CC1101.FREQ0, 0x2E)
        self.write_register(CC1101.MDMCFG4, 0xCA)
        self.write_register(CC1101.MDMCFG3, 0x83)
        #self.write_register(CC1101.MDMCFG2, 0x13) # MDMCFG2 Modem Configuration 0x13
        self.write_register(CC1101.MDMCFG2, 0x93)
        self.write_register(CC1101.MDMCFG1, 0x22)
        self.write_register(CC1101.MDMCFG0, 0xF8)
        self.write_register(CC1101.DEVIATN, 0x34)
        self.write_register(CC1101.MCSM2, 0x07)
        self.write_register(CC1101.MCSM1, 0x30)
        self.write_register(CC1101.MCSM0, 0x18)
        self.write_register(CC1101.FOCCFG, 0x16)
        self.write_register(CC1101.BSCFG, 0x6C)
        self.write_register(CC1101.AGCCTRL2, 0x43)
        self.write_register(CC1101.AGCCTRL1, 0x40)
        self.write_register(CC1101.AGCCTRL0, 0x91)
        self.write_register(CC1101.WOREVT1, 0x87)
        self.write_register(CC1101.WOREVT0, 0x6B)
        self.write_register(CC1101.WORCTRL, 0xF8)
        self.write_register(CC1101.FREND1, 0x56)
        self.write_register(CC1101.FREND0, 0x10)
        self.write_register(CC1101.FSCAL3, 0xE9)
        self.write_register(CC1101.FSCAL2, 0x2A)
        self.write_register(CC1101.FSCAL1, 0x00)
        self.write_register(CC1101.FSCAL0, 0x1F)
        self.write_register(CC1101.RCCTRL1, 0x41)
        self.write_register(CC1101.RCCTRL0, 0x00)
        self.write_register(CC1101.FSTEST, 0x59)
        self.write_register(CC1101.PTEST, 0x7F)
        self.write_register(CC1101.AGCTEST, 0x3F)
        self.write_register(CC1101.TEST2, 0x81)
        self.write_register(CC1101.TEST1, 0x35)
        self.write_register(CC1101.TEST0, 0x09)
        
        self.check_reg()

    def select(self):
        """ CC1101 chip select """
        self.ss.value(0)

    def deselect(self):
        """ CC1101 chip deselect """
        self.ss.value(1)

    def spi_wait_miso(self):
        """ Wait for CC1101 SO to go low """
        while self.miso.value() != 0:
            pass

    def reset(self):
        """ CC1101 reset """
        self.deselect()
        time.sleep_us(5)
        self.select()
        time.sleep_us(10)
        self.deselect()
        time.sleep_us(45)
        self.select()

        self.spi_wait_miso()
        self.write_command(CC1101.SRES)
        time.sleep_ms(10)
        self.deselect()
        
    def check_reg(self):
        print("0x00,0x29=0x00,",hex(self.read_register(0x00)))
        print("0x01,0x2E=0x01,",hex(self.read_register(0x01)))
        print("0x02,0x06=0x02,",hex(self.read_register(0x02)))
        print("0x03,0x47=0x03,",hex(self.read_register(0x03)))
        print("0x04,0xD3=0x04,",hex(self.read_register(0x04)))
        print("0x05,0x91=0x05,",hex(self.read_register(0x05)))
        print("0x06,0xFF=0x06,",hex(self.read_register(0x06)))
        print("0x07,0x04=0x07,",hex(self.read_register(0x07)))
        print("0x08,0x05=0x08,",hex(self.read_register(0x08)))
        print("0x09,0x00=0x09,",hex(self.read_register(0x09)))
        print("0x0A,0x00=0x0A,",hex(self.read_register(0x0A)))
        print("0x0B,0x08=0x0B,",hex(self.read_register(0x0B)))
        print("0x0C,0x00=0x0C,",hex(self.read_register(0x0C)))
        print("0x0D,0x10=0x0D,",hex(self.read_register(0x0D)))
        print("0x0E,0xB4=0x0E,",hex(self.read_register(0x0E)))
        print("0x0F,0x2E=0x0F,",hex(self.read_register(0x0F)))
        print("0x10,0xCA=0x10,",hex(self.read_register(0x10)))
        print("0x11,0x83=0x11,",hex(self.read_register(0x11)))
        print("0x12,0x93=0x12,",hex(self.read_register(0x12)))
        print("0x13,0x22=0x13,",hex(self.read_register(0x13)))
        print("0x14,0xF8=0x14,",hex(self.read_register(0x14)))
        print("0x15,0x34=0x15,",hex(self.read_register(0x15)))
        print("0x16,0x07=0x16,",hex(self.read_register(0x16)))
        print("0x17,0x30=0x17,",hex(self.read_register(0x17)))
        print("0x18,0x18=0x18,",hex(self.read_register(0x18)))
        print("0x19,0x16=0x19,",hex(self.read_register(0x19)))
        print("0x1A,0x6C=0x1A,",hex(self.read_register(0x1A)))
        print("0x1B,0x43=0x1B,",hex(self.read_register(0x1B)))
        print("0x1C,0x40=0x1C,",hex(self.read_register(0x1C)))
        print("0x1D,0x91=0x1D,",hex(self.read_register(0x1D)))
        print("0x1E,0x87=0x1E,",hex(self.read_register(0x1E)))
        print("0x1F,0x6B=0x1F,",hex(self.read_register(0x1F)))
        print("0x20,0xF8=0x20,",hex(self.read_register(0x20)))
        print("0x21,0x56=0x21,",hex(self.read_register(0x21)))
        print("0x22,0x10=0x22,",hex(self.read_register(0x22)))
        print("0x23,0xE9=0x23,",hex(self.read_register(0x23)))
        print("0x24,0x2A=0x24,",hex(self.read_register(0x24)))
        print("0x25,0x00=0x25,",hex(self.read_register(0x25)))
        print("0x26,0x1F=0x26,",hex(self.read_register(0x26)))
        print("0x27,0x41=0x27,",hex(self.read_register(0x27)))
        print("0x28,0x00=0x28,",hex(self.read_register(0x28)))
        print("0x29,0x59=0x29,",hex(self.read_register(0x29)))
        print("0x2A,0x7F=0x2A,",hex(self.read_register(0x2A)))
        print("0x2B,0x3F=0x2B,",hex(self.read_register(0x2B)))
        print("0x2C,0x81=0x2C,",hex(self.read_register(0x2C)))
        print("0x2D,0x35=0x2D,",hex(self.read_register(0x2D)))
        print("0x2E,0x09=0x2E,",hex(self.read_register(0x2E)))
        print(" ")
        
    def write_command(self, command):
        """ Write command strobe

        Command strobes share addresses with the status registers
        (address 0x30 to 0x3F). A command strobe must have the
        burst bit set to 0.

        :param int command: strobe byte
        :return int: status byte
        """
        buf = bytearray((command,))
        self.select()
        self.spi_wait_miso()
        self.spi.write(buf)
        self.deselect()
        return buf[0]
    
    def SetRx(self):
        self.strobe_REG(0x36)
        self.strobe_REG(0x34)        
        CC1101.trxstate=2
        
    def CheckRxFifo(self,delay):
        gdo0 = Pin(2)
        self.strobe_REG(0x36);#CC1101_SIDLE
        self.strobe_REG(0x34);   #CC1101_SRX     
        if(self.read_register(0x3B) & 0x7F):#CC1101_RXBYTES-0x3B BYTES_IN_RXFIFO-
            time.sleep(delay/1000);
            return 1
        else:
            return 0
        
    def write_register(self, address, data):
        """ Write single byte to configuration register

        Note that status registers cannot be written to (as that would be
        a command strobe).

        :param int address: byte address of register
        :param int data: byte to write to register
         """
        buf = bytearray(2)
        buf[0] = address | CC1101.WRITE_SINGLE_BYTE
        buf[1] = data
        self.select()
        self.spi_wait_miso()
        self.spi.write(buf)
        self.deselect()
    
    def SpiReadStatus(self,address,spi_id):
        self.miso = Pin(config_1.MISO_PIN_PER_SPI_ID[str(spi_id)])
        value = bytearray(2)
        temp = bytearray(2)
        temp[0]= address | READ_BURST
        self.select()
        self.spi.write_readinto(temp, value)
        self.deselect()
        return value
            
    def SpiReadReg(self,address,spi_id):
        self.miso = Pin(config_1.MISO_PIN_PER_SPI_ID[str(spi_id)])
        value = bytearray(2)
        temp = bytearray(2)
        temp[0]= address | READ_SINGLE_BYTE
        self.select()
        self.spi.write_readinto(temp, value)
        self.deselect()
        return value
    
            
    def CheckCRC(self):
        lqi=self.read_register(0x33)#read_register_status CC1101_LQI-0x33
        crc_ok = get_bit(lqi, 7)
        if (crc_ok == 1):
            return 1
        else:
            self.write_command(CC1101.SFRX)
            self.write_command(CC1101.SRX)
            return 0
                
    def read_register(self, address, register_type=0x80):
        """ Read value from configuration or status register

        Status registers share addresses with command strobes (address 0x30
        to 0x3F). To access a status register the burst bit must be set to 1.
        This is handled by the mask in parameter register_type.

        :param int address: byte address of register
        :param int register_type: C1101.CONFIG_REGISTER (default) or STATUS_REGISTER
        :return int: register value (byte)
        """
        read_buf = bytearray(2)
        write_buf = bytearray(2)
        write_buf[0] = address | register_type
        self.select()
        self.spi_wait_miso()
        self.spi.write_readinto(write_buf, read_buf)

        """ CC1101 SPI/26 Mhz synchronization bug - see CC1101 errata
            When reading the following registers two consecutive reads
            must give the same result to be OK. """
        if address in [CC1101.FREQEST, CC1101.MARCSTATE, CC1101.RXBYTES,
                       CC1101.TXBYTES, CC1101.WORTIME0, CC1101.WORTIME1]:
            value = read_buf[1]
            while True:
                self.spi.write_readinto(write_buf, read_buf)
                if value == read_buf[1]:
                    break
                value = read_buf[1]

        self.deselect()
        return read_buf[1]
    
    def read_register_status(self, address, register_type=0xC0):
        """ Read value from configuration or status register

        Status registers share addresses with command strobes (address 0x30
        to 0x3F). To access a status register the burst bit must be set to 1.
        This is handled by the mask in parameter register_type.

        :param int address: byte address of register
        :param int register_type: C1101.CONFIG_REGISTER (default) or STATUS_REGISTER
        :return int: register value (byte)
        """
        read_buf = bytearray(2)
        write_buf = bytearray(2)
        write_buf[0] = address | register_type
        self.select()
        self.spi_wait_miso()
        self.spi.write_readinto(write_buf, read_buf)

        """ CC1101 SPI/26 Mhz synchronization bug - see CC1101 errata
            When reading the following registers two consecutive reads
            must give the same result to be OK. """
        if address in [CC1101.FREQEST, CC1101.MARCSTATE, CC1101.RXBYTES,
                       CC1101.TXBYTES, CC1101.WORTIME0, CC1101.WORTIME1]:
            value = read_buf[1]
            while True:
                self.spi.write_readinto(write_buf, read_buf)
                if value == read_buf[1]:
                    break
                value = read_buf[1]

        self.deselect()
        return read_buf[1]
    
    def read_register2(self, address, register_type=0x80):
        """ Read value from configuration or status register

        Status registers share addresses with command strobes (address 0x30
        to 0x3F). To access a status register the burst bit must be set to 1.
        This is handled by the mask in parameter register_type.

        :param int address: byte address of register
        :param int register_type: C1101.CONFIG_REGISTER (default) or STATUS_REGISTER
        :return int: register value (byte)
        """
        read_buf = bytearray(2)
        write_buf = bytearray(2)
        write_buf[0] = address | register_type
        self.select() #ss low
        self.spi_wait_miso()
        self.spi.write_readinto(write_buf, read_buf)

        """ CC1101 SPI/26 Mhz synchronization bug - see CC1101 errata
            When reading the following registers two consecutive reads
            must give the same result to be OK. """
        if address in [CC1101.FREQEST, CC1101.MARCSTATE, CC1101.RXBYTES,
                       CC1101.TXBYTES, CC1101.WORTIME0, CC1101.WORTIME1]:
            value = read_buf[1]
            while True:
                self.spi.write_readinto(write_buf, read_buf)
                if value == read_buf[1]:
                    break
                value = read_buf[1]

        self.deselect()#ss high
        return read_buf[1]

    def read_register_median_of_3(self, address):
        """ Read register 3 times and return median value """
        lst = list()
        for _ in range(3):
            lst.append(self.read_register(address))
        lst.sort()
        return lst[1]

    def read_burst(self, address, length):
        """ Read values from consecutive configuration registers

        :param int address: start register address
        :param int length: number of registers to read
        :return bytearray: values read (bytes)
        """
        buf = bytearray(length + 1)
        buf[0] = address | CC1101.READ_BURST
        self.select()
        self.spi_wait_miso()
        self.spi.write_readinto(buf, buf)
        self.deselect()
        return buf[1:]
   
    def write_burst(self, address, data):
        """ Write data to consecutive registers

        :param int address: start register address
        :param bytearray data: values to write (full array is written)
        """
        buf = bytearray(1)
        buf[0] = address | CC1101.WRITE_BURST
        buf[1:1] = data  # append data
        self.select()
        self.spi_wait_miso()
        self.spi.write(buf)
        self.deselect()
        
    def get_lqi(self):
        lqi=0
        lqi=self.read_register(0x33);
        return lqi;
    def get_rssi(self):
        rssi=0
        rssi=self.read_register(0x34)
        if (rssi >= 128):
            rssi = (rssi-256)/2-74
        else:
            rssi = (rssi/2)-74
        return rssi;
    def ReceiveData(self, buffer):
        size = 0
        byte = 0 
        
        if (1==1):
            size=self.read_register_status(0x3F)
            buffer=self.read_burst(0x3F,size)#2-number of bytes
            status=self.read_burst(0x3F,2)
            print("buf",buffer)
            print("status",status)
            print("size",size)
            self.write_command(0x36)
            if (1==1):#10<size>15
                return buffer
        else:
            self.write_command(0x3A)
            self.write_command(0x34)
            return 0;
    
    def strobe_REG(self,address):
        buf = bytearray(2)
        buf[0] = address | CC1101.WRITE_SINGLE_BYTE
        buf[1] = 0
        self.select()
        self.spi_wait_miso()
        self.spi.write(buf)
        self.deselect()
        
    def receive_data(self, length):
        """ Read available bytes from the FIFO

        :param int length: max number of bytes to read
        :return bytearray: bytes read (can have len() of 0)
        """
        rx_bytes = self.read_register(CC1101.RXBYTES, CC1101.STATUS_REGISTER) & CC1101.BITS_RX_BYTES_IN_FIFO

        # Check for
        if (self.read_register(CC1101.MARCSTATE, CC1101.STATUS_REGISTER) & CC1101.BITS_MARCSTATE) == CC1101.MARCSTATE_RXFIFO_OVERFLOW:
            buf = self.read_burst(CC1101.RXFIFO, rx_bytes)  # RX FIFO overflow: return empty array
        else:
            buf = self.read_burst(CC1101.RXFIFO, rx_bytes)

        self.write_command(CC1101.SIDLE)
        self.write_command(CC1101.SFRX)  # Flush RX buffer
        self.write_command(CC1101.SRX)  # Switch to RX state

        return buf
    def send_data(self, data):
        """ Send data

        :param bytearray data: bytes to send (len(data) may exceed FIFO size)
        """
        DATA_LEN = CC1101.FIFO_BUFFER_SIZE - 5

        self.write_command(CC1101.SIDLE)

        # Clear TX FIFO if needed
        if self.read_register(CC1101.TXBYTES, CC1101.STATUS_REGISTER) & CC1101.BITS_TX_FIFO_UNDERFLOW:
            self.write_command(CC1101.SIDLE)
            self.write_command(CC1101.SFTX)

        self.write_command(CC1101.SIDLE)

        length = len(data) if len(data) <= DATA_LEN else DATA_LEN

        self.write_burst(CC1101.TXFIFO, data[:length])

        self.write_command(CC1101.SIDLE)
        self.write_command(CC1101.STX)  # Start sending packet

        index = 0

        if len(data) > DATA_LEN:
            # More data to send
            index += length

            while index < len(data):
                while True:
                    tx_status = self.read_register_median_of_3(CC1101.TXBYTES | CC1101.STATUS_REGISTER) & CC1101.BITS_RX_BYTES_IN_FIFO
                    if tx_status <= (DATA_LEN - 2):
                        break

                length = DATA_LEN - tx_status
                length = len(data) - index if (len(data) - index) < length else length

                for i in range(length):
                    self.write_register(CC1101.TXFIFO, data[index + i])

                index += length

        # Wait until transmission is finished (TXOFF_MODE is expected to be set to 0/IDLE or TXFIFO_UNDERFLOW)
        while True:
            marcstate = self.read_register(CC1101.MARCSTATE, CC1101.STATUS_REGISTER) & CC1101.BITS_MARCSTATE
            if marcstate in [CC1101.MARCSTATE_IDLE, CC1101.MARCSTATE_TXFIFO_UNDERFLOW]:
                break


if __name__ == "__main__":
    # Demo the connection to a CC1101 by reading values from the chip
    ESP32_SPI=ESP32(ESP32_S3)
    cc1101 = CC1101(ESP32_SPI.SPI_ID, ESP32_SPI.SS_PIN, ESP32_SPI.GD02_PIN)

    # Read status byte
    status = cc1101.write_command(CC1101.SNOP)
    print("Status byte", hex(status), bin(status))

    # Read version
    version = cc1101.read_register(CC1101.VERSION, CC1101.STATUS_REGISTER)
    print("VERSION", hex(version))

    # Prove burst and single register access deliver same results
    burst = cc1101.read_burst(CC1101.IOCFG2, 5)
    for i in range(len(burst)):
        print(hex(burst[i]), end=' ')
    print()

    for register in (CC1101.IOCFG2, CC1101.IOCFG1, CC1101.IOCFG0):
        print(hex(cc1101.read_register(register)), end=' ')
    print()

