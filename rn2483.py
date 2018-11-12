#!/usr/bin/python3

import time as t
import sys
import serial

from tools import GPIO

TTYADDR = "/dev/ttyAMA0"


class RN2843():
    """Microchip RN2483 LoRa Wireless Module

    This class implements all the functions that are available in the
    modules for evaluation.
    """

    COMMANDS = {

        # SYS Commmands
        "SYS_SLEEP": b"sys sleep {}",
        "SYS_RST": b"sys reset",
        "SYS_FACRST": b"sys factoryRESET",
        "SYS_ERASEFW": b"sys eraseFW",

        # SYS set commands
        "SYS_NVM_SET": b"sys set nvm {} {}",
        "SYS_PINMODE_SET": b"sys set pinmode {} {}",
        "SYS_PINCFG_SET": b"sys set pindig {} {}",

        # SYS get commands
        "SYS_VER": b"sys get ver",
        "SYS_NVM": b"sys get nvm {}",
        "SYS_VDD": b"sys get vdd",
        "SYS_HWEUI": b"sys get hweui",
        "SYS_PINDIG": b"sys get pindig {}",
        "SYS_PINANA": b"sys get pinana {}",

        # MAC Commands
        "MAC_RSTBAND": b"mac reset {}",
        "MAC_TX": b"mac tx {} {} {}",
        "MAC_JOIN": b"mac join {}",
        "MAC_SAVE": b"mac save",
        "MAC_FORCE": b"mac forceENABLE",
        "MAC_PAUSE": b"mac pause",
        "MAC_RESUME": b"mac resume",

        # MAC set Commands
        "MAC_DEVADDR_SET": b"mac set devaddr {}",
        "MAC_DEVEUI_SET": b"mac set deveui {}",
        "MAC_APPEUI_SET": b"mac set appeui {}",
        "MAC_NWKSKEY_SET": b"mac set nwkskey {}",
        "MAC_APPSKEY_SET": b"mac set appskey {}",
        "MAC_APPKEY_SET": b"mac set appkey {}",
        "MAC_PWRIDX_SET": b"mac set pwridx {}",
        "MAC_DR_SET": b"mac set dr {}",
        "MAC_ADR_SET": b"mac set adr {}",
        "MAC_BAT_SET": b"mac set bat {}",
        "MAC_RETX_SET": b"mac set retx {}",
        "MAC_LINKCHK_SET": b"mac set linkchk {}",
        "MAC_RXDELAY1_SET": b"mac set rxdelay1 {}",
        "MAC_AR_SET": b"mac set ar {}",
        "MAC_RX_SET": b"mac set rx2 {} {}",
        "MAC_SYNC_SET": b"mac set sync {}",
        "MAC_UPCTR_SET": b"mac set upctr {}",
        "MAC_DNCTR_SET": b"mac set dnctr {}",

        # MAC get Commands
        "MAC_DEVADDR": b"mac get devaddr",
        "MAC_DEVEUI": b"mac get deveui",
        "MAC_APPEUI": b"mac get appeui",
        "MAC_DR": b"mac get dr",
        "MAC_BAND": b"mac get band",
        "MAC_PWRIDX": b"mac get pwridx",
        "MAC_ADR": b"mac get adr",
        "MAC_RETX": b"mac get retx",
        "MAC_RXDELAY1": b"mac get rxdelay1",
        "MAC_RXDELAY2": b"mac get rxdelay2",
        "MAC_AR": b"mac get ar",
        "MAC_RX2": b"mac get rx2 {}",
        "MAC_DYCLEPS": b"mac get dcycleps",
        "MAC_MRGN": b"mac get mrgn",
        "MAC_GWNB": b"mac get gwnb",
        "MAC_STATUS": b"mac get status",
        "MAC_SYNC": b"mac get sync",
        "MAC_UPCTR": b"mac get upctr",
        "MAC_DNCTR": b"mac get dnctr",

        # RADIO Commands
        "RADIO_RX": b"radio rx {}",
        "RADIO_TX": b"radio tx {}",
        "RADIO_CW": b"radio cw {}",

        # RADIO set Commands
        "RADIO_BT_SET": b"radio set bt {}",
        "RADIO_MOD_SET": b"radio set mod {}",
        "RADIO_FREQ_SET": b"radio set freq {}",
        "RADIO_PWR_SET": b"radio set pwr {}",
        "RADIO_SF_SET": b"radio set sf {}",
        "RADIO_AFCBW_SET": b"radio set afcbw {}",
        "RADIO_RXBW_SET": b"radio set rxbw {}",
        "RADIO_BITRATE_SET": b"radio set bitrate {}",
        "RADIO_FDEV_SET": b"radio set fdev {}",
        "RADIO_PRLEN_SET": b"radio set prlen {}",
        "RADIO_CRC_SET": b"radio set crc {}",
        "RADIO_IQI_SET": b"radio set iqi {}",
        "RADIO_CR_SET": b"radio set cr {}",
        "RADIO_WDT_SET": b"radio set wdt {}",
        "RADIO_SYNC_SET": b"radio set sync {}",
        "RADIO_BW_SET": b"radio set bw {}",

        # RADIO get Commands
        "RADIO_BT": b"radio get bt",
        "RADIO_MOD": b"radio get mod",
        "RADIO_FREQ": b"radio get freq",
        "RADIO_PWR": b"radio get pwr",
        "RADIO_SF": b"radio get sf",
        "RADIO_AFCBW": b"radio get afcbw",
        "RADIO_RXBW": b"radio get rxbw",
        "RADIO_BITRATE": b"radio get bitrate",
        "RADIO_FDEV": b"radio get fdev",
        "RADIO_PRLEN": b"radio get prlen",
        "RADIO_CRC": b"radio get crc",
        "RADIO_IQI": b"radio get iqi",
        "RADIO_CR": b"radio get cr",
        "RADIO_WDT": b"radio get wdt",
        "RADIO_SYNC": b"radio get sync",
        "RADIO_BW": b"radio get bw",
    }

    def __init__(self, port=None, reset_gpio_path=None):
        #reset the module
        self._reset_gpio = GPIO(reset_gpio_path, "out", 0)
        t.sleep(0.5)
        self._reset_gpio.set_value(1)

        try:
            self._serial = serial.Serial(port,
                                         baudrate=57600,
                                         bytesize=serial.EIGHTBITS,
                                         parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_ONE,
                                         write_timeout=1.0)
        except:
            return -1

        self._check_connection()


    def _check_connection(self):

    def _write_command(self, command):
        cmd
        self._serial.


    def getConn(self):
        return self.ser

    def closeConn(self):
        if not sys.platform == "pyboard":
            self.ser.close()
        else:
            self.ser.deinit()

    def execCmd(self, cmd):
        if self.verbose:
            print("Attempting to execute command: {}\r\n".format(cmd))
        self.ser.write(cmd)
        self.ser.write("\r\n")
        line = self.ser.readline()
        if self.verbose:
            print("Command response: {}\r\n".format(line))
        return line

    # SYS COMMANDS

    def getSysVersion(self):
        return self.execCmd(self.COMMANDS["SYS_VER"])

    def getSysVdd(self):
        return self.execCmd(self.COMMANDS["SYS_VDD"])

    def getSysHweui(self):
        return self.execCmd(self.COMMANDS["SYS_HWEUI"])

    def getSysNvmAt(self, nvmaddr):
        return self.execCmd(self.COMMANDS["SYS_NVMAT"].format(nvmaddr))

    def setSysNvmAg(self, nvmaddr, hexByte):
        return self.execCmd(self.COMMANDS["SYS_NVMSET"].format(nvmaddr, hexByte))

    def sysSleep(self, millis):
        return self.execCmd(self.COMMANDS["SYS_SLEEP"].format(millis))

    def sysReset(self):
        return self.execCmd(self.COMMANDS["SYS_RST"])

    # MAC COMMANDS

    def getMacDeveui(self):
        return self.execCmd(self.COMMANDS["MAC_DEVEUI"])


def main():
    """ """
    if not sys.platform == "pyboard":
        # in this case a Raspi
        ser = serial.Serial(TTYADDR, 57600, timeout=5)
    else:
        ser = pyb.UART(4, baudrate=57600, timeout=5000)

    loraClient = RN2XX3(ser=ser, verbose=False)

    print("\r\nCHIP FIRMWARE VERSION IS: {}".format(loraClient.getSysVersion()))
    time.sleep(1)
    print("\r\nCHIP HARDWARE EUI IS: {}".format(loraClient.getSysHweui()))
    time.sleep(1)
    print("\r\nCHIP VDD READING IS: {}".format(loraClient.getSysVdd()))
    time.sleep(1)
    print("\r\nCHIP MAC DEVEUI IS: {}".format(loraClient.getMacDeveui()))
    print("SLEEPING WITH RN2XX3 CHIP\r\n {}".format(loraClient.sysSleep(3000)))
    print("SETTING NVM AT 3FF TO 88: {}".format(loraClient.setSysNvmAg('3FF', '88')))
    time.sleep(0.25)
    print("\nNVM MEMORY VALUE AT: 3FF IS: {}".format(loraClient.getSysNvmAt('3FF')))
    print("RESETTING CLIENT {}".format(loraClient.sysReset()))
    loraClient.closeConn()


if __name__ == "__main__":
    main()
