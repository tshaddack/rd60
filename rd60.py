#!/usr/bin/python3

import socket
from struct import pack,unpack
from time import sleep,monotonic
from sys import argv,exit,stdin,stderr

# other imports are placed where they are needed, to avoid crashing whole software instead of a single function on a missing dependency


DEFAULT_SERPORT='/dev/ttyUSB0'
DEFAULT_BAUDRATE=115200
DEFAULT_TCPPORT=8888

DEFAULT_MODBUS_ADDR=1

stdlog=stderr


####################
##
##  LOW LEVEL SERIAL
##
####################


class LowLevelSerPort:
  import serial
  serport='/dev/ttyUSB0'            # target serial port
  baudrate=0
  port=None                         # physical port instance
  verbconn=False
  verbport=False # DEBUG
  #verbport=True # DEBUG
  timeout=3
  connretries=5
  connected=False

  def __init__(self,portname='/dev/ttyUSB0',baudrate=DEFAULT_BAUDRATE):
    self.serport=portname
    self.baudrate=baudrate
    pass

  def connect(self):
    if self.verbconn: print('SERPORT:connecting to',self.serport,'@',self.baudrate,file=stdlog)
    for t in range(0,self.connretries):
      try:
        if t>0: print('retrying...',t)
        #self.port=self.serial.Serial(self.serport,self.baudrate, timeout=self.timeout)
        self.port=self.serial.serial_for_url(self.serport,self.baudrate, timeout=self.timeout)
        self.connected=True
        break
      except Exception as e:
        print('PORTCONNERR:',e)
        sleep(1)
    if not self.connected:
      print('ERRPORTCONN: cannot connect to',self.serport,'- too many retries. Aborting.',file=stdlog)
      exit(12)
    if self.verbconn: print('SERPORT:connected',file=stdlog)
    return None

  def close(self):
    self.port.close()
    if self.verbconn: print('SERPORT:closed',file=stdlog)
    return None

  def send(self,raw,showpacket=None):
    if showpacket!=None and self.verbconn: showpacket(raw,name='SERPORT:SEND',check=False)
    self.port.write(raw)
    return False

  def recv(self,l,showpacket=None):
    res=self.port.read(l)
    if showpacket!=None and self.verbconn: showpacket(res,name='SERPORT:RECV',check=False)
    return res

  def recvflush(self):
    #n=self.port.in_waiting
    self.port.reset_input_buffer()
    return 0



####################
##
##  LOW LEVEL TCP/IP
##
####################

class LowLevelTcpPort:
  ipaddr=None             # target IP
  ipport=None             # target port
  sock=None
  verbconn=False
  verbport=False
  timeout=3
  flushtimeout=0.02
  connretries=5
  connected=False

  reconnect=True

  def __init__(self,addr,port):
    self.ipaddr=addr
    self.ipport=port

  def connect(self):
    if self.verbconn: print('SOCK:connecting to',self.ipaddr,':',self.ipport,file=stdlog)
    self.sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.settimeout(self.timeout)
    for t in range(0,self.connretries):
      try:
        if t>0: print('SOCK:connection retrying...',t,file=stdlog)
        self.sock.connect( (self.ipaddr,self.ipport) )
        self.connected=True
        break
      except Exception as e:
        print('SOCKCONNERR:',e)
        sleep(min(0.5+t,5)) # increase retries delay, max. 5s
    if not self.connected:
      print('ERRSOCKCONN: cannot connect to',self.ipaddr,':',self.ipport,'- too many retries. Aborting.',file=stdlog)
      exit(12)
    #raise(BaseException('MODBUS failure: too many retries!')) # TODO: cleaner fail
    if self.verbconn: print('SOCK:connected',file=stdlog)
    return self.sock

  def close(self):
    if self.verbconn: print('SOCK:closed',file=stdlog)
    self.sock.close()

  def send(self,raw,showpacket=None):
    if showpacket!=None and self.verbport: showpacket(raw,name='SOCK:SEND',check=False,file=stdlog)
    try:  return self.sock.sendall(raw)
    except socket.error as e:
      print('SOCK:SEND:ERR:',e,file=stderr)
      if self.reconnect: self.connect()

  def recv(self,l,showpacket=None):
    try: res=self.sock.recv(l)
    except socket.error as e:
      print('SOCK:RECV:ERR:',e,file=stderr)
      if self.reconnect:
        try: self.close()
        except Exception as e: print('SOCK:CLOSE:ERR:',e,file=stderr)
        self.connect()
        return b''
      else:
        print('SOCK:RECV:aborting',file=stderr)
        exit(15)
    if showpacket!=None and self.verbport: showpacket(res,name='SOCK:RECV',check=False,file=stdlog)
    return res

  def recvflush(self):
    #print('flush')
    self.sock.settimeout(self.flushtimeout)
    try: n=len(self.sock.recv(1024))
    except socket.timeout: n=0
    self.sock.settimeout(self.timeout)
    return n


############################
##
##  MODBUS OVER SERIAL OR IP
##
############################


class RDModbus:
  # dummy init

  verb=True
  verbm=False       # show modbus-level tx/rx data

  dodelay=False

  usesocket=False
  comm=None

  timeout=3
  retries=10

  MODBUS_FUNC_READMULTI16=0x03
  MODBUS_FUNC_WRITEMULTI16=0x10
  MODBUS_FUNC_WRITESINGLE16=0x06


  def __init__(self,addr=DEFAULT_MODBUS_ADDR):
    self.addr=addr

  def initport(self,comm):
    self.comm=comm

  def setverb(self,verb):
    self.comm.verbconn=verb;self.verbm=verb

  def setverbm(self,verb):
    self.comm.verbconn=False;self.verbm=verb

  def strpacket(self,p):
    return ':'.join(f'{x:02x}' for x in p)

  # show hexdump of packet, optionally check crc
  def showpacket(self,p,name='',check=False):
    if self.verb:
      print(name,':'.join(f'{x:02x}' for x in p),end='',file=stdlog)
      if p==b'': print(name,'(empty)')
      elif check:
        if self.modbus_check_crc(p): print(' (correct)',file=stdlog)
        else: print(' (CRC FAIL)',file=stdlog)
      else: print(file=stdlog)


#### INCLUDED THIRD-PARTY FILE, slightly reformatted for compactness
#### from modbus_crc module, to avoid third-party dependency to install
#### https://pypi.org/project/modbus-crc/
  """
  CRC-16 calculation for Modbus protocol
  Copyright (c) 2023 webtoucher
  Distributed under the BSD 3-Clause license. See LICENSE for more info.
  """
  # WARN: broken strings must start on beginning or unwanted \x20 are introduced and calculations fail mysteriously
  MODBUSCRC_LOW_BYTES = b'\
\x00\xC0\xC1\x01\xC3\x03\x02\xC2\xC6\x06\x07\xC7\x05\xC5\xC4\x04\xCC\x0C\x0D\xCD\x0F\xCF\xCE\x0E\x0A\xCA\xCB\x0B\xC9\x09\x08\xC8\xD8\x18\x19\xD9\x1B\xDB\xDA\x1A\x1E\xDE\xDF\x1F\xDD\x1D\x1C\xDC\x14\xD4\xD5\x15\xD7\x17\x16\xD6\xD2\x12\x13\xD3\x11\xD1\xD0\x10\
\xF0\x30\x31\xF1\x33\xF3\xF2\x32\x36\xF6\xF7\x37\xF5\x35\x34\xF4\x3C\xFC\xFD\x3D\xFF\x3F\x3E\xFE\xFA\x3A\x3B\xFB\x39\xF9\xF8\x38\x28\xE8\xE9\x29\xEB\x2B\x2A\xEA\xEE\x2E\x2F\xEF\x2D\xED\xEC\x2C\xE4\x24\x25\xE5\x27\xE7\xE6\x26\x22\xE2\xE3\x23\xE1\x21\x20\xE0\
\xA0\x60\x61\xA1\x63\xA3\xA2\x62\x66\xA6\xA7\x67\xA5\x65\x64\xA4\x6C\xAC\xAD\x6D\xAF\x6F\x6E\xAE\xAA\x6A\x6B\xAB\x69\xA9\xA8\x68\x78\xB8\xB9\x79\xBB\x7B\x7A\xBA\xBE\x7E\x7F\xBF\x7D\xBD\xBC\x7C\xB4\x74\x75\xB5\x77\xB7\xB6\x76\x72\xB2\xB3\x73\xB1\x71\x70\xB0\
\x50\x90\x91\x51\x93\x53\x52\x92\x96\x56\x57\x97\x55\x95\x94\x54\x9C\x5C\x5D\x9D\x5F\x9F\x9E\x5E\x5A\x9A\x9B\x5B\x99\x59\x58\x98\x88\x48\x49\x89\x4B\x8B\x8A\x4A\x4E\x8E\x8F\x4F\x8D\x4D\x4C\x8C\x44\x84\x85\x45\x87\x47\x46\x86\x82\x42\x43\x83\x41\x81\x80\x40'
  MODBUSCRC_HIGH_BYTES = b'\
\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\
\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\
\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\
\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40\x00\xC1\x81\x40\x01\xC0\x80\x41\x00\xC1\x81\x40\x01\xC0\x80\x41\x01\xC0\x80\x41\x00\xC1\x81\x40'
  def modbus_crc16(self,data: bytes) -> bytes:  # Calculate CRC-16 for Modbus
      crc_high = 0xFF;crc_low = 0xFF
      for byte in data:
          index = crc_high ^ int(byte);crc_high = crc_low ^ self.MODBUSCRC_HIGH_BYTES[index];crc_low = self.MODBUSCRC_LOW_BYTES[index]
      return bytes([crc_high, crc_low])
  def modbus_add_crc(self,packet: bytes) -> bytes: #Add CRC-16 to bytes packet
      return packet + self.modbus_crc16(packet)
  def modbus_check_crc(self,packet: bytes) -> bool: #Validate signed bytes packet
      return self.modbus_crc16(packet) == b'\x00\x00'
#### INCLUDE END


  # add crc to packet, send
  def msend(self,packet):
    if self.comm.recvflush()>0: self.mprinterr('nonzero recv flush')
    packet=self.modbus_add_crc(packet)
    if self.verbm: self.showpacket(packet,name='MODBUS:SEND',check=True)
    self.comm.send(packet,self.showpacket)

  # receive packet of known length
  def mrecvlen(self,l):
    packet=self.comm.recv(l,self.showpacket)
    return packet

  def mrecvvar(self):
    packet=self.comm.recv(3,self.showpacket)
    if len(packet)<3: return packet
    num=packet[2]+2 # length in byte3, plus 2 bytes for CRC
    packet+=self.comm.recv(num,self.showpacket)
    return packet

  # receive packet of variable length specified in 3rd byte
  def mrecv(self,l=0):
    if l==0: packet=self.mrecvvar()
    else:    packet=self.mrecvlen(l)
    if self.verbm: self.showpacket(packet,name='MODBUS:RECV',check=True)
    return packet,self.modbus_check_crc(packet)

  # print error; todo, stderr here
  def mprinterr(self,*s):
    print('ERRMODBUS:',*s,file=stdlog)

  def getmodbuserrstr(self,func, packet,errno):
    packetstr=self.strpacket(packet)+':xx:xx' # CRC is added later
    s=' packet='+packetstr
    s2=f' func={func} addr={packet[2]:02x}{packet[3]:02x} len={packet[4]:02x}{packet[5]:02x}'+s
    if errno==1:  return 'Illegal function:'+s2
    if errno==2:  return 'Illegal data address:'+s2
    if errno==3:  return 'Illegal data value:'+s2
    if errno==4:  return 'Slave device failure:'+s2
    if errno==5:  return 'acknowledge (not error)'
    if errno==6:  return 'Slave busy'
    if errno==7:  return 'NACK, negative acknowledge:'+s2
    if errno==8:  return 'Memory parity error:'+s2
    if errno==10: return 'Gateway path unavailable:'+s2
    if errno==11: return 'Gateway target not responding:'+s2
    return f'UNKNOWN ERROR CODE 0x{errno:02x}:'+s2


  # send-receive pair, with timeout
  def msendrecv_attempt(self,func,packet,resplen=0,expectlen=-1):
    packet=pack('>BB',self.addr,func)+packet
    if self.dodelay: self.dodelay=True;sleep(0.005)
    self.msend(packet)
    try:
      res,state=self.mrecv(resplen)
    except socket.timeout: self.mprinterr('timeout');return False
    except Exception as e: self.mprinterr('exception:',e);return False
    if state==False:
      if res==b'': self.mprinterr('recv no data!'); return False
      else: self.mprinterr('recv checksum failed!'); return False
    # check if we're getting the right response, with the right length
    if res[0]!=packet[0]: self.mprinterr('recv response address not matching!');return False
    if res[1]!=packet[1]:
      if res[1]&0x80==0x80:
        self.mprinterr(self.getmodbuserrstr(func,packet,res[2]))
      else: self.mprinterr('recv response function not matching!');return False
    if func==self.MODBUS_FUNC_READMULTI16:
      if int(res[2])!=expectlen: self.mprinterr('recv expected var length not matching!',res[2],expectlen);return False
    else:
      if len(res)!=8: self.mprinterr('recv fixed length not matching!',res[2],8);return False
    return res

  # send-receive pair, calling timeoutable pair, retrying on failure
  def msendrecv(self,func,packet,resplen=0,expectlen=-1):
    for t in range(0,self.retries):
      if t>0 and (self.comm.verbconn or self.comm.verbport or self.verbm): print(f'retrying ({t+1}/{self.retries})...',file=stdlog)
      resp=self.msendrecv_attempt(func,packet,resplen=resplen,expectlen=expectlen)
      if resp != False: return resp
    print('ERROR: MODBUS failure, too many retries. Aborting.',file=stdlog)
    exit(10)
    #raise(BaseException('MODBUS failure: too many retries!')) # TODO: cleaner fail


  # unpack array of bytes to array of 16bit unsigned integers
  def get16bitarr(self,packet):
    l=int(len(packet)/2)
    #print(len(packet),l,'>'+'H'*l,packet)
    a=unpack('>'+'H'*l,packet)
    #print('array:',a)
    return list(a)


  # responds with array of words
  def modbus_readregs(self,start,num):
    if self.verbm: print('MODBUS:CMND:READREGS',start,num,file=stdlog)
    packet=pack('>HH',start,num)
    res=self.msendrecv(self.MODBUS_FUNC_READMULTI16,packet,expectlen=num*2)
    return self.get16bitarr(res[3:-2])


  # echoes back the written value
  def modbus_writereg(self,start,val):
    if isinstance(val,float): val=int(val)
    if self.verbm: print('MODBUS:CMND:WRITEREG',start,'=',val,file=stdlog)
    if start<0: print('ERR: start address < 0');return None
    packet=pack('>HH',start,val)
    return self.msendrecv(self.MODBUS_FUNC_WRITESINGLE16,packet,resplen=8)


  def modbus_writeregs(self,start,valarr):
    if self.verbm: print('MODBUS:CMND:WRITEREGS',start,'=',valarr,file=stdlog)
    if start<0: print('ERR: start address < 0');return None
    packet=pack('>HHB'+len(valarr)*'H',start,len(valarr),2*len(valarr),*valarr)
    return self.msendrecv(self.MODBUS_FUNC_WRITEMULTI16,packet,resplen=8)



# array of numbers into array of bytes
def num2packet(arr): return pack('B'*len(arr),*arr)
# test packet with reading of couple registers from address 1
modbus_test=num2packet([1,3,0,0x50,0,4])








##################################
##
##  HIGH LEVEL INSTRUMENT-SPECIFIC
##
##################################


class PSU_RD60XX:
  # registers list borrowed from https://github.com/ShayBox/Riden
  # who borrowed it from https://github.com/Baldanos/rd6006
  # RD6024 has 512 registers, most zero
  RD_REGS={ # number: [shortname, read/write, description]
  # Init
       0 : [ 'ID'       , 'r'  , 'type ID' ],
       1 : [ 'SN_H'     , 'r'  , 'serial number H' ],
       2 : [ 'SN_L'     , 'r'  , 'serial number L' ],
       3 : [ 'FW'       , 'r'  , 'firmware version *100' ],
  # Info
       4 : [ 'INT_C_S'  , 'r'  , 'internal sensor failure' ],
       5 : [ 'INT_C'    , 'r'  , 'internal temperature C' ],
       6 : [ 'INT_F_S'  , 'r'  , 'internal sensor failure' ],
       7 : [ 'INT_F'    , 'r'  , 'internal temperature F' ],
       8 : [ 'V_SET'    , 'rw' , 'set voltage' ],
       9 : [ 'I_SET'    , 'rw' , 'set current' ],
      10 : [ 'V_OUT'    , 'r'  , 'actual voltage' ],
      11 : [ 'I_OUT'    , 'r'  , 'actual current' ],
      12 : [ 'AH'       , 'r'  , 'AH' ], # what is this? readonly or rw?
      13 : [ 'P_OUT'    , 'r'  , 'actual power' ],
      14 : [ 'V_IN'     , 'r'  , 'input voltage' ],
      15 : [ 'KEYPAD'   , 'r'  , 'keyboard locked' ],
      16 : [ 'OVP_OCP'  , 'r'  , 'OVP-OCP active' ],
      17 : [ 'CV_CC'    , 'r'  , 'output at constant current' ],
      18 : [ 'OUTPUT'   , 'rw' , 'output enabled' ],
      19 : [ 'PRESET'   , 'rw' , 'preset number' ], # selects memory to use
      20 : [ 'I_RANGE'  , 'r'  , 'RD6012p current range' ],   # Used on RD6012p; imult=1000 in 6A range (I_RANGE=0), imult=100 in 12A range (I_RANGE=1)
  # Unused/Unknown 21-31
      32 : [ 'BAT_MODE' , 'r'  , 'battery mode enabled' ],
      33 : [ 'V_BAT'    , 'r'  , 'battery voltage' ],
      34 : [ 'EXT_C_S'  , 'r'  , 'external sensor failure' ],
      35 : [ 'EXT_C'    , 'r'  , 'external sensor temperature C' ],
      36 : [ 'EXT_F_S'  , 'r'  , 'external sensor failure' ],
      37 : [ 'EXT_F'    , 'r'  , 'external sensor temperature F' ],
      38 : [ 'AH_H'     , 'r'  , 'amphour H' ],
      39 : [ 'AH_L'     , 'r'  , 'amphour L' ],
      40 : [ 'WH_H'     , 'r'  , 'watthour H' ],
      41 : [ 'WH_L'     , 'r'  , 'watthour L' ],
  # Unused/Unknown 42-47
  # Date
      48 : [ 'YEAR'     , 'rw' , 'clock year' ],
      49 : [ 'MONTH'    , 'rw' , 'clock month' ],
      50 : [ 'DAY'      , 'rw' , 'clock day' ],
  # Time
      51 : [ 'HOUR'     , 'rw' , 'clock hour' ],
      52 : [ 'MINUTE'   , 'rw' , 'clock minute' ],
      53 : [ 'SECOND'   , 'rw' , 'clock second' ],
  # Unused/Unknown 54
  # Calibration
  # DO NOT CHANGE Unless you know what you're doing!
      55 : [ 'V_OUT_ZERO'   , 'rw' , 'calibration V_OUT_ZERO' ],
      56 : [ 'V_OUT_SCALE'  , 'rw' , 'calibration V_OUT_SCALE' ],
      57 : [ 'V_BACK_ZERO'  , 'rw' , 'calibration V_BACK_ZERO' ],
      58 : [ 'V_BACK_SCALE' , 'rw' , 'calibration V_BACK_SCALE' ],
      59 : [ 'I_OUT_ZERO'   , 'rw' , 'calibration I_OUT_ZERO' ],
      60 : [ 'I_OUT_SCALE'  , 'rw' , 'calibration I_OUT_SCALE' ],
      61 : [ 'I_BACK_ZERO'  , 'rw' , 'calibration I_BACK_ZERO' ],
      62 : [ 'I_BACK_SCALE' , 'rw' , 'calibration I_BACK_SCALE' ],
  # Unused/Unknown 63-65
  # Settings/Options
      66 : [ 'OPT_TAKE_OK'  , 'rw' , 'confirm memory select' ],
      67 : [ 'OPT_TAKE_OUT' , 'rw' , 'output enabled on memory select' ],
      68 : [ 'OPT_BOOT_POW' , 'rw' , 'output enabled on boot' ],
      69 : [ 'OPT_BUZZ'     , 'rw' , 'buzzer enabled' ],
      70 : [ 'OPT_LOGO'     , 'rw' , 'logo enabled on boot' ],
      71 : [ 'OPT_LANG'     , 'rw' , 'interface language' ],
      72 : [ 'OPT_LIGHT'    , 'rw' , 'display backlight level' ],
  # Unused/Unknown 73-79
  # Presets
      80 : [ 'M0_V'     , 'rw' , 'currently set V' ],
      81 : [ 'M0_I'     , 'rw' , 'currently set I' ],
      82 : [ 'M0_OVP'   , 'rw' , 'currently set OVP' ],
      83 : [ 'M0_OCP'   , 'rw' , 'currently set OCP' ],
      84 : [ 'M1_V'     , 'rw' , 'memory M1 V' ],
      85 : [ 'M1_I'     , 'rw' , 'memory M1 I' ],
      86 : [ 'M1_OVP'   , 'rw' , 'memory M1 OVP' ],
      87 : [ 'M1_OCP'   , 'rw' , 'memory M1 OCP' ],
      88 : [ 'M2_V'     , 'rw' , 'memory M2 V' ],
      89 : [ 'M2_I'     , 'rw' , 'memory M2 I' ],
      90 : [ 'M2_OVP'   , 'rw' , 'memory M2 OVP' ],
      91 : [ 'M2_OCP'   , 'rw' , 'memory M2 OCP' ],
      92 : [ 'M3_V'     , 'rw' , 'memory M3 V' ],
      93 : [ 'M3_I'     , 'rw' , 'memory M3 I' ],
      94 : [ 'M3_OVP'   , 'rw' , 'memory M3 OVP' ],
      95 : [ 'M3_OCP'   , 'rw' , 'memory M3 OCP' ],
      96 : [ 'M4_V'     , 'rw' , 'memory M4 V' ],
      97 : [ 'M4_I'     , 'rw' , 'memory M4 I' ],
      98 : [ 'M4_OVP'   , 'rw' , 'memory M4 OVP' ],
      99 : [ 'M4_OCP'   , 'rw' , 'memory M4 OCP' ],
     100 : [ 'M5_V'     , 'rw' , 'memory M5 V' ],
     101 : [ 'M5_I'     , 'rw' , 'memory M5 I' ],
     102 : [ 'M5_OVP'   , 'rw' , 'memory M5 OVP' ],
     103 : [ 'M5_OCP'   , 'rw' , 'memory M5 OCP' ],
     104 : [ 'M6_V'     , 'rw' , 'memory M6 V' ],
     105 : [ 'M6_I'     , 'rw' , 'memory M6 I' ],
     106 : [ 'M6_OVP'   , 'rw' , 'memory M6 OVP' ],
     107 : [ 'M6_OCP'   , 'rw' , 'memory M6 OCP' ],
     108 : [ 'M7_V'     , 'rw' , 'memory M7 V' ],
     109 : [ 'M7_I'     , 'rw' , 'memory M7 I' ],
     110 : [ 'M7_OVP'   , 'rw' , 'memory M7 OVP' ],
     111 : [ 'M7_OCP'   , 'rw' , 'memory M7 OCP' ],
     112 : [ 'M8_V'     , 'rw' , 'memory M8 V' ],
     113 : [ 'M8_I'     , 'rw' , 'memory M8 I' ],
     114 : [ 'M8_OVP'   , 'rw' , 'memory M8 OVP' ],
     115 : [ 'M8_OCP'   , 'rw' , 'memory M8 OCP' ],
     116 : [ 'M9_V'     , 'rw' , 'memory M9 V' ],
     117 : [ 'M9_I'     , 'rw' , 'memory M9 I' ],
     118 : [ 'M9_OVP'   , 'rw' , 'memory M9 OVP' ],
     119 : [ 'M9_OCP'   , 'rw' , 'memory M9 OCP' ],
  # Unused/Unknown 120-255
  #   256 : [ 'SYSTEM'   , 'rw' , 'SYSTEM' ],
  # NOT REGISTERS - Magic numbers for the registers
  #  5633 : [ 'BOOTLOADER' , 'r'  , 'BOOTLOADER' ],
  }


  # from https://sigrok.org/gitweb/?p=libsigrok.git;a=blob;f=src/hardware/rdtech-dps/api.c;h=c55b589655cd579c2e40ca578082f439be6acaac;hb=HEAD
  # values borrowed from Sigrok's libsigrok rdtech-gps, https://sigrok.org/gitweb/?p=libsigrok.git;a=tree;f=src/hardware/rdtech-dps
  RD_TYPES={
    # little endian, table of registers not implemented, crash on attempt to run
    'DPS3005' : { 'Imax': 5, 'Vmax':30, 'Pmax': 160, 'Imult':1000, 'Vmult': 100 },
    'DPS5005' : { 'Imax': 5, 'Vmax':50, 'Pmax': 250, 'Imult':1000, 'Vmult': 100 },
    'DPS5015' : { 'Imax':15, 'Vmax':50, 'Pmax': 750, 'Imult': 100, 'Vmult': 100 },
    'DPS5020' : { 'Imax':20, 'Vmax':50, 'Pmax':1000, 'Imult': 100, 'Vmult': 100 },
    'DPS8005' : { 'Imax': 5, 'Vmax':80, 'Pmax': 408, 'Imult':1000, 'Vmult': 100 },
    # big endian, implemented table of registers
    'RD6006'  : { 'Imax': 6, 'Vmax':60, 'Pmax': 360, 'Imult':1000, 'Vmult': 100 },
    'RK6006'  : { 'Imax': 6, 'Vmax':60, 'Pmax': 360, 'Imult':1000, 'Vmult': 100 },
    'RD6006P' : { 'Imax': 6, 'Vmax':60, 'Pmax': 360, 'Imult':10000,'Vmult':1000 },
    'RD6012'  : { 'Imax':12, 'Vmax':60, 'Pmax': 720, 'Imult': 100, 'Vmult': 100 },
    #* RD6012P supports two current ranges with differing resolution.
    #* Up to 6A with 4 digits (when RTU reg 20 == 0), up to 12A with 3 digits (when RTU reg 20 == 1).
    'RD6012P'  : { 'Imax': 6, 'Vmax':60, 'Pmax': 360, 'Imult':10000,'Vmult':1000, 'cmt':'high-resolution, two ranges with different Imult' },
    'RD6012P:1': { 'Imax':12, 'Vmax':60, 'Pmax': 720, 'Imult':1000, 'Vmult':1000},
    'RD6018'   : { 'Imax':18, 'Vmax':60, 'Pmax':1080, 'Imult': 100, 'Vmult': 100 },
    'RD6024'   : { 'Imax':24, 'Vmax':60, 'Pmax':1440, 'Imult': 100, 'Vmult': 100 },
  }
    # * Specs for models RD60nn taken from the 2020.12.2 instruction manual,
    # * specs for RD6006P from the 2021.2.26 (english) manual,
    # * specs for RD6012P from the 2021.10.26 (english) manual,
    # * and specs for RD6024P from the 2021.1.7 (english) manual.
  RD_IDNUMS={
    3005: "DPS3005",
    5005: "DPS5005",
    5205: "DPH5005",
    5015: "DPS5015",
    5020: "DPS5020",
    8005: "DPS8005",
    60061: "RD6006",
    60062: "RD6006",
    60065: "RD6006P",
    60066: "RK6006",
    60121: "RD6012",
    60125: "RD6012P",
    60181: "RD6018",
    60241: "RD6024",
  }


  modbus=None
  connected=False
  #written=False

  robust=False # increase timeouts, retries
  verbconn=False # debug port-level transactions

  bat=False   # battery relevant data
  bat_forcemode=False

  deltatime=1 # number of seconds to add to time to account for comm delay, empiric

  regs=[]     # cached registers for reads
  oldregs=[]  # saved registers for comparison
  cachereg={} # cached registers for writes, to facilitate group write
  forceread=False # force register read in sync()

  vmult=100 # instrument-dependent, fixed for now
  imult=100 # instrument-dependent, fixed for now, 1000 for RD6006, 100 or 1000 for RD6012p, and so
  imultvar=100 # instrument-dependent, fixed for now, 1000 for RD6006, 100 or 1000 for RD6012p, and so
  pmult=100 # instrument-dependent, fixed for now
  autorange=False # rd6012p current-dependent
  typename=''

  vfmt='5.2'
  ifmt='5.2'
  pfmt='5.2'

  verbc=False    # verbose cached write processing
  verbcmd=False  # verbose commands processing
  verbauto=False # verbose autodetection

  readcachetimeout=1 # for automatic forced sync
  lastreadtime=-100  # 

  ovp=-1
  ocp=-1
  ovpreg=82
  ocpreg=83

  # initialize device when object created
  def __init__(self,comm=None):
    if comm!=None: self.setport(comm)
    self.modbus=RDModbus()

  def setport(self,comm):
    self.modbus.initport(comm)

  # connect to hardware
  def connect(self):
    self.modbus.comm.connect()
    self.connected=True

  # close connection to hardware
  def close(self):
    if self.connected:
      self.modbus.comm.close()
      self.connected=False

  # connect if not connected already
  def initconn(self):
    if not self.connected:
      if self.robust:
        self.modbus.timeout=6
        self.modbus.retries=60
      if self.verbconn:
        self.modbus.comm.verbconn=True
      self.connect()

  # store registers copy to oldregs
  def saveregs(self):
    #print(self.regs)
    self.oldregs=list(self.regs).copy()

  # if registers not read already, read them in
  def initregs(self):
    #print('reglen',len(self.regs),self.regs)
    if len(self.regs)==0:
      if self.verbc: print('INITREGS',file=stdlog)
      self.readstate()
      self.readovpocp() # only once, assume not changing
      self.saveregs()
      #print(self.regs)

  # read state block of registers
  def readstate(self):
    self.initconn()
    if len(self.regs)<4:
      self.regs=self.modbus.modbus_readregs(0,42)
      if (not self.bat_forcemode) and (self.getreg('BAT_MODE')>0): self.bat=True
    else:
      if self.bat: self.regs=self.regs[:4]+self.modbus.modbus_readregs(4,42-4) # save 8 bytes in transaction, assume type does not change
      else:        self.regs=self.regs[:4]+self.modbus.modbus_readregs(4,20-4) # save 8 bytes in transaction, skip reading battery data
    self.written=False
    self.lastreadtime=monotonic()
    self.forceread=False
    if self.typename=='': self.autoconfig() # detect type, set vmult,imult
    if self.autorange: self.setrange() # RD6006P register I_RANGE dependence


  # resolve register name or number to number
  def getregno(self,name):
    for x in self.RD_REGS:
      if self.RD_REGS[x][0]==name: return x
    return None

  # take string with name or number, return register number or except
  def name2reg(self,s):
    reg=self.normreg(s)
    if reg==None: reg=int(s)
    return reg

  # return normalized register, number or name converted to number
  def normreg(self,name):
    if isinstance(name,str):
      reg=self.getregno(name)
      if name==None: raise(BaseException('Unknown register name: '+name))
      return reg
    return name

  # print single register in formatted way
  def printreg(self,n,val,force=False):
    name='' if n not in self.RD_REGS else self.RD_REGS[n][0]
    desc='register '+str(n) if n not in self.RD_REGS else self.RD_REGS[n][2]
    mode='?' if n not in self.RD_REGS else self.RD_REGS[n][1]
    if name=='' and val==0 and not force: return
    print(f'x{n:02x} ',end='')
    print(f'{n:>3}  {val:>5} 0x{val:04x} {mode:<2}  {name:<14} {desc}')

  # print all registers in formatted way
  def printregs(self,start,arr,force=False):
    self.initregs()
    for x in range(0,len(arr)): self.printreg(start+x,arr[x],force=force)

  # compare registers with saved copy in oldregisters
  def compareregs(self):
    self.writecache()
    if self.written: self.readstate()
    blacklist=['V_IN','EXT_C','EXT_F','KEYPAD'] # these change often, do not annoy
    start=0
    print()
    print('DIFFREG:')
    for x in range(0,len(self.oldregs)):
      if x in self.RD_REGS and self.RD_REGS[x][0] in blacklist: continue
      if self.regs[x]==self.oldregs[x]: continue
      self.printreg(start+x,self.oldregs[x])
      self.printreg(start+x,self.regs[x])
    print()

  def getfmtstr(self,range,len=7):
    if range==0: return str(len)+'.0f'
    if range==10: return str(len)+'.1f'
    if range==100: return str(len)+'.2f'
    if range==1000: return str(len)+'.3f'
    if range==10000: return str(len)+'.4f'
    return str(len)+'.3f'

  def setrange(self):
    idstr=self.typename
    if self.autorange and self.getreg('I_RANGE')>0: idstr+=':1'
    if idstr not in self.RD_TYPES: print('ERRIDENT: unknown device type name:',idstr,file=stderr);return
    self.vmult=self.RD_TYPES[idstr]['Vmult']
    self.imult=self.RD_TYPES[idstr]['Imult']
    self.imultvar=self.imult
    flen=7 if self.vmult>100 or self.imult>100 else 6
    self.vfmt=self.getfmtstr(self.vmult,flen)
    self.ifmt=self.getfmtstr(self.imult,flen)
    self.pfmt=self.getfmtstr(self.imult,flen)
    if self.verbauto: print('AUTOCONFIG: Vmult =',self.vmult,', Imult =',self.imult,', autorange' if self.autorange else '',file=stdlog)

  def autoconfig(self):
    idnum=self.getreg('ID')
    if idnum in self.RD_IDNUMS:
      self.typename=self.RD_IDNUMS[idnum]
      if self.typename=='RD6012P': print('WARN: type 6012P has two different current ranges, autorange code is not tested!',file=stderr)
      if self.typename=='RD6012P': self.autorange=True
      if self.verbauto: print('AUTOCONFIG: detected',self.typename, ' autorange enabled' if self.autorange else '')
      self.setrange()
      if self.typename[:3]=='DPS': print('ERRTYPE: DPS devices unsupported, this one is RDS-only');exit(50)
    else:
      print('ERRIDENT: unknown device type ID:',idnum,file=stderr)
      if idnum<60000 or idnum>60999: exit(50) # not 60xx range of devices
      print('WARN: voltage and current values may be order of magnitude off, assuming Vmult =',self.vmult,', Imult =',self.imult,file=stderr)
      self.typename='[unknown]'


  # read and print registers
  def getprintregs(self,start=0,num=42,force=False):
    self.sync()
    regs=self.modbus.modbus_readregs(start,num)
    self.printregs(start,regs,force=force)

  # set datetime registers group to current
  def setdatetime(self):
    def getdatetime():
      from datetime import datetime,timedelta
      t=datetime.now()+timedelta(seconds=self.deltatime)
      return [t.year,t.month,t.day,t.hour,t.minute,t.second]
    self.initconn()
    self.modbus.modbus_writeregs(self.normreg('YEAR'),getdatetime())
#    self.written=True

  # write list of registers, start at name
  def writeregs(self,name,vals):
    self.initconn()
    addr=self.normreg(name)
    self.modbus.modbus_writeregs(addr,vals)
    if addr<len(self.regs):
      for t in range(0,len(vals)):
        self.regs[t+addr]=vals[t]
 #   self.written=True

  # write single register to MODBUS
  def writereg(self,name,val,cache=True):
    reg=self.normreg(name)
    if not cache:
      self.initconn()
      self.modbus.modbus_writereg(reg,val)
 #     self.written=True
      if reg<len(self.regs): self.regs[reg]=val
    else:
      if self.verbc: print('CACHE WRITE:',reg,'=',val,file=stdlog)
      if (reg > len(self.regs)) or (self.regs[reg]!=val):
        self.cachereg[reg]=int(val)
      elif self.verbc: print('skipped, value identical',file=stdlog)


  # read list of registers, start at name, count of num
  def readregs(self,name,num):
    self.initconn()
    reg=self.normreg(name)
    return self.modbus.modbus_readregs(reg,num)

  # read current OVP/OCP settings
  # BEWARE, may not work well when memories are used
  def readovpocp(self,usepreset=False,addr=0):
    if usepreset: addr=4*self.getreg('PRESET')
    else: addr=4*addr
    ovp,ocp=self.readregs(self.ovpreg+4*addr,2)
    self.ovp=ovp/self.vmult
    self.ocp=ocp/self.vmult

  # read single register, from cache if in its range and not forced, else from MODBUS
  def readreg(self,name,force=False):
    self.initconn()
    reg=self.normreg(name)
    if force or reg>=len(self.regs):
      res=self.readregs(reg,1)
      if len(res)>0: return res[0]
      print(f'ERRREG: register {name} returned no data')
      return -1
    else: return self.regs[reg]

  # debug: print write cache values
  def printcache(self):
    if self.cachereg=={}: return
    print('CACHED:')
    for x in sorted(self.cachereg.keys()): print('   ',x,'=',self.cachereg[x])

  # write single block of cached values
  def writecacheblock(self,cout):
    if self.verbc: print('cacheblock:',cout,file=stdlog)
    if len(cout)==0: return
    sleep(0.02)
    if len(cout)==1:
      key=list(cout.keys())[0]
      self.writereg(key,cout[key],cache=False)
      return
    a=sorted(cout.keys())
    kmin=a[0]
    kmax=a[len(a)-1]
    klen=kmax-kmin+1
    if self.verbc: print('minmax:',kmin,kmax,file=stdlog)
    vals=list(self.regs[kmin:kmax+1])
    if len(vals)<klen: vals=(vals+[0]*klen)[:klen]
    if self.verbc: print('vals:',vals,file=stdlog)
    for x in cout: vals[x-kmin]=cout[x]
    if self.verbc: print('vals:',vals,file=stdlog)
    self.writeregs(kmin,vals)

  # write cached values, in two blocks, for settings and ovp/ocp
  def writecache(self):
    if self.cachereg=={}: return
    # memory block
    clow={}
    chigh={}
    for x in self.cachereg:
      if x>=80: chigh[x]=self.cachereg[x]
      if x<=42: clow[x] =self.cachereg[x]
    if self.verbc: print('lowhi:',clow,chigh,file=stdlog)
    self.writecacheblock(chigh)
    self.writecacheblock(clow)
    self.cachereg={}

  # print state registrers, first 42 ones
  def printstateregs(self):
    self.sync()
    self.printregs(0,self.regs)

  # get 32-byte value from REG_H,REG_L pair
  def getreg32(self,name):
    addr=self.normreg(name)
    return (self.regs[addr]<<16) + self.regs[addr+1]

  # get single register from cache
  def getreg(self,name):
    return self.readreg(name)

  # print power supply type
  def printtype(self):
    self.initregs()
    typenum=self.getreg('ID')
    serno=self.getreg32('SN_H')
    fwno=self.getreg('FW')
    print(f'type={self.typename} typeID={typenum} serno={serno} fwno={fwno/100}',self.RD_TYPES[self.typename] if self.typename in self.RD_TYPES else '')

  # print settings and measurements
  def printstate(self,opts='',ovpocp=True,help=False):
    if help:
      print('          opts:  J=JSON, S=short (V/A only), T=show time, U=show UTC time, B=force battery');return
    json=False
    short=False
    showtime=False
    showtimeutc=False
    bat=self.bat
    opts=opts.upper()
    if 'J' in opts: json=True
    if 'S' in opts: short=True
    if 'B' in opts: bat=True
    if 'T' in opts: showtime=True
    if 'U' in opts: showtimeutc=True;showtime=True

    self.sync(force=True)
    a={}
    if showtime:
      from datetime import datetime
      if showtimeutc: a['time']=datetime.utcnow().isoformat()[:23]
      else: a['time']=datetime.now().isoformat()[:23]
    if not short:
      a['Vset']= self.getreg('V_SET')/self.vmult
      a['Iset']= self.getreg('I_SET')/self.imultvar
    if not short or not json:
      a['out'] = self.getreg('OUTPUT')
    a['Vout']= self.getreg('V_OUT')/self.vmult
    a['Iout']= self.getreg('I_OUT')/self.imultvar
    if not short:
      a['Pout']= self.getreg('P_OUT')/self.pmult
      a['Vin']=  self.getreg('V_IN')/self.vmult
    if not short or not json:
      a['cccv']= self.getreg('CV_CC')
      a['ovpocp']=self.getreg('OVP_OCP')
    if bat:
      if not short: a['isbat']=self.getreg('BAT_MODE')
      a['Vbat']= self.getreg('V_BAT')/self.vmult
      if not short:
        a['Ah']=   self.getreg32('AH_H')/1000
        a['Wh']=   self.getreg32('WH_H')/1000
    if not short:
      if ovpocp:
        a['OVP']=self.ovp
        a['OCP']=self.ocp
      a['tempint']=self.getreg('INT_C')
    istempext=False
    if self.bat and self.getreg('EXT_C_S')==0: istempext=True;a['tempext']=self.getreg('EXT_C')
    if self.autorange: a['autorange']=self.getreg('I_RANGE')
    if json:
      from json import dumps
      print(dumps(a))
    else:
      if showtimeutc: print(f'time = {a["time"]} UTC')
      elif showtime: print(f'time = {a["time"]}')
      if not short:
        print(f'Vin  = {a["Vin"]:7.2f} v',end='')
        print(f'    tempin = {a["tempint"]:>2} \'c',end='')
        if istempext: tempextstr=f'{a["tempext"]:>2} \'c'
        else: tempextstr='(disconnected)'
        print(f'    tempext = {tempextstr}' if self.bat else '')
        print(f'Vset = {a["Vset"]:{self.vfmt}} v',end='')
        print(f'    OVP = {a["OVP"]:{self.vfmt}} v' if ovpocp else '')
        print(f'Iset = {a["Iset"]:{self.ifmt}} a',end='')
        print(f'    OCP = {a["OCP"]:{self.ifmt}} a' if ovpocp else '')
      if a['out']>0: print('OUTPUT ENABLED')
      else: print('OUTPUT DISABLED')
      if a['cccv']>0: print('CURRENT MODE')
      if a['ovpocp']>0: print('OVP_OCP PROTECTION ACTIVE')
      if 'Vbat' in a: print(f'Vbat = {a["Vbat"]:{self.vfmt}} v')
      print(f'Vout = {a["Vout"]:{self.vfmt}} v')
      print(f'Iout = {a["Iout"]:{self.ifmt}} a')
      if 'Pout' in a: print(f'Pout = {a["Pout"]:{self.pfmt}} w')
      if self.bat:
        if 'Ah' in a: print(f'Ahour = {a["Ah"]} Ah')
        if 'Wh' in a: print(f'Whour = {a["Wh"]} Wh')

  # print memories M0..M9
  def printmem(self):
    self.getprintregs('M0_V',10*4,force=False)

  # synchronize states; flush write cache, do initial register read if needed, read state if needed
  def sync(self,force=False):
    self.initregs()
    self.writecache()
    #print('cache age:', round(monotonic()-self.lastreadtime,2))
    if force or self.forceread or monotonic()-self.lastreadtime>self.readcachetimeout: self.readstate()


  # read four values from memory Mx (volts, amps, ovp, ocp)
  def readmem(self,addr):
    self.initconn()
    regs=self.modbus.modbus_readregs(self.normreg('M0_V')+4*addr,4)
    #print('memory',addr,'=',regs)
    return regs

  def printmemarr(self,addr,a):
    #a=self.readmem(addr)
    print(f'M{addr}: {a[0]/self.vmult:>6} v  {a[1]/self.imult:>6} a  {a[2]/self.vmult:>6} v ovp  {a[3]/self.imult:>6} a ocp')

  def printmems(self):
    self.initconn()
    regs=self.modbus.modbus_readregs(self.normreg('M0_V'),4*10)
    print('PRESETS:')
    for t in range(0,10): self.printmemarr(t,regs[4*t:])
    print('Current:',self.getreg('PRESET'))

  def setmemraw(self,addr,arr): # expects [volt,amp,ovolt,oamp]
    self.initconn()
    self.modbus.modbus_writeregs(self.normreg('M0_V')+4*addr,arr)
    self.readmem(addr)

  # set memory to real values, convert from float to int
  def setmem(self,addr,arr): # expects [volt,amp,ovolt,oamp]
    if addr==0: self.ovp=arr[2];self.ocp=arr[3]
    self.setmemraw(addr,[int(arr[0]*self.vmult), int(arr[1]*self.imult), int(arr[2]*self.vmult), int(arr[3]*self.imult) ])

  # set output voltage
  def setvolt(self,val,rel=False):
    self.initregs()
    if rel: val=self.getreg('V_SET')/self.vmult+val
    self.writereg('V_SET',self.vmult*val)

  # set output current
  def setamp(self,val,rel=False):
    self.initregs()
    if rel: val=self.getreg('I_SET')/self.imultvar+val
    self.writereg('I_SET',self.imultvar*val)

  # set overvoltage protection
  def setovp(self,val,rel=False):
    self.initregs()
    if rel:
      val=self.readreg('M0_OVP')/self.vmult+val
    self.ovp=val
    self.writereg('M0_OVP',self.ovp*self.vmult)

  # set overcurrent protection
  def setocp(self,val,rel=False):
    self.initregs()
    if rel:
      print('XX',val,rel,self.readreg('M0_OCP'))
      val=self.readreg('M0_OCP')/self.imult+val
    self.ocp=val
    self.writereg('M0_OCP',self.ocp*self.imult)

  # enable output
  def setON(self):
    self.initregs()
    self.writereg('OUTPUT',1)
    self.sync()

  # disable output
  def setOFF(self):
    self.initregs()
    self.writereg('OUTPUT',0)
    self.sync()

  # toggle output
  def setTOGGLE(self):
    self.initregs()
    if self.getreg('OUTPUT')==1: self.setOFF()
    else: self.setON()
    self.sync()

  #def usemem(self,addr):
  #  a=self.readmem(addr)
  ##  self.ovp=a[2]/self.vmult
  ##  self.ocp=a[3]/self.imult
  #  print(addr,a,self.ovp,self.ocp)
  #  self.writereg('PRESET',addr,cache=False) # no caching, immediate write
  #  self.readovpocp(usepreset=True)
  #  print(addr,a,self.ovp,self.ocp)

# TODO: usemem() with correct ovp/ocp setting copy


###############################
##
##  HIGH LEVEL COMMAND HANDLING
##
###############################



class PowerSupply:

  verbcmd=False
  rdpsu=PSU_RD60XX()
  qend='\n'
  #qend=' '
  lastcmd=''

  # return float and if it is absolute or relative
  def floatrel(self,val):
    rel=False
    if val[0]=='+' or val[0]=='-': rel=True
    f=float(val)
    return f,rel



  # handle individual command; dryrun only checks validity, help only prints command description
  def handlecommand(self,cmdorig,dryrun,help=False,verb=verbcmd):
    if verb or self.verbcmd: print('cmd:',cmdorig,dryrun,file=stdlog)
    cmdarr=(cmdorig+':::').split(':') # enforce minimal length of parameters array
    cmd=cmdarr[0].upper()
    self.lastcmd=cmd
    if cmd=='': return True

    # help
    elif cmd in ['HELP','LIST','?','-H','--HELP']:
      if help: print(' HELP            list commands');return False
      self.helpcommands();exit(0)
    elif cmd=='CFGFILE':
      if help: print('  CFGFILE        generate config file template to stdout');return False
      self.makecfgfile()
    elif cmd=='-':
      if help or not dryrun: print();return True

    # single-word queries
    elif cmd=='QV':
      if help: print('  QV             query actual voltage');return False
      if not dryrun: self.rdpsu.sync();print(self.rdpsu.getreg('V_OUT')/self.rdpsu.vmult,end=self.qend)
    elif cmd=='QA' or cmd=='QI':
      if help: print('  QA             query actual current');return False
      if not dryrun: self.rdpsu.sync();print(self.rdpsu.getreg('I_OUT')/self.rdpsu.imultvar,end=self.qend)
    elif cmd=='QBV' or cmd=='QVB':
      if help: print('  QBV            query battery voltage');return False
      if not dryrun: self.rdpsu.sync();print(self.rdpsu.getreg('V_BAT')/self.rdpsu.vmult,end=self.qend)
    elif cmd=='QMV':
      if help: print('  QMV            query actual voltage, integer millivolts');return False
      if not dryrun: self.rdpsu.sync();print(int(1000*self.rdpsu.getreg('V_OUT')/self.rdpsu.vmult),end=self.qend)
    elif cmd=='QMA':
      if help: print('  QMA            query actual current, integer milliamps');return False
      if not dryrun: self.rdpsu.sync();print(int(1000*self.rdpsu.getreg('I_OUT')/self.rdpsu.imultvar),end=self.qend)
    elif cmd=='QBMV':
      if help: print('  QBMV           query battery voltage, integer millivolts');return False
      if not dryrun: self.rdpsu.sync();print(int(1000*self.rdpsu.getreg('V_BAT')/self.rdpsu.vmult),end=self.qend)
    elif cmd=='QTE':
      if help: print('  QTE            query external temperature');return False
      if not dryrun: self.rdpsu.sync();print(self.rdpsu.getreg('EXT_C'),end=self.qend)
    elif cmd=='QTI':
      if help: print('  QTI            query internal temperature');return False
      if not dryrun: self.rdpsu.sync();print(self.rdpsu.getreg('INT_C'),end=self.qend)

    # direct register-level access
    elif cmd[:4]=='QREG' or cmd[:1]=='Q':
      if help: print('  QREGxx or Qxx  query register xx (name or decimal)');return False
      name=cmd[4:] if cmd[:4]=='QREG' else cmd[1:]
      try: reg=self.rdpsu.name2reg(name)
      except: print('[Unknown register to query:',cmd,'seen as "'+name+'" ]',file=stderr);return False
      if not dryrun: self.rdpsu.sync();print(self.rdpsu.getreg(reg),end=self.qend)
    # individual register writes, lowlevel, CAUTION
    elif cmd[:3]=='REG' and '=' in cmd:
      if help: print('  REGxx=yy       write register xx to value yy (decimal)');return False
      a=cmd[3:].split('=')
      try: reg=self.rdpsu.name2reg(a[0]);val=int(a[1])
      except: print('[unknown register=value to set:',cmd,'seen as '+a[0]+'='+a[1]+' ]',file=stderr);return False
      if not dryrun:
        print('REGISTER WRITE:',reg,'=',val)
        self.rdpsu.writereg(reg,val,cache=False) # no caching, immediate write

    # output switch control
    elif cmd=='ON':
      if help: print('  ON             enable output');return False
      if not dryrun: self.rdpsu.setON()
    elif cmd=='OFF':
      if help: print('  OFF            disable output');return False
      if not dryrun: self.rdpsu.setOFF()
    elif cmd=='TOGGLE':
      if help: print('  TOGGLE         toggle output');return False
      if not dryrun: self.rdpsu.setTOGGLE()

    # volt/amp settings
    elif cmd[-1:]=='V':
      if help: print('  nn.nnV         set output voltage');return False
      try: val,rel=self.floatrel(cmd[:-1])
      except: print('[Unknown voltage to set:',cmd,']',file=stderr);return False
      if not dryrun: self.rdpsu.setvolt(val,rel=rel)
    elif cmd[-2:]=='MA':
      if help: print('  nn.nnMA        set output current');return False
      try: val,rel=self.floatrel(cmd[:-2])
      except: print('[Unknown current to set:',cmd,']',file=stderr);return False
      if not dryrun: self.rdpsu.setamp(val/1000,rel=rel)
    elif cmd[-1:]=='A':
      if help: print('  nn.nnA         set output current');return False
      try: val,rel=self.floatrel(cmd[:-1])
      except: print('[Unknown current to set:',cmd,']',file=stderr);return False
      if not dryrun: self.rdpsu.setamp(val,rel=rel)

    # OVP/OCP settings
    elif cmd[-2:]=='VO':
      if help: print('  nn.nnVO        set overvoltage protection');return False
      try: val,rel=self.floatrel(cmd[:-2])
      except: print('[Unknown voltage to set:',cmd,']',file=stderr);return False
      if not dryrun: self.rdpsu.setovp(val,rel=rel)
    elif cmd[-3:]=='MAO':
      if help: print('  nn.nnMAO       set overcurrent protection');return False
      try: val,rel=self.floatrel(cmd[:-3])
      except: print('[Unknown current to set:',cmd,']',file=stderr);return False
      if not dryrun: self.rdpsu.setocp(val/1000,rel=rel)
    elif cmd[-2:]=='AO':
      if help: print('  nn.nnAO        set overcurrent protection');return False
      try: val,rel=self.floatrel(cmd[:-2])
      except: print('[Unknown current to set:',cmd,']',file=stderr);return False
      if not dryrun: self.rdpsu.setocp(val,rel=rel)

    # clock setting
    elif cmd=='SETCLOCK':
      if help: print('  SETCLOCK       set clock to current datetime');return False
      if not dryrun: self.rdpsu.setdatetime()

    # output formatting
    elif cmd in ['LINE','ONELINE']:
      if help: print('  LINE           output the Q-queries as space-separated instead of newline-separated');return False
      self.qend=' '

    # mode selectors
    elif cmd in ['BAT']:
      if help: print('  BAT            enable battery charging related registers');return False
      self.rdpsu.bat_forcemode=True
      self.rdpsu.bat=True
    elif cmd in ['NOBAT']:
      if help: print('  NOBAT          disable battery charging related registers');return False
      self.rdpsu.bat_forcemode=True
      self.rdpsu.bat=False
    elif cmd in ['ROBUST']:
      if help: print('  ROBUST         increase timeouts and retries');return False
      self.rdpsu.robust=True
    # verbosity
    elif cmd in ['VERB','-V']:
      if help: print('  VERB or -V     list MODBUS transactions; affects subsequent commands');return False
      if not dryrun:
        self.rdpsu.modbus.setverbm(True)
        self.rdpsu.modbus.comm.verbconn=True
        self.rdpsu.verbauto=True
    elif cmd in ['VERBCOMM','VERBPORT']:
      if help: print('  VERBCOMM       list comm port transactions');return False
      self.rdpsu.verbconn=True

    # take commands from stdin, help-only here
    elif cmd in ['STDIN']:
      if help: print('  STDIN          read commands from stdin');return False
      #self.rdpsu.bat=True

    # force delay in processing
    elif cmd[:5]=='SLEEP':
      if help: print('  SLEEPxx        sleep for xx seconds');return False
      s=cmd[5:]+cmdarr[1]
      if s!='':
        try: secs=float(s)
        except: print('['+cmd+': Unknown delay to set:',s,']');return False
      else: secs=1
      if not dryrun: self.rdpsu.sync();sleep(secs);self.rdpsu.forceread=True#self.rdpsu.written=True

    # perform loop, help and validation only here
    elif cmd=='LOOP':
      if help: print('  LOOP:[xx]      loop for xx time or endless if not specified');return False
      no=cmdarr[1]
      if no!='':
        try: int(no)
        except: print('[Unknown loops count:',cmd,' seen as "'+no+'" ]');return False

    # show device type
    elif cmd=='TYPE':
      if help: print('  TYPE           print detected device type');return False
      if not dryrun: self.rdpsu.printtype()

    # list status registers
    elif cmd=='REGS':
      if help: print('  REGS           dump registers');return False
      if not dryrun: self.rdpsu.getprintregs()

    # list presets
    elif cmd=='MEMS':
      if help: print('  MEMS           dump memories');return False
      if not dryrun: self.rdpsu.printmems()

    elif cmd[:6]=='SETMEM' and '=' in cmd:
      if help: print('  SETMEMx=v,a,vo,ao   set memory x to values');return False
      a=cmd.split('=')[1].split(',')
      if len(a)!=4: print('[need 4 parameters:',cmd,']',file=stderr);return False
      try: addr=int(cmd[6:7]);xv=float(a[0]);xa=float(a[1]);xvo=float(a[2]);xao=float(a[3])
      except: print('[Cannot understad values:',cmd,']',file=stderr);return False
      if addr<0 or addr>9: print('[Address',addr,'must be >=0 and <=9.]');return False
      if not dryrun: self.rdpsu.setmem(addr,[xv,xa,xvo,xao])

    # select preset
    # DISABLED: dangerous use, chaos with presets of current limits
    #elif cmd[:6]=='USEMEM':
    #  if help: print('  USEMEMx        switch to memory x');return False
    #  try: val=int(cmd[6:])
    #  except: print('[Unknown memory number to set:',cmd,']',file=stderr);return False
    #  if val<0 or val>9: print('[Invalid memory:',val,']',file=stderr);return False
    #  if not dryrun:
    #    self.rdpsu.usemem(val)



    # list all registers
    elif cmd in ['REGSALL','ALLREGS']:
      if help: print('  REGSALL        dump registers including memories and calibration')
      if help: print('  REGSALL:n      dump n registers');return False
      num=120
      if cmdarr[1]=='':
        if not dryrun: self.rdpsu.getprintregs(num=num)
      else:
        try: num=int(cmdarr[1])
        except: print('[Unknown register count to read:',cmd,']',file=stderr);return False
        if not dryrun:
          start=0
          while num>0:
            if num>0: self.rdpsu.getprintregs(start=start,num=min(num,64),force=True);num-=64
            start+=64
#        self.rdpsu.getprintregs(start=1*122,num=122)
#        self.rdpsu.getprintregs(start=2*122,num=122)
#        self.rdpsu.getprintregs(start=3*122,num=122)
#        self.rdpsu.getprintregs(start=4*122,num=122)
#        self.rdpsu.getprintregs(start=5*122,num=122)
#        for t in range(0,10):
#          sleep(0.1)
#          print('X',t,t*64)
#          self.rdpsu.getprintregs(start=t*64,num=64)

    # list all registers
#    elif cmd[:8] in ['REGSALL:']:
#      if help: print('  REGSALL:n      dump n registers');return False
#      try: num=int(cmd[8:])
#      except: print('[Unknown register count to read:',cmd,']',file=stderr);return False
#      if not dryrun:
#        start=0
#        while num>0:
#          if num>0: self.rdpsu.getprintregs(start=start,num=min(num,64),force=True);num-=64
#          start+=64

    elif cmd in ['REGSDIFF','REGDIFF','DIFFREG']:
      if help: print('  REGSDIFF       show registers difference');return False
      if not dryrun: self.rdpsu.sync();self.rdpsu.compareregs()

    elif cmd in ['STATE','STAT','STATUS']:
      if help: print('  STATE[:opts]   print setting state in JSON format')
      if not dryrun: self.rdpsu.sync();self.rdpsu.printstate(opts=cmdarr[1])

    elif cmd in ['JSTATE','JSTAT','JSTATUS','STATEJ','STATJ','STATUSJ']:
      if help: print('  STATEJ[:opts]  print setting state in JSON format, like opts=J')
      if help: self.rdpsu.printstate(help=True);return False
      if not dryrun: self.rdpsu.sync();self.rdpsu.printstate(opts='J'+cmdarr[1])


    elif cmd[:4]=='TCP=':
      if help: print('  TCP=addr[:port]           set connection via TCP');return False
      if dryrun:
        a=(cmdorig[4:]+':'+str(DEFAULT_TCPPORT)).split(':')
        self.conf['host']=a[0]
        self.conf['port']=int(a[1])
        for x in ['serport','baudrate']:
          if x in self.conf: self.conf.pop(x)

    elif cmd[:5]=='PORT=':
      if help: print('  PORT=/dev/ttyport[@baud]  set connection via serial port');return False
      if dryrun:
        a=(cmdorig[5:]+'@'+str(DEFAULT_BAUDRATE)).split('@')
        self.conf['serport']=DEFAULT_SERPORT if a[0]=='' else a[0]
        self.conf['baudrate']=a[1]
        for x in ['host','port']:
          if x in self.conf: self.conf.pop(x)

    else: print('[Unknown command:',cmd,']');return False
    return True



  # list help of commands
  def helpcommands(self):
    helparr=['ON','OFF',
             '-','xV','xMA','xA','xVO','xMAO','xAO',
             '-','QV','QA','QBV','QTE','QTI','QREG',
             '-','STAT','JSTAT',
             '-','REGx=y','REGS','REGSALL','REGSDIFF',
             '-','TCP=','PORT=','ROBUST',
             '-','BAT','NOBAT','STDIN','LOOP:','SLEEP','VERB','LINE','SETMEMx=','MEMS','SETCLOCK','CFGFILE']
    print('RD60 Riden RD60xx power supply control')
    print('Usage:',argv[0],'<command> [command]...')
    print('Commands:')
    print()
    for x in helparr: self.handlecommand(x.upper(),dryrun=True,help=True)
    print()
    print('For volt and amp setting, prefixing the value with + or - marks it as relative, to be added/subtracted to the current value')
    print('Commands are executed in sequence. Writes are cached and grouped together to minimize bus transactions.')
    print('Commands are case-insensitive.')
    print('Command "-" forces a newline into output.')


  # dryrun of commands list
  def verifycommands(self,cmds):
    ok=True
    for x in cmds:
      if not self.handlecommand(x,True): ok=False
    return ok

  # dryrun of commands to check validity, then run live
  def handlecommands(self,cmds):
    #print("COMMANDS:",cmds)
#    for x in cmds: self.handlecommand(x,False)
    for t in range(0,len(cmds)):
      cmd=cmds[t].upper()
      #print('CMD:',cmd)
      counter=-1 # endless
      # recursive running of loop
      if cmd[:5]=='LOOP:':
        self.rdpsu.sync()
        if len(cmd)>5: counter=int(cmd[5:]) # error check already done in dryrun parse
        while counter!=0:
          counter-=1
          self.handlecommands(cmds[t+1:])
          #if counter==0: break
        break
      # from now, everything comes from stdin
      elif cmd=='STDIN':
        #print('STDIN')
        while True:
          self.rdpsu.sync()
          s=stdin.readline().strip()
          if s=='': break # pipe closed
          if self.handlecommand(s,True): self.handlecommand(s,False) # check validity, execute if good
      else:
        if self.qend==' ' and cmd[:5]=='SLEEP': print()
        self.handlecommand(cmd,False,verb=False)
    self.rdpsu.writecache()







  conf={}
  configfilename=None

  def getprocessbarename(self):
    cmdn=('/'+argv[0]).split('/')[-1]
    if cmdn[-3:]=='.py': cmdn=cmdn[:-3]
    return cmdn

  # generate configuration file name from running file name or from name or direct filename
  # TODO: windows compatibility
  def setconfigfilename(self,name=None,filename=None):
    if filename==None:
      if name==None: name=self.getprocessbarename()
      filename='~/.'+name+'.cfg'
    self.configfilename=filename

  # output configfile template to stdout
  def makecfgfile(self):
    if self.configfilename==None: self.setconfigfilename()
    print('# config file goes to '+self.configfilename)
    print("""
# physical serial TTY
# serport=/dev/ttyUSB0
# baudrate=115200

# plain TCP socket
# host=rd6024.local
# port=8888

# physical port takes precedence if both are defined
""")
    exit(0)

  # integer from string, with error handling
  def cfgint(self,s,msg='Configfile error:',default=-1):
    try: return int(s)
    except:
      print(msg,'Invalid value "'+s+'"',file=stderr)
      print('substituting',default,file=stderr)
      return default

  # read configfile - name=process name, filename=direct config file name
  def readconf(self,name=None,filename=None):
    from os.path import expanduser
    self.setconfigfilename(name=name,filename=filename)
    fn=expanduser(self.configfilename)
    verb='verb' in argv
    if verb: print('CONFIGFILE:filename:',fn,file=stdlog)
    try:
      with open(fn,'r') as f:
        all=f.read().split('\n')
        for s in all:
          if '#' in s: continue
          if '=' not in s: continue
          a=s.split('=')
          self.conf[a[0].strip()]=a[1].strip()
    except Exception as e:
      if verb: print('CONFIGFILE:FAIL:',e,file=stderr)
      return False
    if verb: print('CONFIGFILE:',self.conf,file=stdlog)
    return True

  # initialize port from configuration in self.conf
  def initport(self):
    #global comm,port,self.rdpsu
    if 'serport' in self.conf:
      port=self.conf['serport']
      baud=DEFAULT_BAUDRATE if 'baudrate' not in self.conf else self.cfgint(self.conf['baudrate'],default=DEFAULT_BAUDRATE)
      self.comm=LowLevelSerPort(port,baud)
    elif 'host' in self.conf:
      host=self.conf['host']
      port=DEFAULT_TCPPORT if 'port' not in self.conf else self.cfgint(self.conf['port'],default=DEFAULT_TCPPORT)
      self.comm=LowLevelTcpPort(host,port)
    else:
      print('ERROR: unknown serial port or TCP host',file=stderr)
      print('Use TCP=<host>[:port] or PORT=[/dev/tty...]',file=stderr)
      exit(1)

    self.rdpsu=PSU_RD60XX()
    self.rdpsu.modbus.initport(self.comm)

  # close port
  def close(self):
    self.rdpsu.close()



#######################################
##
##  user code, no more objects
##
#######################################


if __name__=="__main__":
  cmds=argv[1:]

  psu=PowerSupply()
  psu.readconf() # set config file

  if len(cmds)==1 and (cmds[0][:4].upper()=='TCP=' or cmds[0][:5].upper()=='PORT='): cmds.append('STATE')
  elif cmds==[]: cmds=['STATE']

  if not psu.verifycommands(cmds):
    print('Command error.',file=stderr)
    exit(1)

  # now we read the config, processed parameters by a dry run, and know the port to use
  psu.initport()
  if len(cmds)<1: cmds=['STAT']

  if False:
    psu.handlecommands(cmds); psu.close() # no exception catching, for debug
  else:
    try:
      psu.handlecommands(cmds)
    except KeyboardInterrupt: pass # clean break
    finally:
      psu.close()
      if psu.lastcmd[:5]=='SLEEP' and psu.qend==' ': print()


