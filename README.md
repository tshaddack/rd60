Autogenerated from [https://www.improwis.com/projects/sw_rd60/](https://www.improwis.com/projects/sw_rd60/)






RD60.py, python control for RD60xx power supplies







RD60.py, python control for RD60xx power supplies
=================================================



---

[Why](#Why "#Why")  
[How](#How "How")  
      [dependencies](#dependencies "How.dependencies")  
[Usage](#Usage "Usage")  
      [hardware configuration](#hardwareconfiguration "Usage.hardware configuration")  
      [commands](#commands "Usage.commands")  
      [registers](#registers "Usage.registers")  
      [settings](#settings "Usage.settings")  
      [miniscripts](#miniscripts "Usage.miniscripts")  
            [loops](#loops "Usage.miniscripts.loops")  
            [stdin](#stdin "Usage.miniscripts.stdin")  
            [data on one line](#dataononeline "Usage.miniscripts.data on one line")  
            [connection persistence](#connectionpersistence "Usage.miniscripts.connection persistence")  
      [verbosity](#verbosity "Usage.verbosity")  
      [temperatures](#temperatures "Usage.temperatures")  
      [battery mode](#batterymode "Usage.battery mode")  
      [autoconfiguration](#autoconfiguration "Usage.autoconfiguration")  
[Files](#Files "Files")  
[TODO](#TODO "TODO")  


---

Why
---



It was necessary to control a Riden power supply for some lab automation purposes.
The stock software was useless for the purpose.




Fortunately, the protocol is documented enough, and based on MODBUS.




All available software is in this or that form dependent on some MODBUS library, which
relies on the assumption of a fully functional local or at least virtual serial port of /dev/ttySx
or /dev/ttyUSBx or such nature.




The wired ports require... umm... wires. USB to a nearby computer. Which is not always feasible.
The virtual COM ports require OS-specific (and often capricious) port emulating software on the
computer side. All when all that's necessary is a pipe for bytes in, bytes out.




An expedient plain [serial-over-TCP](https://www.improwis.com/projects/hw_SerialOverTCP "local project") ("TasmoCOM") solution was chosen, leveraging ESP8266
and [Tasmota](https://en.wikipedia.org/wiki/Tasmota "Wikipedia link: Tasmota"), a proven cheap and opensource approach.




For the hardware, see [hw\_RD6024powersupply](https://www.improwis.com/projects/hw_RD6024powersupply "local project").




For the control code for an artificial load, see [sw\_dl24](https://www.improwis.com/projects/sw_dl24 "local project").




Tested with:
* RD6024, deviceID 60241
* RK6006, deviceID 60066





---

How
---



rd60.py, a python-based (for portability) script, was written. The software allows both
using a tty-style port and a raw TCP socket, with no fancy RFC2217 support. If the latter is needed, URI-style
pyserial syntax is available with the port.




The software defines a hierarchy of classes:
* class LowLevelSerPort - for wired /dev/ttyX ports or full virtual ports
* class LowLevelTcpPort - for raw TCP sockets, TasmoCOM style
* class RDModbus - [MODBUS](https://en.wikipedia.org/wiki/MODBUS "Wikipedia link: MODBUS") protocol, implementing the Riden function subset (3, 6, 16)
* class PSU\_RD60XX - functions specific for the RD60xx power supplies, register lists
* class PowerSupply - command interpreter, configfile reader



### dependencies



The software tries to minimize dependencies.




The mandatory ones, and mostly standard ones, are:
* socket (for TCP communication)
* time (for sleep)
* struct (pack/unpack, for conversion of packets to/from byte stream)

The nonmandatory, imported only as needed (so the process would run when a missing dependency is not required), are:
* serial (pyserial, for serial ports)
* datetime (for date/time settings)
* json (for JSON format output)





---

Usage
-----



```
RD60 Riden RD60xx power supply control
Usage: ./rd60.py <command> [command]...
Commands:

  ON             enable output
  OFF            disable output

  nn.nnV         set output voltage
  nn.nnMA        set output current
  nn.nnA         set output current
  nn.nnVO        set overvoltage protection
  nn.nnMAO       set overcurrent protection
  nn.nnAO        set overcurrent protection

  QV             query actual voltage
  QA             query actual current
  QBV            query battery voltage
  QTE            query external temperature
  QTI            query internal temperature
  QREGxx or Qxx  query register xx (name or decimal)

  STATE[:opts]   print setting state in JSON format
  STATEJ[:opts]  print setting state in JSON format, like opts=J
          opts:  J=JSON, S=short (V/A only), T=show time, U=show UTC time, B=force battery

  REGxx=yy       write register xx to value yy (decimal)
  REGS           dump registers
  REGSALL        dump registers including memories and calibration
  REGSALL:n      dump n registers
  REGSDIFF       show registers difference

  TCP=addr[:port]           set connection via TCP
  PORT=/dev/ttyport[@baud]  set connection via serial port
  ROBUST         increase timeouts and retries

  BAT            enable battery charging related registers
  NOBAT          disable battery charging related registers
  STDIN          read commands from stdin
  LOOP:[xx]      loop for xx time or endless if not specified
  SLEEPxx        sleep for xx seconds
  VERB or -V     list MODBUS transactions; affects subsequent commands
  LINE           output the Q-queries as space-separated instead of newline-separated
  SETMEMx=v,a,vo,ao   set memory x to values
  MEMS           dump memories
  SETCLOCK       set clock to current datetime
  CFGFILE        generate config file template to stdout

For volt and amp setting, prefixing the value with + or - marks it as relative, to be added/subtracted to the current value
Commands are executed in sequence. Writes are cached and grouped together to minimize bus transactions.
Commands are case-insensitive.
Command "-" forces a newline into output.

```
### hardware configuration



The host:port or serport:baudrate are saved in ~/.rd60.cfg (or other name, where filename is derived from
the command by stripping the .py suffix and prefixing a home directory and a dot). This variability allows
to use several symlinks for different power supplies simultaneously used, eg. as rd60a, rd60b,...




The configfile template can be generated on demand by command CFGFILE.




Directly, the devices may be specified as TCP=<host>[:port]  or PORT=/dev/ttyUSBx@baudrate,
eg. TCP=10.0.1.15:8888 or PORT=/dev/ttyUSB1@115200 or PORT=/dev/ttyUSB1 (default speed is 115200).




The PORT directive, both in command and in config, also supports the [URL form](https://pyserial.readthedocs.io/en/latest/url_handlers.html "remote link: https://pyserial.readthedocs.io/en/latest/url_handlers.html").



### commands



The script takes a sequence of commands from commandline, separated by spaces. Each command is a single token,
optionally containing separator characters.




The commands can be a fixed string (STATE, REGS, QV, ...) or a prefix with value, or value with suffix (12.5V,
SLEEP1.5, REG12=8123, QVIN...)




Register names can be sent as decimal address or as a name. Q3 and QFW are the same.




Some commands look like register commands but refer directly to values:
* QV, QA - for querying output voltage/amperage
* QMV, QMA - same, but integer value in millivolts/milliamps instead of float, for bash comparisons



### registers



The REGS and REGSALL commands read and dump the power supply's MODBS registers. REGS only first 42, REGSALL take in 122 (which
includes calibration registers (do not touch unless you MUST) and the memories (M0 is for the currently set overvoltage and overcurrent
protections (OVP and OCP), M1..M9 are for presets).




The registers dump shows register address in hex and decimal, value in decimal and hex, r/rw for readonly and readwrite, short name, and description.
Registers with zero value and no known name are assumed unused and are skipped.



./rd60.py regs
```
x00   0  60241 0xeb51 r   ID             type ID
x01   1      0 0x0000 r   SN_H           serial number H
x02   2  10542 0xXXXX r   SN_L           serial number L
x03   3    138 0x008a r   FW             firmware version *100
x04   4      0 0x0000 r   INT_C_S        INT_C_S
x05   5     44 0x002c r   INT_C          internal temperature C
x06   6      0 0x0000 r   INT_F_S        INT_F_S
x07   7    111 0x006f r   INT_F          internal temperature F
x08   8   1000 0x03e8 rw  V_SET          set voltage
x09   9    210 0x00d2 rw  I_SET          set current
x0a  10    998 0x03e6 r   V_OUT          actual voltage
x0b  11      0 0x0000 r   I_OUT          actual current
x0c  12      0 0x0000 r   AH             AH
x0d  13      0 0x0000 r   P_OUT          actual power
x0e  14   6789 0x1a85 r   V_IN           input voltage
x0f  15      1 0x0001 r   KEYPAD         keyboard locked
x10  16      0 0x0000 r   OVP_OCP        OVP-OCP active
x11  17      0 0x0000 r   CV_CC          output at constant current
x12  18      1 0x0001 rw  OUTPUT         output enabled
x13  19      0 0x0000 rw  PRESET         preset number
x14  20      0 0x0000 r   I_RANGE        I_RANGE
x20  32      0 0x0000 r   BAT_MODE       battery mode enabled
x21  33      0 0x0000 r   V_BAT          battery voltage
x22  34      1 0x0001 r   EXT_C_S        EXT_C_S
x23  35     89 0x0059 r   EXT_C          battery temperature C
x24  36      1 0x0001 r   EXT_F_S        EXT_F_S
x25  37    129 0x0081 r   EXT_F          battery temperature F
x26  38      0 0x0000 r   AH_H           amphour H
x27  39      0 0x0000 r   AH_L           amphour L
x28  40      0 0x0000 r   WH_H           watthour H
x29  41      0 0x0000 r   WH_L           watthour L

```

or, for REGSALL, also

```
x30  48   2023 0x07e7 rw  YEAR           clock year
x31  49     12 0x000c rw  MONTH          clock month
x32  50     16 0x0010 rw  DAY            clock day
x33  51      0 0x0000 rw  HOUR           clock hour
x34  52     20 0x0014 rw  MINUTE         clock minute
x35  53     44 0x002c rw  SECOND         clock second
x37  55     24 0xXXXX rw  V_OUT_ZERO     calibration V_OUT_ZERO
x38  56  24859 0xXXXX rw  V_OUT_SCALE    calibration V_OUT_SCALE
x39  57     33 0xXXXX rw  V_BACK_ZERO    calibration V_BACK_ZERO
x3a  58  24543 0xXXXX rw  V_BACK_SCALE   calibration V_BACK_SCALE
x3b  59    334 0xXXXX rw  I_OUT_ZERO     calibration I_OUT_ZERO
x3c  60   4327 0xXXXX rw  I_OUT_SCALE    calibration I_OUT_SCALE
x3d  61     59 0xXXXX rw  I_BACK_ZERO    calibration I_BACK_ZERO
x3e  62   4663 0xXXXX rw  I_BACK_SCALE   calibration I_BACK_SCALE
x42  66      1 0x0001 rw  OPT_TAKE_OK    OPT_TAKE_OK
x43  67      0 0x0000 rw  OPT_TAKE_OUT   output enabled on memory select
x44  68      0 0x0000 rw  OPT_BOOT_POW   output enabled on boot
x45  69      1 0x0001 rw  OPT_BUZZ       buzzer enabled
x46  70      1 0x0001 rw  OPT_LOGO       logo enabled on boot
x47  71      0 0x0000 rw  OPT_LANG       interface language
x48  72      3 0x0003 rw  OPT_LIGHT      display backlight level
x50  80    300 0x012c rw  M0_V           currently set V
x51  81    200 0x00c8 rw  M0_I           currently set I
x52  82   2000 0x07d0 rw  M0_OVP         currently set OVP
x53  83    220 0x00dc rw  M0_OCP         currently set OCP
x54  84    600 0x0258 rw  M1_V           memory M1 V
x55  85    200 0x00c8 rw  M1_I           memory M1 I
x56  86    700 0x02bc rw  M1_OVP         memory M1 OVP
x57  87    500 0x01f4 rw  M1_OCP         memory M1 OCP
x58  88    500 0x01f4 rw  M2_V           memory M2 V
x59  89   2410 0x096a rw  M2_I           memory M2 I
x5a  90   6200 0x1838 rw  M2_OVP         memory M2 OVP
x5b  91   2420 0x0974 rw  M2_OCP         memory M2 OCP
x5c  92    500 0x01f4 rw  M3_V           memory M3 V
x5d  93   2410 0x096a rw  M3_I           memory M3 I
x5e  94   6200 0x1838 rw  M3_OVP         memory M3 OVP
x5f  95   2420 0x0974 rw  M3_OCP         memory M3 OCP
x60  96    500 0x01f4 rw  M4_V           memory M4 V
x61  97   2410 0x096a rw  M4_I           memory M4 I
x62  98   6200 0x1838 rw  M4_OVP         memory M4 OVP
x63  99   2420 0x0974 rw  M4_OCP         memory M4 OCP
x64 100    500 0x01f4 rw  M5_V           memory M5 V
x65 101   2410 0x096a rw  M5_I           memory M5 I
x66 102   6200 0x1838 rw  M5_OVP         memory M5 OVP
x67 103   2420 0x0974 rw  M5_OCP         memory M5 OCP
x68 104    500 0x01f4 rw  M6_V           memory M6 V
x69 105   2410 0x096a rw  M6_I           memory M6 I
x6a 106   6200 0x1838 rw  M6_OVP         memory M6 OVP
x6b 107   2420 0x0974 rw  M6_OCP         memory M6 OCP
x6c 108    500 0x01f4 rw  M7_V           memory M7 V
x6d 109   2410 0x096a rw  M7_I           memory M7 I
x6e 110   6200 0x1838 rw  M7_OVP         memory M7 OVP
x6f 111   2420 0x0974 rw  M7_OCP         memory M7 OCP
x70 112    500 0x01f4 rw  M8_V           memory M8 V
x71 113   2410 0x096a rw  M8_I           memory M8 I
x72 114   6200 0x1838 rw  M8_OVP         memory M8 OVP
x73 115   2420 0x0974 rw  M8_OCP         memory M8 OCP
x74 116    500 0x01f4 rw  M9_V           memory M9 V
x75 117   2410 0x096a rw  M9_I           memory M9 I
x76 118   6200 0x1838 rw  M9_OVP         memory M9 OVP
x77 119   2420 0x0974 rw  M9_OCP         memory M9 OCP

```



### settings



The voltage, current, and overvoltage/overcurrent protection can be set with suffix-based commands.
For the value of 1.23, the commands are
* 1.23V - set output voltage
* 1.23A - set output current
* 1230MA - set output current in milliamps
* 1.23VO - set overvoltage protection (OVP)
* 1.23AO - set overcurrent protection (OCP)
* 1230MAO - set overcurrent protection (OCP) in milliamps
* +1.23V - increase output voltage
* -1.23V - decrease output voltage
* +1.23A - increase output current
* +1230MA - increase output current in milliamps




The battery-related registers are not refreshed during usual operations. If the power supply is to be used in battery charging duty,
enable related reporting with command BAT. 



### miniscripts



The commands are executed in order. To minimize bus transactions, the ones from the low memory area (first 42 registers) are grouped into a single write.



* set voltage and current, enable output
+ rd60.py 12v 550ma on

* toggle output, wait half second, show state
+ rd60.py toggle sleep0.5 state

* set output to 5 volts and enable, wait a second, increase by volt, wait a second, increase by another two volts, show state and power off
+ rd60.py on 5v sleep1 +1v sleep1 +2v sleep0.5 state off

* show output voltage and power, one per line (QP\_OUT queries register by name)
+ rd60.py qv qP\_OUT

* show output voltage in millivolts and current in milliamps, space-separated
+ rd60.py qmv qma | tr '\n' ' '

* disable output, set output voltage and current, overvoltage and overcurrent, enable output
+ rd60.py off 4.9v 1250ma 5.5vo 2.1ao on

#### loops



The LOOP: statement can be used for repeating of commands. The subsequent command set is repeated forever, or for specified number of times.



* show status in JSON format, forever
+ rd60.py loop: jstate

* ramp voltage by 20mV over time, from 5 to 15V, watch status:
+ rd60.py 5v on loop:500 +0.02v sleep0.5 jstat

#### stdin



The commands can be sent from another script, via stdin. The STDIN statement has to be the last on the command line, everything after it is ignored.
* enable output, take file with voltages, send in one per second, then disable output
+ cat file.txt | while read x; do echo $x; sleep 1; done | ./rd60.py on stdin; ./rd60.py off



#### data on one line



The LINE command sets the separator character between Q-values from default newline to a space. Groups of values then can be sent as single lines.
* check every 5 seconds, send millivolts/battery millivolts/milliamps/internal temperature/constant-current register state, use battery mode:
+ rd60.py bat line loop: qmv qbmv qma qti qcv\_cc sleep5



#### connection persistence



The connection to the port is opened when first needed, then kept open until the process closes.




In some cases this may be detrimental to reliability (connection fail crashes the process). Running it anew each time may be beneficial then.



### verbosity



To see the port/socket opening/closing, and the bus transactions dumped in hex, use VERB as the first command.



./rd60.py verb state
```
CONFIGFILE:filename: /home/user/.rd60.cfg
CONFIGFILE: {'host': 'rd60', 'port': '8888'}
SOCK:connecting to rd60 : 8888
SOCK:connected
MODBUS:CMND:READREGS 0 42
MODBUS:SEND 01:03:00:00:00:2a:c4:15 (correct)
MODBUS:RECV 01:03:54:eb:51:00:00:XX:XX:00:8a:00:00:00:2c:00:00:00:70:03:e8:00:d2:03:e6:00:00:00:00:00:00:1a:84:00:00:00:00:00:00:00:01:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:01:00:59:00:01:00:81:00:00:00:00:00:00:00:00:XX:cd (correct)
MODBUS:CMND:READREGS 82 2
MODBUS:SEND 01:03:00:52:00:02:65:da (correct)
MODBUS:RECV 01:03:04:07:d0:00:dc:fb:27 (correct)
Vin =67.88 v    tempin =44 'c    tempext=89 'c
Vset=10.00 v    OVP =20.00 v
Iset= 2.10 a    OCP = 2.20 a
OUTPUT ENABLED
Vout= 9.98 v
Iout= 0.00 a
SOCK:closed

```
### temperatures



The INT\_C and EXT\_C registers contain the power supply board temperature and the external NTC probe temperature in degrees C. (The corresponding \_F registers
use the Fahrenheit abomination.)




Caution, if the external probe is not attached the register value contains swinging nonsense.



### battery mode



The power supply has a range of registers for specific use with battery charging (battery voltage measurements, watthours, amphours).
To conserve socket/port transfers volume, this registers block is read only if the power supply is in battery mode.




The battery mode is autodetected when the program starts; for prolonged loop-based operation, this will miss the battery detection event and the mode won't be changed,
as the register is in the area that's read only during first start (unless in battery mode).




The battery mode can be also manually forced by BAT or disabled by NOBAT.



### autoconfiguration



The software borrows a trick (and table of devices) from [Sigrok](https://en.wikipedia.org/wiki/Sigrok "Wikipedia link: Sigrok")'s libsigrok, the [rdtech-dps](https://sigrok.org/gitweb/?p=libsigrok.git;a=tree;f=src/hardware/rdtech-dps "remote link: https://sigrok.org/gitweb/?p=libsigrok.git;a=tree;f=src/hardware/rdtech-dps")
component. From the table of devices, based on the device ID (register 0), the type is identified and the voltage and amperage multipliers are loaded.
(The registers provide data as 16bit integers, which have to be converted to the floats. With multiplier of 100, a 1.28 amp is set or read as 128.
The multiplier has to be known, and is type-specific, otherwise the voltages/currents can be off by a magnitude or two. 100 and 100 is default when the
type is unknown.





---

Files
-----


* **[rd60.py](rd60.py "local file")** - code itself



---

TODO
----


* better windows compatibility
* hardware mod for attaching an adc to the wifi unit, for streaming of measurements independent on the MODBUS
* [SCPI](https://en.wikipedia.org/wiki/Standard_Commands_for_Programmable_Instruments "Wikipedia link: Standard Commands for Programmable Instruments") emulation/gateway






