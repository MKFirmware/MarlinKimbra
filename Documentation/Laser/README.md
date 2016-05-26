K40 laser rasterized images sends fast and large size commands over serial,
so, the common 64 byte buffer in arduino isn't enough.

You can edit "board.txt" in arduino ide, copy the "mega 2560" board declaration, call it 
"mega 2560 serial buffer 256" and then add a line like:

```
mega256.build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=256 -DSERIAL_TX_BUFFER_SIZE=256
```

and then in the arduino ide select "mega 2560 serial buffer 256" board.


In my boards.txt mega256 appears as this:

```
##############################################################
mega256.name=Arduino/Genuino Mega or Mega 2560 256 serialbuf

mega256.vid.0=0x2341
mega256.pid.0=0x0010
mega256.vid.1=0x2341
mega256.pid.1=0x0042
mega256.vid.2=0x2A03
mega256.pid.2=0x0010
mega256.vid.3=0x2A03
mega256.pid.3=0x0042
mega256.vid.4=0x2341
mega256.pid.4=0x0210
mega256.vid.5=0x2341
mega256.pid.5=0x0242

mega256.upload.tool=avrdude
mega256.upload.maximum_data_size=8192

mega256.bootloader.tool=avrdude
mega256.bootloader.low_fuses=0xFF
mega256.bootloader.unlock_bits=0x3F
mega256.bootloader.lock_bits=0x0F

mega256.build.f_cpu=16000000L
mega256.build.core=arduino
mega256.build.variant=mega
mega256.build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=256 -DSERIAL_TX_BUFFER_SIZE=256

# default board may be overridden by the cpu menu
mega256.build.board=AVR_MEGA2560

## Arduino/Genuino Mega w/ ATmega2560
## -------------------------
mega256.menu.cpu.atmega2560=ATmega2560 (Mega 2560)

mega256.menu.cpu.atmega2560.upload.protocol=wiring
mega256.menu.cpu.atmega2560.upload.maximum_size=253952
mega256.menu.cpu.atmega2560.upload.speed=115200

mega256.menu.cpu.atmega2560.bootloader.high_fuses=0xD8
mega256.menu.cpu.atmega2560.bootloader.extended_fuses=0xFD
mega256.menu.cpu.atmega2560.bootloader.file=stk500v2/stk500boot_v2_mega2560.hex

mega256.menu.cpu.atmega2560.build.mcu=atmega2560
mega256.menu.cpu.atmega2560.build.board=AVR_MEGA2560
```
