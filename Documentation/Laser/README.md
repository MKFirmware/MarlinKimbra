K40 laser rasterized images sends fast and large size commands over serial,
so, the common 64 byte buffer in arduino isn't enough.

You can edit "board.txt" in arduino ide, copy the "mega 2560" board declaration, call it 
"mega 2560 serial buffer 256" and then add a line like:

```
mega256.build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=256 -DSERIAL_TX_BUFFER_SIZE=256
```

and then in the arduino ide select "mega 2560 serial buffer 256" board.

