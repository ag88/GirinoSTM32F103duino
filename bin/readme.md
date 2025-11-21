### prerequisite 

- [stm32duino bootloader](https://github.com/rogerclarkmelbourne/STM32duino-bootloader) should be the bootloader installed on the board
- this binary is for maple mini (works for blue pill as well)
- [dfu-util](http://dfu-util.sourceforge.net/)


### install

```
dfu-util -a 2 -RD STM32F1duino-girino.bin
```

#### F401 install

for the F401 binary, it is installed directly to the device directly e.g. st-link, uart, or native DFU.
no custom bootloader for that. 
It is build against the older version of the WeAct F401 board with a 25 Mhz crystal:
https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1
