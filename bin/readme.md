### prerequisite 

- [stm32duino bootloader](https://github.com/rogerclarkmelbourne/STM32duino-bootloader) should be the bootloader installed on the board
- this binary is for maple mini (works for blue pill as well)
- [dfu-util](http://dfu-util.sourceforge.net/)


### install

```
dfu-util -a 2 -RD STM32F1duino-girino.bin
```

#### F401 install

For the F401 binary, it is installed directly to the device directly e.g. st-link, uart, or native DFU.
No custom bootloader for that. The official tool is [stm32cubeprogrammer](https://www.st.com/en/development-tools/stm32cubeprog.html).
It is build against the older version of the WeAct F401 board with a 25 Mhz crystal:
https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1

If one insist on DFU and using dfu-util, there is a 'button dance'
- connect usb (phone) cable 
- press both boot0 and reset
- hold boot0, release reset 
- release boot0 2 sec later

That should put the board in DFU mode, the firmware can be installed over the DFU cable
and the command is like
```
dfu-util -a 0 -s 0x8000000 -RD blackpill_f401.bin
```
