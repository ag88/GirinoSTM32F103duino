### prerequisite 

- [stm32duino bootloader](https://github.com/rogerclarkmelbourne/STM32duino-bootloader) should be the bootloader installed on the board
- this binary is for maple mini (works for blue pill as well)
- [dfu-util](http://dfu-util.sourceforge.net/)


### install

```
dfu-util -a 2 -RD STM32F1duino-girino.bin
```