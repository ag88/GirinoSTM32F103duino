### Protocol

This document the serial protocol implemented for the Girinoscope 'compatible release'  used in this implementation.

The serial protocol is a text based command / response protocol.

the first character is the command followed by a 3-4 character parameter
e.g. t123 - t is the command, 123 is the parameter

| command | parameter | description                    
| ------- |:--------- | :---------------------------------------------------
| s       |  nil      | start adc conversion 
|         |           | response: when the frame is converted 1280 bytes)  
|         |           | the data is transmitted as raw bytes 
|         |           | 
| S       |  nil      | stop conversion
|         |           |
| p       | 2,4,8,16  | set prescaler, parameter is the value
| P       | 32,64,128 | sample rates is 16 mhz / prescaler
|         |           |
| r       |  nnn      | set voltage reference
| R       |           | unused in this implementation
|         |           |
| e       |  nnn      | set trigger event type
| E       |           | 0 - toggle
|         |           | 2 - falling edge
|         |           | 3 - rising edge        
|         |           |
| w       |  nnnn     | set wait duration
| W       |           | 0 - 1280 (should be less than frame length of 1280)
|         |           |
| t       |  nnn      | set threshold (for trigger)
| T       |           | 0 - 255
|         |           |
| d       |  nnn      | dump parameters
| D       |           | display parameters 
             
