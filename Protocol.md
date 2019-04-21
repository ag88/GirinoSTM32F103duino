### Protocol

This document the serial protocol implemented for the Girinoscope 'compatible release'  used in this implementation.

The serial protocol is a text based command / response protocol.

the first character is the command followed by a 3-4 character parameter
e.g. t123 - t is the command, 123 is the parameter

| command | parameter | description                    
| ------- | :-------- | :---------------------------------------------------
| s       |  nil      | start adc conversion, response: when the frame is converted 1280 bytes), the data is transmitted as raw bytes 
| S       |  nil      | stop conversion
| p       | 2,4,8,16  | set prescaler, parameter is the value, sample rates is 16 mhz / prescaler
| P       | nnnn      | set sample rates in k samples per sec
| r, R    |  nnn      | set voltage reference, unused in this implementation
| e, E    |  nnn      | set trigger event type, 0 - toggle, 2 - falling edge, 3 - rising edge        
| w, W    | nnnn      | set wait duration, 0 - 1280 (should be less than frame length of 1280)
| t, T    |  nnn      | set threshold (for trigger), 0 - 255
| d, D    |  nnn      | dump parameters, display parameters 

note that commands are single characters, parameter has varying lengths
