# Future Fixes and Additions for the Manipulator Robot

## Hardware Fixes
### USB
- Remove the USB ID pin connection as it's unneccessary.
- Make the USB D+ and D- lines a differential pair.
- Add a 15K pull-up(?) resistor to the USB D+(?) line to have it denote itself as a full-speed device.

### Motor Control
- Pick motors with a wider range of motion - the MG996Rs only have 120 degrees of mobility
- Pick motors with a better control method - the MG996Rs have their PWM control limited to 5% of the duty cycle of a 50Hz wave. 
This turns into a little over 3000 points of precision on this microcontroller, but it could be better.

### Pins
- Make the pin size 2.54mm, not 2.00mm
- Change the order of the SWD pins such that they are: Data, Ground, Clock, 3.3V

### Mounting holes
- Make 2-4 mounting holes, make them grounded
- Reduce the size of the mounting holes

### Button
- Use a larger button for resets

## Additions for Improvement
- Add an on-board flash or eeprom chip so positioning data can be stored local to the microcontroller (in a safe/efficient manner)
- Add a large e-stop button