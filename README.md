<img align="right" src="Documentation/Logo/MarlinKimbra%20Logo%20GitHub.png"/>
# MK4due 3D Printer Firmware for Arduino due
## Version 4.2.81 dev

### Special thanks
* Wurstnase
* all Marlin8bit-developers.

### New features are:
* Stepping-algorithm optmized now for DRV8825 and A4988 (no need for double or quadstepping; no delays)
* High speed stepping of approx. 295.000 steps/s, if needed (maybe more with less DOUBLE_STEP_FREQUENCY?)

---
# MK4due 3D Printer Firmware
  * [Configuration & Compilation](/Documentation/Compilation.md)
  * Supported
    * [Features](/Documentation/Features.md)
    * [Hardware](/Documentation/Hardware.md)
    * [GCodes](/Documentation/GCodes.md)
  * Notes
    * [Auto Bed Leveling](/Documentation/BedLeveling.md)
    * [Filament Sensor](/Documentation/FilamentSensor.md)
    * [Ramps Servo Power](/Documentation/RampsServoPower.md)
    * [LCD Language - Font - System](Documentation/LCDLanguageFont.md)
  * Version
    * [Change Log](/Documentation/changelog.md)


## Configurator Tool Online

http://marlinkimbra.it


## Quick Information

This version of Marlin was made to accommodate some requests made by the community RepRap Italy http://forums.reprap.org/index.php?349
The new features are:
A single Firmware for all types of printers; Cartesian, Delta, SCARA, CoreXY & CoreXZ.
The possibility of having only one hotend independently from the extruders that you have.
The addition of the 6th extruder.
Management Color Mixing Extruder
System Management MKr4 for 6 extruders with just two drivers or only driver.
Management Multyextruder NPr2, 4/6 extruders with only two engines.
Management Dual Extruder DONDOLO.
Adding commands to facilitate purging of hotend. 
Step per unit varied for each extruder as well as the feedrate and the acceleration.
Adding Debug Dryrun used by repetier.
Added total Power on time writed in SD CARD.
Added total Power consumption writed in SD CARD.
Added total filament printed writed in SD CARD.
Added anti extruder idle oozing system.
Added Hysteresis and Z-Wobble correction (only cartesian printers).

## Credits

The current MarlinKimbra dev team consists of:
 - MagoKimbra - Alberto Cotronei (https://github.com/MagoKimbra)
 - simonepri - Simone Primarosa (https://github.com/simonepri)

More features have been added by:
  - Nico [@wurstnase]
  - Giutrec
  - Drakelive
  - and others...

## License

Marlin is published under the [GPL license](/Documentation/COPYING.md) because I believe in open development.
Do not use this code in products (3D printers, CNC etc) that are closed source or are crippled by a patent.
