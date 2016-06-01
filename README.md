<img align="right" src="Documentation/Logo/MarlinKimbra%20Logo%20GitHub.png" />
# MarlinKimbra 3D Printer Firmware for Arduino
## Version 4.2.82 dev

For K40 laser cutter there are branches with the default config files ready to use:

 * [with cooler and flow sensor](https://git.nexlab.net/machinery/MarlinKimbra/tree/k40_flow_cooler)
 * [without cooler and flow sensor](https://git.nexlab.net/machinery/MarlinKimbra/tree/k40_noflow_nocooler)

### Special thanks
* all Marlin8bit-developers.

---
# MarlinKimbra 3D Printer Firmware
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
Added support reader TAG width MFRC522
Added Cooler and Hot Chamber
Added Laser beam and raster base64

## Credits

The current MarlinKimbra dev team consists of:
 - MagoKimbra - Alberto Cotronei (https://github.com/MagoKimbra)
 - simonepri - Simone Primarosa (https://github.com/simonepri)

More features have been added by:
 - Franco (nextime) Lanza

## License

Marlin is published under the [GPL license](/Documentation/COPYING.md) because I believe in open development.
Please do not use this code in products (3D printers, CNC etc) that are closed source or are crippled by a patent.
