# rays-indifocuser
A relatively simple diy telescope focuser driver with a simple backlash adjustment with serial controls.

This focuser was originally based on an older version of Robert Brown's focuser-pro and has been modified to include a single backlash setting and serial commands to control it(not tested with indi so far). I use a drv8825 driver board mounted directly to an arduino nano with a pin header. The motor output is powered by the 12v battery and a single 4 wire cable plugs into stepper motors on several different telescope OTA's.

New: A new version is in the works using a uln2003 transistor driver that ~~will~~(may) run off a powered usb hub in order to reduce cable drag on the mount. If it works the redesign will only require a single usb line to the telescope body.

todo: Get indi/ekos to accept backlash from moonlite compatable controllers, or convert the driver to a standalone one.
