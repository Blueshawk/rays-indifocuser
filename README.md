# rays-indifocuser
A relatively simple diy telescope focuser driver with a simple backlash adjustment with serial controls.

This focuser was originally based on an older version of Robert Brown's focuser-pro and has been modified to include code from mltple sources with a single backlash setting and serial commands to control it. 

Hardware: 
  I used a drv8825 driver board mounted directly to an arduino nano with a pin header. The motor output is powered by the 12v   battery and a single 4 wire cable plugs into stepper motors on several different telescope OTA's.

News: Switching to a TMC2208 looks to allow single powered operation using a powered usb hub - but apparently not with my old floppy drive motors which require higher voltage to achieve lock. 

todo: Get indi/ekos to accept backlash from moonlite compatable controllers, or convert the driver to a standalone one.
