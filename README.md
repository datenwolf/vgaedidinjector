# VGA EDID Injector

Captures EDID information from a display and presents it to a computer. Useful in situations where the DDC lines are not connected (like the CCH).
* sources	https://github.com/datenwolf/vgaedidinjector
* project at C3VOC wiki	https://c3voc.de/wiki/hardware:vga_edid_injector



The VGA EDID Injector is a tool that aims to make the use of presentation infrastructure less painful. In many conference venues the presentation displays (video projectors aka "beamers") are connected using old fashioned analogue connections using high quality coax cable. While this method offers a very good image quality it suffers from the lack of a DDC channel for transmission of EDID data.

Modern operating systems however depend on the EDID information for mode setting. Even worse on some operating systems it is very hard or even impossible to override the settings obtained from the EDID data. Those operating systems also usually fall back to a set of safe display resolutions and timings if no EDID data could be retrieved via DDC. Unfortunately those safe timings usually don't match the physical resolution of the display device, resulting in bad looking scaling and/or worse distortion of the aspect ratio (e.g. fall back resolution 1024×768 (4:3 aspect), display device physical resolution 1280×720 (16:9) resulting in squashed image, i.e. pancake heads, and so on…).

The VGA EDID Injector provides relief for this kind of situation: It allows to take a snapshot of a display's EDID information and "carry" it to the computer in need.

## Hardware
The VGA EDID Injector has 3 connectors (see image):

* D-Sub 15 H (VGA) female, left side of the board – connect to display device
* D-Sub 15 H (VGA) male, right side of the board – connect to computer
* Micro-USB, lower side of the board – for providing auxiliary power (+5V); upcoming firmware revisions will provide a command line interface



## Usage
### Snapshoting Display EDID information

To snapshot the EDID connection you have to insert the injector in reverse direction. This typically requires you to use the VGA gender changer and the VGA cable.

* Connect the display to the left side, female VGA connector. 
* Plug the right side, male VGA connector into a computer


Using the display control tools refresh/identify connected displays. Repeat this until the attached display is recognized.
The VGA EDID Injector can now be spliced into the computer side of a video signal only (i.e. no DDC signals) connection.


### Replaying snapshoted EDID information
Insert the VGA EDID Injector into the video signal connection close to the computer.

**Important Notice:** There must be no DDC signals present on the display side of the connection. Otherwise the VGA EDID Injector will try to attempt to snapshot the information found there and, if successful, will overwrite the previously snapshoted EDID information.

In that case it is recommended to prepare a special VGA cable in which the signal carried over the pins 12 and 15 is severed (simply removing the pins from one of the connectors with some side cutters and/or pliers will do the trick).

### Micro-USB connector
An upcoming firmware release will provide a command line interface over USB CDC ACM serial device emulation. Until then the Micro-USB connector serves for supplying auxiliary power (+5V). Under normal circumstances the VGA connection will provide +5V, 50mA(max) over pin 9. However long cable lengths or a "weak" system's power supply (think laptops) may prevent the supply to meet the demands.

The circuitry of the VGA EDID Injector incorporates two rectifier diodes to prevent current to flow back into either the VGA port or the Micro-USB if both are connected at the same time, or should a computer get accidently connected on the display side. However those diodes create a 0.7V voltage drop.

Hence should the VGA EDID Injector seem to be nonoperational it is suggested to provide auxiliary power through the Micro-USB connector.

