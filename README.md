---------
OVERVIEW 
---------
MIDI Controler project that includes:
two rotate potentiometers,
one slide potentiometer,
twelve buttons,
two buttons for changing octave,
display.
Picture of the project (MIDIProject.jpg)
------------
DESCRIPTION
------------
Hardware: 
STM32 microcontroler, 
USART interface, 
1 x display,
14 x buttons, 
2 x rotate linear potentiometers (1 k), 
1 x slide linear potentiometer (10 k),
12 x resistors (3,3k), wires.
Software:
hairless-midiserial, 
loopMIDI.
------
TOOLS
------
Software used by creating the project:
STM32CubeMX, 
System Workbench for STM32, 
STMStudio, 
MIDI-OX.
-----------
HOW TO RUN
-----------
1. Build new MIDI port in loopMIDI (readme1.jpg)
2. Connect STM32 MC and USART device.
3. Run project.
4. Build new COM/MIDI OUT bridge in hairless-midiserial (choose USART device COM port; readme2.jpg).
5. Now you can use MIDI Controler in any DAW.

Using expample in FL Studio 20:
1. Potentiometers - right click on any control and click 'Link to controller' then move potentiometer (readme3.jpg).
2. Buttons - the buttons imitate one octave (like in piano from C to B ton). Implicitly program run on 5th octave but 
you can change it with rotate potentiometer from 0-10. The current octave displays on screen. 
---------------
HOW TO COMPILE
---------------
Connection to microcontroler:
PE0-PE7; PB14-PB15 - display GPIO OutPut
PA0-PA2 - potentiometers ADC OutPut
PD2-PD13 - buttons EXTI InPut
PC10/PC11 - USART TXD/RXD

Just copy main.c file and compile
--------------------
FUTURE IMPROVEMENTS
--------------------
Possible improvements contains:
replacing buttons for dynamic buttons that send to DAW inforamtion about power of pressing a button that would change 
a velocity value of the note,
adding more potentiometers,
--------
CREDITS
--------
Konrad Marciniak
Piotr Zajdziński

The project was conducted during the Microprocessor Lab course held by the
Institute of Control and Information Engineering, Poznan University of Technology.
Supervisor: Adam Bondyra
