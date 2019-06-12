# ClapSwitch

This software implements logic for a Clap Switch. Two claps turns it on. Two
claps turns it off again. Features include:

- controllable light hue and brightness
- a mechanical push button for when clapping is inappropriate

Please note that this is not an officially supported Google product. Though I am
employed by Google, this is a personal project.

Speaking of which, I'm not a professional hardware engineer, just a very
enthusiastic hacker. Let me know if you find bugs, I'll try to fix them.

This project is documented on hackaday.io at
https://hackaday.io/project/165448-happy-clap-switch.

## Software

The implementation consists of three files:

*   `.atmelstart/atmel_start_config.atstart` The Atmel START configuration file,
    used to generate hardware intitialization, libraries and boilerplate code.
    Edit this file using the online editor at start.atmel.com.
*   `main.c` The actual code that does things.
*   `driver_isr.c` An empty file that replaces the default `driver_isr.c` file
    generated by Atmel START.

This should be compatible with any IDE/environment that uses Atmel START but has
only been tested with Atmel Studio.

To use this repository with Atmel Studio:
1.  Clone this repository to a temporary location. This is important because
    Atmel Studio wants to create its own directories.
2.  Upload the `atmel_start_config.atstart` file to start.atmel.com and
    generate and Atmel Studio configuration.
3.  Open Atmel Studio and import the configuration. You will now have a project
    full of boilerplate code.
4.  Copy the temporary repository directory into the Atmel Studio project
    directory. This will overwrite `main.c` and `driver_isr.c`. Make sure to
    copy the `.git` directory and `.gitignore` files too.

## Hardware

This was written to run on an Atmel ATtiny3217 processor. It takes input from a
microphone, a push button and two rotary encoders. Output is to a short string
of WS2812s LEDs.


