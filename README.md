# GBA for MiSTer
This core is currently non-functional for any practical purpose and is not being actively developed.

It is a port from this project: https://github.com/soctar/GBA

Currently only one game will run, pong, which is found in the text_games directory.

To use this, the 16kb GBA bios must be boot.rom in the GBA folder on your SD card.

## What this core needs
A lot. I spend time cleaning up the code and fixing a few hundred warnings. The memory controller in particular needs evaluation and either SDRAM or DDRAM needs to be used for loading roms. Because the largest roms for GBA are 32mb they would just barely fit in SDRAM, but due to the 16 bit nature of the SDRAM and the 32 bit nature of the GBA bus, I think that DDRAM would be a better choice for this. In the docs folder, the "final report.pdf" file has a very detailed overview of the state of the code.