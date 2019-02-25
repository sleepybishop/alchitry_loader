# alchitry_loader

this is the Alchitry C++ loader rewritten in C and the closed sourced D2XX drivers replaced with libftdi for easier portability to SoC's for which D2XX libraries are not available.

if you have a platform which is supported by the original loaders it would be advisable to stick with those.

TODO:
* SPI support for the CU
* handle cases when FT2232H is blank
* better allocation cleanup

