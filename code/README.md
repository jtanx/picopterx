Server code
=============
This folder contains all the code runs on the hexacopter. This code has been
written in C++, conforming to the C++11 standard.

## Requirements
* git
* cmake >= 2.8 (version 3 is recommended)
* make
* gcc/g++ >= 4.7 (4.8 or later is recommended)
* wiringPi (if compiling on the RPi)
* boost - only for `shared_ptr` and only for the server (src/server/picopter.cpp) due to Thrift dependency
* OpenCV - version 2.4 or higher (anything lower is not supported!)
* Thrift >= 0.9 (0.9.2 is used) - need the Thrift compiler, as well as the Thrift C++ and PHP backends.

### Optionals
* Doxygen (for generating source code documentation)
* LaTeX (for generating source code documentation)
* `libgps-dev` (GPSD) - if you want to use the XSens GPS via GPSD (e.g. never because it's a POS).
* `python-gps` (for `gpsfake`), `gpsd-clients` (for `cgps`)
* [ArduCopter](https://github.com/diydrones/ardupilot) compiled for SITL (if you want to run it on your own computer)

## Installation
Ensure you have git installed:

    sudo apt-get install git

Now install the compiler:
* At the time of writing, Debian Wheezy bundles gcc 4.7. If you want to install a later version, [see this guide](http://somewideopenspace.wordpress.com/2014/02/28/gcc-4-8-on-raspberry-pi-wheezy/).

~~~~~
sudo apt-get install make cmake gcc g++
~~~~~

If you want the GPSD GPS backend (unnecessary now with Pixhawk/MAVLink), install these too:

    sudo apt-get install gpsd libgps-dev python-gps gpsd-clients
	
If you want to build the documentation (not recommended for the RPi; installation requires ~1GB of space).

    sudo apt-get install doxygen
	
## Building

~~~~~
git clone --recursive https://github.com/jtanx/picopterx
cd picopterx/code
./configure
make
~~~~~ 

If you want to make the test suite too, then do this instead:

~~~~~~
git clone --recursive https://github.com/jtanx/picopterx
cd picopterx/code
./configure -Dtest=ON
make
~~~~~

To force emulation of RPi specific components, specify `-DFORCE_EMULATION=ON`. To not build the server component, which depends on Boost and Thrift, specify `-DDISABLE_SERVER=YES`. To build the test applications in the `apps` folder, specify `-DBUILD_OPTIONALS=YES`. There may also be other options - look in the top-level CMakeLists.txt to see what else there is.

To make the source code documentation (CMake version 3 or greater only):

~~~~~~
make doc
~~~~~~

Otherwise (assuming you have Doxygen and LaTeX installed):

~~~~~
cd picopterx
doxygen
cd doc/latex
make
~~~~~

The documentation is located at `doc/latex/refman.pdf`.


To run the test suite:

~~~~~
make test
~~~~~
