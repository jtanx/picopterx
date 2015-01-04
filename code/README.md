Server code
=============
This folder contains all the code runs on the hexacopter. This code has been
written in C++, conforming to the C++11 standard.

## Requirements
* git
* cmake >= 2.6 (version 3 is recommended)
* make
* gcc/g++ >= 4.7 (4.8 or later is recommended)
* gpsd, libgps-dev (hexacopter only)

### Optionals
* Doxygen (for generating source code documentation)
* LaTeX (for generating source code documentation)
* python-gps (for `gpsfake`), gpsd-clients (for `cgps`)

## Installation
Ensure you have git installed:

    sudo apt-get install git

Now install the compiler:
* At the time of writing, Debian Wheezy bundles gcc 4.7. If you want to install a later version, [see this guide](http://somewideopenspace.wordpress.com/2014/02/28/gcc-4-8-on-raspberry-pi-wheezy/).

~~~~~
sudo apt-get install make cmake gcc g++
~~~~~

If you are building on the hexacopter (RPi),

    sudo apt-get install gpsd libgps-dev python-gps gpsd-clients
	
If you want to build the documentation (not recommended for the RPi; installation requires ~1GB of space).

    sudo apt-get install doxygen
	
## Building

~~~~~
git clone https://github.com/jtanx/picopterx
cd picopterx
git submodule init
git submodule update
cd code
./configure
make
~~~~~ 

If you want to make the test suite too, then do this instead:

~~~~~~
git clone https://github.com/jtanx/picopterx
cd picopterx
git submodule init
git submodule update
cd code
./configure -Dtest=ON
make
~~~~~

To make the source code documentation:

~~~~~~
make doc
~~~~~~

If you have CMake version 3 or greater and you have Doxygen and LaTeX installed,
the output will be in `doc/latex/refman.pdf`.
