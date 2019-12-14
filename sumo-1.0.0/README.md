[![Linux and MacOS Build Status](https://travis-ci.org/eclipse/sumo.svg?branch=master)](https://travis-ci.org/eclipse/sumo)
[![Windows Build status](https://ci.appveyor.com/api/projects/status/github/eclipse/sumo?svg=true)](https://ci.appveyor.com/project/eclipsewebmaster/sumo)



Eclipse SUMO - Simulation of Urban MObility 
===========================================

What is SUMO
------------

["Simulation of Urban MObility" (SUMO)](http://sumo.dlr.de/) is an open source,
highly portable, microscopic traffic simulation package designed to handle
large road networks and different modes of transport.

It is mainly developed by employees of the [Institute of Transportation Systems
at the German Aerospace Center](http://www.dlr.de/ts).


Where to get it
---------------

You can download SUMO from SourceForge via our [downloads site](http://sumo.dlr.de/wiki/Downloads).

As the program is still under development and is extended continuously, we advice you to
use the latest sources from our GitHub repository. Using a command line client
the following command should work:

        git clone --recursive https://github.com/eclipse/sumo


Mailing List
------------

To stay informed, we have a mailing list for SUMO. You can subscribe at
https://dev.eclipse.org/mailman/listinfo/sumo-user.
Messages to the list can be sent to sumo-user@eclipse.org.
SUMO announcements will be made through the sumo-announce@eclipse.org list;
you can subscribe to this list at https://dev.eclipse.org/mailman/listinfo/sumo-announce.


Build and Installation
----------------------

For Windows we provide pre-compiled binaries and CMake files to generate Visual Studio projects.
Using Linux a simple "./configure && make" should be enough for the distributions, if you
have installed all needed libraries properly. Using the repository checkout you
need to issue "make -f Makefile.cvs" before "./configure && make" in order to run
the autoconf utilities creating configure and the Makefiles.
If configure does not find the libraries or includes needed, please check
"./configure --help" for information on how to specify the paths needed.

For [detailed build instructions have a look at our wiki](http://sumo.dlr.de/wiki/Developer/Main#Build_instructions).


Getting started
---------------

To get started with SUMO, take a look at the docs/tutorial and examples directories,
which contain some example networks with routing data and configuration files.
There is also user documentation provided in the docs/ directory and on the
homepage.


Bugs
----

Please use for bugs and requests the [GitHub bug tracking tool](https://github.com/eclipse/sumo/issues)
or file them to the list sumo-user@eclipse.org. Before
filing a bug, please consider to check with a current repository checkout
whether the problem has already been fixed.


License
-------

SUMO is licensed under the [Eclipse Public License Version 2](https://eclipse.org/legal/epl-v20.html).
For the licenses of the different libraries and supplementary code information is in the
subdirectories and the [wiki](http://sumo.dlr.de/wiki/License).
