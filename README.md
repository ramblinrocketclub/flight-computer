
# Contributing

The flight software may be edited with any text editor other than CubeMX or STM32CubeIDE. Contributing comes in
many forms: reporting issues, submitting pull requests, and logging bugs.

Do not use any external libraries such as the HAL provided by STM. Use only the libraries present.

## Prerequisites

In order to download the necessary tools, clone the repository, and install necessary dependencies, you
need network access.

You'll need the following tools:
* [Git](https://git-scm.com/)
* A C/C++ compiler tool chain for your platform:
    * Windows 10/11
        * [MinGW](https://www.youtube.com/watch?v=DHekr3EtDOA)
    * macOS
        * [Xcode](https://developer.apple.com/xcode/downloads/) and the Command Line Tools, which will
        install `gcc` and the related toolchain containing `make`
            * Run `xcode-select --install` to install the Command Line Tools
    * Linux
        * On Debian-based Linux: `sudo apt install build-essential`
* [ARM GNU Toolchain](https://developer.arm.com/downloads/-/gnu-rm)
* [Stlink](https://github.com/stlink-org/stlink/tree/master)

## Repository Structure

Knowing the project structure is essential to understanding the workings of this project and where exactly contribution
to the project takes place.

### Analysis

The `Analysis` directory contains scripts that aid in analyzing the behavior of the flight software.

### Core

The `Core` directory contains the main code that will be modified and updated to progress the flight software. This is the
only directory contributers may access and edit.

### Docs

The `Docs` directory contains automatically generated documentation for the project via Doxygen. Do not edit the contents
of this directory directly.

### Drivers

The `Drivers` directory contains the necessary libraries and drivers for the microcontroller such as register addresses
and low-level functions.

### Middlewares

The `Middlewares` directory contains middle layers in software such as an RTOS.

### Doxyfile

The `Doxyfile` is a configuration file for Doxygen, which automatically generates documentation for the project based
on comments place within the code.

### Makefile

The `Makefile` eases the process of compiling the project.

## Build and Run

### Getting the sources

First, fork the flight-computer repository so that you can make a pull request. Then, clone your fork locally:

```
git clone https://github.com/<<<your-github-account>>>/flight-computer.git
```

Ocasionally you will want to merge the changes in the upstream repository (the official code repo) with
your fork

```
cd flight-computer
git checkout master
git pull https://github.com/ramblinrocketclub/flight-computer.git master
```

Manage any merge conflicts, commit them, and then push them to your fork.

### Build

Compile the source code using `make`:

```
cd flight-computer
make
```

### Run

You must have a development board in order to run and test code. Flashing code onto the microcontroller
is done by the following:

```
cd flight-computer
make flash
```

### Making modifications

When making modifications, always ensure that you are doing them within a separate branch. For instance,
make a branch named `fix-imu-parsing` to fix an issue with the imu parser. Make sure that the names you assign to your
branches are specific and concise. If an idea you have may take a lot to implement, break it down into bite-sized chunks
and make all of those chunks branches. This way, testing and development will be much easier. When you have finished writing
modifications, commit the code and move on to the next step: making a pull request.

### Making a pull request

One you're done with your modifications, it's time to make a pull request and submit your changes. To make a
pull request, click on the green `Compare & pull request` button that appears when you go back to your forked repository.
From there, make a descriptive comment about the changes and additions made, and finally submit the pull request to be
reviewed.
