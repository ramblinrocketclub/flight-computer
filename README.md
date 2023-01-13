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
* [GCC](https://gcc.gnu.org/install/binaries.html)
* [ARM GNU Toolchain](https://developer.arm.com/downloads/-/gnu-rm)
* [Stlink](https://github.com/stlink-org/stlink/tree/master)

## Build and Run

### Getting the sources

First, fork the gtxr repository so that you can make a pull request. Then, clone your fork locally:

```
git clone https://github.com/<<<your-github-account>>>/gtxr.git
```

Ocasionally you will want to merge the changes in the upstream repository (the official code repo) with
your fork

```
cd gtxr
git checkout master
git pull https://github.com/ramblinrocketclub/gtxr.git master
```

Manage any merge conflicts, commit them, and then push them to your fork.

### Build

Compile the source code using `make`:

```
cd gtxr
make
```

### Run

You must have a development board in order to run and test code. Flashing code onto the microcontroller
is done by the following:

```
cd gtxr
make flash
```



The flight computer software is written in c using only the standard peripheral libraries provided
by STM32 for the STM32H723ZGT6 microcontroller. The program may be compiled using `make` and
flashed to the microcontroller using `make flash` while in the root project directory.
