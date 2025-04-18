PERQemu Readme


1.0 Introduction
================

PERQemu is an emulation of the venerable Three Rivers Computer Corporation (3RCC)
PERQ-1A system.

Emulating this system has been an immense challenge, but so far it's been a lot
of fun...  See section 3.0 to see what works and what doesn't.

PERQemu is written in C# under the .NET Framework 2.0.  The source code is now
available on GitHub at https://github.com/jdersch/PERQemu/.  I've been working
on it on and off since August of 2006.  Thanks for taking a look at it.

Thanks,
- J. Dersch


1.1 A Little PERQ history
-------------------------

The PERQ is an early microcoded graphical workstation with a high-resolution
bitmapped display and is arguably one of the first commercially-available
"workstation" class computers to meet the "3 M" criteria: a megabyte of RAM,
a million pixel display, and a million instructions-per-second of processing
power.  Heavily influenced by the Xerox PARC Alto and taking inspiration from
the DEC PDP-11 for some of its performance tricks, it was first demonstrated
in 1979 but not released in production quantities until late 1980.  It is
estimated that fewer than 5,000 PERQs were built, mostly sold to universities.

The PERQ-1 hardware consists of the following:

    - A custom bit-slice, microcoded CPU with 48-bit microinstruction words
    - 4K (PERQ-1) or 16K (PERQ-1A) of writable control store
    - 512KB to 2MB of RAM (in 16-bit words)
    - A high resolution bitmapped display at 768 x 1024 pixels (1bpp)
    - Custom RasterOp hardware to accelerate bitmap operations

The original PERQs feature a standard set of peripherals:

    - A 12 or 24mb Shugart SA4000-series hard disk (14" platters)
    - A single 8" Shugart floppy drive
    - A GPIB interface, typically used to interface with a Summagraphics
      BitPadOne digitizer tablet
    - One programmable RS232 port, up to 9600 baud
    - A CVSD chip for audio output

Optional IO boards can be fitted, which provide:

    - 3Mbit or 10Mbit Ethernet interfaces
    - Canon LBP-10 (or Canon CX) laser printer interface
    - QIC streamer tape connection

Introduced in 1983, the PERQ-2 series extends the original design in some
significant ways and adds a number of additional IO options:

    - A landscape display, with 1280 x 1024 pixels (1bpp)
    - An 8" Micropolis disk (PERQ-2 or PERQ-2/T1) or up to two 5.25" MFM/ST506
      disks (PERQ-2/T2 or /T4) replace the Shugart 14" hard drive
    - The standard I/O board incorporates an Ethernet controller and a faster
      Z80 subsystem with RTC chip, second RS-232 port
    - A 24-bit version of the 16K CPU extends the memory capacity to 8MB in
      the rare PERQ-2/T4 model

PERQemu will emulate all of the standard PERQ-1 and PERQ-2 configurations and
peripherals.


1.2 Current Status
------------------

PERQemu versions through 0.4.5 focused exclusively on PERQ-1 support.  Before
the Great Refactoring (v0.5.0 and beyond) the WinForms-based emulator could run
only the PERQ-1 "old" Z80 with a single Shugart hard disk.  The emulation was
fairly complete, but the options for OS and software to run were limited.

The v0.5.0 release was a major leap in functionality and the available software
base expanded greatly, but it still only emulated PERQ-1 configurations.  Work
on the experimental branch shifted to expanding emulation options, adding new
peripheral support (Ethernet, laser printer), bug fixes, and new UI features.

PERQemu v0.5.5 added Canon printer support and expanded experimental Ethernet
support.  It was an interim release to skeezicsb/main and wasn't submitted for
a merge into the master.

PERQemu v0.5.8 introduces the first milestone in PERQ-2/EIO support, adding a
Micropolis 8" disk driver and enough of the EIO/Z80 components to allow POS G,
Accent S6 and PNX 2 and 3 to boot.  There are still a number of rough edges and
missing pieces, but this interim development snapshot is reliable enough to let
it into the wild for testing from skeezicsb/main.

PERQemu v0.6.5 delivers nearly complete PERQ-2 and 2/Tx support; at this point
only the 24-bit PERQ-2/T4 is unfinished (but in testing).

Please check back often for updates!


1.3 System Requirements
-----------------------

You will need a Windows machine with the .NET Framework 4.8 installed, or a
Linux/UNIX/Mac OS machine with the Mono 6.x runtime installed.

PERQemu is a nearly cycle-accurate, register-level emulation of a complex 
microcoded processor AND a Z80 subsystem -- essentially two emulations running
side-by-side.  Your computer should have a reasonably current processor with at
least 2-4 CPU cores to run the emulator at speed.  While a CPU hog, PERQemu's
memory footprint is very modest, requiring less than 100MB of RAM with a typical
24MB Shugart hard drive configured.

To gauge how faithfully your computer is able to emulate the PERQ, the title of
the display window updates every few seconds to report the frame rate ("fps")
and the average cycle time of the PERQ and Z80 processors.  At full speed the
PERQ will display 60fps, with the CPU executing at 170ns and the Z80 at 407ns.

Please consult the User's Guide for more information about system requirements
and performance (rate limiting) options.


2.0 Getting Started
===================

This is a very basic "quick start" guide.  For more information about running
the emulator, please consult the UserGuide.pdf included in this distribution.
This comprehensive User's Guide includes a complete command reference.


If you've gotten this far you've unpacked the Zip archive and you have a 
directory containing the emulator executable, PERQemu.exe.

There are several subdirectories:

    Conf/
        Contains a collection of "prefab" system configurations as well as
        device data required for operation.  By default, all custom PERQ
        configurations are also saved in and loaded from this directory.

    Disks/
        Contains disk images that the emulator can access.  Included with
        the distribution are:

        d6.prqm:
            (Josh) A dump from my very own PERQ1's disk, which has POS
            D.61 and Accent S4 installed.

        f0.prqm:
            (Skeezics) A dump of my POS F.0 drive, which has a working
            MPOS E.29 installation as well!
              
        f1.prqm:
            A disk containing a pretty complete installation of POS F.1,
            including source, documentation, the Pascal compiler and a
            number of games, demos, and applications.  This was created
            from floppy images on Bitsavers.

        f15.prqm:
        f15dev.prqm:
            Updated Shugart images containing the offshoot POS F.15
            distribution, in both basic and developer (full source)
            versions.  Includes Amendments 1 & 2.

        g6mic.prqm:
        g6mfm.prqm:
            New POS G.6 images for the PERQ-2 and PERQ-2/T2.  These are
            somewhat cobbled together from floppy and tape images but
            include patches through Amendment 3, full source code, and
            lots of apps, games and demos!

        g7.prqm:
            The first PERQmedia-formatted Shugart image for use with the
            "CIO" board.  Contains a POS G.7 installation (binary only)
            with a few extras, and a basic Accent S6 installation without
            any extra apps (yet).  This image includes some fun new demos
            not previously available on PERQemu!

        s6lisp.prqm:
            Update of the g7 image with Spice Lisp version M3!

        pnx1.prqm:
            A bundle of the basic PNX 1.3 installation from the PERQmedia
            repository, but reformatted as a .prqm image.

        pnx2mic.prqm:
            An 8" Micropolis disk for PERQ-2 containing the available
            PNX 2 installation.  Many improvements over PNX 1, but the
            man pages are missing and no extra apps/demos are available.

        Additional "stock" hard drive or floppy images may be included as
        well.  Any custom disk images you create or import are loaded from
        and saved in the Disks/ directory by default.  Please consult the
        UserGuide for information about working with PERQemu media files.

    Output/
        When logging debug output to disk is enabled, log files go here by
        default.  Unnamed files copied to the RSX: device, screenshots and
        Canon laser printer output is directed here as well.  (The output
        directory path is a settable preference.)

    PROM/
    Resources/
        These directories contain dumps of PERQ ROMs, Z80 source code and
        other files necessary for operation.


To start the emulator, just run PERQemu.exe:

    Windows: double-click the icon.  PERQemu is a "console application,"
        so a command window will appear and you'll be at the command prompt.

    Unix/Linux/Mac: invoke "mono PERQemu.exe" from the command-line in a
        terminal window.  The PERQemu debugger will announce itself, same
        as in the Windows version.

The top-level command prompt is a single '>'.  The command-line interface (CLI)
now organizes the extensive command set into a hierarchical set of "subsystems".
The prompt will change according to the current level in the hierarchy, such as:

    configure>
or
    settings>

The CLI provides extensive prompting through tab completion and other on-line
help.  Consult the UserGuide for help navigating and running commands.

The first time you run PERQemu, the default configuration is selected.  The
default is a PERQ-1A with the POS F.1 image already assigned.  This default
configuration is the only one built-in to the emulator; all the rest are loaded
from the Conf/ directory.

To start the emulation, type the shortcut:

    > go

At this point the PERQ will "power on," the Display window will appear and the
virtual machine will load the f1.phd hard disk image and start executing.
Eventually you'll be greeted with the POS login prompt.

As with previous versions of PERQemu, pressing ^C will pause the running PERQ.
Unlike the older command interface, however, the CLI now remains active even
when the emulator is running!  You may type "stop" or other commands to control
or interact with the emulator.  Consult the UserGuide for more information.

To shut down the PERQ, you can simply close the Display window.  The emulator
will report that the machine is shutting down and return you to the prompt.
You may also type the commands:

    > stop

to pause, or just

    > power off

to immediately halt and "power down" the virtual machine.  If you are in a
real hurry:

    > quit

will exit the program, shutting down the machine if it is running.  Note that
PERQemu does NOT automatically save any disk images that you load, so if you
want to save changes to the hard drive or floppy disk images you should do so
manually before issuing the "power off" or quit commands.  However, the new
"settings" subsystem offers a set of preferences to making saving automatic.

There is a LOT more to explore in PERQemu now, with dynamic configuration, new
storage devices, extended debugging, user preference settings, and more on the
way.  Say it with me now: "Consult the UserGuide for more information!"


2.1 Operating System Support
----------------------------

As of v0.4.2, the following PERQ operating systems are known to boot:

  - POS versions D.6, F.0 and F.1 (official 3RCC releases);
  - POS version F.15 (released by Boondoggle Heavy Industries, Ltd);
  - MPOS version E.29 (unreleased by 3RCC);
  - Accent S4 (an early version from CMU, unreleased);
  - PNX 1.3 (first public release by ICL).

As of v0.4.6, these PERQ operating systems also boot:

  - POS version G.6 (last official 3RCC release);
  - POS version G.7 (released by Accent Systems, 1986?);
  - Accent S6 (Release II from PERQ Systems Corporation, 1985).

Fixed in v0.5.3:
  - A workaround for PNX 2 video support is included (disk image TBA).
  - Accent S7 (Release III, unreleased) also verified as well.

Added in v0.5.8:
  - PNX 3 is now also confirmed to boot and run (from floppy) with an
    included video patch.  (The available floppy installation set is
    incomplete and needs to be re-imaged before a full hard disk
    installation is possible, however.  Watch this space.)
  - Debug pre-release for early Micropolis disk support testing.

Added in v0.6.x:
  - 5.25" MFM disk support for PERQ-2/Tx models (POS G, Accent tested;
    PNX not yet available)

NOTE: PNX 1 only supports 1MB of memory and will crash if configured with more.
PNX 2, PNX 3, POS, MPOS and Accent have no trouble with a full megaword (2MB)
of memory.

Accent mouse tracking takes a little getting used to since it runs in relative
mode.  To simulate mouse "swipes" you have to use the Alt key (Option key on
Mac) to tell PERQemu the mouse is "off tablet", reposition, then release the
key to start tracking again.  It's a little clumsy at first.


If you have any other PERQ software that does not run successfully under
PERQemu, send us a copy and we'll find out why!


3.0 What's Implemented
======================

The following hardware has been implemented in the emulator:

  Processors:
    - 4K and 16K CPUs (20-bit) are tested and complete;
    - 16K CPU (24-bit) is complete but not tested; waiting on PERQ-2/EIO;
    - The CPU, memory and video run on a separate thread.

  Memory/VideoController:
    - Now can be configured at runtime, up to 8MB in the 24-bit models;
    - Only tested for operation with the 20-bit processors (max 2MB / 1MW).

  Hard disk:
    - The original PERQ-1 14" Shugart SA4000-series drives and controllers are
      tested to work with IOB (old Z80) and CIO (new Z80) implementations;
    - The PERQ-2 8" Micropolis 1200-series controller and Disk Interface Board
      is now emulated for use with the PERQ-2/EIO configuration (one drive only);
    - One or two 5.25" MFM drives can be configured if the PERQ-2/Tx chassis is
      selected.
      
  Floppy disk:
    - Rewritten to work with the new Z80 and floppy disk controller;
    - Supports dynamic loading and unloading of all media types (single- and
      double-sided diskettes, in single- and double-density).

  Tape drive:
    - The streaming tape drive is now available as the Tape option when the
      OIO board is selected.  It emulates the Archive Sidewinder QIC tape
      drive, providing 20MB of storage.  The UserGuide has more details!

  Displays:
    - The standard 768 x 1024 portrait display is available for all models;
    - The 1280 x 1024 landscape display is supported and tested with POS G.7
      and Accent S6!  Although PERQ-1 landscape configurations were very rare,
      the emulator runs 'em just fine!  Became standard equipment on most
      PERQ-2 models.

  Z80 I/O Processor:
    - Simulation replaced by a real Z80 emulator running actual PERQ ROM code;
    - Runs asynchronously on its own thread to improve performance;
    - Allows different ROMs to be loaded to support CIO and EIO boards;
    - New register-level interface written to support Z80 DMA, CTC, SIO, PIT,
      FDC and GPIB controller chips;
    - Z80 Debugger support includes single stepping and source code display
      (for the current v8.7 ROMs; v100.017 source disassembly for EIO is now
      complete, CIO in progress).  Limited breakpoint support is available.

  Keyboard:
    - Now uses the SDL2 interface so no more horrible hacks required for MacOS;
    - Support for the VT100-style PERQ-2 keyboard is now included and is working
      (but is not fully tested and has some limitations);
    - Currently caps lock is problematic and can get out of sync with the host.
      This is a minor inconvenience but it's on the bug list.  [TODO: check if
      this is still the case under SDL2.]

  RS-232:
    - The Z80 SIO chip is implemented to work with the new Z80 emulator;
    - Software running under emulation can control a real physical serial port
      on the host; second RS-232 port available with the PERQ-2/EIO models [not
      yet fully tested];
    - The RSX: pseudo-device for transferring text files from the host to POS
      has been reinstated.

  GPIB:
    - The TMS9914 controller chip is implemented to work with the new Z80, but
      it is still incomplete and occasionally seems to confuse POS (reporting
      non-fatal errors that don't seem to negatively affect operation);
    - Supports basic System Controller, Talker and Listener features, but just
      enough to support what the PERQ needs.  Being able to drive a real GPIB
      card in the host computer would be pretty darn cool but I wouldn't hold
      my breath on that one.

  Tablets:
    - The proprietary Kriz tablet works with the new SIO chip; it is only
      useful on POS F.1 and later (no support in D.6, F.0, PNX 1, or Accent S4);
    - The simulated Summagraphics BitPadOne works with the new GPIB and is
      supported on all PERQ OSes;
    - PERQ-2/EIO configuration support: Kriz tablet and GPIB BitPad are working.

  Ethernet:
    - A "null" bare-bones interface is now available when the "Ether" option for
      the OIO option board is configured.  This allows Accent to initialize its
      NetMsgServer so other peripheral server processes (floppy, serial, etc)
      can start up.  Consult the User Guide for more details!
    - The EIO version of the interface is being tested for use with PERQ-2;
    - Implementation of a real host Ethernet interface is available for testing,
      but with caveats.  Check the User Guide for details.

  Canon:
    - Laser printer interface can now be enabled as an OIO option.  It provides
      high quality output in PNG or TIFF format at 240- or 300-dpi.


There is a ton of additional detail about the internals of PERQemu itself in
the source distribution.  See Readme-source.txt, or the copious notes in the
Docs/ directory for way, way more information than you need.  Way more.


3.1 What's Not
--------------
 
- Ethernet.  In development! [See above]

- Option boards:  3Mbit Ethernet.  On the list.
 
- PERQLink.  Unimplemented other than a stub that tells the microcode that
  there's nothing connected to it.
 
- Sound.  Yet to be rewritten to work with the new Z80/SIO and hooked up to
  any sort of host output device.

- Multibus option and SMD disk/9-track tape support.  Dream on!

- A proper GUI.  Sigh.

Additionally, some debugger commands are planned/in development but are not
documented or complete, and some CLI enhancements are unfinished.


4.0 Quirks, Bugs, Misfeatures
=============================

The following known issues are present, with workarounds given where possible.
Undoubtedly there are numerous other small bugs, glitches and aesthetic issues
but the aim here is to document the most serious of them.  Feedback is welcome!


1. Console sometimes loses track of the current input line.

Symptoms:  Sometimes the console output will wrap around to the top of the
window and overwrite previous output, rather than scrolling, making it difficult
to see the input prompt.  This seems most prevalent on Windows 10.

Workaround:  On Windows, enable "legacy mode" in the console Properties.  This
seems to fix some of the glitches but it's still a mess.  Mac and Linux terminal
applications don't tend to misbehave as badly.  Hitting ^L now clears and resets
the window in case things are messy. [This is kinda broken at the moment and is
being looked into.]


2. Minimizing the Display window makes it disappear for good / very difficult
to restore.

Symptoms:  Pressing the window's minimize button sends it to never-never-land.

Workaround:  On Windows, disable "autohide" for the task bar.  Otherwise you
have to do some weird Ctrl-right-click-Restore gyrations to force the Display
window back onto the screen.  This was fixed?  Kind of?  Except when it isn't?


3. Reading from the serial port is unreliable.

Symptoms:  On Linux, reading data from a COM port (/dev/ttyS0) stalls unless
output is transmitted (to prod the receiver).  Windows and Mac serial devices
seem to struggle less, but this isn't exactly "production ready."  Flow control
on all three platforms is unreliable and data may be garbled or lost.

Workaround:  None, yet.  This is largely due to serious deficiencies in the
C#/Mono System.IO.Ports.SerialPort implementation that will require a reworking
of the emulator's port handling.


4. PNX boot failure at DDS 142.

Symptom:  Attempting to boot from the PNX installation floppies causes the DDS
to stop at 142 and the process to hang.

Workaround:  The version of the VFY microcode included on the available floppies
contains a bug -- or relies on a quirk of the hardware -- that causes the Victim
test to fail when a 16K CPU is configured.  For PNX versions 2 or 3, when the
DDS stops at 142, type "debug jump $808" to resume execution, bypassing the
failed test.  Note that the DDS status will be off for the remainder of the
session, and will not stop at 255 when PNX has completed booting.

Solution:  A patch to detect and fix this automatically was included in PERQemu
v0.5.8 through v0.6.5; the CPU was modified to correct the issue and the patch
removed in v0.6.6 (experiments branch).


5. PNX video glitches.

Symptom: The PNX 2 window manager sometimes randomly paints its background 
pattern with strange stripes or other visual anomalies.

Solution:  Corrected in PERQemu v0.6.9.


6. PNX 5 kernel panic after boot.

Symptom:  Initial PNX 5 floppy boot drops into the kernel debugger.  Attempts to
continue eventually cause a kernel panic.

Workaround:  None at this time.  Investigating whether this is a corrupted boot
image or yet another PNX difference in how their microcode drives the hardware.
Affects PERQ2-T2 configurations only (if anyone else is attempting to perform a
bare metal install from the Bitsavers floppies).  Some damn source code would be
awfully helpful, ICL.  :-(


5.0 Version History and Roadmap
===============================

v1.0 - TBD
  Sometime before the heat death of the universe:
  - Feature complete, with a nice GUI, full screen mode, VR, scratch 'n sniff
  - Massive software library organized, catalogued, available for use and study
  - Working audio output :-)

v0.9 - TBD
  Fill in the final pieces:
  - Full-featured Ethernet
  - 24-bit "T4" model with larger memory
  - See if CIO Micropolis has any real software support?
  - Multibus and SMD disks?

v0.7.5 - RSN
  Leverage the new architecture to roll out new models, new peripherals and
  open up the full range of available operating systems!
  - Add more bundled configurations and hard disk images
  - PERQ-2 EIO emulation support: expanded IO Board with faster Z80, second
    serial port, RTC chip, support for two hard disks
  - PERQ-2 peripherals: 8" and 5.25" disk drives, VT100-style keyboard

v0.7.0 - Experiments branch
  - Comprehensive RasterOp update, refactoring to fix PNX glitches
  - Video update so floppy activity icon is visible during slow floppy boots
  - Update for EIO RTC chip programming and time handling
  - Bug fixes and some documentation updates

v0.6.5 - Main branch (v0.7 pre-release)
  - MFM support for PERQ-2/Tx, multiple 5.25" hard disks
  - Low-level formatting for 8" and 5.25" hard disks works
  - Improved hard disk seek timing
  - Update to PERQmedia library to allow more robust handling of partial
    or corrupted IMD-format floppy images
  - PERQ-2 backplane S/N PROM added

v0.5.8 - Main branch (v0.7 pre-release)
  - Micropolis 8" disk (high-level operation works! low-level formatting
    support is incomplete/untested)
  - EIO Z80 ROM source code formatted for use with the debugger
  - EIO Z80 peripherals added:  RTC chip, VT100-style keyboard, Am9519 IRQ
    control and i8237 DMAC
  - Kriz tablet and GPIB updated for EIO; PNX Vfy bug workaround devised
  - Ethernet drivers updated for EIO (no new functionality, yet)

v0.5.5 - Main branch (v0.7 pre-release)
  - Ethernet running (but requires root/admin access)
  - Patch for Turkish keyboard in CLI
  - Limited Micropolis 8" disk support
  - Refactored Z80 subsystem for EIO support
  - Patch to support PNX 2's non-standard video display list handling
  - Canon laser printer support
  - Screenshots available from the CLI

v0.5.0 - New baseline
  (Pending) Merge the experiments back into the master branch:
  - True Z80 emulation
  - PERQ-1 CIO (new Z80) support: updated to run new Z80 ROMs
  - 64-bit Mono/MacOS build (no 32-bit WinForms limitation)
  - SDL2 for improved display performance
  - Landscape display!
  - Unified PERQ media storage architecture and file format
  - Dynamic runtime configuration of all PERQ models and features
  - Enhanced command line interface with more prompts, in-line help
  - Expanded logging and enhanced debugging support
  - Persistent user preference settings

v0.4.9 - Experimental branch (v0.5.0 pre-release) 
  - Added a minimal Ethernet interface, bug fixes.
  - Bundled a working Spice Lisp disk image.

v0.4.8 - Main branch (v0.5.0 pre-relase)
  - Added streamer tape support!

v0.4.6 - Experimental branch (v0.5.0 pre-release)
  - All v0.5.0 features above, in a snapshot release prior to merge back into
    master.

v0.4.5beta - Unreleased
  - A one-off build for VCF PNW with some experimental video hacks to improve
    display performance.

v0.4.4 - Fourth major release
  - RasterOp streamlining and refinements allowed us to remove the
    "microcode bailed early" hack.  Sweet.
  - New and updated disk images for release.
  - Numerous documentation updates to support first Github tagged release.

v0.4.3 - Accent update
  - Massive one-byte change to get mouse tracking to work in Accent S4.
  - Expanded GPIB implementation to support talker/listener selection, 
    can now read and write commands and data for multiple bus targets beyond
    just the Bitpad.
  - Z80 ready/busy status reworked, and other small fixes and streamlining.
  - VideoController streamlined and timing glitch fixed to correct visual
    artifacting during vertical blanking.

v0.4.2.12 - Major Memory update
  - Rewrote the Memory implementation again, to improve performance.
  - Other small enhancements and bug fixes.

v0.4.2 - Major RasterOp update
  - Extensive reimplementation of the RasterOp pipeline; removed the "fake"
    RasterOp code entirely.
  - Additional debugging support added.

v0.4.0 - Development update
  - Completed 16K mul/divstep hardware.
  - First reimplementation of the "real RasterOp" pipeline.
  - Memory changes to support new Rasterop.
  - Platform-independent Keyboard rewrite, no more P/Invokes.
  - Somewhere between v0.3 and v0.4 is the snapshot uploaded to Github.
  - The state save/restore serialization code removed.

v0.30 - Third major release
  - Added 16K CPU support.
  - Rolled up changes for first source code release.

v0.25 - Interim release
  - Added 16K CPU support (minus mul/divstep hardware)
  - "Real" RasterOp emulation is 50% completed, still not working correctly.
  - Additional performance improvements due to caching, I/O clocking changes, and
    optimizations to the ALU.
  - Refactored debugger code, and added functionality for modifying PERQ registers
    and loading microcode
  - Fixed bug in EStack implementation that prevented PERQMan from running properly.
    (PERQMan RNG uCode leaves an extra entry on the stack with every invocation,
    which eventually overflows the eStack.  On a real PERQ this wraps around, my
    implementation was clipping the pointer into range.  This caused odd errors.)
  - Added special RSX: hook. 
  - Fixed keyboard handling: Now Ctrl+Shift+<key> registers properly.
  - Changed display update code to work around a Mono WinForms implementation issue.
    Should run without the display thread crashing under Mono now.

v0.21 - Minor update
  - Fixed small issues preventing PERQemu from running well under Mono.  Should
    run acceptably now!
  - Added some very hacky support for double-density floppy images.
  - Some minor optimizations to the CPU code result in a 5% performance gain.  

v0.2 - Second release
  - Z80 simulation is mostly complete, along with most subdevices:
    - RS232 is 99% done and will talk to a real serial port on the host machine
    - Kriz Tablet is 99% done and works sufficiently well to allow applications 
      that use the pointer to function.
    - Speech support is there but output is not hooked up to the host.    
    - GPIB support is only a stub.
    - Floppy disk support is 80% done and supports all of the functionality
      needed to deal with double-sided, single-density floppy images.
    - Keyboard support is 100% done.      
 - Hardware cursor implementation refined, vertical and horizontal positioning
   are now correct.  Vertical positioning occasionally is thrown off by a few
   pixels.  
 - A few minor speed improvements (nothing major) gained from only clocking the
   Z80 devices every 16 PERQ cpu clocks.  
 - Floppy disk R/W support added, Hard disk Write support added. 
 - Debugger commands for loading/saving and creating disk images for hard disks
   and floppies added.

v0.1 - First public release


Update history:

4/17/2025 - skeezicsb - v0.7.0 (experiments)
12/8/2024 - skeezicsb - v0.6.5 (main)
6/19/2024 - skeezicsb - v0.5.8 (main)
2/18/2024 - skeezicsb - v0.5.5 (main)
1/24/2023 - jdersch - v0.5.0
1/17/2023 - skeezicsb - v0.4.9 (main)
12/28/2022 - skeezicsb - v0.4.8 (main)
11/1/2022 - skeezicsb - v0.4.6 (experiments)
3/14/2019 - skeezicsb - v0.4.5beta (unreleased)
6/24/2018 - skeezicsb - v0.4 - v0.4.4
6/24/2010 - jdersch - v0.1 - v0.3
