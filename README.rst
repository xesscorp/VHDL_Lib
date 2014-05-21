========================================
XESS VHDL Library Files
========================================

These are VHDL files for modules that are useful in a variety of larger designs for XESS 
FPGA boards (and possibly others). 


Library Organization
========================================

The main part of this library is in the top-level directory, and it is pretty simple: it's just 
a bunch of VHDL files containing modules that perform certain functions, hopefully functions 
you actually want. Some of the modules reference others. For example, many of the modules 
reference the ``CommonPckg`` package in the ``common.vhd`` file. 

In the ``Board_Packages`` subdirectory below the top-level are some very important VHDL 
files which are not actually part of the library. Each of these files contains a package of 
constants and definitions for a particular XESS FPGA board. By adding one of these files to 
your project, you customize this library for that FPGA board.
 
The ``Board_Packages`` subdirectory also has constraint files (with a ``.ucf`` suffix) 
containing the FPGA pin assignments for each XESS FPGA board.         


How to Use This Library
========================================

#. Place these files in a directory somewhere.

#. Start a Xilinx ISE project.

#. Select the **Project => New VHDL Library...** menu item.
   In the **New VHDL Library** window, enter ``XESS`` as the name of the library.
   Then enter the directory where these VHDL files are stored as the library location.
   Finally, click **OK**.
   
#. Select the **Project => Add Source...** menu item.
   In the **Add Source** window, go to the ``Board_Packages`` subdirectory in the library 
   directory and select the file associated with the XESS board you are designing for.
   Then click on the **Open** button. (**Add Source...** is used here so you'll always
   access the most current board configuration in case there are corrections or additions.)
   
#. Select the **Project => Add Copy of Source...** menu item.
   In the **Add Copy of Source** window, go to the ``Board_Packages`` subdirectory in the 
   library directory and select the UCF file associated with the XESS board you are designing for.
   Then click on the **Open** button. (**Add Copy of Source...** is used here so you'll
   have a local copy of pin assignments that you can edit to give them names appropriate
   for your particular project without affecting the pin assignments in other projects.)
   
#. Create the rest of the VHDL files for your design. To access the elements of this library,
   place code like this into your files::
   
        library XESS;
        use XESS.CommonPckg.all;
        use XESS.PwmPckg.all;
        ...
    
   (You can find the available packages by looking inside each of the VHDL files.)
   
   Do this if you want to access any of the constants found in the board package file::
   
        use work.XessBoardPckg.all;
        
   (Since you used **Project => Add Source...** to add the board definition file to your project,
   then it is automatically included in the default ``work`` library.)
   
   
How to Fix a Project with Missing Files
========================================

After you've downloaded an XESS design project, you may get errors with missing files when you
try to open or compile it. That's probably because this library isn't in the place where the
project expects to find it. You could move this library to the place the project wants it, but
it's probably better to just modify the project as follows:

#. In the **Libraries** tab of the ISE Navigator window, right-click on the **XESS** library
   entry and select **Remove** from the pop-up menu.
   
#. In the **Files** tab of the ISE Navigator window, right-click on any files that are
   in the ``VHDL_Lib\Board_Packages`` subdirectory and select **Remove** from the pop-up menu.
   
Once these items are removed, you can add them back using the procedure described in the previous
section. Then the missing file errors should be gone.


Library Contents
========================================

    Audio:
        An interface to an AK4565 audio codec for sampling and generating
        20-bit stereo signals.
        
    ButtonDebounver.vhd:
        A debouncing circuit for a pushbutton.
        
    ButtonScanner.vhd:
        An interface for scanning the button array on the StickIt! Buttons module.

    ClkGen.vhd:
        A clock generator module that uses a DCM to multiply or divide an input clock to create an
        output clock with a higher or lower frequency. Also includes a simpler module that divides
        a master clock to generate lower frequencies.

    Common.vhd:
        Contains useful constants and functions.
        
    Delay.vhd:
        Modules to delay signals or buses by one or more clock cycles.
        
    Fifo.vhd:
        Simple FIFO modules in common-clock and independent-clock versions.

    FlashCntl.vhd:
        A module that provides read/write access to the serial flash device on the XuLA board.
        
    Hcsr04.vhd:
        An interface to an HCSR04 ultrasonic distance sensor.

    HostIo.vhd:
        A set of modules that let a host PC pass data back-and-forth with
        a user design running in the FPGA.
        
    HostIoComm.vhd:
        An interface that lets a PC pass data back-and-forth with a user design
        running in the FPGA via a set of download and upload FIFO buffers.
        
    HostIoToI2c.vhd:
        An interface that lets the host PC pass data back-and-forth with
        a chip that has an I2C interface.
        
    HostIoToSpi.vhd:
        An interface that lets the host PC pass data back-and-forth with
        a chip that has an SPI interface.
        
    I2c.vhd:
        A master-to-slave I2C interface.
        
    LedDigits.vhd:
        An interface to the Charlieplexed LED array on the StickIt! LED Digits module.
        
    Misc.vhd:
        Miscellaneous modules that don't fit under any other category.

    MemTest.vhd:
        A module that writes a random stream of values to memory.
        
    PatternGen.vhd:
        A module to output a sequence of words stored in a ROM.

    Pwm.vhd:
        A simple pulse-width modulator and a single-shot pulse generator.
        
    RandGen.vhd:
        An LFSR-based module for generating random values.
        
    RotaryEncoder.vhd:
        An interface to detect CW/CCW rotation of a rotary encoder.
        
    SDCard.vhdl:
        An interface module that simplifies reading/writing to a Secure Digital Flash card.

    SdramCntl.vhd:
        An interface module that makes an SDRAM appear as a simple SRAM-like memory to
        a user design in the FPGA.
        
    Spi.vhd:
        A master-to-slave SPI interface.

    SyncToClk.vhd:
        Modules that sync one or more signals crossing from one clock domain to another.

    TestBoardCore.vhd:
        A module that tests the functioning of a XuLA board by writing a random stream of values into SDRAM
        and then reading it back and comparing it to the original.

    Vga.vhd:
        Modules for generating bitmapped and character mapped displays on VGA monitors.
