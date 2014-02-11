--**********************************************************************
-- Copyright (c) 1997-2014 by XESS Corp <http://www.xess.com>.
-- All rights reserved.
--
-- This library is free software; you can redistribute it and/or
-- modify it under the terms of the GNU Lesser General Public
-- License as published by the Free Software Foundation; either
-- version 3.0 of the License, or (at your option) any later version.
-- 
-- This library is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
-- Lesser General Public License for more details.
-- 
-- You should have received a copy of the GNU Lesser General Public
-- License along with this library.  If not, see 
-- <http://www.gnu.org/licenses/>.
--**********************************************************************

----------------------------------------------------------------------------------
-- Flash controller.
--------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;

package FlashCntlPckg is

  component FlashCntl
    generic(
      DATA_WIDTH_G : natural := 8;      -- data width of Flash chip
      ADDR_WIDTH_G : natural := 24;     -- address width of Flash chip
      BLOCK_SIZE_G : natural := 256  -- size of RAM block that buffers data programmed into Flash
      );
    port(
      reset         : in  std_logic;    -- reset input
      clk           : in  std_logic;    -- main clock input
      h_rd          : in  std_logic;    -- read enable
      h_rd_continue : in  std_logic;    -- enable contiguous reads
      h_wr          : in  std_logic;    -- port A write enable
      h_erase       : in  std_logic;    -- flash chip erase enable
      h_blk_pgm     : in  std_logic;    -- block program enable
      h_addr        : in  std_logic_vector(ADDR_WIDTH_G-1 downto 0);  -- address for read/write
      h_di          : in  std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- data from JTAG instr. unit
      h_do          : out std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- data output from flash
      h_begun       : out std_logic;    -- true when flash operation begun
      h_busy        : out std_logic;    -- true when operation in progress
      h_done        : out std_logic;    -- true when flash operation done
      f_cs_n        : out std_logic;    -- flash chip-enable
      sclk          : out std_logic;    -- serial data clock
      si            : in  std_logic;    -- serial data input
      so            : out std_logic     -- serial data output
      );
  end component;
end package;



library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_ARITH.all;
use IEEE.STD_LOGIC_UNSIGNED.all;
use XESS.CommonPckg.all;
use XESS.FlashCntlPckg.all;

library UNISIM;
use UNISIM.VComponents.all;


entity FlashCntl is
  generic(
    DATA_WIDTH_G : natural := 8;        -- data width of Flash chip
    ADDR_WIDTH_G : natural := 24;       -- address width of Flash chip
    BLOCK_SIZE_G : natural := 256  -- size of RAM block that buffers data programmed into Flash
    );
  port(
    reset         : in  std_logic;      -- reset input
    clk           : in  std_logic;      -- main clock input
    h_rd          : in  std_logic;      -- read enable
    h_rd_continue : in  std_logic;      -- enable contiguous reads
    h_wr          : in  std_logic;      -- port A write enable
    h_erase       : in  std_logic;      -- flash chip erase enable
    h_blk_pgm     : in  std_logic;      -- block program enable
    h_addr        : in  std_logic_vector(ADDR_WIDTH_G-1 downto 0);  -- address for read/write
    h_di          : in  std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- data from JTAG instr. unit
    h_do          : out std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- data output from flash
    h_begun       : out std_logic;      -- true when flash operation begun
    h_busy        : out std_logic;      -- true when operation in progress
    h_done        : out std_logic;      -- true when flash operation done
    f_cs_n        : out std_logic;      -- flash chip-enable
    sclk          : out std_logic;      -- serial data clock
    si            : in  std_logic;      -- serial data input
    so            : out std_logic       -- serial data output
    );
end entity;


architecture arch of FlashCntl is

  constant ERASED : std_logic_vector(DATA_WIDTH_G-1 downto 0) := "11111111";  -- value of erased flash byte

  subtype cmd is std_logic_vector(DATA_WIDTH_G-1 downto 0);
  constant WRITE_ENABLE_CMD   : cmd := "00000110";
  constant CHIP_ERASE_CMD     : cmd := "11000111";
  constant PAGE_PROGRAM_CMD   : cmd := "00000010";
  constant READ_DATA_CMD      : cmd := "00000011";
  constant FAST_READ_CMD      : cmd := "00001011";
  constant READ_STATUS_CMD    : cmd := "00000101";
  constant READ_DEVICE_ID_CMD : cmd := "10010000";

  constant BUSY_BIT : natural := 0;  -- position of BUSY bit in flash status register

  -- States of the flash controller FSM.
  type cntlState is (
    RAM_BLK_INIT,
    FLASH_NOP,
    FLASH_ERASE, FLASH_ERASE_2, FLASH_ERASE_3, FLASH_ERASE_4, FLASH_ERASE_5,
    FLASH_BLK_PGM, FLASH_BLK_PGM_2, FLASH_BLK_PGM_3, FLASH_BLK_PGM_4, FLASH_BLK_PGM_5,
    FLASH_BLK_PGM_6, FLASH_BLK_PGM_7, FLASH_BLK_PGM_8, FLASH_BLK_PGM_9, FLASH_BLK_PGM_10,
    FLASH_READ, FLASH_READ_2, FLASH_READ_3, FLASH_READ_4, FLASH_READ_5, FLASH_READ_6, FLASH_READ_7, FLASH_READ_DUMMY,
    FLASH_IO_WAIT, FLASH_IO_WAIT_2,
    FLASH_IO, FLASH_IO_2, FLASH_IO_3, FLASH_IO_4, FLASH_IO_5
    );
  signal flash_state     : cntlState;   -- FSM state register
  signal rtn_flash_state : cntlState;   -- return-to state for FSM subroutines

  -- Signals to/from dualport block RAM to FSM.
  constant BLOCK_ADDR_WIDTH_G : natural := Log2(BLOCK_SIZE_G);
  signal addra, addrb         : std_logic_vector(10 downto 0);
  signal blk_addr_b           : std_logic_vector(BLOCK_ADDR_WIDTH_G-1 downto 0);  -- port B address
  signal blk_di_b             : std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- port B data input
  signal blk_do_b             : std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- port B data output
  signal blk_en_b             : std_logic;  -- port B enable
  signal blk_we_b             : std_logic;  -- port B write enable

  signal f_addr : std_logic_vector(h_addr'range);  -- address of location in serial flash

  -- Signals to/from serial flash I/O controller.
  signal cntr : natural range DATA_WIDTH_G-1 downto 0;  -- I/O shift register bit counter.
  signal f_di : std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- parallel data input to flash.
  signal f_do : std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- parallel data output from flash.

  constant CS_HIGH_WAIT : natural := 30;
  signal wait_cntr      : natural range CS_HIGH_WAIT-1 downto 0;

  signal read_cntr : natural range BLOCK_SIZE_G-1 downto 0 := 0;

begin

  -- The dual-port block RAM buffers data that is to be written to the flash.
  -- Port A is written with data, while port B is used to read the data and program it into the flash.
  blk_ram : RAMB16_S9_S9
    port map (
      SSRA  => reset,
      CLKA  => clk,
      ADDRA => addra,  -- lower portion of host address is used to address block RAM
      DIA   => h_di,  -- data word received from the PC through the JTAG interface
      DIPA  => "0",                     -- parity input (not used)  
      DOA   => open,                    -- output databus (not used)   
      ENA   => h_wr,  -- enable port A when host writes to the flash
      WEA   => h_wr,  -- write-enable for port A when host writes to the flash
      SSRB  => reset,
      CLKB  => clk,
      ADDRB => addrb,
      DIB   => ERASED,  -- input databus is set to erased Flash value for block RAM initialization     
      DIPB  => "0",                     -- parity input (not used)   
      DOB   => blk_do_b,                -- output databus
      ENB   => blk_en_b,                -- enable port B
      WEB   => blk_we_b                 -- write-enable port B
      );

  process(h_addr, blk_addr_b)
  begin
    addra                   <= (others => '0');
    addra(blk_addr_b'range) <= h_addr(blk_addr_b'range);
    addrb                   <= (others => '0');
    addrb(blk_addr_b'range) <= blk_addr_b;
  end process;

  -- This FSM performs the sequences of write/reads to the Flash chip that will
  -- erase/program/read the Flash contents.
  process(reset, clk)
  begin
    
    if reset = YES then
      -- don't do anything when reset
      blk_addr_b      <= (others => '0');  -- starting block RAM address for initialization that follows
      blk_en_b        <= NO;
      blk_we_b        <= NO;
      f_cs_n          <= '1';
      sclk            <= '0';
      f_di            <= (others => '0');
      h_begun         <= NO;
      h_busy          <= NO;
      h_done          <= NO;
      flash_state     <= RAM_BLK_INIT;  -- starting state after reset
      rtn_flash_state <= FLASH_NOP;  -- go to this state after RAM initialization
      
    elsif rising_edge(clk) then
      case flash_state is

        -- Initialize the block RAM with the value stored in an erased Flash location.
        -- This means the block RAM can be partially filled and then programmed into Flash
        -- and the unused Flash locations will be left in their erased state.
        when RAM_BLK_INIT =>
          blk_en_b   <= YES;
          blk_we_b   <= YES;
          blk_addr_b <= blk_addr_b + 1;  -- progress through the RAM block
          -- End initialization when address is BLOCK_SIZE_G-1.
          -- One more write to location BLOCK_SIZE_G will occur after this.
          -- Since the address will rollover, the actual write occurs to location 0
          -- so the entire block RAM gets initialized to the erase value.
          if blk_addr_b = BLOCK_SIZE_G-1 then
            flash_state <= rtn_flash_state;
          end if;

        -- Wait for a Flash erase, program or read operation to arrive.  
        when FLASH_NOP =>
          blk_addr_b <= (others => '0');  -- set address to zero in case Flash programming is initiated
          blk_en_b   <= NO;             -- disable block RAM until it is needed
          blk_we_b   <= NO;  -- disable writes to block RAM after initialization is done
          f_cs_n     <= '1';
          h_done     <= NO;  -- remove any done signal that was asserted when the previous operation finished
          h_begun    <= YES;            -- assume some operation has begun
          h_busy     <= YES;        -- assume we are busy doing some operation
          if h_erase = YES then
            flash_state <= FLASH_ERASE;  -- go to states where Flash erasure occurs
          elsif h_blk_pgm = YES then
            f_addr      <= h_addr;  -- send the address to write to in the flash
            flash_state <= FLASH_BLK_PGM;  -- go to states where flash programming occurs
          elsif h_rd = YES then
            f_addr      <= h_addr;  -- send the address to read from to the external Flash interface
            flash_state <= FLASH_READ;  -- go to the states where Flash reading occurs
          else  -- cancel the begun and busy indicators since no operation was initiated
            h_begun <= NO;              -- no operation has been initiated
            h_busy  <= NO;              -- no operation is in-progress
          end if;

        -- Erase the entire Flash chip:
        --  1) Enable writing of the flash.
        --  2) Issue the erase command.
        --  3) Read status register until the erase operation is done.
        when FLASH_ERASE =>
          h_begun         <= NO;   -- remove the begun signal after one cycle
          f_di            <= WRITE_ENABLE_CMD;  -- enable writing to the flash
          flash_state     <= FLASH_IO_WAIT;    -- send the command to the flash
          rtn_flash_state <= FLASH_ERASE_2;
        when FLASH_ERASE_2 =>
          f_cs_n          <= '1';  -- raise flash CS after write enable command so it will take effect
          f_di            <= CHIP_ERASE_CMD;   -- issue the chip erase command 
          flash_state     <= FLASH_IO_WAIT;
          rtn_flash_state <= FLASH_ERASE_3;
        when FLASH_ERASE_3 =>
          f_cs_n          <= '1';  -- raise flash CS after chip erase command so it will take effect
          f_di            <= READ_STATUS_CMD;  -- issue the command to read the flash status
          flash_state     <= FLASH_IO_WAIT;
          rtn_flash_state <= FLASH_ERASE_4;
        when FLASH_ERASE_4 =>
          flash_state     <= FLASH_IO;  -- after this, the flash status is in the f_do register
          rtn_flash_state <= FLASH_ERASE_5;
        when FLASH_ERASE_5 =>
          if f_do(BUSY_BIT) = '1' then
            flash_state <= FLASH_ERASE_4;  -- keep reading flash status until the flash is not busy
          else
            -- flash has completed the erase operation
            f_cs_n      <= '1';  -- terminate the reading of the flash status
            h_done      <= YES;  -- signal the host that this operation is done
            flash_state <= FLASH_NOP;   -- return to wait for another operation
          end if;

        -- Program the contents of the block RAM into the flash.
        --  1) Enable writing of the flash.
        --  2) Issue the programming command.
        --  3) Send the starting address.
        --  4) Send 256 bytes of data stored in the block RAM.
        --  5) Read status register until the programming operation is done.
        when FLASH_BLK_PGM =>
          h_begun         <= NO;   -- remove the begun signal after one cycle
          f_di            <= WRITE_ENABLE_CMD;  -- enable writing to the flash
          flash_state     <= FLASH_IO_WAIT;  -- send the command to the flash
          rtn_flash_state <= FLASH_BLK_PGM_2;
        when FLASH_BLK_PGM_2 =>
          f_cs_n          <= '1';  -- raise flash CS after write enable command so it will take effect
          f_di            <= PAGE_PROGRAM_CMD;  -- issue the programming command
          flash_state     <= FLASH_IO_WAIT;
          rtn_flash_state <= FLASH_BLK_PGM_3;
        when FLASH_BLK_PGM_3 =>
          f_di            <= f_addr(23 downto 16);  -- send the MSByte of the flash address
          flash_state     <= FLASH_IO;
          rtn_flash_state <= FLASH_BLK_PGM_4;
        when FLASH_BLK_PGM_4 =>
          f_di            <= f_addr(15 downto 8);  -- send the middle byte of the flash address
          flash_state     <= FLASH_IO;
          rtn_flash_state <= FLASH_BLK_PGM_5;
        when FLASH_BLK_PGM_5 =>
          -- the entire block of RAM is written to the flash, so we can always start with the LSB set to 0 and increment to 255.
          f_di            <= (others => '0');
          blk_addr_b      <= (others => '0');  -- start reading block RAM at same position as in flash page
          blk_en_b        <= YES;  -- enable the block RAM in order to read its contents
          flash_state     <= FLASH_IO;
          rtn_flash_state <= FLASH_BLK_PGM_6;
        when FLASH_BLK_PGM_6 =>
          f_di        <= blk_do_b;  -- send the data from the block RAM to the flash
          flash_state <= FLASH_IO;
          blk_addr_b  <= blk_addr_b + 1;  -- increment the address for the next write to flash
          if blk_addr_b = BLOCK_SIZE_G-1 then
            rtn_flash_state <= FLASH_BLK_PGM_7;  -- break out of loop once all the data is written to flash
          else
            rtn_flash_state <= FLASH_BLK_PGM_6;  -- loop until all data is written from the block RAM to flash
          end if;
        when FLASH_BLK_PGM_7 =>
          f_cs_n          <= '1';  -- raise flash CS so the data will be programmed into the flash
          f_di            <= READ_STATUS_CMD;  -- issue the command to read the flash status
          flash_state     <= FLASH_IO_WAIT;
          rtn_flash_state <= FLASH_BLK_PGM_8;
        when FLASH_BLK_PGM_8 =>
          flash_state     <= FLASH_IO;  -- after this, the flash status is in the f_do register
          rtn_flash_state <= FLASH_BLK_PGM_9;
        when FLASH_BLK_PGM_9 =>
          if f_do(BUSY_BIT) = '1' then
            flash_state <= FLASH_BLK_PGM_8;  -- keep reading flash status until the flash is not busy
          else
            -- flash has completed the page programming operation
            f_cs_n          <= '1';     -- stop reading flash status
            flash_state     <= RAM_BLK_INIT;   -- re-initialize the block RAM
            rtn_flash_state <= FLASH_BLK_PGM_10;
          end if;
        when FLASH_BLK_PGM_10 =>
          h_done      <= YES;  -- signal the host that this operation is done
          flash_state <= FLASH_NOP;     -- return to wait for another operation

        -- Read values from the flash address registered during the NOP state:
        --  1) Issue the flash read data command.
        --  2) Send the starting address.
        --  3) Read bytes from the serial flash.
        when FLASH_READ =>
          h_begun         <= NO;  -- remove the begun signal after one cycle
          f_di            <= FAST_READ_CMD;  -- issue the fast read data command
          flash_state     <= FLASH_IO_WAIT;
          rtn_flash_state <= FLASH_READ_2;
        when FLASH_READ_2 =>
          f_di            <= f_addr(23 downto 16);  -- send the MSByte of the flash address
          flash_state     <= FLASH_IO;
          rtn_flash_state <= FLASH_READ_3;
        when FLASH_READ_3 =>
          f_di            <= f_addr(15 downto 8);  -- send the middle byte of the flash address
          flash_state     <= FLASH_IO;
          rtn_flash_state <= FLASH_READ_4;
        when FLASH_READ_4 =>
          f_di            <= f_addr(7 downto 0);  -- send the LSByte of the flash address
          flash_state     <= FLASH_IO;
          rtn_flash_state <= FLASH_READ_DUMMY;
        when FLASH_READ_DUMMY =>
          flash_state     <= FLASH_IO;  -- send dummy byte required for fast read command
          rtn_flash_state <= FLASH_READ_5;
        when FLASH_READ_5 =>
          h_begun         <= NO;  -- remove begun signal raised when coming from FLASH_READ_7 state
          flash_state     <= FLASH_IO;  -- after this completes, the data read from flash appears is in the f_do register
          rtn_flash_state <= FLASH_READ_6;
        when FLASH_READ_6 =>
          h_do   <= f_do;         -- register the data read from the Flash
          h_done <= YES;  -- signal the host that this operation is done
          -- now wait until the read signal has been released, so we know the data has been read
          if h_rd = NO then
            flash_state <= FLASH_READ_7;
          end if;
        when FLASH_READ_7 =>
          h_done <= NO;                 -- the operation is done
          if h_rd_continue = NO then
            f_cs_n      <= '1';         -- terminate the read data command
            flash_state <= FLASH_NOP;   -- return to wait for another operation
          elsif h_rd = YES then
            h_begun     <= YES;         -- begin a new read operation
            flash_state <= FLASH_READ_5;
          else
            null;
          end if;

        -- This is an FSM subroutine that writes a value to and reads a value from the serial flash.
        when FLASH_IO_WAIT =>
          wait_cntr   <= CS_HIGH_WAIT - 1;
          flash_state <= FLASH_IO_WAIT_2;
        when FLASH_IO_WAIT_2 =>
          wait_cntr <= wait_cntr - 1;
          if wait_cntr /= 0 then
            flash_state <= FLASH_IO_WAIT_2;
          else
            f_cs_n      <= '0';  -- enable the flash so it will accept and deliver serial data
            flash_state <= FLASH_IO;
          end if;
        when FLASH_IO =>
          f_cs_n      <= '0';  -- enable the flash so it will accept and deliver serial data
          cntr        <= DATA_WIDTH_G-1;
          flash_state <= FLASH_IO_2;
        when FLASH_IO_2 =>
          sclk        <= '1';
          flash_state <= FLASH_IO_3;
        when FLASH_IO_3 =>
          sclk <= '0';
          f_do <= f_do(DATA_WIDTH_G-2 downto 0) & si;
          f_di <= f_di(DATA_WIDTH_G-2 downto 0) & '0';
          cntr <= cntr - 1;
          if cntr /= 0 then
            flash_state <= FLASH_IO_2;
          else
            flash_state <= rtn_flash_state;
--            read_cntr       <= read_cntr + 1;
--            f_do <= CONV_STD_LOGIC_VECTOR(read_cntr,f_do'length);
          end if;

        when others =>
          null;
      end case;
    end if;
  end process;

  so <= f_di(DATA_WIDTH_G-1);

end architecture;
