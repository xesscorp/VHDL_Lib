--**********************************************************************
-- Copyright (c) 2014 by XESS Corp <http://www.xess.com>.
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

--**********************************************************************
-- This module provides a communication interface between a host PC and 
-- a module in the FPGA that operates as follows:
--
-- 1. A Python program on the host PC issues commands for reading or
--    writing a communication FIFO inside the XuLA FPGA board.
-- 2. The commands are turned into JTAG bit strings that can be sent
--    to the FPGA through its JTAG port.
-- 3. The JTAG bit strings are packaged as USB packets and are sent to
--    the interface microcontroller on the XuLA board.
-- 4. The microcontroller unpacks the JTAG bitstrings and injects them
--    into the FPGA through its JTAG port.
-- 5. The HostIoToRam module in the FPGA decodes the bit strings and
--    interprets them as reads or writes to the registers of the
--    FIFOs.
-- 6. The Down FIFO accepts data from the PC and delivers it when a
--    module in the FPGA reads it.
-- 7. The Up FIFO accepts data from the FPGA module and serializes
--    it into a JTAG bit string that is returned to the microcontroller.
-- 8. The microcontroller returns the JTAG bit strings to the host
--    PC as USB packets.
-- 9. The JTAG bit strings are unpacked from the USB packets and
--    turned into data or status bytes.
--
--                   +------------------XuLA Board----------------------------------------+
-- +---------+       | +---uC---+          +---------FPGA-------------------------------+ |
-- |         |       | |        |          |                                            | |
-- |         |       | |        |          |              /---> Down FIFO --->\         | |
-- | Host PC |<--USB-->| PIC18F |<--JTAG-->| HostIoToRam <                     > Module | |
-- |         |       | |        |          |              \<---  Up FIFO  <---/         | |
-- |         |       | |        |          |                                            | |
-- +---------+       | +--------+          +--------------------------------------------+ |
--                   +--------------------------------------------------------------------+
--
--**********************************************************************


  library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.math_real.all;
use XESS.CommonPckg.all;
use XESS.HostIoPckg.all;
use work.XessBoardPckg.all;

package HostIoCommPckg is

  component HostIoComm is
    generic (
      ID_G               : std_logic_vector := "11111111";  -- The ID this module responds to.
      PYLD_CNTR_LENGTH_G : natural          := 32;  -- Length of payload bit counter.
      FPGA_DEVICE_G      : FpgaFamily_t     := FPGA_FAMILY_C;  -- FPGA device type.
      TAP_USER_INSTR_G   : TapUserInstr_t   := TAP_USER_INSTR_C;  -- USER instruction this module responds to.
      SIMPLE_G           : boolean          := false;  -- If true, include BscanToHostIo module in this module.
      FIFO_LENGTH_G      : natural          := 16;  -- Number of data words in the Up and Down FIFOs.
      WORD_WIDTH_G       : natural          := 8  -- Number of bits in each FIFO word.
      );
    port (
      reset_i     : in  std_logic := LO;  -- Active-high reset signal.
      clk_i       : in  std_logic;      -- Master clock.
      -- Interface to BscanHostIo. (Used only if SIMPLE_G is false.)
      inShiftDr_i : in  std_logic := LO;  -- True when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
      drck_i      : in  std_logic := LO;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
      tdi_i       : in  std_logic := LO;  -- Bit from the host to the memory.
      tdo_o       : out std_logic;      -- Bit from the memory to the host.
      -- FPGA-side interface.
      add_i       : in  std_logic;  -- Add data to the Up FIFO that sends data to the host.
      data_i      : in  std_logic_vector(WORD_WIDTH_G-1 downto 0);  -- Data to send to the host.
      rmv_i       : in  std_logic;  -- Remove data from the Down FIFO that receives data from the host.
      data_o      : out std_logic_vector(WORD_WIDTH_G-1 downto 0);  -- Data downloaded from the host.
      upEmpty_o   : out std_logic;  -- True if the Up FIFO to the host is empty.
      upFull_o    : out std_logic;  -- True if the Up FIFO to the host is full.
      upLevel_o   : out std_logic_vector(natural(ceil(log2(real(FIFO_LENGTH_G+1))))-1 downto 0);  -- # of data words waiting in the Up FIFO to send to the host.
      dnEmpty_o   : out std_logic;  -- True if the Down FIFO from the host is empty.
      dnFull_o    : out std_logic;  -- True if the Down FIFO from the host is full.
      dnLevel_o   : out std_logic_vector(natural(ceil(log2(real(FIFO_LENGTH_G+1))))-1 downto 0)  -- # of data words available in the Down FIFO from the host.
      );
  end component;

  component WbHostUart is
    generic (
      ID_G               : std_logic_vector := "11111111";  -- The ID this module responds to.
      VENDOR_ID_G        : std_logic_vector := x"08";  -- ZPUino.
      PRODUCT_ID_G       : std_logic_vector := x"11";  -- UART.
      PYLD_CNTR_LENGTH_G : natural          := 32;  -- Length of payload bit counter.
      SIMPLE_G           : boolean          := false;  -- If true, include BscanToHostIo module in this module.
      WORD_WIDTH_G       : natural          := 8;  -- Width of data word sent to/from the host.
      FIFO_LENGTH_G      : natural          := 16  -- Number of data words in the Up and Down FIFOs.
      );
    port (
      -- Wishbone interface.
      wb_clk_i    : in  std_logic;
      wb_rst_i    : in  std_logic;
      wb_dat_o    : out std_logic_vector;
      wb_dat_i    : in  std_logic_vector;
      wb_adr_i    : in  std_logic_vector;
      wb_we_i     : in  std_logic;
      wb_cyc_i    : in  std_logic;
      wb_stb_i    : in  std_logic;
      wb_ack_o    : out std_logic;
      wb_inta_o   : out std_logic;
      id          : out std_logic_vector;
      -- Interface to BscanHostIo. (Used only if SIMPLE_G is false.)
      inShiftDr_i : in  std_logic := LO;
      drck_i      : in  std_logic := LO;
      tdi_i       : in  std_logic := LO;
      tdo_o       : out std_logic
      );
  end component;

end package;




library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use XESS.CommonPckg.all;
use XESS.HostIoPckg.all;
use XESS.FifoPckg.all;
use xess.DelayPckg.all;
use work.XessBoardPckg.all;

entity HostIoComm is
  generic (
    ID_G               : std_logic_vector := "11111111";  -- The ID this module responds to.
    PYLD_CNTR_LENGTH_G : natural          := 32;  -- Length of payload bit counter.
    FPGA_DEVICE_G      : FpgaFamily_t     := FPGA_FAMILY_C;  -- FPGA device type.
    TAP_USER_INSTR_G   : TapUserInstr_t   := TAP_USER_INSTR_C;  -- USER instruction this module responds to.
    SIMPLE_G           : boolean          := false;  -- If true, include BscanToHostIo module in this module.
    FIFO_LENGTH_G      : natural          := 16;  -- Number of data words in the Up and Down FIFOs.
    WORD_WIDTH_G       : natural          := 8  -- Number of bits in each FIFO word.
    );
  port (
    reset_i     : in  std_logic := LO;  -- Active-high reset signal.
    clk_i       : in  std_logic;        -- Master clock.
    -- Interface to BscanHostIo. (Used only if SIMPLE_G is false.)
    inShiftDr_i : in  std_logic := LO;  -- True when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
    drck_i      : in  std_logic := LO;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
    tdi_i       : in  std_logic := LO;  -- Bit from the host to the memory.
    tdo_o       : out std_logic;        -- Bit from the memory to the host.
    -- FPGA-side interface.
    add_i       : in  std_logic;  -- Add data to the Up FIFO that sends data to the host.
    data_i      : in  std_logic_vector(WORD_WIDTH_G-1 downto 0);  -- Data to send to the host.
    rmv_i       : in  std_logic;  -- Remove data from the Down FIFO that receives data from the host.
    data_o      : out std_logic_vector(WORD_WIDTH_G-1 downto 0);  -- Data downloaded from the host.
    upEmpty_o   : out std_logic;  -- True if the Up FIFO to the host is empty.
    upFull_o    : out std_logic;  -- True if the Up FIFO to the host is full.
    upLevel_o   : out std_logic_vector(natural(ceil(log2(real(FIFO_LENGTH_G+1))))-1 downto 0);  -- # of data words waiting in the Up FIFO to send to the host.
    dnEmpty_o   : out std_logic;  -- True if the Down FIFO from the host is empty.
    dnFull_o    : out std_logic;  -- True if the Down FIFO from the host is full.
    dnLevel_o   : out std_logic_vector(natural(ceil(log2(real(FIFO_LENGTH_G+1))))-1 downto 0)  -- # of data words available in the Down FIFO from the host.
    );
end entity;


architecture arch of HostIoComm is
  signal hostReset_s    : std_logic;    -- Reset signal issued from host.
  signal reset_s        : std_logic;  -- OR combination of host-side and FPGA-side resets.
  type RegAddr_t is (FIFO_REG, STATUS_CONTROL_REG, DN_FREE_REG, UP_USED_REG);
  signal regAddr_s      : std_logic_vector(2 downto 0);  -- Register address from host.
  signal wr_s           : std_logic;    -- Write signal from host.
  signal rd_s           : std_logic;    -- Read signal from host.
  signal done_s         : std_logic;    -- Host read/write done signal.
  signal upRmv_s        : std_logic;  -- Remove data from the front of the Up FIFO.
  signal upRmvDelayed_s : std_logic;  -- Remove data from the front of the Up FIFO after read is done.
  signal upEmpty_s      : std_logic;    -- True if Up FIFO is empty.
  signal upFull_s       : std_logic;    -- True if Up FIFO is full.
  signal upLevel_s      : std_logic_vector(upLevel_o'range);  -- # of data words in Up FIFO.
  signal dnAdd_s        : std_logic;    -- Add data to back of Down FIFO.
  signal dnEmpty_s      : std_logic;    -- True if Down FIFO is empty.
  signal dnFull_s       : std_logic;    -- True if Down FIFO is full.
  signal dnLevel_s      : std_logic_vector(dnLevel_o'range);  -- # of data words in Down FIFO.
  signal busFromHost_s  : std_logic_vector(WORD_WIDTH_G-1 downto 0);  -- Data from host to FPGA.
  signal busToHost_s    : std_logic_vector(WORD_WIDTH_G-1 downto 0);  -- Data from FPGA to host.
  signal upFront_s      : std_logic_vector(WORD_WIDTH_G-1 downto 0);  -- Data from Up FIFO to host.
  signal dnEmptySel_s   : std_logic;  -- True when the amount of free space in the Up FIFO is read.
  signal upFilledSel_s  : std_logic;  -- True when the amount of used space in the Down FIFO is read.
  signal dnEmptySR_r    : std_logic_vector(31 downto 0);  -- Holds amount of free space in the Down FIFO.
  signal upFilledSR_r   : std_logic_vector(31 downto 0);  -- Holds amount of used space in the Up FIFO.
begin

  -- Reset this module from the FPGA-side or the host-side.
  reset_s <= reset_i or hostReset_s;

  -- Instantiate an interface between the JTAG port and the Up/Down FIFOs.
  u1 : HostIoToRam
    generic map(
      ID_G               => ID_G,
      PYLD_CNTR_LENGTH_G => PYLD_CNTR_LENGTH_G,
      FPGA_DEVICE_G      => FPGA_DEVICE_G,
      TAP_USER_INSTR_G   => TAP_USER_INSTR_G,
      SIMPLE_G           => SIMPLE_G,
      SYNC_G             => true,
      ADDR_INC_G         => 0  -- R/W sequences do not automatically advance the register address.
     -- (This is done to support multi-read/writes of FIFO data.)
      )
    port map(
      reset_i        => reset_s,        -- Active-high reset input.
      clk_i          => clk_i,          -- Master clock input.
      -- JTAG interface.
      inShiftDr_i    => inShiftDr_i,
      drck_i         => drck_i,
      tdi_i          => tdi_i,
      tdo_o          => tdo_o,
      -- Host interface to FIFOs.      
      addr_o         => regAddr_s,      -- Register address from host.
      rd_o           => rd_s,           -- Host read signal.
      dataToHost_i   => busToHost_s,    -- Data or status info to the host.
      wr_o           => wr_s,           -- Host write signal.
      dataFromHost_o => busFromHost_s,  -- Data or control from the host.
      opBegun_i      => LO,  -- This keeps read/write controls active for the entire bus cycle.
      done_i         => done_s  -- True when the read or write is completed.
      );

  -- FIFO R/W operations are done a single cycle after they are initiated by the host.
  uDoneDelay : delayLine
    generic map(NUM_DELAY_CYCLES_G => 1)
    port map(clk_i                 => clk_i, a_i => (rd_s or wr_s), aDelayed_o => done_s);

  -- Decode the register address from the host to initiate the correct comm channel operations.
  RegAddrDecoder : process(regAddr_s, rd_s, wr_s, upFront_s, upEmpty_s, upFull_s, upLevel_s, dnEmpty_s, dnFull_s, dnLevel_s)
  begin
    -- Default control signal values to prevent latches from being synthesized.
    upRmv_s       <= LO;  -- Don't remove the data at the front of the Up FIFO.
    dnAdd_s       <= LO;  -- Don't add data to the back of the Down FIFO.
    hostReset_s   <= LO;                -- Don't reset the FIFOs.
    busToHost_s   <= (others => '0');  -- Default the data to the host to zero.
    dnEmptySel_s  <= LO;
    upFilledSel_s <= LO;

    case RegAddr_t'val(TO_INTEGER(unsigned(regAddr_s))) is
      when FIFO_REG =>     -- Host add/remove data to/from the Down/Up FIFOs.
        dnAdd_s     <= wr_s;  -- A write by the host adds data from the host to the Down FIFO.
        upRmv_s     <= rd_s;  -- A read by the host removes data from the front of the Up FIFO.
        busToHost_s <= upFront_s;  -- Send the data at the front of the Up FIFO to the host.
      when STATUS_CONTROL_REG =>  -- Host read of FIFO statuses or reset of the FIFOs.
        busToHost_s(3 downto 0) <= upFull_s & upEmpty_s & dnFull_s & dnEmpty_s;  -- A read by the host gets the FIFO statuses.
        hostReset_s             <= wr_s;  -- A write by the host to this address resets the FIFOs.
      when DN_FREE_REG =>  -- Host reads the # of available slots for additional data in the Down FIFO.
        busToHost_s  <= dnEmptySR_r(busToHost_s'range);
        dnEmptySel_s <= rd_s;  -- Only select the Down FIFO free-amount register if it is being read.
      when UP_USED_REG =>  -- Host reads the # of filled slots waiting in the Up FIFO for xfer to the host.
        busToHost_s   <= upFilledSR_r(busToHost_s'range);
        upFilledSel_s <= rd_s;  -- Only select the Up FIFO filled-amount register if it is being read.
      when others =>
        null;
    end case;
  end process;

  -- Shift register that holds the amount of empty space in the Down FIFO and dispenses it one word at a time.
  -- This is necessary because the width of the bus going back to the host may be too small to transfer the
  -- count in a single transfer.
  DownEmptySReg : process(clk_i)
    variable shiftEnable_v : std_logic := NO;  -- Enable register shifting using the delayed dnEmptySel_s signal.
    variable cnt_v         : natural range 0 to dnEmptySR_r'length;  -- Counts the # of bits left to shift out.
  begin
    if rising_edge(clk_i) then
      if reset_s = YES then  -- Upon reset, amount of free space equals the size of the FIFO.
        cnt_v := 0;
      elsif cnt_v = 0 then  -- Reload the shift reg with current FIFO level after it is read or not selected.
        dnEmptySR_r                          <= (others => ZERO);
        dnEmptySR_r(dnLevel_s'high downto 0) <= std_logic_vector(TO_UNSIGNED(FIFO_LENGTH_G - TO_INTEGER(unsigned(dnLevel_s)), dnLevel_s'length));
        cnt_v                                := dnEmptySR_r'length;
      elsif done_s = YES then  -- Update the shift register after any read or write.
        if shiftEnable_v = YES then  -- Shift this register by WORD_WIDTH bits whenever it is selected for reading.
          dnEmptySR_r                                         <= (others => ZERO);  -- Shift in zeroes.
          dnEmptySR_r(dnEmptySR_r'high-WORD_WIDTH_G downto 0) <= dnEmptySR_r(dnEmptySR_r'high downto WORD_WIDTH_G);
          cnt_v                                               := cnt_v - WORD_WIDTH_G;  -- Reduce the bit counter by the number of bits shifted out.
        else
          cnt_v := 0;       -- Causes the shift register to be reloaded.
        end if;
      end if;
      shiftEnable_v := dnEmptySel_s;  -- This 1-cycle delay makes the shift enable coincide with the done signal.
    end if;
  end process;

  -- Shift register that holds the amount of space used in the Up FIFO and dispenses it one word at a time.
  -- This is necessary because the width of the bus going back to the host may be too small to transfer the
  -- count in a single transfer.
  UpFilledSReg : process(clk_i)
    variable shiftEnable_v : std_logic := NO;  -- Enable register shifting using the delayed upFilledSel_s signal.
    variable cnt_v         : natural range 0 to upFilledSR_r'length;  -- Counts the # of bits left to shift out.
  begin
    if rising_edge(clk_i) then
      if reset_s = YES then  -- Upon reset, the Up FIFO is emptied and the filled space is zero.
        cnt_v := 0;
      elsif cnt_v = 0 then  -- Reload the shift reg with current FIFO level after it is read or not selected.
        upFilledSR_r                          <= (others => ZERO);
        upFilledSR_r(upLevel_s'high downto 0) <= upLevel_s;
        cnt_v                                 := upFilledSR_r'length;
      elsif done_s = YES then  -- Update the shift register after any read or write.
        if shiftEnable_v = YES then  -- Shift this register by WORD_WIDTH bits whenever it is selected for reading.
          upFilledSR_r                                          <= (others => ZERO);  -- Shift in zeroes.
          upFilledSR_r(upFilledSR_r'high-WORD_WIDTH_G downto 0) <= upFilledSR_r(upFilledSR_r'high downto WORD_WIDTH_G);
          cnt_v                                                 := cnt_v - WORD_WIDTH_G;  -- Reduce the bit counter by the number of bits shifted out.
        else
          cnt_v := 0;       -- Causes the shift register to be reloaded.
        end if;
      end if;
      shiftEnable_v := upFilledSel_s;  -- This 1-cycle delay makes the shift enable coincide with the done signal.
    end if;
  end process;

  -- The Down FIFO takes data from the host and delivers it to the FPGA module.
  DownFifo : FifoCc
    generic map(
      WIDTH_G  => WORD_WIDTH_G,
      LENGTH_G => FIFO_LENGTH_G
      )
    port map(
      rst_i   => reset_s,
      clk_i   => clk_i,
      add_i   => dnAdd_s,
      data_i  => busFromHost_s,
      rmv_i   => rmv_i,
      data_o  => data_o,
      full_o  => dnFull_s,
      empty_o => dnEmpty_s,
      level_o => dnLevel_s
      );

  -- Delay removal of the data from the upload FIFO until the host has a chance to read it.
  uRmvDelay : delayLine
    generic map(NUM_DELAY_CYCLES_G => 1)
    port map(clk_i                 => clk_i, a_i => upRmv_s, aDelayed_o => upRmvDelayed_s);

  -- The Up FIFO takes data from the FPGA module and delivers it to the host.
  UpFifo : FifoCc
    generic map(
      WIDTH_G  => WORD_WIDTH_G,
      LENGTH_G => FIFO_LENGTH_G
      )
    port map(
      rst_i   => reset_s,
      clk_i   => clk_i,
      add_i   => add_i,
      data_i  => data_i,
      rmv_i   => upRmvDelayed_s,
      data_o  => upFront_s,
      full_o  => upFull_s,
      empty_o => upEmpty_s,
      level_o => upLevel_s
      );

  -- Output the Up & Down FIFO statuses to the FPGA module.
  upFull_o  <= upFull_s;
  upEmpty_o <= upEmpty_s;
  upLevel_o <= upLevel_s;
  dnFull_o  <= dnFull_s;
  dnEmpty_o <= dnEmpty_s;
  dnLevel_o <= dnLevel_s;

end architecture;




--**********************************************************************
-- HostIoComm with Wishbone interface to the FPGA.
--   Address    Function
--      0       Read data from the UART RX queue.
--              Write data to the UART TX queue.
--      1       Read status: Bit 0: True if UART RX queue has data waiting.
--                           Bit 1: True if UART TX queue is full.
--              Write control: No control bits.
--**********************************************************************

library IEEE, XESS;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use xess.CommonPckg.all;
use xess.HostIoCommPckg.all;
use work.XessboardPckg.all;

entity WbHostUart is
  generic (
    ID_G               : std_logic_vector := "11111111";  -- The ID this module responds to.
    VENDOR_ID_G        : std_logic_vector := x"08";  -- ZPUino.
    PRODUCT_ID_G       : std_logic_vector := x"11";  -- UART.
    PYLD_CNTR_LENGTH_G : natural          := 32;  -- Length of payload bit counter.
    SIMPLE_G           : boolean          := false;  -- If true, include BscanToHostIo module in this module.
    WORD_WIDTH_G       : natural          := 8;  -- Width of data word sent to/from the host.
    FIFO_LENGTH_G      : natural          := 16  -- Number of data words in the Up and Down FIFOs.
    );
  port (
    -- Wishbone interface.
    wb_clk_i    : in  std_logic;
    wb_rst_i    : in  std_logic;
    wb_dat_o    : out std_logic_vector;
    wb_dat_i    : in  std_logic_vector;
    wb_adr_i    : in  std_logic_vector;
    wb_we_i     : in  std_logic;
    wb_cyc_i    : in  std_logic;
    wb_stb_i    : in  std_logic;
    wb_ack_o    : out std_logic;
    wb_inta_o   : out std_logic;
    id          : out std_logic_vector;
    -- Interface to BscanHostIo. (Used only if SIMPLE_G is false.)
    inShiftDr_i : in  std_logic := LO;
    drck_i      : in  std_logic := LO;
    tdi_i       : in  std_logic := LO;
    tdo_o       : out std_logic
    );
end entity;

architecture arch of WbHostUart is
  signal wbActive_s : std_logic;  -- True when this device is read/written over Wishbone bus.
  signal rmv_s      : std_logic;  -- True when data is being removed from the download FIFO from the host.
  signal add_s      : std_logic;  -- True when data is being added to the upload FIFO to the host.
  signal dnData_s   : std_logic_vector(7 downto 0);  -- Data from the download FIFO.
  signal dnEmpty_s  : std_logic;  -- True when the download FIFO from the host is empty.
  signal upFull_s   : std_logic;  -- True when the upload FIFO to the host is full.
begin

  id <= VENDOR_ID_G & PRODUCT_ID_G;  -- Output the vendor and product IDs so the ZPUino can identify it.

  wbActive_s <= wb_cyc_i and wb_stb_i;  -- True when this device is read/written over Wishbone bus.

  wb_inta_o <= NO;                      -- No interrupts come from this module.
  wb_ack_o  <= wbActive_s;  -- Immediately acknowledge any read or write operation.

  process(wbActive_s, wb_adr_i, wb_we_i, dnData_s, dnEmpty_s, upFull_s)
  begin
    -- Set default values for these signals when this device is not being read/written.
    rmv_s    <= NO;
    add_s    <= NO;
    wb_dat_o <= (others => '0');

    if wbActive_s = YES then            -- This device is being accessed.
      if wb_we_i = YES then             -- Write UART operation.
        case wb_adr_i(2) is
          when '0' =>                   -- Write UART TX register.
            add_s <= YES;  -- Write data from ZPUino to FIFO uploaded to the host.
          when '1' =>                   -- Write UART control register.
            null;          -- There's no control bits for this UART.
          when others =>
            null;
        end case;
      else                              -- Read UART operation.
        case wb_adr_i(2) is
          when '0' =>                   -- Read UART RX register.
            rmv_s                <= YES;  -- Read data from FIFO downloaded from the host.
            wb_dat_o             <= (others => '0');
            wb_dat_o(7 downto 0) <= dnData_s;
          when '1' =>                   -- Read UART status register.
            wb_dat_o    <= (others => '0');
            wb_dat_o(0) <= not dnEmpty_s;  -- RX data is available if download FIFO is not empty.
            wb_dat_o(1) <= upFull_s;  -- Indicate TX is busy if upload FIFO is full.
            wb_dat_o(2) <= NO;          -- TX is never "in transmit".
          when others =>
            null;
        end case;
      end if;
    end if;
  end process;

  u0 : HostIoComm
    generic map(
      ID_G               => ID_G,
      PYLD_CNTR_LENGTH_G => PYLD_CNTR_LENGTH_G,
      SIMPLE_G           => SIMPLE_G,
      FIFO_LENGTH_G      => FIFO_LENGTH_G,
      WORD_WIDTH_G       => WORD_WIDTH_G
      )
    port map(
      reset_i     => wb_rst_i,
      clk_i       => wb_clk_i,
      -- Interface to BscanHostIo. (Used only if SIMPLE_G is false.)
      inShiftDr_i => inShiftDr_i,
      drck_i      => drck_i,
      tdi_i       => tdi_i,
      tdo_o       => tdo_o,
      -- FPGA-side interface.
      add_i       => add_s,             -- Add data to upload FIFO.
      data_i      => wb_dat_i(7 downto 0),  -- Data to upload to host via USB.
      upFull_o    => upFull_s,          -- Upload FIFO to host is full.
      rmv_i       => rmv_s,             -- Remove data from download FIFO.
      data_o      => dnData_s,          -- Data downloaded from host via USB.
      dnEmpty_o   => dnEmpty_s          -- Download FIFO from host is empty.
      );

end architecture;




--**********************************************************************
-- This is a simple design for testing communications between a host
-- and an FPGA module.
--**********************************************************************

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use XESS.CommonPckg.all;
use XESS.ClkGenPckg.all;
use XESS.HostIoCommPckg.all;
use work.XessBoardPckg.all;

entity HostIoCommTest is
  port (
    fpgaClk_i : in std_logic            -- XuLA 12 MHz clock.
    );
end entity;

architecture arch of HostIoCommTest is
  type FsmState_t is (FILL_WITH_DATA, CHANGE_DATA, WAIT_FOR_STATUS);
  signal state_r              : FsmState_t := FILL_WITH_DATA;
  signal clk_s                : std_logic;         -- Clock.
  signal reset_s              : std_logic  := LO;  -- Active-high reset.
  signal rd_s, wr_s           : std_logic;
  signal dataFromHost_s       : std_logic_vector(7 downto 0);
  signal dataToHost_s, data_r : unsigned(7 downto 0);
  signal empty_s, full_s      : std_logic;
begin

  -- Generate 100 MHz clock from 12 MHz XuLA clock.
  u0 : ClkGen generic map(CLK_MUL_G => 25, CLK_DIV_G => 3) port map(i => fpgaClk_i, o => clk_s);

  -- Generate a reset pulse to initialize the modules.
  process (clk_s)
    variable rstCnt_v : integer range 0 to 15 := 10;  -- Set length of rst pulse.
  begin
    if rising_edge(clk_s) then
      reset_s <= HI;                    -- Activate rst.
      if rstCnt_v = 0 then
        reset_s <= LO;                  -- Release rst when counter hits 0.
      else
        rstCnt_v := rstCnt_v - 1;
      end if;
    end if;
  end process;

  -- Instantiate the communication interface.
  u1 : HostIoComm
    generic map(
      SIMPLE_G      => true,
      FIFO_LENGTH_G => 64
      )
    port map(
      reset_i   => reset_s,
      clk_i     => clk_s,
      rmv_i     => rd_s,
      data_o    => dataFromHost_s,
      dnEmpty_o => empty_s,
      dnFull_o  => open,
      dnLevel_o => open,
      add_i     => wr_s,
      data_i    => std_logic_vector(dataToHost_s),
      upEmpty_o => open,
      upFull_o  => full_s,
      upLevel_o => open
      );

  dataProcess : process(clk_s, reset_s)
  begin
    if rising_edge(clk_s) then
      rd_s <= LO;
      wr_s <= LO;
      if reset_s = HI then
        state_r <= FILL_WITH_DATA;
        data_r  <= "00001010";
      else
        case state_r is
          when FILL_WITH_DATA =>        -- Init the Down FIFO with some data.
            dataToHost_s <= data_r;
            data_r       <= data_r + 1;
            wr_s         <= HI;
            if data_r = 30 then
              state_r <= CHANGE_DATA;
            end if;
          when CHANGE_DATA =>  -- Read from the Down FIFO and write to the Up FIFO.
            if (empty_s = NO) and (full_s = NO) then
              dataToHost_s <= unsigned(dataFromHost_s) + 3;  -- Alter the data.
              wr_s         <= HI;
              rd_s         <= HI;
              state_r      <= WAIT_FOR_STATUS;
            end if;
          when WAIT_FOR_STATUS =>  -- Wait for the FIFO empty/full statuses to update.
            state_r <= CHANGE_DATA;
        end case;
      end if;
    end if;
  end process;

end architecture;

