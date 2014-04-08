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
-- This module provides a coounication interface between a host PC and 
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
      FIFO_LENGTH_G      : natural          := 16; -- Number of data words in the Up and Down FIFOs.
      WORD_WIDTH_G       : natural          := 8 -- Number of bits in each FIFO word.
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
    end component;

end package;




library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use XESS.CommonPckg.all;
use XESS.HostIoPckg.all;
use XESS.FifoPckg.all;
use work.XessBoardPckg.all;

entity HostIoComm is
  generic (
    ID_G               : std_logic_vector := "11111111";  -- The ID this module responds to.
    PYLD_CNTR_LENGTH_G : natural          := 32;  -- Length of payload bit counter.
    FPGA_DEVICE_G      : FpgaFamily_t     := FPGA_FAMILY_C;  -- FPGA device type.
    TAP_USER_INSTR_G   : TapUserInstr_t   := TAP_USER_INSTR_C;  -- USER instruction this module responds to.
    SIMPLE_G           : boolean          := false;  -- If true, include BscanToHostIo module in this module.
    FIFO_LENGTH_G      : natural          := 16; -- Number of data words in the Up and Down FIFOs.
    WORD_WIDTH_G       : natural          := 8 -- Number of bits in each FIFO word.
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
  signal hostReset_s   : std_logic;     -- Reset signal issued from host.
  signal reset_s       : std_logic;  -- OR combination of host-side and FPGA-side resets.
  type RegAddr_t is (FIFO_REG, STATUS_CONTROL_REG, DN_FREE_REG, UP_USED_REG);
  signal regAddr_s     : std_logic_vector(2 downto 0);  -- Register address from host.
  signal wr_s          : std_logic; -- Write signal from host.
  signal rd_s          : std_logic; -- Read signal from host.
  signal done_s        : std_logic; -- Host read/write done signal.
  signal upRmv_s       : std_logic; -- Remove data from the front of the Up FIFO.
  signal upEmpty_s     : std_logic; -- True if Up FIFO is empty.
  signal upFull_s      : std_logic; -- True if Up FIFO is full.
  signal upLevel_s     : std_logic_vector(upLevel_o'range); -- # of data words in Up FIFO.
  signal dnAdd_s       : std_logic; -- Add data to back of Down FIFO.
  signal dnEmpty_s     : std_logic; -- True if Down FIFO is empty.
  signal dnFull_s      : std_logic; -- True if Down FIFO is full.
  signal dnLevel_s     : std_logic_vector(dnLevel_o'range); -- # of data words in Down FIFO.
  signal busFromHost_s : std_logic_vector(WORD_WIDTH_G-1 downto 0);  -- Data from host to FPGA.
  signal busToHost_s   : std_logic_vector(WORD_WIDTH_G-1 downto 0);  -- Data from FPGA to host.
  signal upFront_s     : std_logic_vector(WORD_WIDTH_G-1 downto 0);  -- Data from Up FIFO to host.
  
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
      ADDR_INC           => 0  -- R/W sequences do not automatically advance the register address.
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

  -- FIFO operations are done within a single cycle after they are initiated by the host.
  done_s <= rd_s or wr_s;

  -- Decode the register address from the host to perform the right comm channel operations.
  RegAddrDecoder : process(regAddr_s, rd_s, wr_s, upFront_s, upEmpty_s, upFull_s, upLevel_s, dnEmpty_s, dnFull_s, dnLevel_s)
  begin
    -- Default control signal values to prevent latches from being synthesized.
    upRmv_s     <= LO; -- Don't remove the data at the front of the Up FIFO.
    dnAdd_s     <= LO; -- Don't add data to the back of the Down FIFO.
    hostReset_s <= LO; -- Don't reset the FIFOs.
    busToHost_s <= (others => '0'); -- Default the data to the host to zero.
    
    case RegAddr_t'val(TO_INTEGER(unsigned(regAddr_s))) is
      when FIFO_REG => -- Host add/remove data to/from the Down/Up FIFOs.
        dnAdd_s     <= wr_s; -- A write by the host adds data from the host to the Down FIFO.
        upRmv_s     <= rd_s; -- A read by the host removes data from the front of the Up FIFO.
        busToHost_s <= upFront_s; -- Send the data at the front of the Up FIFO to the host.
      when STATUS_CONTROL_REG => -- Host read of FIFO statuses or reset of the FIFOs.
        busToHost_s(3 downto 0) <= upFull_s & upEmpty_s & dnFull_s & dnEmpty_s; -- A read by the host gets the FIFO statuses.
        hostReset_s             <= wr_s; -- A write by the host to this address resets the FIFOs.
      when DN_FREE_REG => -- Host reads the # of available slots for additional data in the Down FIFO.
        busToHost_s <= std_logic_vector(TO_UNSIGNED(FIFO_LENGTH_G - TO_INTEGER(unsigned(dnLevel_s)), busToHost_s'length));
      when UP_USED_REG => -- Host reads the # of filled slots waiting in the Up FIFO for xfer to the host.
        busToHost_s <= std_logic_vector(TO_UNSIGNED(TO_INTEGER(unsigned(upLevel_s)), busToHost_s'length));
      when others =>
        null;
    end case;
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
      rmv_i   => upRmv_s,
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
      SIMPLE_G => true,
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
          when FILL_WITH_DATA => -- Init the Down FIFO with some data.
            dataToHost_s <= data_r;
            data_r       <= data_r + 1;
            wr_s         <= HI;
            if data_r = 30 then
              state_r <= CHANGE_DATA;
            end if;
          when CHANGE_DATA => -- Read from the Down FIFO and write to the Up FIFO.
            if (empty_s = NO) and (full_s = NO) then
              dataToHost_s <= unsigned(dataFromHost_s) + 3; -- Alter the data.
              wr_s         <= HI;
              rd_s         <= HI;
              state_r      <= WAIT_FOR_STATUS;
            end if;
          when WAIT_FOR_STATUS => -- Wait for the FIFO empty/full statuses to update.
            state_r <= CHANGE_DATA;
        end case;
      end if;
    end if;
  end process;

end architecture;

