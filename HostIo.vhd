--**********************************************************************
-- Copyright (c) 2011-2014 by XESS Corp <http://www.xess.com>.
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
-- Modules for passing bits back and forth from the host PC
-- to FPGA application logic through the JTAG port.
----------------------------------------------------------------------------------


library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;
use work.XessBoardPckg.all;

package HostIoPckg is

  -- Use one of these to select the memory operation to perform via the JTAG port.
  constant NOP_OPCODE_C   : std_logic_vector(1 downto 0) := "00";
  constant SIZE_OPCODE_C  : std_logic_vector(1 downto 0) := "01";
  constant WRITE_OPCODE_C : std_logic_vector(1 downto 0) := "10";
  constant READ_OPCODE_C  : std_logic_vector(1 downto 0) := "11";

  component BscanToHostIo is
    generic (
      FPGA_DEVICE_G    : FpgaFamily_t   := FPGA_FAMILY_C;  -- FPGA device type.
      TAP_USER_INSTR_G : TapUserInstr_t := TAP_USER_INSTR_C  -- USER instruction this module responds to.
      );
    port (
      -- Interface to HostIoHdrScanner.
      inShiftDr_o : out std_logic;  -- True when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
      drck_o      : out std_logic;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
      tdi_o       : out std_logic;  -- Bit from the host to the FPGA application logic.
      tdo_i       : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdoa_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdob_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdoc_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdod_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdoe_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdof_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdog_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdoh_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdoi_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdoj_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdok_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdol_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdom_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdon_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdoo_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdop_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdoq_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdor_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdos_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdot_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdou_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdov_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdow_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdox_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdoy_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
      tdoz_i      : in  std_logic := LO  -- Bit from the FPGA application logic to the host.
      );
  end component;

  component HostIoHdrScanner is
    generic (
      ID_G               : std_logic_vector := "11111111";  -- The ID this module responds to.
      PYLD_CNTR_LENGTH_G : natural          := 32  -- Length of payload bit counter.
      );
    port (
      -- Interface to BscanHostIo.
      inShiftDr_i : in  std_logic;  -- True when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
      drck_i      : in  std_logic;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
      tdi_i       : in  std_logic;  -- Bit from the host to the FPGA application logic.
      -- Interface to FPGA application logic.
      pyldCntr_o  : out std_logic_vector(PYLD_CNTR_LENGTH_G-1 downto 0);  -- This counts down the number of payload bits still to be received.
      active_o    : out std_logic  -- Tell the external circuitry it has been activated.
      );
  end component;

  component RamCtrlSync is
    port (
      drck_i    : in  std_logic;        -- Clock from JTAG domain.
      clk_i     : in  std_logic;        -- Clock from RAM domain.
      ctrlIn_i  : in  std_logic;        -- Control signal from JTAG domain.
      ctrlOut_o : out std_logic;        -- Control signal to RAM domain.
      opBegun_i : in  std_logic := LO;  -- R/W operation begun signal from RAM domain.
      doneIn_i  : in  std_logic := LO;  -- R/W operation done signal from RAM domain.
      doneOut_o : out std_logic  -- R/W operation done signal to the JTAG domain. 
      );
  end component;

  component HostIoToRamCore is
    generic (
      ID_G               : std_logic_vector := "11111111";  -- The ID this module responds to.
      PYLD_CNTR_LENGTH_G : natural          := 32;  -- Length of payload bit counter.
      ADDR_INC_G         : integer          := 1  -- Add to address after each memory R/W operation.
      );
    port (
      reset_i        : in  std_logic := LO;  -- Active-high reset signal.
      -- Interface to BscanHostIo.
      inShiftDr_i    : in  std_logic := LO;  -- true when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
      drck_i         : in  std_logic := LO;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
      tdi_i          : in  std_logic := LO;  -- Bit from the host to the FPGA application logic.
      tdo_o          : out std_logic;  -- Bit from the FPGA application logic to the host.
      -- Interface to the memory.
      addr_o         : out std_logic_vector;      -- Address to memory.
      wr_o           : out std_logic;   -- Write data to memory when high.
      dataFromHost_o : out std_logic_vector;      -- Data written to memory.
      rd_o           : out std_logic;   -- Read data from memory when high.
      dataToHost_i   : in  std_logic_vector;      -- Data read from memory.
      rwDone_i       : in  std_logic := HI  -- True when memory read/write operation is done.
      );
  end component;

  component HostIoToRam is
    generic (
      ID_G               : std_logic_vector := "11111111";  -- The ID this module responds to.
      PYLD_CNTR_LENGTH_G : natural          := 32;  -- Length of payload bit counter.
      ADDR_INC_G         : integer          := 1;  -- Add to address after each memory R/W operation.
      FPGA_DEVICE_G      : FpgaFamily_t     := FPGA_FAMILY_C;  -- FPGA device type.
      TAP_USER_INSTR_G   : TapUserInstr_t   := TAP_USER_INSTR_C;  -- USER instruction this module responds to.
      SIMPLE_G           : boolean          := false;  -- If true, include BscanToHostIo module in this module.
      SYNC_G             : boolean          := true  -- If true, sync this module with the memory clock domain.
      );
    port (
      reset_i        : in  std_logic := LO;  -- Active-high reset signal.
      -- Interface to BscanHostIo.
      inShiftDr_i    : in  std_logic := LO;  -- true when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
      drck_i         : in  std_logic := LO;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
      tdi_i          : in  std_logic := LO;  -- Bit from the host to the memory.
      tdo_o          : out std_logic;   -- Bit from the memory to the host.
      -- Interface to the memory.
      clk_i          : in  std_logic := LO;  -- Clock from FPGA application logic. 
      addr_o         : out std_logic_vector;       -- Address to memory.
      wr_o           : out std_logic;   -- Write data to memory when high.
      dataFromHost_o : out std_logic_vector;       -- Data written to memory.
      rd_o           : out std_logic;   -- Read data from memory when high.
      dataToHost_i   : in  std_logic_vector;       -- Data read from memory.
      opBegun_i      : in  std_logic := LO;  -- High when memory read/write operation has begun.
      done_i         : in  std_logic := LO  -- High when memory read/write operation is done.
      );
  end component;

  component HostIoToDut is
    generic (
      ID_G                 : std_logic_vector := "11111111";  -- The ID this module responds to.
      PYLD_CNTR_LENGTH_G   : natural          := 32;  -- Length of payload bit counter.
      FPGA_DEVICE_G        : FpgaFamily_t     := FPGA_FAMILY_C;  -- FPGA device type.
      TAP_USER_INSTR_G     : TapUserInstr_t   := TAP_USER_INSTR_C;  -- USER instruction this module responds to.
      SIMPLE_G             : boolean          := false;  -- If true, include BscanToHostIo module in this module.
      OUTPUT_RESET_VALUE_G : std_logic        := LO  -- Value assigned to vectorToDut_o outputs upon reset.
      );
    port (
      reset_i         : in  std_logic := LO;  -- Active-high reset signal.
      -- Interface to BscanHostIo.
      inShiftDr_i     : in  std_logic := LO;  -- True when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
      drck_i          : in  std_logic := LO;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
      tdi_i           : in  std_logic := LO;  -- Bit from the host to the DUT.
      tdo_o           : out std_logic;  -- Bit from the DUT to the host.
      -- Interface to DUT.
      clkToDut_o      : out std_logic;  -- Rising edge clock signals arrival of vector to DUT.
      vectorFromDut_i : in  std_logic_vector;  -- Gather inputs to send back to host thru this bus.
      vectorToDut_o   : out std_logic_vector  -- Output test vector from the host to DUT thru this bus.
      );
  end component;

end package;



--**************************************************************************************************
-- This module connects the BSCAN primitive to a HostIo module.
--**************************************************************************************************

library IEEE, UNISIM, XESS;
use IEEE.std_logic_1164.all;
use UNISIM.VComponents.all;
use XESS.CommonPckg.all;
use XESS.HostIoPckg.all;
use work.XessBoardPckg.all;

entity BscanToHostIo is
  generic (
    FPGA_DEVICE_G    : FpgaFamily_t   := FPGA_FAMILY_C;    -- FPGA device type.
    TAP_USER_INSTR_G : TapUserInstr_t := TAP_USER_INSTR_C  -- USER instruction this module responds to.
    );
  port (
    -- Interface to HostIoHdrScanner.
    inShiftDr_o : out std_logic;  -- True when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
    drck_o      : out std_logic;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
    tdi_o       : out std_logic;  -- Bit from the host to the FPGA application logic.
    tdo_i       : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdoa_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdob_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdoc_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdod_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdoe_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdof_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdog_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdoh_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdoi_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdoj_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdok_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdol_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdom_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdon_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdoo_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdop_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdoq_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdor_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdos_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdot_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdou_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdov_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdow_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdox_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdoy_i      : in  std_logic := LO;  -- Bit from the FPGA application logic to the host.
    tdoz_i      : in  std_logic := LO  -- Bit from the FPGA application logic to the host.
    );
end entity;


architecture arch of BscanToHostIo is
  -- Signals from BSCAN primitive.
  signal bscanReset_s : std_logic;
  signal bscanShift_s : std_logic;
  signal bscanDrck1_s : std_logic;
  signal bscanDrck2_s : std_logic;
  signal bscanSel1_s  : std_logic;
  signal bscanSel2_s  : std_logic;
  signal bscanSel_s   : std_logic;
  signal tdo_s        : std_logic;
begin

  spartan3a_bscan : if FPGA_DEVICE_G = SPARTAN3A_E generate
    -- Boundary-scan interface to FPGA JTAG port.
    UBscanUser : BSCAN_SPARTAN3A
      port map(
        DRCK1 => bscanDrck1_s,  -- data clock after USER1 instruction received.
        DRCK2 => bscanDrck2_s,  -- data clock after USER2 instruction received.
        RESET => bscanReset_s,          -- JTAG TAP FSM reset.
        SEL1  => bscanSel1_s,           -- USER1 instruction enables user-I/O.
        SEL2  => bscanSel2_s,           -- USER2 instruction enables user-I/O.
        SHIFT => bscanShift_s,  -- True when JTAG TAP FSM is in the SHIFT-DR state.
        TDI   => tdi_o,  -- Data bits from the host arrive through here.
        TDO1  => tdo_s,  -- Bits from the FPGA app. logic go to the TDO pin and back to the host.
        TDO2  => tdo_s  -- Bits from the FPGA app. logic go to the TDO pin and back to the host.
        );

    -- All the bits from the application logic are OR'ed together since only one HostIo module
    -- will be active and the others will pull their TDO outputs to logic 0.
    tdo_s <= tdo_i or tdoa_i or tdob_i or tdoc_i or tdod_i or tdoe_i or tdof_i or tdog_i or tdoh_i
             or tdoi_i or tdoj_i or tdok_i or tdol_i or tdom_i or tdon_i or tdoo_i or tdop_i or tdoq_i
             or tdor_i or tdos_i or tdot_i or tdou_i or tdov_i or tdow_i or tdox_i or tdoy_i or tdoz_i;

    -- Select the appropriate sel signal based upon which USER instruction this module responds to.
    bscanSel_s <= bscanSel1_s when TAP_USER_INSTR_G = USER1_E else bscanSel2_s;

    -- Detect when a USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
    inShiftDr_o <= YES when bscanReset_s = LO and bscanShift_s = HI and bscanSel_s = HI else NO;

    -- Output the appropriate drck signal to the HostIo module.
    drck_o <= bscanDrck1_s when TAP_USER_INSTR_G = USER1_E else bscanDrck2_s;
  end generate;

  spartan6_bscan : if FPGA_DEVICE_G = SPARTAN6_E generate
    -- Boundary-scan interface to FPGA JTAG port.
    UBscanUser : BSCAN_SPARTAN6
      generic map(
        JTAG_CHAIN => TapUserInstr_t'pos(TAP_USER_INSTR_G)
        )
      port map(
        DRCK  => drck_o,        -- Data clock after USER instruction received.
        RESET => bscanReset_s,          -- JTAG TAP FSM reset.
        SEL   => bscanSel_s,    -- True when USER instruction enters IR.
        SHIFT => bscanShift_s,  -- True when JTAG TAP FSM is in the SHIFT-DR state.
        TDI   => tdi_o,         -- Data bits from the host arrive through here.
        TDO   => tdo_s  -- Bits from the FPGA app. logic go to the TDO pin and back to the host.
        );

    -- All the bits from the application logic are OR'ed together since only one HostIo module
    -- will be active and the others will pull their TDO outputs to logic 0.
    tdo_s <= tdo_i or tdoa_i or tdob_i or tdoc_i or tdod_i or tdoe_i or tdof_i or tdog_i or tdoh_i
             or tdoi_i or tdoj_i or tdok_i or tdol_i or tdom_i or tdon_i or tdoo_i or tdop_i or tdoq_i
             or tdor_i or tdos_i or tdot_i or tdou_i or tdov_i or tdow_i or tdox_i or tdoy_i or tdoz_i;

    -- Detect when a USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
    inShiftDr_o <= YES when bscanReset_s = LO and bscanShift_s = HI and bscanSel_s = HI else NO;
  end generate;

end architecture;



--**************************************************************************************************
-- This module accepts a bitstream from a BscanHostIo module and extracts an ID and
-- the number of payload bits that follow.  It triggers an attached module if the received
-- ID matches the ID passed in by the generic parameter. The attached module accepts the
-- bitstream until all the payload bits are processed.  The attached module can also return
-- results that it has produced (usually from an operation initiated by a previous instruction). 
-- After the payload bit counter decrements to zero, this module is reset and it
-- repeats the entire process for the next instruction.
--
-- If the received ID does not match, the attached module is not triggered. The bitstream 
-- continues downstream to look for a possible match with another module.
--
--             |                    Complete Instruction                      |
--             |          Header reception           |   Payload reception    |
-- TDI:        |       ID        | # of payload bits | Payload bits from host |
-- TDO:        |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx| Result bits from FPGA  |
-- Bit counter |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|  N-1 N-2 ...... 2 1 0  |
--**************************************************************************************************

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use XESS.CommonPckg.all;

entity HostIoHdrScanner is
  generic (
    ID_G               : std_logic_vector := "11111111";  -- The ID this module responds to.
    PYLD_CNTR_LENGTH_G : natural          := 32  -- Length of payload bit counter.
    );
  port (
    -- Interface to BscanHostIo.
    inShiftDr_i : in  std_logic;  -- True when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
    drck_i      : in  std_logic;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
    tdi_i       : in  std_logic;  -- Bit from the host to the FPGA application logic.
    -- Interface to FPGA application logic.
    pyldCntr_o  : out std_logic_vector(PYLD_CNTR_LENGTH_G-1 downto 0);  -- This counts down the number of payload bits still to be received.
    active_o    : out std_logic  -- Tell the external circuitry it has been activated.
    );
end entity;


architecture arch of HostIoHdrScanner is
  -- The header register consists of the ID field and a field with the # of payload bits that follow.
  signal id_r       : std_logic_vector(ID_G'high downto ID_G'low);
  signal pyldCntr_r : std_logic_vector(pyldCntr_o'range);
  signal hdrRcvd_r  : std_logic;  -- High after an ID and # of payload bits have been shifted in.
begin

  -- Scan in the header and any following payload bits.
  process(drck_i)
  begin
    if rising_edge(drck_i) then

      -- Reset the header register if BSCAN drops out of USER SHIFT-DR state or if the current instruction is done.
      -- Detect when an instruction is done, i.e. has received all its payload bits.
      -- Detection uses payload counter value of 1 (not 0) because the last bit has entered at that point.
      if inShiftDr_i = NO or (hdrRcvd_r = YES and pyldCntr_r = 1) then
        -- Clear the header register and set MSbit of payload bit counter.  A header has
        -- been received when this bit has been shifted all the way through the counter and ID
        -- registers and into the header received flag.
        id_r                        <= (others => ZERO);
        pyldCntr_r                  <= (others => ZERO);
        pyldCntr_r(pyldCntr_r'high) <= YES;
        hdrRcvd_r                   <= NO;
        
      else    -- Otherwise, shift in header bits on the rising edge of DRCK.
        if hdrRcvd_r = NO then  -- Shift bits into the header register until a complete header is received.
          pyldCntr_r <= tdi_i & pyldCntr_r(pyldCntr_r'high downto 1);
          id_r       <= pyldCntr_r(0) & id_r(id_r'high downto 1);
          hdrRcvd_r  <= id_r(0);
        else  -- After a header has been received, count down the number of payload bits following the header.
          pyldCntr_r <= pyldCntr_r - 1;
        end if;
      end if;
    end if;
  end process;

  -- The attached module is activated if it matches the ID in the header.
  active_o <= HI when (id_r(id_r'high downto 0) = ID_G(0 to ID_G'high)) and (hdrRcvd_r = HI) else LO;

  -- Output the number of payload bits still to be received.
  pyldCntr_o <= pyldCntr_r;
  
end architecture;



--**************************************************************************************************
-- This module interfaces with BscanToHostIo to perform read/write operations to memory devices.
--
-- Write operations:
-- Once the HostIoHdrScanner module extracts the ID and number of payload bits,
-- a write operation is activated by the opcode in the first two bits in the payload.
-- This module then extracts a starting address from the payload bitstream.
-- Then this module extracts data words from the payload bitstream and writes them to
-- the memory device at sequentially increasing addresses beginning from that address.
--
--       |     Header reception     |                    Payload bits                        |
-- TDI:  |  ID  | # of payload bits | Opcode | Starting address |  Data1  | ..... | DataN |
-- TDO:  |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|
-- Addr: |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx| Addr1 | ..... | AddrN |
-- Data: |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|   Data1  | ..... | DataN        |
--
-- Read operations:
-- Once the HostIoHdrScanner module extracts the ID and number of payload bits,
-- a read operation is activated by the opcode in the first two bits in the payload.
-- This module then extracts a starting address from the payload bitstream.
-- Then this module reads data from the memory device at sequentially increasing addresses
-- starting from that address, and it shifts them serially back to the host.
-- (Valid data on TDO starts after the first read of the memory completes.) 
--
--       |     Header reception     |        Payload bits       |  RAM data goes back to host  |
-- TDI:  |  ID  | # of payload bits | Opcode | Starting address |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|
-- TDO:  |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx| Data1 | ... | DataN-1 |
-- Addr: |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx| Addr1 | Addr2 | ... |  AddrN  |
-- Data: |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx| Data1 | Data2 | ... |  DataN  |
--
-- Parameter query operation:
-- Once the HostIoHdrScanner module extracts the ID and number of payload bits,
-- a parameter query operation is activated by the opcode in the first two bits in the payload.
-- This module then places the width of the memory address and data buses into a register
-- and shifts it serially back to the host.
--
--       |     Header reception     | Payload bits |  Parameter data goes back to host  |
-- TDI:  |  ID  | # of payload bits |    Opcode    |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|
-- TDO:  |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|   Address width   |   Data width   |
-- Addr: |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|
-- Data: |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|
--**************************************************************************************************

library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_ARITH.all;
use IEEE.STD_LOGIC_UNSIGNED.all;
use XESS.CommonPckg.all;
use XESS.HostIoPckg.all;

entity HostIoToRamCore is
  generic (
    ID_G               : std_logic_vector := "11111111";  -- The ID this module responds to.
    PYLD_CNTR_LENGTH_G : natural          := 32;  -- Length of payload bit counter.
    ADDR_INC_G         : integer          := 1  -- Add to address after each memory R/W operation.
    );
  port (
    reset_i        : in  std_logic := LO;  -- Active-high reset signal.
    -- Interface to BscanHostIo.
    inShiftDr_i    : in  std_logic := LO;  -- true when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
    drck_i         : in  std_logic := LO;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
    tdi_i          : in  std_logic := LO;  -- Bit from the host to the FPGA application logic.
    tdo_o          : out std_logic;  -- Bit from the FPGA application logic to the host.
    -- Interface to the memory.
    addr_o         : out std_logic_vector;      -- Address to memory.
    wr_o           : out std_logic;     -- Write data to memory when high.
    dataFromHost_o : out std_logic_vector;      -- Data written to memory.
    rd_o           : out std_logic;     -- Read data from memory when high.
    dataToHost_i   : in  std_logic_vector;      -- Data read from memory.
    rwDone_i       : in  std_logic := HI  -- True when memory read/write operation is done.
    );
end entity;


architecture arch of HostIoToRamCore is
  signal pyldCntr_s         : std_logic_vector(PYLD_CNTR_LENGTH_G-1 downto 0);
  signal active_s           : std_logic;
  signal opcode_r           : std_logic_vector(NOP_OPCODE_C'range)          := NOP_OPCODE_C;
  signal opcodeRcvd_r       : std_logic                                     := NO;
  signal addrFromHost_r     : std_logic_vector(addr_o'high downto 0)        := (others => ZERO);
  signal addrFromHostRcvd_r : std_logic                                     := NO;
  constant PARAM_SIZE_C     : natural                                       := 16;
  constant SHIFT_REG_SIZE_C : natural                                       := IntMax(PARAM_SIZE_C, dataFromHost_o'length);
  signal shiftReg_r         : std_logic_vector(SHIFT_REG_SIZE_C-1 downto 0) := (others => ZERO);
  signal bitCntr_r          : natural range 0 to SHIFT_REG_SIZE_C           := 0;
  signal wrToMemory_r       : std_logic                                     := NO;
  signal rdFromMemory_r     : std_logic                                     := NO;
  signal dataFromMemory_r   : std_logic_vector(dataToHost_i'high downto 0);
begin

  -- Scan the bits from the host looking for an instruction header.
  UHdrScannner : HostIoHdrScanner
    generic map (
      ID_G               => ID_G,
      PYLD_CNTR_LENGTH_G => PYLD_CNTR_LENGTH_G
      )
    port map (
      -- Interface to BSCAN primitive.
      inShiftDr_i => inShiftDr_i,
      drck_i      => drck_i,
      tdi_i       => tdi_i,
      -- Interface to FPGA application logic.
      pyldCntr_o  => pyldCntr_s,
      active_o    => active_s
      );

  -- Process the instruction bits as they arrive from the host.
  process(drck_i)
  begin
    if rising_edge(drck_i) then

      -- Keep processing as long as this module is activated or writing to memory.
      if (active_s = YES or wrToMemory_r = HI) and (reset_i = LO) then

        -- First, get the opcode from the host.
        if opcodeRcvd_r = NO then
          opcode_r     <= tdi_i & opcode_r(opcode_r'high downto 1);
          opcodeRcvd_r <= opcode_r(0);  -- Opcode complete once LSB is set.

        -- Next, process the received opcode.
        else
          case opcode_r is

            when SIZE_OPCODE_C =>  -- Return memory address and data bus parameters.
              if bitCntr_r = 0 then  -- Load the memory parameters into the host shift register.
                shiftReg_r(PARAM_SIZE_C-1 downto 0) <= CONV_STD_LOGIC_VECTOR(dataFromHost_o'length, PARAM_SIZE_C/2)
                                                       & CONV_STD_LOGIC_VECTOR(addr_o'length, PARAM_SIZE_C/2);
                bitCntr_r <= PARAM_SIZE_C;  -- Set the number of data bits to send.
              else  -- Shift next bit of memory parameters to the host.
                shiftReg_r <= ZERO & shiftReg_r(shiftReg_r'high downto 1);  -- Shift register contents.
                bitCntr_r  <= bitCntr_r - 1;  -- One more bit has been sent to the host.
              end if;

            when WRITE_OPCODE_C =>      -- Perform write to memory.
              if addrFromHostRcvd_r = NO then  -- Receiving the memory write address from the host.
                addrFromHost_r                  <= tdi_i & addrFromHost_r(addrFromHost_r'high downto 1);
                addrFromHostRcvd_r              <= addrFromHost_r(0);  -- Address complete once LSB is set.
                wrToMemory_r                    <= NO;
                shiftReg_r                      <= (others => ZERO);
                shiftReg_r(dataFromHost_o'high) <= ONE;
              else    -- Now get data to write to memory from the host.
                if wrToMemory_r = YES and rwDone_i = YES then  -- Write to memory is done.
                  wrToMemory_r   <= NO;  -- Stop any further writes till another complete data word arrives from host.
                  addrFromHost_r <= addrFromHost_r + ADDR_INC_G;  -- Point to next memory location to be written (if needed).
                end if;
                if shiftReg_r(0) = LO then  -- Shifting in data from host before writing it to memory. 
                  shiftReg_r(dataFromHost_o'range) <= tdi_i & shiftReg_r(dataFromHost_o'high downto 1);
                else  -- Data from host received, now write it into the memory.
                  dataFromHost_o                  <= tdi_i & shiftReg_r(dataFromHost_o'high downto 1);  -- Store host data so it doesn't change if more bits arrive from host.
                  -- Clear shift register so it can receive more data from the host.
                  shiftReg_r                      <= (others => ZERO);
                  shiftReg_r(dataFromHost_o'high) <= HI;
                  wrToMemory_r                    <= YES;  -- Initiate write of host data to memory.
                end if;
              end if;

            -- Perform read of memory.
            when READ_OPCODE_C =>
              if addrFromHostRcvd_r = NO then  -- Receiving the memory read address from the host.
                addrFromHost_r     <= tdi_i & addrFromHost_r(addrFromHost_r'high downto 1);
                addrFromHostRcvd_r <= addrFromHost_r(0);  -- Address complete once LSB is set.
                rdFromMemory_r     <= addrFromHost_r(0);  -- Initiate read as soon as address is received.
                bitCntr_r          <= dataFromMemory_r'length - 1;  -- Output garbage word until 1st read has a chance to complete.
              else
                if rdFromMemory_r = YES and rwDone_i = YES then  -- Receive a complete data word from the host.
                  rdFromMemory_r   <= NO;  -- OK, data is here so stop the reading the memory.
                  dataFromMemory_r <= dataToHost_i;  -- Store the memory data until it can be loaded into the host shift reg.
                end if;
                if bitCntr_r = 0 then   -- Shift register is empty.
                  shiftReg_r(dataFromMemory_r'range) <= dataFromMemory_r;  --  Reload it with new data from memory.
                  bitCntr_r                          <= dataFromMemory_r'length-1;  -- Reload the bit counter.
                  if pyldCntr_s >= shiftReg_r'length then  -- Is more data expected by the host?
                    addrFromHost_r <= addrFromHost_r + ADDR_INC_G;  -- Point to next memory location to read from.
                    rdFromMemory_r <= YES;     -- Initiate the read operation.
                  end if;
                else  -- Shift register is shifting its contents to the host.
                  shiftReg_r <= ZERO & shiftReg_r(shiftReg_r'high downto 1);  -- Shift register contents.
                  bitCntr_r  <= bitCntr_r - 1;  -- One more bit has been sent to the host.
                end if;
              end if;

            -- Default case is NOP.
            when others =>
              null;
              
          end case;
        end if;
        
      else  -- Reset everything when this module is not selected or is reset.
        opcode_r                            <= (others => ZERO);
        opcode_r(opcode_r'high)             <= ONE;
        opcodeRcvd_r                        <= NO;
        addrFromHost_r                      <= (others => ZERO);
        addrFromHost_r(addrFromHost_r'high) <= ONE;
        addrFromHostRcvd_r                  <= NO;
        shiftReg_r                          <= (others => ZERO);
        shiftReg_r(dataFromHost_o'high)     <= ONE;
        bitCntr_r                           <= 0;
        wrToMemory_r                        <= NO;
        rdFromMemory_r                      <= NO;
      end if;
    end if;

  end process;

  -- Force output low if this module has not been selected. 
  -- This allows the bit outputs of multiple modules to be OR'ed together
  -- and sent to the TDO input of the BSCAN primitive.
  tdo_o <= shiftReg_r(0) when active_s = YES else LO;

  -- Attach this module to a memory.
  addr_o <= addrFromHost_r;
  wr_o   <= wrToMemory_r;
  rd_o   <= rdFromMemory_r;
  
end architecture;



--**************************************************************************************************
-- This module synchronizes a HostIoToRamCore read or write control signal to the clock domain
-- of the memory device.
--**************************************************************************************************

library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use XESS.CommonPckg.all;
use XESS.SyncToClockPckg.all;

entity RamCtrlSync is
  port (
    drck_i    : in  std_logic;          -- Clock from JTAG domain.
    clk_i     : in  std_logic;          -- Clock from RAM domain.
    ctrlIn_i  : in  std_logic;          -- Control signal from JTAG domain.
    ctrlOut_o : out std_logic;          -- Control signal to RAM domain.
    opBegun_i : in  std_logic := LO;  -- R/W operation begun signal from RAM domain.
    doneIn_i  : in  std_logic := LO;  -- R/W operation done signal from RAM domain.
    doneOut_o : out std_logic  -- R/W operation done signal to the JTAG domain. 
    );
end entity;

architecture arch of RamCtrlSync is
  signal ctrlIn_s  : std_logic := LO;  -- JTAG domain control signal sync'ed to RAM domain.
  signal ctrlOut_r : std_logic := LO;
  signal doneOut_r : std_logic := LO;
begin

  -- Sync the RAM control signal from the JTAG clock domain to the RAM domain.
  UCtrlSync : SyncToClock
    generic map (NUM_SYNC_STAGES_G => 2)
    port map (clk_i                => clk_i, unsynced_i => ctrlIn_i, synced_o => ctrlIn_s);

  -- Now raise-and-hold the output control signal to the RAM upon a rising edge of the input control signal.
  -- Lower the output control signal if the input control signal goes low or if the RAM signals that the 
  -- operation has begun or has finished.  
  process(clk_i)
    variable prevCtrlIn_v : std_logic := LO;  -- Previous value of the input control signal.
  begin
    if rising_edge(clk_i) then
      if ctrlIn_s = LO then
        -- Lower the RAM control signal if the input signal has been deactivated.
        ctrlOut_r <= LO;
      elsif prevCtrlIn_v = LO then
        -- Raise the RAM control signal upon a rising edge of the input control signal.
        ctrlOut_r <= HI;
      elsif opBegun_i = HI or doneIn_i = HI then
        -- Lower the RAM control signal once the RAM has begun or completed the R/W operation.
        ctrlOut_r <= LO;
      end if;
      prevCtrlIn_v := CtrlIn_s;  -- Store the previous value of the input control signal.
    end if;
  end process;

  -- Make sure the RAM control signal is lowered as soon as the RAM starts doing or has completed
  -- its operation, otherwise the operation may be repeated.
  ctrlOut_o <= ctrlOut_r and not (doneIn_i or opBegun_i);

  -- Inform the HostIoToRamCore when the memory operation is done. Latch the done signal
  -- from the RAM until the HostIoToRamCore sees it and lowers its control signal.
  -- Once the control signal is lowered, the RAM will eventually lower its done signal.
  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if ctrlIn_s = LO then
        doneOut_r <= LO;
      elsif doneIn_i = HI then
        doneOut_r <= HI;
      end if;
    end if;
  end process;

  UDoneSync : SyncToClock port map (clk_i => drck_i, unsynced_i => doneOut_r, synced_o => doneOut_o);

end architecture;



--**************************************************************************************************
-- This module combines the HostIoToRamCore with two RamCtrlSync modules for the R/W control
-- signals to form a complete interface between the JTAG port and a memory device.
--**************************************************************************************************

library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use XESS.CommonPckg.all;
use XESS.SyncToClockPckg.all;
use XESS.HostIoPckg.all;
use work.XessBoardPckg.all;

entity HostIoToRam is
  generic (
    ID_G               : std_logic_vector := "11111111";  -- The ID this module responds to.
    PYLD_CNTR_LENGTH_G : natural          := 32;  -- Length of payload bit counter.
    ADDR_INC_G         : integer          := 1;  -- Add to address after each memory R/W operation.
    FPGA_DEVICE_G      : FpgaFamily_t     := FPGA_FAMILY_C;  -- FPGA device type.
    TAP_USER_INSTR_G   : TapUserInstr_t   := TAP_USER_INSTR_C;  -- USER instruction this module responds to.
    SIMPLE_G           : boolean          := false;  -- If true, include BscanToHostIo module in this module.
    SYNC_G             : boolean          := true  -- If true, sync this module with the memory clock domain.
    );
  port (
    reset_i        : in  std_logic := LO;  -- Active-high reset signal.
    -- Interface to BscanHostIo.
    inShiftDr_i    : in  std_logic := LO;  -- true when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
    drck_i         : in  std_logic := LO;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
    tdi_i          : in  std_logic := LO;  -- Bit from the host to the memory.
    tdo_o          : out std_logic;     -- Bit from the memory to the host.
    -- Interface to the memory.
    clk_i          : in  std_logic := LO;  -- Clock from FPGA application logic. 
    addr_o         : out std_logic_vector;       -- Address to memory.
    wr_o           : out std_logic;     -- Write data to memory when high.
    dataFromHost_o : out std_logic_vector;       -- Data written to memory.
    rd_o           : out std_logic;     -- Read data from memory when high.
    dataToHost_i   : in  std_logic_vector;       -- Data read from memory.
    opBegun_i      : in  std_logic := LO;  -- High when memory read/write operation has begun.
    done_i         : in  std_logic := LO  -- High when memory read/write operation is done.
    );
end entity;


architecture arch of HostIoToRam is
  -- Internal memory signals.
  signal wr_s         : std_logic;
  signal rd_s         : std_logic;
  signal wrDone_s     : std_logic;
  signal rdDone_s     : std_logic;
  signal rwDone_s     : std_logic;
  signal dataToHost_r : std_logic_vector(dataToHost_i'range);
  signal dataToHost_s : std_logic_vector(dataToHost_i'range);
  -- Internal JTAG signals.
  signal inShiftDr_s  : std_logic;
  signal drck_s       : std_logic;
  signal tdi_s        : std_logic;
  signal tdo_s        : std_logic;
begin

  -- If you're only interfacing the JTAG port to a single module, then the
  -- SIMPLE_G parameter lets you build the JTAG interface right into this interface
  -- and connect it to the internal JTAG signals.
  USimple : if SIMPLE_G = true generate
  begin
    UBscanHostIo : BscanToHostIo
      generic map(
        FPGA_DEVICE_G    => FPGA_DEVICE_G,
        TAP_USER_INSTR_G => TAP_USER_INSTR_G
        )
      port map(
        inShiftDr_o => inShiftDr_s,
        drck_o      => drck_s,
        tdi_o       => tdi_s,
        tdo_i       => tdo_s
        );
  end generate;

  -- If you're interfacing several modules to the JTAG port, then you'll be using
  -- an external JTAG module. So just connect the I/O ports to the internal
  -- JTAG signals.
  UComplex : if SIMPLE_G = false generate
  begin
    inShiftDr_s <= inShiftDr_i;
    drck_s      <= drck_i;
    tdi_s       <= tdi_i;
    tdo_o       <= tdo_s;
  end generate;

  -- Connect the HostIoToRamCore to the internal JTAG and memory signals.
  UHostIoToRamCore : HostIoToRamCore
    generic map (
      ID_G               => ID_G,
      PYLD_CNTR_LENGTH_G => PYLD_CNTR_LENGTH_G,
      ADDR_INC_G         => ADDR_INC_G
      )
    port map(
      reset_i        => reset_i,
      inShiftDr_i    => inShiftDr_s,
      drck_i         => drck_s,
      tdi_i          => tdi_s,
      tdo_o          => tdo_s,
      addr_o         => addr_o,
      wr_o           => wr_s,
      dataFromHost_o => dataFromHost_o,
      rd_o           => rd_s,
      dataToHost_i   => dataToHost_s,
      rwDone_i       => rwDone_s
      );

  -- Synchronize the JTAG interface to the memory clock domain.
  USync : if SYNC_G = true generate
  begin
    UWrRamCtrlSync : RamCtrlSync
      port map (
        drck_i    => drck_s,
        clk_i     => clk_i,
        ctrlIn_i  => wr_s,
        ctrlOut_o => wr_o,
        opBegun_i => opBegun_i,
        doneIn_i  => done_i,
        doneOut_o => wrDone_s
        );

    URdRamCtrlSync : RamCtrlSync
      port map (
        drck_i    => drck_s,
        clk_i     => clk_i,
        ctrlIn_i  => rd_s,
        ctrlOut_o => rd_o,
        opBegun_i => opBegun_i,
        doneIn_i  => done_i,
        doneOut_o => rdDone_s
        );

    -- Hold the data in a register so it doesn't change until the 
    -- slower host has a chance to read it.
    process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (rd_s = YES) and (done_i = YES) then
          dataToHost_r <= dataToHost_i;
        end if;
      end if;
    end process;
    dataToHost_s <= dataToHost_r;

    -- Only one read or write memory operation can be in-process at a time,
    -- so OR their done signals together to create a unified
    -- "memory operation done" signal.
    rwDone_s <= rdDone_s or wrDone_s;
  end generate;

  -- Don't synchronize the JTAG interface to the memory clock domain.
  UUnsync : if SYNC_G = false generate
  begin
    rwDone_s     <= done_i;
    rd_o         <= rd_s;
    wr_o         <= wr_s;
    dataToHost_s <= dataToHost_i;
  end generate;

end architecture;



--**************************************************************************************************
-- This module interfaces with BscanToHostIo to send/receive test vectors to/from a device-under-test (DUT).
--
-- Write operations:
-- Once the HostIoHdrScanner module extracts the ID and number of payload bits,
-- a write operation is activated by the opcode in the first two bits in the payload.
-- This module then captures N bits serially from TDI and outputs them in parallel
-- onto the inputs of the DUT. A clock pulse is also generated which can be used to clock
-- the DUT if desired.
--
--          |     Header reception     |         Payload bits           | One clock cycle |
-- TDI:     |  ID  | # of payload bits | Opcode | b1 b2 b3 b4 b5 ... bN |
-- TDO:     |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|
-- DUT INP: |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|  b1b2b3b4...bN  |
-- DUT clk: |____________________________________________________________________|^^^^^^^^|____
--
-- Read operations:
-- Once the HostIoHdrScanner module extracts the ID and number of payload bits,
-- a read operation is activated by the opcode in the first two bits in the payload.
-- This module captures all the outputs from the DUT in parallel and then shifts them
-- back to the host on TDO.
--
--           |     Header reception     | Payload bits | One clock cycle |  DUT output bits   |
-- TDI:      |  ID  | # of payload bits |    Opcode    |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|
-- TDO:      |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx| b1 b2 b3 b4 ... bN |
-- DUT OUTP: |                                  b1b2b3b4...bN                                 |
--
-- Parameter query operation:
-- Once the HostIoHdrScanner module extracts the ID and number of payload bits,
-- a parameter query operation is activated by the opcode in the first two bits in the payload.
-- This module then places the number of DUT inputs and outputs into a register and shifts it 
-- serially back to the host.
--
--       |     Header reception     | Payload bits |  Parameter data goes back to host  |
-- TDI:  |  ID  | # of payload bits |    Opcode    |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|
-- TDO:  |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|   # DUT inputs   |  # DUT outputs  |
--**************************************************************************************************

library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_ARITH.all;
use IEEE.STD_LOGIC_UNSIGNED.all;
use XESS.CommonPckg.all;
use XESS.HostIoPckg.all;
use work.XessBoardPckg.all;

entity HostIoToDut is
  generic (
    ID_G                 : std_logic_vector := "11111111";  -- The ID this module responds to.
    PYLD_CNTR_LENGTH_G   : natural          := 32;  -- Length of payload bit counter.
    FPGA_DEVICE_G        : FpgaFamily_t     := FPGA_FAMILY_C;  -- FPGA device type.
    TAP_USER_INSTR_G     : TapUserInstr_t   := TAP_USER_INSTR_C;  -- USER instruction this module responds to.
    SIMPLE_G             : boolean          := false;  -- If true, include BscanToHostIo module in this module.
    OUTPUT_RESET_VALUE_G : std_logic        := LO  -- Value assigned to vectorToDut_o outputs upon reset.
    );
  port (
    reset_i         : in  std_logic := LO;  -- Active-high reset signal.
    -- Interface to BscanHostIo.
    inShiftDr_i     : in  std_logic := LO;  -- True when USER JTAG instruction is active and the TAP FSM is in the Shift-DR state.
    drck_i          : in  std_logic := LO;  -- Bit clock. TDI clocked in on rising edge, TDO sampled on falling edge.
    tdi_i           : in  std_logic := LO;  -- Bit from the host to the DUT.
    tdo_o           : out std_logic;    -- Bit from the DUT to the host.
    -- Interface to DUT.
    clkToDut_o      : out std_logic;  -- Rising edge clock signals arrival of vector to DUT.
    vectorFromDut_i : in  std_logic_vector;  -- Gather inputs to send back to host thru this bus.
    vectorToDut_o   : out std_logic_vector  -- Output test vector from the host to DUT thru this bus.
    );
end entity;


architecture arch of HostIoToDut is
  signal active_s           : std_logic;  -- True when the module is activated by a matching ID.
  signal opcode_r           : std_logic_vector(NOP_OPCODE_C'range);  -- Stores opcode received via JTAG.
  signal opcodeRcvd_r       : std_logic;  -- True once opcode is received.
  constant PARAM_SIZE_C     : natural := 16;
  constant SHIFT_REG_SIZE_C : natural := IntMax(IntMax(PARAM_SIZE_C, vectorFromDut_i'length), vectorToDut_o'length);
  signal shiftReg_r         : std_logic_vector(SHIFT_REG_SIZE_C-1 downto 0);  -- Stores DUT input & output bits.
  signal bitCntr_r          : natural range 0 to SHIFT_REG_SIZE_C;
  signal activateClk_r      : std_logic;
  -- Internal JTAG signals.
  signal inShiftDr_s        : std_logic;
  signal drck_s             : std_logic;
  signal tdi_s              : std_logic;
  signal tdo_s              : std_logic;
begin

  -- If you're only interfacing the JTAG port to a single module, then the
  -- SIMPLE_G parameter lets you build the JTAG interface right into this interface
  -- and connect it to the internal JTAG signals.
  USimple : if SIMPLE_G = true generate
  begin
    UBscanHostIo : BscanToHostIo
      generic map(
        FPGA_DEVICE_G    => FPGA_DEVICE_G,
        TAP_USER_INSTR_G => TAP_USER_INSTR_G
        )
      port map(
        inShiftDr_o => inShiftDr_s,
        drck_o      => drck_s,
        tdi_o       => tdi_s,
        tdo_i       => tdo_s
        );
  end generate;

  -- If you're interfacing several modules to the JTAG port, then you'll be using
  -- an external JTAG module. So just connect the I/O ports to the internal
  -- JTAG signals.
  UComplex : if SIMPLE_G = false generate
  begin
    inShiftDr_s <= inShiftDr_i;
    drck_s      <= drck_i;
    tdi_s       <= tdi_i;
    tdo_o       <= tdo_s;
  end generate;

  -- Scan the bits from the host looking for an instruction header.
  UHdrScanner : HostIoHdrScanner
    generic map (
      ID_G               => ID_G,
      PYLD_CNTR_LENGTH_G => PYLD_CNTR_LENGTH_G
      )
    port map (
      -- Interface to BSCAN primitive.
      inShiftDr_i => inShiftDr_s,
      drck_i      => drck_s,
      tdi_i       => tdi_s,
      -- Interface to DUT.
      active_o    => active_s
      );

  -- Process the instruction bits as they arrive from the host.
  process(drck_s)
  begin
    if rising_edge(drck_s) then

      activateClk_r <= LO;  -- By default, keep clock to DUT inactive.

      -- Keep processing as long as this module is activated.
      if active_s = YES and reset_i = LO then

        -- First, get the opcode from the host.
        if opcodeRcvd_r = NO then
          opcode_r     <= tdi_s & opcode_r(opcode_r'high downto 1);
          opcodeRcvd_r <= opcode_r(0);  -- Opcode complete once LSB is set.

        -- Next, process the received opcode.
        else
          case opcode_r is

            when SIZE_OPCODE_C =>  -- Return DUT input and output-width parameters.
              if bitCntr_r = 0 then  -- Load the I/O parameters into the host shift register.
                shiftReg_r(PARAM_SIZE_C-1 downto 0) <= CONV_STD_LOGIC_VECTOR(vectorFromDut_i'length, PARAM_SIZE_C/2)
                                                       & CONV_STD_LOGIC_VECTOR(vectorToDut_o'length, PARAM_SIZE_C/2);
                bitCntr_r <= PARAM_SIZE_C;  -- Set the number of bits to send.
              else  -- Shift next bit of I/O parameters to the host.
                shiftReg_r <= ZERO & shiftReg_r(shiftReg_r'high downto 1);  -- Shift register contents.
                bitCntr_r  <= bitCntr_r - 1;  -- One more bit has been sent to the host.
              end if;

            when WRITE_OPCODE_C =>      -- Output a test vector to the DUT.
              case vectorToDut_o'length is
                when 0 =>
                  -- No inputs to the DUT, so just pulse the clock.
                  activateClk_r <= HI;
                when 1 =>
                  -- Only one input to DUT, so drive it directly from the TDI pin.
                  vectorToDut_o(vectorToDut_o'low) <= tdi_s;
                  activateClk_r                    <= HI;
                when others =>
                  if shiftReg_r(0) = NO then  -- Shifting in data from host before applying it to the DUT. 
                    shiftReg_r(vectorToDut_o'length-1 downto 0) <= tdi_s & shiftReg_r(vectorToDut_o'length-1 downto 1);
                  else  -- Vector from host received, now apply it to the DUT.
                    vectorToDut_o                      <= tdi_s & shiftReg_r(vectorToDut_o'length-1 downto 1);  -- Output test vector to DUT.
                    activateClk_r                      <= HI;  -- Pulse vector clock for one cycle after vector is received.
                    -- Clear shift register so it can receive another vector from the host.
                    shiftReg_r                         <= (others => ZERO);  -- Clear all shift register bits and ...
                    shiftReg_r(vectorToDut_o'length-1) <= HI;  -- ... set MSbit so we can tell when all bits are received.
                  end if;
              end case;

            when READ_OPCODE_C =>  -- Get results of a test vector from the DUT.
              if bitCntr_r /= 0 then    -- Shifting DUT outputs to the host.
                shiftReg_r <= ZERO & shiftReg_r(shiftReg_r'high downto 1);  -- Shift register contents.
                bitCntr_r  <= bitCntr_r - 1;  -- One more bit has been sent to the host.
              else  -- Load the DUT output bits into the shift register.
                shiftReg_r(vectorFromDut_i'length-1 downto 0) <= vectorFromDut_i;  -- Load the DUT outputs into the host shift register.
                bitCntr_r                                     <= vectorFromDut_i'length - 1;  -- Set the number of bits to send.
              end if;

            -- Default case is NOP.
            when others =>
              null;
              
          end case;
        end if;
        
      else  -- Reset everything when this module is not selected or is reset.
        opcode_r                <= (others => ZERO);  -- Clear all opcode shift register bits and ...
        opcode_r(opcode_r'high) <= ONE;  -- ... set MSbit so we can tell when a complete opcode is received.
        opcodeRcvd_r            <= NO;
        shiftReg_r              <= (others => ZERO);  -- Clear all shift register bits and ...
        if vectorToDut_o'length > 0 then
          shiftReg_r(vectorToDut_o'length-1) <= ONE;  -- ... set MSbit so we can tell when all bits are received.
        end if;
        bitCntr_r <= 0;
        if reset_i = HI then  -- Only reset input vector to DUT when a hard reset occurs.
          vectorToDut_o <= (others => OUTPUT_RESET_VALUE_G);
        end if;
      end if;
    end if;

  end process;

  -- The inputs to the DUT are changed and the clock-enable signal is set on the rising edge of the DRCK.
  -- The following statement generates a rising clock edge to the DUT when DRCK goes low. This gives
  -- the inputs to the DUT time to settle.
  clkToDut_o <= not drck_s when activateClk_r = HI else LO;

  -- Force output low if this module has not been selected. 
  -- This allows the bit outputs of multiple modules to be OR'ed together
  -- and sent to the TDO input of the BSCAN primitive.
  tdo_s <= shiftReg_r(0) when active_s = YES else LO;
  
end architecture;



--**********************************************************************
-- HostIo Test Jig
--
-- Test of modules for passing bits back and forth from the host PC
-- to FPGA application logic through the JTAG port.
--**********************************************************************

library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_ARITH.all;
use IEEE.STD_LOGIC_UNSIGNED.all;
use XESS.CommonPckg.all;
use XESS.ClkgenPckg.all;
use XESS.HostIoPckg.all;

---- Uncomment the following library declaration if instantiating
---- any Xilinx primitives in this code.
library UNISIM;
use UNISIM.VComponents.all;

entity hostio_test is
  port (
    fpgaClk_i : in  std_logic;
    chan_io   : out std_logic_vector(7 downto 0)
    );
end hostio_test;


architecture Behavioral of hostio_test is
  
  signal clk_s          : std_logic;
  signal inShiftDr_s    : std_logic;
  signal drck_s         : std_logic;
  signal tdi_s          : std_logic;
  signal tdo_s          : std_logic;
  signal tdoReg_s       : std_logic;
  signal reg_r          : std_logic_vector(31 downto 0) := "11011110101011011011111011101111";
  signal addrReg_s      : std_logic_vector(0 downto 0);
  signal dataToReg_s    : std_logic_vector(reg_r'range);
  signal wrReg_s        : std_logic;
  signal tdoBram_s      : std_logic;
  signal addrBram_s     : std_logic_vector(9 downto 0);
  signal dataToBram_s   : std_logic_vector(15 downto 0);
  signal dataFromBram_s : std_logic_vector(15 downto 0);
  signal wrBram_s       : std_logic;
  signal tdoCntr_s      : std_logic;
  signal cntr_r         : std_logic_vector(3 downto 0)  := "1011";
  signal cntrUpDn_s     : std_logic_vector(0 downto 0);
  signal cntrClk_s      : std_logic;
  signal tdoSub_s       : std_logic;
  signal toSub_s        : std_logic_vector(15 downto 0);
  signal difference_s   : std_logic_vector(8 downto 0);

begin

  -- Generate a faster clock from the 12 MHz clock for the FPGA application logic.
  UClkGen : ClkGen generic map (CLK_MUL_G => 3, CLK_DIV_G => 3) port map (I => fpgaClk_i, O => clk_s);

  -- This is the main entry point for the JTAG signals that communicate with this design.
  UBscanToHostIo : BscanToHostIo
    port map (
      inShiftDr_o => inShiftDr_s,  -- True when bits are shifting between the PC host and the FPGA.
      drck_o      => drck_s,            -- Bit shift clock.
      tdi_o       => tdi_s,             -- Bits from the host PC.
      tdo_i       => tdo_s              -- Bits that go back to the host PC.
      );

  -- OR the bits from all the user's modules and send them back to the PC.
  -- (Non-selected modules pull their TDO outputs low, so only bits from the active module are transferred.)
  tdo_s <= tdoReg_s or tdoBram_s or tdoCntr_s or tdoSub_s;

  -- This module interfaces a single register to the JTAG port so that it can be read/written by the PC host.
  UHostIoToReg : HostIoToRam
    generic map (
      ID_G => "00000001"  -- The identifier used by the PC host to access this module.
      )
    port map (
      -- Connections to the JTAG signals.
      inShiftDr_i    => inShiftDr_s,
      drck_i         => drck_s,
      tdi_i          => tdi_s,
      tdo_o          => tdoReg_s,  -- Bits from the attached register back to the PC host.
      -- Interface to the memory.
      addr_o         => addrReg_s,  -- This is just a dummy output to set the bus-width of this unconstrained output.
      clk_i          => clk_s,  -- Put this interface in the same clock domain as the register.
      wr_o           => wrReg_s,
      dataFromHost_o => dataToReg_s,  -- Get new register contents from PC host here.
      dataToHost_i   => reg_r   -- Send register contents back to PC host here.
      );

  -- This is the simple readable/writable register that interfaces to the module above.
  process(clk_s)
  begin
    if rising_edge(clk_s) then
      if wrReg_s = HI then
        reg_r <= dataToReg_s;
      end if;
    end if;
  end process;

  -- This module interfaces a block RAM to the JTAG port so that it can be read/written by the PC host.
  UHostIoToBram : HostIoToRam
    generic map (
      ID_G => "00000010"  -- The identifier used by the PC host to access this module.
      )
    port map (
      inShiftDr_i    => inShiftDr_s,
      drck_i         => drck_s,
      tdi_i          => tdi_s,
      tdo_o          => tdoBram_s,  -- Bits from the attached BRAM to the PC host.
      -- Interface to the memory.
      clk_i          => clk_s,  -- Put this interface in the same clock domain as the BRAM.
      addr_o         => addrBram_s,
      wr_o           => wrBram_s,
      dataFromHost_o => dataToBram_s,
      dataToHost_i   => dataFromBram_s
      );

  -- This is the block RAM that interfaces to the module above.
  URAMB16_S18 : RAMB16_S18
    generic map (
      INIT       => X"000000000",  --  Value of output RAM registers at startup
      SRVAL      => X"000000000",       --  Ouput value upon SSR assertion
      write_mode => "WRITE_FIRST"  --  WRITE_FIRST, READ_FIRST or NO_CHANGE
      )
    port map (
      DO   => dataFromBram_s,           -- 32-bit Data Output
      ADDR => addrBram_s,               -- 9-bit Address Input
      CLK  => clk_s,                    -- Clock
      DI   => dataToBram_s,             -- 32-bit Data Input
      DIP  => "00",
      EN   => HI,                       -- RAM Enable Input
      SSR  => LO,                       -- Synchronous Set/Reset Input
      WE   => wrBram_s                  -- Write Enable Input
      );

  -- This module interfaces a counter to the JTAG port so that it can be read/modified by the PC host.
  UHostIoToCntr : HostIoToDut
    generic map (
      ID_G => "00000011"  -- The identifier used by the PC host to access this module.
      )
    port map (
      inShiftDr_i     => inShiftDr_s,
      drck_i          => drck_s,
      tdi_i           => tdi_s,
      tdo_o           => tdoCntr_s,
      -- Test vector I/O.
      clkToDut_o      => cntrClk_s,
      vectorToDut_o   => cntrUpDn_s,
      vectorFromDut_i => cntr_r
      );

  -- This is the counter that interfaces to the module above.
  process(cntrClk_s)
  begin
    if rising_edge(cntrClk_s) then
      case cntrUpDn_s is
        when "1" =>
          cntr_r <= cntr_r + 1;
        when others =>
          cntr_r <= cntr_r - 1;
      end case;
    end if;
  end process;

  -- This module interfaces a subtractor to the JTAG port so that it can be exercised by the PC host.
  UHostIoToSubtractor : HostIoToDut
    generic map (
      ID_G => "00000100"  -- The identifier used by the PC host to access this module.
      )
    port map (
      inShiftDr_i     => inShiftDr_s,
      drck_i          => drck_s,
      tdi_i           => tdi_s,
      tdo_o           => tdoSub_s,
      -- Test vector I/O.
      vectorToDut_o   => toSub_s,
      vectorFromDut_i => difference_s
      );

  -- This is the subtractor that interfaces to the module above.
  difference_s <= ('0' & toSub_s(7 downto 0)) - toSub_s(15 downto 8);

  -- Output a byte of the test register so we can probe it externally.
  chan_io(7 downto 0) <= reg_r(7 downto 0);

end Behavioral;
