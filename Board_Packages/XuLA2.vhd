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


--*********************************************************************
-- Definitions for the XuLA2 board.
--*********************************************************************


library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;

package XessBoardPckg is

  constant XESS_BOARD_C : XessBoard_t := XULA2_E;

  -- FPGA device family that's used to select the appropriate BSCAN primitive.
  constant FPGA_FAMILY_C : FpgaFamily_t := SPARTAN6_E;

  -- Use one of these to select which USER opcode enables the I/O to the host through the JTAG port.
  -- (I realize there is no USER0 instruction for the Spartan6, but it needs to be in there because
  -- the *position* of the user instruction within the enumeration determines the actual number for
  -- the user instruction in the VHDL code. DON'T TAKE IT OUT!)
  type TapUserInstr_t is (USER0_E, USER1_E, USER2_E, USER3_E, USER4_E);

  constant TAP_USER_INSTR_C : TapUserInstr_t := USER1_E;

  constant BASE_FREQ_C : real := 12.0;  -- Base crystal frequency.

  -- Parameters for Winbond W9825G6JH-75 (all times are in nanoseconds).
  constant SDRAM_NROWS_C       : natural := 8192;  -- Number of rows in SDRAM array.
  constant SDRAM_NCOLS_C       : natural := 512;  -- Number of columns in SDRAM array.
  constant SDRAM_DATA_WIDTH_C  : natural := 16;   -- Host & SDRAM data width.
  constant SDRAM_HADDR_WIDTH_C : natural := 24;   -- Host-side address width.
  constant SDRAM_SADDR_WIDTH_C : natural := 13;   -- SDRAM-side address width.
  constant SDRAM_T_INIT_C      : real    := 200_000.0;  -- Min initialization interval (ns).
  constant SDRAM_T_RAS_C       : real    := 45.0;  -- Min interval between active to precharge commands (ns).
  constant SDRAM_T_RCD_C       : real    := 20.0;  -- Min interval between active and R/W commands (ns).
  constant SDRAM_T_REF_C       : real    := 64_000_000.0;  -- Maximum refresh interval (ns).
  constant SDRAM_T_RFC_C       : real    := 65.0;  -- Duration of refresh operation (ns).
  constant SDRAM_T_RP_C        : real    := 20.0;  -- Min precharge command duration (ns).
  constant SDRAM_T_XSR_C       : real    := 75.0;  -- Exit self-refresh time (ns).

  constant SDRAM_FREQ_C                 : real    := 100.0;  -- Operating frequency in MHz.
  constant SDRAM_IN_PHASE_C             : boolean := true;  -- SDRAM and controller XESS on same or opposite clock edge.
  constant SDRAM_PIPE_EN_C              : boolean := false;  -- If true, enable pipelined read operations.
  constant SDRAM_ENABLE_REFRESH_C       : boolean := true;  -- If true, row refreshes are automatically inserted.
  constant SDRAM_MULTIPLE_ACTIVE_ROWS_C : boolean := false;  -- If true, allow an active row in each bank.
  constant SDRAM_MAX_NOP_C              : natural := 10000;  -- Number of NOPs before entering self-refresh.
  constant SDRAM_BEG_ADDR_C             : natural := 16#00_0000#;  -- Beginning SDRAM address.
  constant SDRAM_END_ADDR_C             : natural := 16#FF_FFFF#;  -- Ending SDRAM address.

end package;
