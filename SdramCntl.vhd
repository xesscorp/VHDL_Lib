--**********************************************************************
-- Copyright (c) 2001-2014 by XESS Corp <http://www.xess.com>.
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
-- SDRAM controller and dual-port interface.
--*********************************************************************



library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;
use work.XessBoardPckg.all;

package SdramCntlPckg is

  component SdramCntl is
    generic(
      FREQ_G                 : real    := SDRAM_FREQ_C;  -- Operating frequency in MHz.
      IN_PHASE_G             : boolean := SDRAM_IN_PHASE_C;  -- SDRAM and controller XESS on same or opposite clock edge.
      PIPE_EN_G              : boolean := SDRAM_PIPE_EN_C;  -- If true, enable pipelined read operations.
      MAX_NOP_G              : natural := SDRAM_MAX_NOP_C;  -- Number of NOPs before entering self-refresh.
      ENABLE_REFRESH_G       : boolean := SDRAM_ENABLE_REFRESH_C;  -- If true, row refreshes are automatically inserted.
      MULTIPLE_ACTIVE_ROWS_G : boolean := SDRAM_MULTIPLE_ACTIVE_ROWS_C;  -- If true, allow an active row in each bank.
      -- Parameters for SDRAM.
      DATA_WIDTH_G           : natural := SDRAM_DATA_WIDTH_C;  -- Host & SDRAM data width.
      NROWS_G                : natural := SDRAM_NROWS_C;  -- Number of rows in SDRAM array.
      NCOLS_G                : natural := SDRAM_NCOLS_C;  -- Number of columns in SDRAM array.
      HADDR_WIDTH_G          : natural := SDRAM_HADDR_WIDTH_C;  -- Host-side address width.
      SADDR_WIDTH_G          : natural := SDRAM_SADDR_WIDTH_C;  -- SDRAM-side address width.
      T_INIT_G               : real    := SDRAM_T_INIT_C;  -- min initialization interval (ns).
      T_RAS_G                : real    := SDRAM_T_RAS_C;  -- min interval between active to precharge commands (ns).
      T_RCD_G                : real    := SDRAM_T_RCD_C;  -- min interval between active and R/W commands (ns).
      T_REF_G                : real    := SDRAM_T_REF_C;  -- maximum refresh interval (ns).
      T_RFC_G                : real    := SDRAM_T_RFC_C;  -- duration of refresh operation (ns).
      T_RP_G                 : real    := SDRAM_T_RP_C;  -- min precharge command duration (ns).
      T_XSR_G                : real    := SDRAM_T_XSR_C  -- exit self-refresh time (ns).
      );
    port(
      -- Host side.
      clk_i          : in  std_logic;   -- Master clock.
      lock_i         : in  std_logic                                  := YES;  -- True if clock is stable.
      rst_i          : in  std_logic                                  := NO;  -- Reset.
      rd_i           : in  std_logic                                  := NO;  -- Initiate read operation.
      wr_i           : in  std_logic                                  := NO;  -- Initiate write operation.
      earlyOpBegun_o : out std_logic;  -- Read/write/self-refresh op has begun (async).
      opBegun_o      : out std_logic;  -- Read/write/self-refresh op has begun (clocked).
      rdPending_o    : out std_logic;  -- True if read operation(s) are still in the pipeline.
      done_o         : out std_logic;   -- Read or write operation is done_o.
      rdDone_o       : out std_logic;  -- Read operation is done_o and data is available.
      addr_i         : in  std_logic_vector(HADDR_WIDTH_G-1 downto 0) := (others => ZERO);  -- Address from host to SDRAM.
      data_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);  -- Data from host to SDRAM.
      data_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- Data from SDRAM to host.
      status_o       : out std_logic_vector(3 downto 0);  -- Diagnostic status of the FSM         .

      -- SDRAM side.
      sdCke_o   : out   std_logic;      -- Clock-enable to SDRAM.
      sdCe_bo   : out   std_logic;      -- Chip-select to SDRAM.
      sdRas_bo  : out   std_logic;      -- SDRAM row address strobe.
      sdCas_bo  : out   std_logic;      -- SDRAM column address strobe.
      sdWe_bo   : out   std_logic;      -- SDRAM write enable.
      sdBs_o    : out   std_logic_vector(1 downto 0);  -- SDRAM bank address.
      sdAddr_o  : out   std_logic_vector(SADDR_WIDTH_G-1 downto 0);  -- SDRAM row/column address.
      sdData_io : inout std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- Data to/from SDRAM.
      sdDqmh_o  : out   std_logic;  -- Enable upper-byte of SDRAM databus if true.
      sdDqml_o  : out   std_logic  -- Enable lower-byte of SDRAM databus if true.
      );
  end component;

  component DualPort is
    generic(
      PIPE_EN_G         : boolean                       := SDRAM_PIPE_EN_C;  -- enable pipelined read operations.
      PORT_TIME_SLOTS_G : std_logic_vector(15 downto 0) := "1111000011110000";
      DATA_WIDTH_G      : natural                       := SDRAM_DATA_WIDTH_C;  -- host & SDRAM data width.
      HADDR_WIDTH_G     : natural                       := SDRAM_HADDR_WIDTH_C  -- host-side address width.
      );
    port(
      clk_i : in std_logic;             -- master clock.

      -- Host-side port 0.
      rst0_i          : in  std_logic                                  := NO;  -- reset.
      rd0_i           : in  std_logic                                  := NO;  -- initiate read operation.
      wr0_i           : in  std_logic                                  := NO;  -- initiate write operation.
      earlyOpBegun0_o : out std_logic;  -- read/write op has begun (async).
      opBegun0_o      : out std_logic                                  := NO;  -- read/write op has begun (clocked).
      rdPending0_o    : out std_logic;  -- true if read operation(s) are still in the pipeline.
      done0_o         : out std_logic;  -- read or write operation is done_i.
      rdDone0_o       : out std_logic;  -- read operation is done_i and data is available.
      addr0_i         : in  std_logic_vector(HADDR_WIDTH_G-1 downto 0) := (others => ZERO);  -- address from host to SDRAM.
      data0_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);  -- data from host to SDRAM.
      data0_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);  -- data from SDRAM to host.
      status0_o       : out std_logic_vector(3 downto 0);  -- diagnostic status of the SDRAM controller FSM         .

      -- Host-side port 1.
      rst1_i          : in  std_logic                                  := NO;
      rd1_i           : in  std_logic                                  := NO;
      wr1_i           : in  std_logic                                  := NO;
      earlyOpBegun1_o : out std_logic;
      opBegun1_o      : out std_logic                                  := NO;
      rdPending1_o    : out std_logic;
      done1_o         : out std_logic;
      rdDone1_o       : out std_logic;
      addr1_i         : in  std_logic_vector(HADDR_WIDTH_G-1 downto 0) := (others => ZERO);
      data1_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);
      data1_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);
      status1_o       : out std_logic_vector(3 downto 0);

      -- SDRAM controller host-side port.
      rst_o          : out std_logic;
      rd_o           : out std_logic;
      wr_o           : out std_logic;
      earlyOpBegun_i : in  std_logic;
      opBegun_i      : in  std_logic;
      rdPending_i    : in  std_logic;
      done_i         : in  std_logic;
      rdDone_i       : in  std_logic;
      addr_o         : out std_logic_vector(HADDR_WIDTH_G-1 downto 0);
      data_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0);
      data_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0);
      status_i       : in  std_logic_vector(3 downto 0)
      );
  end component;

  component DualPortSdram is
    generic(
      PORT_TIME_SLOTS_G      : std_logic_vector(15 downto 0) := "1111000011110000";
      FREQ_G                 : real                          := SDRAM_FREQ_C;  -- Operating frequency in MHz.
      IN_PHASE_G             : boolean                       := SDRAM_IN_PHASE_C;  -- SDRAM and controller XESS on same or opposite clock edge.
      PIPE_EN_G              : boolean                       := SDRAM_PIPE_EN_C;  -- If true, enable pipelined read operations.
      MAX_NOP_G              : natural                       := SDRAM_MAX_NOP_C;  -- Number of NOPs before entering self-refresh.
      ENABLE_REFRESH_G       : boolean                       := SDRAM_ENABLE_REFRESH_C;  -- If true, row refreshes are automatically inserted.
      MULTIPLE_ACTIVE_ROWS_G : boolean                       := SDRAM_MULTIPLE_ACTIVE_ROWS_C;  -- If true, allow an active row in each bank.
      -- Parameters for SDRAM.
      DATA_WIDTH_G           : natural                       := SDRAM_DATA_WIDTH_C;  -- Host & SDRAM data width.
      NROWS_G                : natural                       := SDRAM_NROWS_C;  -- Number of rows in SDRAM array.
      NCOLS_G                : natural                       := SDRAM_NCOLS_C;  -- Number of columns in SDRAM array.
      HADDR_WIDTH_G          : natural                       := SDRAM_HADDR_WIDTH_C;  -- Host-side address width.
      SADDR_WIDTH_G          : natural                       := SDRAM_SADDR_WIDTH_C;  -- SDRAM-side address width.
      T_INIT_G               : real                          := SDRAM_T_INIT_C;  -- min initialization interval (ns).
      T_RAS_G                : real                          := SDRAM_T_RAS_C;  -- min interval between active to precharge commands (ns).
      T_RCD_G                : real                          := SDRAM_T_RCD_C;  -- min interval between active and R/W commands (ns).
      T_REF_G                : real                          := SDRAM_T_REF_C;  -- maximum refresh interval (ns).
      T_RFC_G                : real                          := SDRAM_T_RFC_C;  -- duration of refresh operation (ns).
      T_RP_G                 : real                          := SDRAM_T_RP_C;  -- min precharge command duration (ns).
      T_XSR_G                : real                          := SDRAM_T_XSR_C  -- exit self-refresh time (ns).
      );
    port(
      clk_i : in std_logic;             -- master clock.

      -- Host-side port 0.
      rst0_i          : in  std_logic                                  := NO;  -- reset.
      rd0_i           : in  std_logic                                  := NO;  -- initiate read operation.
      wr0_i           : in  std_logic                                  := NO;  -- initiate write operation.
      earlyOpBegun0_o : out std_logic;  -- read/write op has begun (async).
      opBegun0_o      : out std_logic                                  := NO;  -- read/write op has begun (clocked).
      rdPending0_o    : out std_logic;  -- true if read operation(s) are still in the pipeline.
      done0_o         : out std_logic;  -- read or write operation is done_i.
      rdDone0_o       : out std_logic;  -- read operation is done_i and data is available.
      addr0_i         : in  std_logic_vector(HADDR_WIDTH_G-1 downto 0) := (others => ZERO);  -- address from host to SDRAM.
      data0_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);  -- data from host to SDRAM.
      data0_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);  -- data from SDRAM to host.
      status0_o       : out std_logic_vector(3 downto 0);  -- diagnostic status of the SDRAM controller FSM         .

      -- Host-side port 1.
      rst1_i          : in  std_logic                                  := NO;
      rd1_i           : in  std_logic                                  := NO;
      wr1_i           : in  std_logic                                  := NO;
      earlyOpBegun1_o : out std_logic;
      opBegun1_o      : out std_logic                                  := NO;
      rdPending1_o    : out std_logic;
      done1_o         : out std_logic;
      rdDone1_o       : out std_logic;
      addr1_i         : in  std_logic_vector(HADDR_WIDTH_G-1 downto 0) := (others => ZERO);
      data1_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);
      data1_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);
      status1_o       : out std_logic_vector(3 downto 0);

      -- SDRAM side.
      sdCke_o   : out   std_logic;      -- Clock-enable to SDRAM.
      sdCe_bo   : out   std_logic;      -- Chip-select to SDRAM.
      sdRas_bo  : out   std_logic;      -- SDRAM row address strobe.
      sdCas_bo  : out   std_logic;      -- SDRAM column address strobe.
      sdWe_bo   : out   std_logic;      -- SDRAM write enable.
      sdBs_o    : out   std_logic_vector(1 downto 0);  -- SDRAM bank address.
      sdAddr_o  : out   std_logic_vector(SADDR_WIDTH_G-1 downto 0);  -- SDRAM row/column address.
      sdData_io : inout std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- Data to/from SDRAM.
      sdDqmh_o  : out   std_logic;  -- Enable upper-byte of SDRAM databus if true.
      sdDqml_o  : out   std_logic  -- Enable lower-byte of SDRAM databus if true.
      );
  end component;

  component RWHandshake is
    port (
      rd_i   : in  std_logic;           -- Read control signal.
      rd_o   : out std_logic;  -- Read control signal that goes to host-side of SDRAM controller.
      wr_i   : in  std_logic;           -- Write control signal.
      wr_o   : out std_logic;  -- Write control signal that goes to host-side of SDRAM controller.
      done_i : in  std_logic;  -- R/W op done signal from host-side of SDRAM controller.
      done_o : out std_logic            -- R/W done status signal.
      );
  end component;

end package;



--*********************************************************************
-- SDRAM controller.
--*********************************************************************

library IEEE, UNISIM, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use XESS.CommonPckg.all;
use work.XessBoardPckg.all;

entity SdramCntl is
  generic(
    FREQ_G                 : real    := SDRAM_FREQ_C;  -- Operating frequency in MHz.
    IN_PHASE_G             : boolean := SDRAM_IN_PHASE_C;  -- SDRAM and controller XESS on same or opposite clock edge.
    PIPE_EN_G              : boolean := SDRAM_PIPE_EN_C;  -- If true, enable pipelined read operations.
    MAX_NOP_G              : natural := SDRAM_MAX_NOP_C;  -- Number of NOPs before entering self-refresh.
    ENABLE_REFRESH_G       : boolean := SDRAM_ENABLE_REFRESH_C;  -- If true, row refreshes are automatically inserted.
    MULTIPLE_ACTIVE_ROWS_G : boolean := SDRAM_MULTIPLE_ACTIVE_ROWS_C;  -- If true, allow an active row in each bank.
    -- Parameters for SDRAM.
    DATA_WIDTH_G           : natural := SDRAM_DATA_WIDTH_C;  -- Host & SDRAM data width.
    NROWS_G                : natural := SDRAM_NROWS_C;  -- Number of rows in SDRAM array.
    NCOLS_G                : natural := SDRAM_NCOLS_C;  -- Number of columns in SDRAM array.
    HADDR_WIDTH_G          : natural := SDRAM_HADDR_WIDTH_C;  -- Host-side address width.
    SADDR_WIDTH_G          : natural := SDRAM_SADDR_WIDTH_C;  -- SDRAM-side address width.
    T_INIT_G               : real    := SDRAM_T_INIT_C;  -- min initialization interval (ns).
    T_RAS_G                : real    := SDRAM_T_RAS_C;  -- min interval between active to precharge commands (ns).
    T_RCD_G                : real    := SDRAM_T_RCD_C;  -- min interval between active and R/W commands (ns).
    T_REF_G                : real    := SDRAM_T_REF_C;  -- maximum refresh interval (ns).
    T_RFC_G                : real    := SDRAM_T_RFC_C;  -- duration of refresh operation (ns).
    T_RP_G                 : real    := SDRAM_T_RP_C;  -- min precharge command duration (ns).
    T_XSR_G                : real    := SDRAM_T_XSR_C  -- exit self-refresh time (ns).
    );
  port(
    -- Host side.
    clk_i          : in  std_logic;     -- Master clock.
    lock_i         : in  std_logic                                  := YES;  -- True if clock is stable.
    rst_i          : in  std_logic                                  := NO;  -- Reset.
    rd_i           : in  std_logic                                  := NO;  -- Initiate read operation.
    wr_i           : in  std_logic                                  := NO;  -- Initiate write operation.
    earlyOpBegun_o : out std_logic;  -- Read/write/self-refresh op has begun (async).
    opBegun_o      : out std_logic;  -- Read/write/self-refresh op has begun (clocked).
    rdPending_o    : out std_logic;  -- True if read operation(s) are still in the pipeline.
    done_o         : out std_logic;     -- Read or write operation is done_o.
    rdDone_o       : out std_logic;  -- Read operation is done_o and data is available.
    addr_i         : in  std_logic_vector(HADDR_WIDTH_G-1 downto 0) := (others => ZERO);  -- Address from host to SDRAM.
    data_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);  -- Data from host to SDRAM.
    data_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- Data from SDRAM to host.
    status_o       : out std_logic_vector(3 downto 0);  -- Diagnostic status of the FSM         .

    -- SDRAM side.
    sdCke_o   : out   std_logic;        -- Clock-enable to SDRAM.
    sdCe_bo   : out   std_logic;        -- Chip-select to SDRAM.
    sdRas_bo  : out   std_logic;        -- SDRAM row address strobe.
    sdCas_bo  : out   std_logic;        -- SDRAM column address strobe.
    sdWe_bo   : out   std_logic;        -- SDRAM write enable.
    sdBs_o    : out   std_logic_vector(1 downto 0);  -- SDRAM bank address.
    sdAddr_o  : out   std_logic_vector(SADDR_WIDTH_G-1 downto 0);  -- SDRAM row/column address.
    sdData_io : inout std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- Data to/from SDRAM.
    sdDqmh_o  : out   std_logic;  -- Enable upper-byte of SDRAM databus if true.
    sdDqml_o  : out   std_logic  -- Enable lower-byte of SDRAM databus if true.
    );
end entity;



architecture arch of SdramCntl is

  constant OUTPUT_C : std_logic := '1';  -- direction of dataflow w.r.t. this controller.
  constant INPUT_C  : std_logic := '0';
  constant NOP_C    : std_logic := '0';  -- no operation.
  constant READ_C   : std_logic := '1';  -- read operation.
  constant WRITE_C  : std_logic := '1';  -- write operation.

  -- SDRAM timing parameters converted into clock cycles (based on FREQ_G).
  constant FREQ_GHZ_C    : real    := FREQ_G/1000.0;  -- GHz = 1/ns.
  constant INIT_CYCLES_C : natural := integer(ceil(T_INIT_G*FREQ_GHZ_C));  -- SDRAM power-on initialization interval.
  constant RAS_CYCLES_C  : natural := integer(ceil(T_RAS_G*FREQ_GHZ_C));  -- active-to-precharge interval.
  constant RCD_CYCLES_C  : natural := integer(ceil(T_RCD_G*FREQ_GHZ_C));  -- active-to-R/W interval.
  constant REF_CYCLES_C  : natural := integer(ceil(T_REF_G*FREQ_GHZ_C/real(NROWS_G)));  -- interval between row refreshes.
  constant RFC_CYCLES_C  : natural := integer(ceil(T_RFC_G*FREQ_GHZ_C));  -- refresh operation interval.
  constant RP_CYCLES_C   : natural := integer(ceil(T_RP_G*FREQ_GHZ_C));  -- precharge operation interval.
  constant WR_CYCLES_C   : natural := 2;  -- write recovery time.
  constant XSR_CYCLES_C  : natural := integer(ceil(T_XSR_G*FREQ_GHZ_C));  -- exit self-refresh time.
  constant MODE_CYCLES_C : natural := 2;  -- mode register setup time.
  constant CAS_CYCLES_C  : natural := 3;  -- CAS latency.
  constant RFSH_OPS_C    : natural := 8;  -- number of refresh operations needed to init SDRAM.

  -- timer registers that count down times for various SDRAM operations.
  signal timer_r, timer_x       : natural range 0 to INIT_CYCLES_C := 0;  -- current SDRAM op time.
  signal rasTimer_r, rasTimer_x : natural range 0 to RAS_CYCLES_C  := 0;  -- active-to-precharge time.
  signal wrTimer_r, wrTimer_x   : natural range 0 to WR_CYCLES_C   := 0;  -- write-to-precharge time.
  signal refTimer_r, refTimer_x : natural range 0 to REF_CYCLES_C  := REF_CYCLES_C;  -- time between row refreshes.
  signal rfshCntr_r, rfshCntr_x : natural range 0 to NROWS_G       := 0;  -- counts refreshes that are neede.
  signal nopCntr_r, nopCntr_x   : natural range 0 to MAX_NOP_G     := 0;  -- counts consecutive NOP_C operations.

  signal doSelfRfsh_s : std_logic;  -- active when the NOP counter hits zero and self-refresh can start.

  -- states of the SDRAM controller state machine.
  type CntlStateType is (
    INITWAIT,  -- initialization - waiting for power-on initialization to complete.
    INITPCHG,  -- initialization - initial precharge of SDRAM banks.
    INITSETMODE,                        -- initialization - set SDRAM mode.
    INITRFSH,  -- initialization - do initial refreshes.
    RW,                                 -- read/write/refresh the SDRAM.
    ACTIVATE,  -- open a row of the SDRAM for reading/writing.
    REFRESHROW,                         -- refresh a row of the SDRAM.
    SELFREFRESH   -- keep SDRAM in self-refresh mode with CKE low.
    );
  signal state_r, state_x : CntlStateType := INITWAIT;  -- state register and next state.

  -- commands that are sent to the SDRAM to make it perform certain operations.
  -- commands use these SDRAM input pins (ce_bo,ras_bo,cas_bo,we_bo,dqmh_o,dqml_o).
  subtype sdramCmdType is unsigned(5 downto 0);
  constant NOP_CMD_C    : SdramCmdType := "011100";
  constant ACTIVE_CMD_C : SdramCmdType := "001100";
  constant READ_CMD_C   : SdramCmdType := "010100";
  constant WRITE_CMD_C  : SdramCmdType := "010000";
  constant PCHG_CMD_C   : SdramCmdType := "001000";
  constant MODE_CMD_C   : SdramCmdType := "000000";
  constant RFSH_CMD_C   : SdramCmdType := "000100";

  -- SDRAM mode register.
  -- the SDRAM is placed in a non-burst mode (burst length = 1) with a 3-cycle CAS.
  subtype SdramModeType is std_logic_vector(11 downto 0);
  constant MODE_C : SdramModeType := "00" & "0" & "00" & "011" & "0" & "000";

  -- the host address is decomposed into these sets of SDRAM address components.
  constant ROW_LEN_C : natural := Log2(NROWS_G);  -- number of row address bits.
  constant COL_LEN_C : natural := Log2(NCOLS_G);  -- number of column address bits.
  signal bank_s      : std_logic_vector(sdBs_o'range);    -- bank address bits.
  signal row_s       : std_logic_vector(ROW_LEN_C - 1 downto 0);  -- row address within bank.
  signal col_s       : std_logic_vector(sdAddr_o'range);  -- column address within row.

  -- registers that store the currently active row in each bank of the SDRAM.
  constant NUM_ACTIVE_ROWS_C        : integer                                    := IntSelect(MULTIPLE_ACTIVE_ROWS_G = false, 1, 2**sdBs_o'length);
  type ActiveRowType is array(0 to NUM_ACTIVE_ROWS_C-1) of std_logic_vector(row_s'range);
  signal activeRow_r, activeRow_x   : ActiveRowType;
  signal activeFlag_r, activeFlag_x : std_logic_vector(0 to NUM_ACTIVE_ROWS_C-1) := (others => NO);  -- indicates that some row in a bank is active.
  signal bankIndex_s                : natural range 0 to NUM_ACTIVE_ROWS_C-1;  -- bank address bits.
  signal activeBank_r, activeBank_x : std_logic_vector(sdBs_o'range);  -- indicates the bank with the active row.
  signal doActivate_s               : std_logic;  -- indicates when a new row in a bank needs to be activated.

  -- there is a command bit embedded within the SDRAM column address.
  constant CMDBIT_POS_C    : natural   := 10;   -- position of command bit.
  constant AUTO_PCHG_ON_C  : std_logic := '1';  -- CMDBIT value to auto-precharge the bank.
  constant AUTO_PCHG_OFF_C : std_logic := '0';  -- CMDBIT value to disable auto-precharge.
  constant ONE_BANK_C      : std_logic := '0';  -- CMDBIT value to select one bank.
  constant ALL_BANKS_C     : std_logic := '1';  -- CMDBIT value to select all banks.

  -- status signals that indicate when certain operations are in progress.
  signal wrInProgress_s       : std_logic;  -- write operation in progress.
  signal rdInProgress_s       : std_logic;  -- read operation in progress.
  signal activateInProgress_s : std_logic;  -- row activation is in progress.

  -- these registers track the progress of read and write operations.
  signal rdPipeline_r, rdPipeline_x : std_logic_vector(CAS_CYCLES_C+1 downto 0) := (others => '0');  -- pipeline of read ops in progress.
  signal wrPipeline_r, wrPipeline_x : std_logic_vector(0 downto 0)              := (others => '0');  -- pipeline of write ops (only need 1 cycle).

  -- registered outputs to host.
  signal opBegun_r, opBegun_x                     : std_logic                      := NO;  -- true when SDRAM read or write operation is started.
  signal sdramData_r, sdramData_x                 : std_logic_vector(data_o'range) := (others => '0');  -- holds data read from SDRAM and sent to the host.
  signal sdramDataOppPhase_r, sdramDataOppPhase_x : std_logic_vector(data_o'range);  -- holds data read from SDRAM on opposite clock edge.

  -- registered outputs to SDRAM.
  signal cke_r, cke_x           : std_logic                         := NO;  -- Clock-enable bit.
  signal cmd_r, cmd_x           : SdramCmdType                      := NOP_CMD_C;  -- SDRAM command bits.
  signal ba_r, ba_x             : std_logic_vector(sdBs_o'range)    := (others => '0');  -- SDRAM bank address bits.
  signal sAddr_r, sAddr_x       : std_logic_vector(sdAddr_o'range)  := (others => '0');  -- SDRAM row/column address.
  signal sData_r, sData_x       : std_logic_vector(sdData_io'range) := (others => '0');  -- SDRAM out databus.
  signal sDataDir_r, sDataDir_x : std_logic                         := INPUT_C;  -- SDRAM databus direction control bit.

begin

  --*********************************************************************
  -- attach some internal signals to the I/O ports 
  --*********************************************************************

  -- attach registered SDRAM control signals to SDRAM input pins
  (sdCe_bo, sdRas_bo, sdCas_bo, sdWe_bo, sdDqmh_o, sdDqml_o) <= cmd_r;  -- SDRAM operation control bits
  sdCke_o                                                    <= cke_r;  -- SDRAM clock enable
  sdBs_o                                                     <= ba_r;  -- SDRAM bank address
  sdAddr_o                                                   <= sAddr_r;  -- SDRAM address
  sdData_io                                                  <= sData_r when sDataDir_r = OUTPUT_C else (others => 'Z');  -- SDRAM output data bus
  -- attach some port signals
  data_o                                                     <= sdramData_r;  -- data back to host
  opBegun_o                                                  <= opBegun_r;  -- true if requested operation has begun


  --*********************************************************************
  -- compute the next state and outputs 
  --*********************************************************************

  combinatorial : process(rd_i, wr_i, addr_i, data_i, sdramData_r, sdData_io, state_r, opBegun_x,
                          activeFlag_r, activeRow_r, activeBank_r, rdPipeline_r, wrPipeline_r,
                          wrInProgress_s, rdInProgress_s, activateInProgress_s, doActivate_s, doSelfRfsh_s,
                          sdramDataOppPhase_r, nopCntr_r, lock_i, rfshCntr_r, timer_r, rasTimer_r,
                          wrTimer_r, refTimer_r, cmd_r, row_s, col_s, ba_r, ba_x, bank_s, bankIndex_s, cke_r)
  begin

    --*********************************************************************
    -- setup default values for signals 
    --*********************************************************************

    opBegun_x      <= NO;               -- no operations have begun
    earlyOpBegun_o <= opBegun_x;
    cke_x          <= YES;              -- enable SDRAM clock
    cmd_x          <= NOP_CMD_C;        -- set SDRAM command to no-operation
    sDataDir_x     <= INPUT_C;          -- accept data from the SDRAM
    sData_x        <= data_i(sData_x'range);  -- output data from host to SDRAM
    state_x        <= state_r;          -- reload these registers and flags
    activeFlag_x   <= activeFlag_r;  --              with their existing values
    activeRow_x    <= activeRow_r;
    activeBank_x   <= activeBank_r;
    rfshCntr_x     <= rfshCntr_r;

    --*********************************************************************
    -- setup default value for the SDRAM address 
    --*********************************************************************

    -- extract bank field from host address
    ba_x <= addr_i(sdBs_o'length + ROW_LEN_C + COL_LEN_C - 1 downto ROW_LEN_C + COL_LEN_C);
    if MULTIPLE_ACTIVE_ROWS_G = true then
      bank_s      <= (others => '0');
      bankIndex_s <= CONV_INTEGER(ba_x);
    else
      bank_s      <= ba_x;
      bankIndex_s <= 0;
    end if;
    -- extract row, column fields from host address
    row_s                       <= addr_i(ROW_LEN_C + COL_LEN_C - 1 downto COL_LEN_C);
    -- extend column (if needed) until it is as large as the (SDRAM address bus - 1)
    col_s                       <= (others => '0');  -- set it to all zeroes
    col_s(COL_LEN_C-1 downto 0) <= addr_i(COL_LEN_C-1 downto 0);
    -- by default, set SDRAM address to the column address with interspersed
    -- command bit set to disable auto-precharge
    sAddr_x                     <= col_s(col_s'high-1 downto CMDBIT_POS_C) & AUTO_PCHG_OFF_C
                                   & col_s(CMDBIT_POS_C-1 downto 0);

    --*********************************************************************
    -- manage the read and write operation pipelines
    --*********************************************************************

    -- determine if read operations are in progress by the presence of
    -- READ flags in the read pipeline 
    if rdPipeline_r(rdPipeline_r'high downto 1) /= 0 then
      rdInProgress_s <= YES;
    else
      rdInProgress_s <= NO;
    end if;
    rdPending_o <= rdInProgress_s;  -- tell the host if read operations are in progress

    -- enter NOPs into the read and write pipeline shift registers by default
    rdPipeline_x    <= NOP_C & rdPipeline_r(rdPipeline_r'high downto 1);
    wrPipeline_x(0) <= NOP_C;

    -- transfer data from SDRAM to the host data register if a read flag has exited the pipeline
    -- (the transfer occurs 1 cycle before we tell the host the read operation is done)
    if rdPipeline_r(1) = READ_C then
      sdramDataOppPhase_x <= sdData_io(data_o'range);  -- gets value on the SDRAM databus on the opposite phase
      if IN_PHASE_G then
        -- get the SDRAM data for the host directly from the SDRAM if the controller and SDRAM are in-phase
        sdramData_x <= sdData_io(data_o'range);
      else
        -- otherwise get the SDRAM data that was gathered on the previous opposite clock edge
        sdramData_x <= sdramDataOppPhase_r(data_o'range);
      end if;
    else
      -- retain contents of host data registers if no data from the SDRAM has arrived yet
      sdramDataOppPhase_x <= sdramDataOppPhase_r;
      sdramData_x         <= sdramData_r;
    end if;

    done_o   <= rdPipeline_r(0) or wrPipeline_r(0);  -- a read or write operation is done
    rdDone_o <= rdPipeline_r(0);  -- SDRAM data available when a READ flag exits the pipeline 

    --*********************************************************************
    -- manage row activation
    --*********************************************************************

    -- request a row activation operation if the row of the current address
    -- does not match the currently active row in the bank, or if no row
    -- in the bank is currently active
    if (bank_s /= activeBank_r) or (row_s /= activeRow_r(bankIndex_s)) or (activeFlag_r(bankIndex_s) = NO) then
      doActivate_s <= YES;
    else
      doActivate_s <= NO;
    end if;

    --*********************************************************************
    -- manage self-refresh
    --*********************************************************************

    -- enter self-refresh if neither a read or write is requested for MAX_NOP consecutive cycles.
    if (rd_i = YES) or (wr_i = YES) then
      -- any read or write resets NOP counter and exits self-refresh state
      nopCntr_x    <= 0;
      doSelfRfsh_s <= NO;
    elsif nopCntr_r /= MAX_NOP_G then
      -- increment NOP counter whenever there is no read or write operation 
      nopCntr_x    <= nopCntr_r + 1;
      doSelfRfsh_s <= NO;
    else
      -- start self-refresh when counter hits maximum NOP count and leave counter unchanged
      nopCntr_x    <= nopCntr_r;
      doSelfRfsh_s <= YES;
    end if;

    --*********************************************************************
    -- update the timers 
    --*********************************************************************

    -- row activation timer
    if rasTimer_r /= 0 then
      -- decrement a non-zero timer and set the flag
      -- to indicate the row activation is still inprogress
      rasTimer_x           <= rasTimer_r - 1;
      activateInProgress_s <= YES;
    else
      -- on timeout, keep the timer at zero     and reset the flag
      -- to indicate the row activation operation is done
      rasTimer_x           <= rasTimer_r;
      activateInProgress_s <= NO;
    end if;

    -- write operation timer            
    if wrTimer_r /= 0 then
      -- decrement a non-zero timer and set the flag
      -- to indicate the write operation is still inprogress
      wrTimer_x      <= wrTimer_r - 1;
      wrInPRogress_s <= YES;
    else
      -- on timeout, keep the timer at zero and reset the flag that
      -- indicates a write operation is in progress
      wrTimer_x      <= wrTimer_r;
      wrInPRogress_s <= NO;
    end if;

    -- refresh timer            
    if refTimer_r /= 0 then
      refTimer_x <= refTimer_r - 1;
    else
      -- on timeout, reload the timer with the interval between row refreshes
      -- and increment the counter for the number of row refreshes that are needed
      refTimer_x <= REF_CYCLES_C;
      if ENABLE_REFRESH_G then
        rfshCntr_x <= rfshCntr_r + 1;
      else
        rfshCntr_x <= 0;  -- refresh never occurs if this counter never gets above zero
      end if;
    end if;

    -- main timer for sequencing SDRAM operations               
    if timer_r /= 0 then
      -- decrement the timer and do nothing else since the previous operation has not completed yet.
      timer_x  <= timer_r - 1;
      status_o <= "0000";
    else
      -- the previous operation has completed once the timer hits zero
      timer_x <= timer_r;               -- by default, leave the timer at zero

      --*********************************************************************
      -- compute the next state and outputs 
      --*********************************************************************
      case state_r is

        --*********************************************************************
        -- let clock stabilize and then wait for the SDRAM to initialize 
        --*********************************************************************
        when INITWAIT =>
          if lock_i = YES then
            -- wait for SDRAM power-on initialization once the clock is stable
            timer_x <= INIT_CYCLES_C;  -- set timer for initialization duration
            state_x <= INITPCHG;
          else
            -- disable SDRAM clock and return to this state if the clock is not stable
            -- this insures the clock is stable before enabling the SDRAM
            -- it also insures a clean startup if the SDRAM is currently in self-refresh mode
            cke_x <= NO;
          end if;
          status_o <= "0001";

        --*********************************************************************
        -- precharge all SDRAM banks after power-on initialization 
        --*********************************************************************
        when INITPCHG =>
          cmd_x                 <= PCHG_CMD_C;
          sAddr_x(CMDBIT_POS_C) <= ALL_BANKS_C;  -- precharge all banks
          timer_x               <= RP_CYCLES_C;  -- set timer for precharge operation duration
          rfshCntr_x            <= RFSH_OPS_C;  -- set counter for refresh ops needed after precharge
          state_x               <= INITRFSH;
          status_o              <= "0010";

        --*********************************************************************
        -- refresh the SDRAM a number of times after initial precharge 
        --*********************************************************************
        when INITRFSH =>
          cmd_x      <= RFSH_CMD_C;
          timer_x    <= RFC_CYCLES_C;  -- set timer to refresh operation duration
          rfshCntr_x <= rfshCntr_r - 1;  -- decrement refresh operation counter
          if rfshCntr_r = 1 then
            state_x <= INITSETMODE;  -- set the SDRAM mode once all refresh ops are done
          end if;
          status_o <= "0011";

        --*********************************************************************
        -- set the mode register of the SDRAM 
        --*********************************************************************
        when INITSETMODE =>
          cmd_x                 <= MODE_CMD_C;
          sAddr_x               <= (others => '0');
          sAddr_x(MODE_C'range) <= MODE_C;  -- output mode register bits on the SDRAM address bits
          timer_x               <= MODE_CYCLES_C;  -- set timer for mode setting operation duration
          state_x               <= RW;
          status_o              <= "0100";

        --*********************************************************************
        -- process read/write/refresh operations after initialization is done 
        --*********************************************************************
        when RW =>
          --*********************************************************************
          -- highest priority operation: row refresh 
          -- do a refresh operation if the refresh counter is non-zero
          --*********************************************************************
          if rfshCntr_r /= 0 then
            -- wait for any row activations, writes or reads to finish before doing a precharge
            if (activateInProgress_s = NO) and (wrInProgress_s = NO) and (rdInProgress_s = NO) then
              cmd_x                 <= PCHG_CMD_C;  -- initiate precharge of the SDRAM
              sAddr_x(CMDBIT_POS_C) <= ALL_BANKS_C;  -- precharge all banks
              timer_x               <= RP_CYCLES_C;  -- set timer for this operation
              activeFlag_x          <= (others => NO);  -- all rows are inactive after a precharge operation
              state_x               <= REFRESHROW;  -- refresh the SDRAM after the precharge
            end if;
            status_o <= "0101";
          --*********************************************************************
          -- do a host-initiated read operation 
          --*********************************************************************
          elsif rd_i = YES then
            -- Wait one clock cycle if the bank address has just changed and each bank has its own active row.
            -- This gives extra time for the row activation circuitry.
            if (ba_x = ba_r) or (MULTIPLE_ACTIVE_ROWS_G = false) then
              -- activate a new row if the current read is outside the active row or bank
              if doActivate_s = YES then
                -- activate new row only if all previous activations, writes, reads are done
                if (activateInProgress_s = NO) and (wrInProgress_s = NO) and (rdInProgress_s = NO) then
                  cmd_x                     <= PCHG_CMD_C;  -- initiate precharge of the SDRAM
                  sAddr_x(CMDBIT_POS_C)     <= ONE_BANK_C;  -- precharge this bank
                  timer_x                   <= RP_CYCLES_C;  -- set timer for this operation
                  activeFlag_x(bankIndex_s) <= NO;  -- rows in this bank are inactive after a precharge operation
                  state_x                   <= ACTIVATE;  -- activate the new row after the precharge is done
                end if;
              -- read from the currently active row if no previous read operation
              -- is in progress or if pipeline reads are enabled
              -- we can always initiate a read even if a write is already in progress
              elsif (rdInProgress_s = NO) or PIPE_EN_G then
                cmd_x        <= READ_CMD_C;   -- initiate a read of the SDRAM
                -- insert a flag into the pipeline shift register that will exit the end
                -- of the shift register when the data from the SDRAM is available
                rdPipeline_x <= READ_C & rdPipeline_r(rdPipeline_r'high downto 1);
                opBegun_x    <= YES;  -- tell the host the requested operation has begun
              end if;
            end if;
            status_o <= "0110";
          --*********************************************************************
          -- do a host-initiated write operation 
          --*********************************************************************
          elsif wr_i = YES then
            -- Wait one clock cycle if the bank address has just changed and each bank has its own active row.
            -- This gives extra time for the row activation circuitry.
            if (ba_x = ba_r) or (MULTIPLE_ACTIVE_ROWS_G = false) then
              -- activate a new row if the current write is outside the active row or bank
              if doActivate_s = YES then
                -- activate new row only if all previous activations, writes, reads are done
                if (activateInProgress_s = NO) and (wrInProgress_s = NO) and (rdInProgress_s = NO) then
                  cmd_x                     <= PCHG_CMD_C;  -- initiate precharge of the SDRAM
                  sAddr_x(CMDBIT_POS_C)     <= ONE_BANK_C;  -- precharge this bank
                  timer_x                   <= RP_CYCLES_C;  -- set timer for this operation
                  activeFlag_x(bankIndex_s) <= NO;  -- rows in this bank are inactive after a precharge operation
                  state_x                   <= ACTIVATE;  -- activate the new row after the precharge is done
                end if;
              -- write to the currently active row if no previous read operations are in progress
              elsif rdInProgress_s = NO then
                cmd_x           <= WRITE_CMD_C;  -- initiate the write operation
                sDataDir_x      <= OUTPUT_C;  -- turn on drivers to send data to SDRAM
                -- set timer so precharge doesn't occur too soon after write operation
                wrTimer_x       <= WR_CYCLES_C;
                -- insert a flag into the 1-bit pipeline shift register that will exit on the
                -- next cycle.  The write into SDRAM is not actually done by that time, but
                -- this doesn't matter to the host
                wrPipeline_x(0) <= WRITE_C;
                opBegun_x       <= YES;  -- tell the host the requested operation has begun
              end if;
            end if;
            status_o <= "0111";
          --*********************************************************************
          -- do a host-initiated self-refresh operation 
          --*********************************************************************
          elsif doSelfRfsh_s = YES then
            -- wait until all previous activations, writes, reads are done
            if (activateInProgress_s = NO) and (wrInProgress_s = NO) and (rdInProgress_s = NO) then
              cmd_x                 <= PCHG_CMD_C;  -- initiate precharge of the SDRAM
              sAddr_x(CMDBIT_POS_C) <= ALL_BANKS_C;  -- precharge all banks
              timer_x               <= RP_CYCLES_C;  -- set timer for this operation
              activeFlag_x          <= (others => NO);  -- all rows are inactive after a precharge operation
              state_x               <= SELFREFRESH;  -- self-refresh the SDRAM after the precharge
            end if;
            status_o <= "1000";
          --*********************************************************************
          -- no operation
          --*********************************************************************
          else
            state_x  <= RW;  -- continue to look for SDRAM operations to execute
            status_o <= "1001";
          end if;

        --*********************************************************************
        -- activate a row of the SDRAM 
        --*********************************************************************
        when ACTIVATE =>
          cmd_x                     <= ACTIVE_CMD_C;
          sAddr_x                   <= (others => '0');  -- output the address for the row to be activated
          sAddr_x(row_s'range)      <= row_s;
          activeBank_x              <= bank_s;
          activeRow_x(bankIndex_s)  <= row_s;  -- store the new active SDRAM row address
          activeFlag_x(bankIndex_s) <= YES;    -- the SDRAM is now active
          rasTimer_x                <= RAS_CYCLES_C;  -- minimum time before another precharge can occur 
          timer_x                   <= RCD_CYCLES_C;  -- minimum time before a read/write operation can occur
          state_x                   <= RW;  -- return to do read/write operation that initiated this activation
          status_o                  <= "1010";

        --*********************************************************************
        -- refresh a row of the SDRAM         
        --*********************************************************************
        when REFRESHROW =>
          cmd_x      <= RFSH_CMD_C;
          timer_x    <= RFC_CYCLES_C;   -- refresh operation interval
          rfshCntr_x <= rfshCntr_r - 1;  -- decrement the number of needed row refreshes
          state_x    <= RW;  -- process more SDRAM operations after refresh is done
          status_o   <= "1011";

        --*********************************************************************
        -- place the SDRAM into self-refresh and keep it there until further notice           
        --*********************************************************************
        when SELFREFRESH =>
          if (doSelfRfsh_s = YES) or (lock_i = NO) then
            -- keep the SDRAM in self-refresh mode as long as requested and until there is a stable clock
            cmd_x <= RFSH_CMD_C;  -- output the refresh command; this is only needed on the first clock cycle
            cke_x <= NO;                -- disable the SDRAM clock
          else
            -- else exit self-refresh mode and start processing read and write operations
            cke_x        <= YES;        -- restart the SDRAM clock
            rfshCntr_x   <= 0;  -- no refreshes are needed immediately after leaving self-refresh
            activeFlag_x <= (others => NO);  -- self-refresh deactivates all rows
            timer_x      <= XSR_CYCLES_C;  -- wait this long until read and write operations can resume
            state_x      <= RW;
          end if;
          status_o <= "1100";

        --*********************************************************************
        -- unknown state
        --*********************************************************************
        when others =>
          state_x  <= INITWAIT;         -- reset state if in erroneous state
          status_o <= "1101";

      end case;
    end if;
  end process combinatorial;


  --*********************************************************************
  -- update registers on the appropriate clock edge     
  --*********************************************************************

  update : process(rst_i, clk_i)
  begin

    if rst_i = YES then
      -- asynchronous reset
      state_r      <= INITWAIT;
      activeFlag_r <= (others => NO);
      rfshCntr_r   <= 0;
      timer_r      <= 0;
      refTimer_r   <= REF_CYCLES_C;
      rasTimer_r   <= 0;
      wrTimer_r    <= 0;
      nopCntr_r    <= 0;
      opBegun_r    <= NO;
      rdPipeline_r <= (others => '0');
      wrPipeline_r <= (others => '0');
      cke_r        <= NO;
      cmd_r        <= NOP_CMD_C;
      ba_r         <= (others => '0');
      sAddr_r      <= (others => '0');
      sData_r      <= (others => '0');
      sDataDir_r   <= INPUT_C;
      sdramData_r  <= (others => '0');
    elsif rising_edge(clk_i) then
      state_r      <= state_x;
      activeBank_r <= activeBank_x;
      activeRow_r  <= activeRow_x;
      activeFlag_r <= activeFlag_x;
      rfshCntr_r   <= rfshCntr_x;
      timer_r      <= timer_x;
      refTimer_r   <= refTimer_x;
      rasTimer_r   <= rasTimer_x;
      wrTimer_r    <= wrTimer_x;
      nopCntr_r    <= nopCntr_x;
      opBegun_r    <= opBegun_x;
      rdPipeline_r <= rdPipeline_x;
      wrPipeline_r <= wrPipeline_x;
      cke_r        <= cke_x;
      cmd_r        <= cmd_x;
      ba_r         <= ba_x;
      sAddr_r      <= sAddr_x;
      sData_r      <= sData_x;
      sDataDir_r   <= sDataDir_x;
      sdramData_r  <= sdramData_x;
    end if;

    -- The register that gets data from the SDRAM and holds it for the host
    -- is clocked on the opposite edge.  We don't use this register if IN_PHASE_G=TRUE.
    if rst_i = YES then
      sdramDataOppPhase_r <= (others => '0');
    elsif falling_edge(clk_i) then
      sdramDataOppPhase_r <= sdramDataOppPhase_x;
    end if;

  end process update;

end architecture;




--*********************************************************************
-- Handshake circuit for SDRAM controller R/W control signals.
--
-- This circuit transforms the R/W/done control signals into a handshake
-- interface: you have to raise the control (rd_i or wr_i), then wait for
-- the done_o signal to go high, then lower the control signal, and then
-- the done_o signal goes low. Meanwhile, the control signals that go to
-- the SDRAM controller (rd_o and wr_o) are sequenced so that there is no
-- chance of doing a double read or write, and the one-cycle done_i signal
-- from the SDRAM controller is held high by the handshake circuit so the
-- initiator has a chance to see it. 
--
--         R/W_i   _____/--------------------------\_____
--         R/W_o   _____/--------------\_________________
--         done_i  ____________________/--\______________
--         done_o  ____________________/-----------\_____
--*********************************************************************

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;
use XESS.MiscPckg.all;
use work.XessBoardPckg.all;

entity RWHandshake is
  port (
    rd_i   : in  std_logic;             -- Read control signal.
    rd_o   : out std_logic;  -- Read control signal that goes to host-side of SDRAM controller.
    wr_i   : in  std_logic;             -- Write control signal.
    wr_o   : out std_logic;  -- Write control signal that goes to host-side of SDRAM controller.
    done_i : in  std_logic;  -- R/W op done signal from host-side of SDRAM controller.
    done_o : out std_logic              -- R/W done status signal.
    );
end entity;

architecture arch of RWHandshake is
  signal rdDone_s : std_logic;
  signal wrDone_s : std_logic;
begin

  uRead : HandshakeIntfc
    port map(
      ctrl_i => rd_i,
      ctrl_o => rd_o,
      done_i => done_i,
      done_o => rdDone_s
      );

  uWrite : HandshakeIntfc
    port map(
      ctrl_i => wr_i,
      ctrl_o => wr_o,
      done_i => done_i,
      done_o => wrDone_s
      );

  -- Read and write operations never occur simultaneously, so we can present a
  -- single, unified done signal to the initiator.
  done_o <= rdDone_s or wrDone_s;
  
end architecture;





--*********************************************************************
-- Dual-port interface to SDRAM controller.
--*********************************************************************

library IEEE, UNISIM, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use IEEE.numeric_std.all;
use XESS.CommonPckg.all;
use work.XessBoardPckg.all;

entity DualPort is
  generic(
    PIPE_EN_G         : boolean                       := SDRAM_PIPE_EN_C;  -- enable pipelined read operations.
    PORT_TIME_SLOTS_G : std_logic_vector(15 downto 0) := "1111000011110000";
    DATA_WIDTH_G      : natural                       := SDRAM_DATA_WIDTH_C;  -- host & SDRAM data width.
    HADDR_WIDTH_G     : natural                       := SDRAM_HADDR_WIDTH_C  -- host-side address width.
    );
  port(
    clk_i : in std_logic;               -- master clock.

    -- Host-side port 0.
    rst0_i          : in  std_logic                                  := NO;  -- reset.
    rd0_i           : in  std_logic                                  := NO;  -- initiate read operation.
    wr0_i           : in  std_logic                                  := NO;  -- initiate write operation.
    earlyOpBegun0_o : out std_logic;    -- read/write op has begun (async).
    opBegun0_o      : out std_logic                                  := NO;  -- read/write op has begun (clocked).
    rdPending0_o    : out std_logic;  -- true if read operation(s) are still in the pipeline.
    done0_o         : out std_logic;    -- read or write operation is done_i.
    rdDone0_o       : out std_logic;  -- read operation is done_i and data is available.
    addr0_i         : in  std_logic_vector(HADDR_WIDTH_G-1 downto 0) := (others => ZERO);  -- address from host to SDRAM.
    data0_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);  -- data from host to SDRAM.
    data0_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);  -- data from SDRAM to host.
    status0_o       : out std_logic_vector(3 downto 0);  -- diagnostic status of the SDRAM controller FSM         .

    -- Host-side port 1.
    rst1_i          : in  std_logic                                  := NO;
    rd1_i           : in  std_logic                                  := NO;
    wr1_i           : in  std_logic                                  := NO;
    earlyOpBegun1_o : out std_logic;
    opBegun1_o      : out std_logic                                  := NO;
    rdPending1_o    : out std_logic;
    done1_o         : out std_logic;
    rdDone1_o       : out std_logic;
    addr1_i         : in  std_logic_vector(HADDR_WIDTH_G-1 downto 0) := (others => ZERO);
    data1_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);
    data1_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);
    status1_o       : out std_logic_vector(3 downto 0);

    -- SDRAM controller host-side port.
    rst_o          : out std_logic;
    rd_o           : out std_logic;
    wr_o           : out std_logic;
    earlyOpBegun_i : in  std_logic;
    opBegun_i      : in  std_logic;
    rdPending_i    : in  std_logic;
    done_i         : in  std_logic;
    rdDone_i       : in  std_logic;
    addr_o         : out std_logic_vector(HADDR_WIDTH_G-1 downto 0);
    data_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0);
    data_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0);
    status_i       : in  std_logic_vector(3 downto 0)
    );
end entity;



architecture arch of DualPort is
  -- The door signal controls whether the read/write signal from the active port
  -- is allowed through to the read/write inputs of the SDRAM controller.
  type DoorStateType is (OPENED_C, CLOSED_C);
  signal door_r, door_x : DoorStateType := CLOSED_C;

  -- The port signal indicates which port is connected to the SDRAM controller.
  type PortStateType is (PORT0_C, PORT1_C);
  signal port_r, port_x : PortStateType := PORT0_C;

  signal switch_s                         : std_logic;  -- indicates that the active port should be switched.
  signal inProgress_s                     : std_logic;  -- the active port has a read/write op in-progress.
  signal rd_s                             : std_logic;  -- read signal to the SDRAM controller (internal copy).
  signal wr_s                             : std_logic;  -- write signal to the SDRAM controller (internal copy).
  signal earlyOpBegun0_s, earlyOpBegun1_s : std_logic;  -- (internal copies).
  signal slot_r, slot_x                   : std_logic_vector(PORT_TIME_SLOTS_G'range) := PORT_TIME_SLOTS_G;  -- time-slot allocation shift-register.
begin

  --*********************************************************************
  -- multiplex the SDRAM controller port signals to/from the dual host-side ports  
  --*********************************************************************

  -- send the SDRAM controller the address and data from the currently active port.
  addr_o <= addr0_i when port_r = PORT0_C else addr1_i;
  data_o <= data0_i when port_r = PORT0_C else data1_i;

  -- both ports get the data from the SDRAM but only the active port will use it.
  data0_o <= data_i;
  data1_o <= data_i;

  -- send the SDRAM controller status to the active port and give the inactive port an inactive status code.
  status0_o <= status_i when port_r = PORT0_C else "1111";
  status1_o <= status_i when port_r = PORT1_C else "1111";

  -- either port can reset the SDRAM controller.
  rst_o <= rst0_i or rst1_i;

  -- apply the read signal from the active port to the SDRAM controller only if the door is open..
  rd_s <= rd0_i when (port_r = PORT0_C) and (door_r = OPENED_C) else
          rd1_i when (port_r = PORT1_C) and (door_r = OPENED_C) else
          NO;
  rd_o <= rd_s;

  -- apply the write signal from the active port to the SDRAM controller only if the door is open..
  wr_s <= wr0_i when (port_r = PORT0_C) and (door_r = OPENED_C) else
          wr1_i when (port_r = PORT1_C) and (door_r = OPENED_C) else
          NO;
  wr_o <= wr_s;

  -- send the status signals for various SDRAM controller operations back to the active port.
  earlyOpBegun0_s <= earlyOpBegun_i when port_r = PORT0_C else NO;
  earlyOpBegun0_o <= earlyOpBegun0_s;
  earlyOpBegun1_s <= earlyOpBegun_i when port_r = PORT1_C else NO;
  earlyOpBegun1_o <= earlyOpBegun1_s;
  rdPending0_o    <= rdPending_i    when port_r = PORT0_C else NO;
  rdPending1_o    <= rdPending_i    when port_r = PORT1_C else NO;
  done0_o         <= done_i         when port_r = PORT0_C else NO;
  done1_o         <= done_i         when port_r = PORT1_C else NO;
  rdDone0_o       <= rdDone_i       when port_r = PORT0_C else NO;
  rdDone1_o       <= rdDone_i       when port_r = PORT1_C else NO;

  --*********************************************************************
  -- Indicate when the active port needs to be switched.  A switch occurs if
  -- a read or write operation is requested on the port that is not currently active and:
  -- 1) no R/W operation is being performed on the active port or 
  -- 2) a R/W operation is in progress on the active port, but the time-slot allocation 
  --    register is giving precedence to the inactive port.  (The R/W operation on the
  --    active port will be completed before the switch is made.)
  -- This rule keeps the active port from hogging all the bandwidth.
  --*********************************************************************
  switch_s <= (rd0_i or wr0_i) when (port_r = PORT1_C) and (((rd1_i = NO) and (wr1_i = NO)) or (slot_r(0) = '0')) else
              (rd1_i or wr1_i) when (port_r = PORT0_C) and (((rd0_i = NO) and (wr0_i = NO)) or (slot_r(0) = '1')) else
              NO;

  --*********************************************************************
  -- Indicate when an operation on the active port is in-progress and
  -- can't be interrupted by a switch to the other port.  (Only read operations
  -- are looked at since write operations always complete in one cycle once they
  -- are initiated.)
  --*********************************************************************
  inProgress_s <= rdPending_i or (rd_s and earlyOpBegun_i);

  --*********************************************************************
  -- Update the time-slot allocation shift-register.  The port with priority is indicated by the
  -- least-significant bit of the register.  The register is rotated right if:
  -- 1) the current R/W operation has started, and
  -- 2) both ports are requesting R/W operations (indicating contention), and
  -- 3) the currently active port matches the port that currently has priority.
  -- Under these conditions, the current time slot port allocation has been used so
  -- the shift register is rotated right to bring the next port time-slot allocation
  -- bit into play.
  --*********************************************************************
  slot_x <= slot_r(0) & slot_r(slot_r'high downto 1) when (earlyOpBegun_i = YES) and
            (((rd0_i = YES) or (wr0_i = YES)) and ((rd1_i = YES) or (wr1_i = YES))) and
            (((port_r = PORT0_C) and (slot_r(0) = '0')) or ((port_r = PORT1_C) and (slot_r(0) = '1')))
            else slot_r;

  --*********************************************************************
  -- Determine which port will be active on the next cycle.  The active port is switched if:
  -- 1) there are no pending operations in progress, and
  -- 2) the port switch indicator is active.
  --*********************************************************************
  port_process : process(port_r, inProgress_s, switch_s, done_i)
  begin
    port_x <= port_r;  -- by default, the active port is not changed
    case port_r is
      when PORT0_C =>
        if (inProgress_s = NO) and (switch_s = YES) then
          port_x <= PORT1_C;
        end if;
      when PORT1_C =>
        if (inProgress_s = NO) and (switch_s = YES) then
          port_x <= PORT0_C;
        end if;
      when others =>
        port_x <= port_r;
    end case;
  end process port_process;

  --*********************************************************************
  -- Determine if the door is open for the active port to initiate new R/W operations to
  -- the SDRAM controller.  If the door is open and R/W operations are in progress but
  -- a switch to the other port is indicated, then the door is closed to prevent any
  -- further R/W operations from the active port.  The door is re-opened once all
  -- in-progress operations are completed, at which time the switch to the other port
  -- is also completed so it can issue its own R/W commands.
  --*********************************************************************
  door_process : process(door_r, inProgress_s, switch_s)
  begin
    door_x <= door_r;  -- by default, the door remains as it is.
    case door_r is
      when OPENED_C =>
        if (inProgress_s = YES) and (switch_s = YES) then
          door_x <= CLOSED_C;
        end if;
      when CLOSED_C =>
        if inProgress_s = NO then
          door_x <= OPENED_C;
        end if;
      when others =>
        door_x <= door_r;
    end case;
  end process door_process;

  --*********************************************************************
  -- update registers on the appropriate clock edge.
  --*********************************************************************
  update : process(rst0_i, rst1_i, clk_i)
  begin
    if (rst0_i = YES) or (rst1_i = YES) then
      -- asynchronous reset.
      door_r     <= CLOSED_C;
      port_r     <= PORT0_C;
      slot_r     <= PORT_TIME_SLOTS_G;
      opBegun0_o <= NO;
      opBegun1_o <= NO;
    elsif rising_edge(clk_i) then
      door_r     <= door_x;
      port_r     <= port_x;
      slot_r     <= slot_x;
      --*********************************************************************
      -- opBegun signals are cycle-delayed versions of earlyOpBegun signals.
      -- We can't use the actual opBegun signal from the SDRAM controller
      -- because it would be turned off if the active port was switched on the
      -- cycle immediately after earlyOpBegun went active.
      --*********************************************************************
      opBegun0_o <= earlyOpBegun0_s;
      opBegun1_o <= earlyOpBegun1_s;
    end if;
  end process update;

end architecture;




--*********************************************************************
-- Dual-port interface + SDRAM controller.
--*********************************************************************

library IEEE, UNISIM, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use IEEE.numeric_std.all;
use XESS.CommonPckg.all;
use XESS.SdramCntlPckg.all;
use work.XessBoardPckg.all;

entity DualPortSdram is
  generic(
    PORT_TIME_SLOTS_G      : std_logic_vector(15 downto 0) := "1111000011110000";
    FREQ_G                 : real                          := SDRAM_FREQ_C;  -- Operating frequency in MHz.
    IN_PHASE_G             : boolean                       := SDRAM_IN_PHASE_C;  -- SDRAM and controller XESS on same or opposite clock edge.
    PIPE_EN_G              : boolean                       := SDRAM_PIPE_EN_C;  -- If true, enable pipelined read operations.
    MAX_NOP_G              : natural                       := SDRAM_MAX_NOP_C;  -- Number of NOPs before entering self-refresh.
    ENABLE_REFRESH_G       : boolean                       := SDRAM_ENABLE_REFRESH_C;  -- If true, row refreshes are automatically inserted.
    MULTIPLE_ACTIVE_ROWS_G : boolean                       := SDRAM_MULTIPLE_ACTIVE_ROWS_C;  -- If true, allow an active row in each bank.
    -- Parameters for SDRAM.
    DATA_WIDTH_G           : natural                       := SDRAM_DATA_WIDTH_C;  -- Host & SDRAM data width.
    NROWS_G                : natural                       := SDRAM_NROWS_C;  -- Number of rows in SDRAM array.
    NCOLS_G                : natural                       := SDRAM_NCOLS_C;  -- Number of columns in SDRAM array.
    HADDR_WIDTH_G          : natural                       := SDRAM_HADDR_WIDTH_C;  -- Host-side address width.
    SADDR_WIDTH_G          : natural                       := SDRAM_SADDR_WIDTH_C;  -- SDRAM-side address width.
    T_INIT_G               : real                          := SDRAM_T_INIT_C;  -- min initialization interval (ns).
    T_RAS_G                : real                          := SDRAM_T_RAS_C;  -- min interval between active to precharge commands (ns).
    T_RCD_G                : real                          := SDRAM_T_RCD_C;  -- min interval between active and R/W commands (ns).
    T_REF_G                : real                          := SDRAM_T_REF_C;  -- maximum refresh interval (ns).
    T_RFC_G                : real                          := SDRAM_T_RFC_C;  -- duration of refresh operation (ns).
    T_RP_G                 : real                          := SDRAM_T_RP_C;  -- min precharge command duration (ns).
    T_XSR_G                : real                          := SDRAM_T_XSR_C  -- exit self-refresh time (ns).
    );
  port(
    clk_i : in std_logic;               -- master clock.

    -- Host-side port 0.
    rst0_i          : in  std_logic                                  := NO;  -- reset.
    rd0_i           : in  std_logic                                  := NO;  -- initiate read operation.
    wr0_i           : in  std_logic                                  := NO;  -- initiate write operation.
    earlyOpBegun0_o : out std_logic;    -- read/write op has begun (async).
    opBegun0_o      : out std_logic                                  := NO;  -- read/write op has begun (clocked).
    rdPending0_o    : out std_logic;  -- true if read operation(s) are still in the pipeline.
    done0_o         : out std_logic;    -- read or write operation is done_i.
    rdDone0_o       : out std_logic;  -- read operation is done_i and data is available.
    addr0_i         : in  std_logic_vector(HADDR_WIDTH_G-1 downto 0) := (others => ZERO);  -- address from host to SDRAM.
    data0_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);  -- data from host to SDRAM.
    data0_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);  -- data from SDRAM to host.
    status0_o       : out std_logic_vector(3 downto 0);  -- diagnostic status of the SDRAM controller FSM         .

    -- Host-side port 1.
    rst1_i          : in  std_logic                                  := NO;
    rd1_i           : in  std_logic                                  := NO;
    wr1_i           : in  std_logic                                  := NO;
    earlyOpBegun1_o : out std_logic;
    opBegun1_o      : out std_logic                                  := NO;
    rdPending1_o    : out std_logic;
    done1_o         : out std_logic;
    rdDone1_o       : out std_logic;
    addr1_i         : in  std_logic_vector(HADDR_WIDTH_G-1 downto 0) := (others => ZERO);
    data1_i         : in  std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);
    data1_o         : out std_logic_vector(DATA_WIDTH_G-1 downto 0)  := (others => ZERO);
    status1_o       : out std_logic_vector(3 downto 0);

    -- SDRAM side.
    sdCke_o   : out   std_logic;        -- Clock-enable to SDRAM.
    sdCe_bo   : out   std_logic;        -- Chip-select to SDRAM.
    sdRas_bo  : out   std_logic;        -- SDRAM row address strobe.
    sdCas_bo  : out   std_logic;        -- SDRAM column address strobe.
    sdWe_bo   : out   std_logic;        -- SDRAM write enable.
    sdBs_o    : out   std_logic_vector(1 downto 0);  -- SDRAM bank address.
    sdAddr_o  : out   std_logic_vector(SADDR_WIDTH_G-1 downto 0);  -- SDRAM row/column address.
    sdData_io : inout std_logic_vector(DATA_WIDTH_G-1 downto 0);  -- Data to/from SDRAM.
    sdDqmh_o  : out   std_logic;  -- Enable upper-byte of SDRAM databus if true.
    sdDqml_o  : out   std_logic  -- Enable lower-byte of SDRAM databus if true.
    );
end entity;


architecture arch of DualPortSdram is
  signal rst_s          : std_logic;
  signal rd_s           : std_logic;
  signal wr_s           : std_logic;
  signal earlyOpBegun_s : std_logic;
  signal opBegun_s      : std_logic;
  signal rdPending_s    : std_logic;
  signal done_s         : std_logic;
  signal rdDone_s       : std_logic;
  signal addr_s         : std_logic_vector(addr0_i'range);
  signal dataFromHost_s : std_logic_vector(sdData_io'range);
  signal dataToHost_s   : std_logic_vector(sdData_io'range);
  signal status_s       : std_logic_vector(status0_o'range);
begin

  u0 : DualPort
    generic map(
      PIPE_EN_G         => PIPE_EN_G,
      PORT_TIME_SLOTS_G => PORT_TIME_SLOTS_G,
      DATA_WIDTH_G      => DATA_WIDTH_G,
      HADDR_WIDTH_G     => HADDR_WIDTH_G
      )
    port map(
      clk_i => clk_i,

      -- Host-side port 0.
      rst0_i          => rst0_i,
      rd0_i           => rd0_i,
      wr0_i           => wr0_i,
      earlyOpBegun0_o => earlyOpBegun0_o,
      opBegun0_o      => opBegun0_o,
      rdPending0_o    => rdPending0_o,
      done0_o         => done0_o,
      rdDone0_o       => rdDone0_o,
      addr0_i         => addr0_i,
      data0_i         => data0_i,
      data0_o         => data0_o,
      status0_o       => status0_o,

      -- Host-side port 1.
      rst1_i          => rst1_i,
      rd1_i           => rd1_i,
      wr1_i           => wr1_i,
      earlyOpBegun1_o => earlyOpBegun1_o,
      opBegun1_o      => opBegun1_o,
      rdPending1_o    => rdPending1_o,
      done1_o         => done1_o,
      rdDone1_o       => rdDone1_o,
      addr1_i         => addr1_i,
      data1_i         => data1_i,
      data1_o         => data1_o,
      status1_o       => status1_o,

      -- SDRAM controller host-side port.
      rst_o          => rst_s,
      rd_o           => rd_s,
      wr_o           => wr_s,
      earlyOpBegun_i => earlyOpBegun_s,
      opBegun_i      => opBegun_s,
      rdPending_i    => rdPending_s,
      done_i         => done_s,
      rdDone_i       => rdDone_s,
      addr_o         => addr_s,
      data_o         => dataFromHost_s,
      data_i         => dataToHost_s,
      status_i       => status_s
      );

  u1 : SdramCntl
    generic map(
      FREQ_G        => FREQ_G,
      IN_PHASE_G    => IN_PHASE_G,
      PIPE_EN_G     => PIPE_EN_G,
      MAX_NOP_G     => MAX_NOP_G,
      DATA_WIDTH_G  => DATA_WIDTH_G,
      NROWS_G       => NROWS_G,
      NCOLS_G       => NCOLS_G,
      HADDR_WIDTH_G => HADDR_WIDTH_G,
      SADDR_WIDTH_G => SADDR_WIDTH_G
      )
    port map(
      clk_i          => clk_i,  -- master clock from external clock source (unbuffered)
      lock_i         => YES,   -- no DLLs, so frequency is always locked
      rst_i          => rst_s,          -- reset
      rd_i           => rd_s,  -- host-side SDRAM read control from memory tester
      wr_i           => wr_s,  -- host-side SDRAM write control from memory tester
      earlyOpBegun_o => earlyOpBegun_s,  -- early indicator that memory operation has begun
      opBegun_o      => opBegun_s,    -- indicates memory read/write has begun
      rdPending_o    => rdPending_s,  -- read operation to SDRAM is in progress_o
      done_o         => done_s,  -- SDRAM memory read/write done indicator
      rdDone_o       => rdDone_s,  -- indicates SDRAM memory read operation is done
      addr_i         => addr_s,  -- host-side address from memory tester to SDRAM
      data_i         => dataFromHost_s,  -- test data pattern from memory tester to SDRAM
      data_o         => dataToHost_s,   -- SDRAM data output to memory tester
      status_o       => status_s,  -- SDRAM controller state (for diagnostics)
      sdCke_o        => sdCke_o,        -- Clock-enable to SDRAM.
      sdCe_bo        => sdCe_bo,        -- Chip-select to SDRAM.
      sdRas_bo       => sdRas_bo,       -- SDRAM RAS
      sdCas_bo       => sdCas_bo,       -- SDRAM CAS
      sdWe_bo        => sdWe_bo,        -- SDRAM write-enable
      sdBs_o         => sdBs_o,         -- SDRAM bank address
      sdAddr_o       => sdAddr_o,       -- SDRAM address
      sdData_io      => sdData_io,      -- data to/from SDRAM
      sdDqmh_o       => sdDqmh_o,  -- Enable upper-byte of SDRAM databus if true.
      sdDqml_o       => sdDqml_o  -- Enable lower-byte of SDRAM databus if true.
      );

end architecture;
