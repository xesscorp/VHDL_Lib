--**********************************************************************
-- Copyright (c) 2011-2015 by XESS Corp <http://www.xess.com>.
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
-- Modules for generating a clock frequency from a master clock and for transferring
-- a clock signal from the clock network to a logic input or an output pin.
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.all;

package ClkGenPckg is

  --**********************************************************************
  -- Generate a clock frequency from a master clock.
  --**********************************************************************
  component ClkGen is
    generic (
      BASE_FREQ_G : real                  := 12.0;  -- Input frequency in MHz.
      CLK_MUL_G   : natural range 1 to 32 := 25;    -- Frequency multiplier.
      CLK_DIV_G   : natural range 1 to 32 := 3      -- Frequency divider.
      );
    port (
      i            : in  std_logic;     -- Clock input (12 MHz by default).
      o            : out std_logic;  -- Generated clock output (100 MHz by default).
      o_b          : out std_logic;  -- Negative-phase generated clock output (inverse of 'o' output).
      clkToLogic_o : out std_logic  -- Clock signal that can go to an output pin or logic-gate input.
      );
  end component;

  --**********************************************************************
  -- Send a clock signal to an output pin or some logic that's not
  -- on an FPGA clock network.
  --**********************************************************************
  component ClkToLogic is
    port (
      clk_i  : in  std_logic;           -- Positive-phase of clock input.
      clk_ib : in  std_logic;           -- Negative-phase of clock input.
      clk_o  : out std_logic  -- Clock output that's suitable as a logic input.
      );
  end component;

  --**********************************************************************
  -- Send a clock signal to an output pin or some logic that's not
  -- on an FPGA clock network. This uses some flip-flops and an XOR
  -- to generate the output clock, so it's edges won't be very close
  -- to the edges of the input clock.  
  --**********************************************************************
  component DirtyClkToLogic is
    port (
      clk_i : in  std_logic;            -- Clock input.
      clk_o : out std_logic   -- Clock output that's suitable as a logic input.
      );
  end component;

  --**********************************************************************
  -- Generate a slow clock from a faster clock.
  --**********************************************************************
  component SlowClkGen is
    generic(
      INPUT_FREQ_G  : real;             -- Input frequency (MHz).
      OUTPUT_FREQ_G : real              -- Output frequency (MHz).
      );
    port(
      clk_i : in  std_logic;            -- Input clock.
      clk_o : out std_logic             -- Output clock.
      );
  end component;
  
end package;


library IEEE, UNISIM, XESS;
use IEEE.STD_LOGIC_1164.all;
use XESS.CommonPckg.all;
use XESS.ClkGenPckg.all;
use UNISIM.VComponents.all;

--**********************************************************************
-- Generate a clock frequency from a master clock.
--**********************************************************************
entity ClkGen is
  generic (
    BASE_FREQ_G : real                  := 12.0;  -- Input frequency in MHz.
    CLK_MUL_G   : natural range 1 to 32 := 25;    -- Frequency multiplier.
    CLK_DIV_G   : natural range 1 to 32 := 3      -- Frequency divider.
    );
  port (
    i            : in  std_logic;       -- Clock input (12 MHz by default).
    o            : out std_logic;  -- Generated clock output (100 MHz by default).
    o_b          : out std_logic;  -- Negative-phase generated clock output (inverse of 'o' output).
    clkToLogic_o : out std_logic  -- Clock signal that can go to an output pin or logic-gate input.
    );
end entity;

architecture arch of ClkGen is
  signal genClkP_s : std_logic;         -- Positive phase of generated clock.
  signal genClkN_s : std_logic;         -- Negative phase of generated clock.
begin

  u0 : DCM_SP
    generic map (
      CLKDV_DIVIDE          => 2.0,
      CLKFX_DIVIDE          => CLK_DIV_G,  --  Can be any interger from 1 to 32
      CLKFX_MULTIPLY        => CLK_MUL_G,  --  Can be any integer from 1 to 32
      CLKIN_DIVIDE_BY_2     => false,  --  TRUE/FALSE to enable CLKIN divide by two feature
      CLKIN_PERIOD          => 1000.0 / BASE_FREQ_G,  --  Specify period of input clock in ns
      CLKOUT_PHASE_SHIFT    => "NONE",  --  Specify phase shift of NONE, FIXED or VARIABLE
      CLK_FEEDBACK          => "NONE",  --  Specify clock feedback of NONE, 1X or 2X
      DESKEW_ADJUST         => "SYSTEM_SYNCHRONOUS",
      DLL_FREQUENCY_MODE    => "LOW",   --  HIGH or LOW frequency mode for DLL
      DUTY_CYCLE_CORRECTION => true,   --  Duty cycle correction, TRUE or FALSE
      PHASE_SHIFT           => 0,  --  Amount of fixed phase shift from -255 to 255
      STARTUP_WAIT          => false)  --  Delay configuration DONE until DCM LOCK, TRUE/FALSE
    port map (
      RST      => '0',                  -- DCM asynchronous reset input
      CLKIN    => i,               -- Clock input (from IBUFG, BUFG or DCM)
      CLKFX    => genClkP_s,       -- Positive-phase of generated clock output
      CLKFX180 => genClkN_s        -- Negative-phase of generated clock output.
      );

  o   <= genClkP_s;
  o_b <= genClkN_s;

  -- Create a clock signal that can go to an output pin or to a logic-gate input.
  u1 : ClkToLogic
    port map (
      clk_i  => genClkP_s,
      clk_ib => genClkN_s,
      clk_o  => clkToLogic_o
      );
end architecture;


library IEEE, UNISIM, XESS;
use IEEE.STD_LOGIC_1164.all;
use XESS.CommonPckg.all;
use UNISIM.VComponents.all;

--**********************************************************************
-- Send a clock signal to an output pin or some logic that's not
-- on an FPGA clock network.
--**********************************************************************
entity ClkToLogic is
  port (
    clk_i  : in  std_logic;             -- Positive-phase of clock input.
    clk_ib : in  std_logic;             -- Negative-phase of clock input.
    clk_o  : out std_logic   -- Clock output that's suitable as a logic input.
    );
end entity;

architecture arch of ClkToLogic is
begin
  -- Use ODDR2 to transfer clock signal from FPGA's clock network to the logic fabric.
  -- (This stops the synthesis tools from complaining about using a clock as an input
  -- to a logic gate or when driving a pin for an external clock signal.)
  u1 : ODDR2
    port map (
      Q  => clk_o,
      C0 => clk_i,
      C1 => clk_ib,
      CE => YES,
      D0 => ONE,
      D1 => ZERO,
      R  => ZERO,
      S  => ZERO
      );
end architecture;


library IEEE, UNISIM, XESS;
use IEEE.STD_LOGIC_1164.all;
use XESS.CommonPckg.all;
use UNISIM.VComponents.all;

--**********************************************************************
-- Send a clock signal to an output pin or some logic that's not
-- on an FPGA clock network. This uses some flip-flops and an XOR
-- to generate the output clock, so it's edges won't be very close
-- to the edges of the input clock.  
--**********************************************************************
entity DirtyClkToLogic is
  port (
    clk_i : in  std_logic;              -- Clock input.
    clk_o : out std_logic   -- Clock output that's suitable as a logic input.
    );
end entity;

architecture arch of DirtyClkToLogic is
begin
  process(clk_i)
    variable phase1_v : std_logic := ZERO;
    variable phase2_v : std_logic := ZERO;
  begin
    if rising_edge(clk_i) then
      phase1_v := not phase1_v;
    end if;
    if falling_edge(clk_i) then
      phase2_v := not phase2_v;
    end if;
    clk_o <= (phase1_v xor phase2_v);
  end process;
end architecture;


library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use IEEE.math_real.all;
use XESS.CommonPckg.all;

--**********************************************************************
-- Generate a slow clock from a faster clock.
--**********************************************************************
entity SlowClkGen is
  generic(
    INPUT_FREQ_G  : real;               -- Input frequency (MHz).
    OUTPUT_FREQ_G : real                -- Output frequency (MHz).
    );
  port(
    clk_i : in  std_logic;              -- Input clock.
    clk_o : out std_logic               -- Output clock.
    );
end entity;

architecture arch of SlowClkGen is
  constant DIVISOR_C  : natural := integer(round(INPUT_FREQ_G / OUTPUT_FREQ_G));
  constant LOW_CNT_C  : natural := DIVISOR_C / 2;
  constant HIGH_CNT_C : natural := DIVISOR_C - LOW_CNT_C;
begin
  process(clk_i)
    variable cnt_v    : natural range 0 to HIGH_CNT_C-1;
    variable output_v : std_logic := LO;
  begin
    if rising_edge(clk_i) then
      if cnt_v = 0 then
        clk_o <= output_v;
        if output_v = LO then
          cnt_v := LOW_CNT_C-1;
        else
          cnt_v := HIGH_CNT_C-1;
        end if;
        output_v := not output_v;
      else
        cnt_v := cnt_v - 1;
      end if;
    end if;
  end process;
end architecture;
