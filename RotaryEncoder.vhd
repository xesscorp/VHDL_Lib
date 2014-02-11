--**********************************************************************
-- Copyright (c) 2012-2014 by XESS Corp <http://www.xess.com>.
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
-- A module for sensing a rotary encoder and outputing an enable
-- pulse and a direction (CW/CCW) of rotation.
--*********************************************************************


library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;

package RotaryEncoderPckg is

  component RotaryEncoder is
    port (
      clk_i : in  std_logic;            -- Input clock.
      a_i   : in  std_logic;            -- A phase of rotary encoder.
      b_i   : in  std_logic;            -- B phase of rotary encoder.
      en_o  : out std_logic;            -- True when rotation increment occurs.
      cw_o  : out std_logic  -- True if rotation is clockwise, false if counter-clockwise.
      );
  end component;

  component RotaryEncoderWithCounter is
    generic (
      ALLOW_ROLLOVER_G : boolean := false;  -- If true, allow counter to rollover from 0xf..ff to 0.
      INITIAL_CNT_G    : integer := 0   -- Initial value of counter.
      );
    port (
      clk_i   : in  std_logic;          -- Input clock.
      reset_i : in  std_logic := NO;    -- Reset.
      a_i     : in  std_logic;          -- A phase of rotary encoder.
      b_i     : in  std_logic;          -- B phase of rotary encoder.
      cnt_o   : out std_logic_vector    -- counter output.
      );
  end component;

end package;




--*********************************************************************
-- A module for sensing a rotary encoder and outputing an enable
-- pulse and a direction (CW/CCW) of rotation.
--*********************************************************************

library IEEE, UNISIM, XESS;
use IEEE.MATH_REAL.all;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use UNISIM.vcomponents.all;
use XESS.CommonPckg.all;

entity RotaryEncoder is
  port (
    clk_i : in  std_logic;              -- Input clock.
    a_i   : in  std_logic;              -- A phase of rotary encoder.
    b_i   : in  std_logic;              -- B phase of rotary encoder.
    en_o  : out std_logic;              -- True when rotation increment occurs.
    cw_o  : out std_logic  -- True if rotation is clockwise, false if counter-clockwise.
    );
end entity;

architecture arch of RotaryEncoder is
  signal aDelay_r : std_logic_vector(2 downto 0);  -- Delay lines for sync'ing A and B inputs to clock.
  signal bDelay_r : std_logic_vector(aDelay_r'range);
begin
  
  process(clk_i)
  begin
    -- Load A and B inputs into delay lines.
    if rising_edge(clk_i) then
      aDelay_r <= a_i & aDelay_r(aDelay_r'high downto 1);
      bDelay_r <= b_i & bDelay_r(bDelay_r'high downto 1);
    end if;
  end process;

  -- Enable output goes high whenever there is a transition on A or B.
  en_o <= aDelay_r(1) xor aDelay_r(0) xor bDelay_r(1) xor bDelay_r(0);
  -- Direction of rotation is determined by current value of A and previous value of B.
  cw_o <= aDelay_r(1) xor bDelay_r(0);
  
end architecture;



--*********************************************************************
-- A module for sensing a rotary encoder and outputing a counter
-- value for the number of rotational increments that have been seen.
--*********************************************************************

library IEEE, UNISIM, XESS;
use IEEE.MATH_REAL.all;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use UNISIM.vcomponents.all;
use XESS.CommonPckg.all;
use XESS.RotaryEncoderPckg.all;

entity RotaryEncoderWithCounter is
  generic (
    ALLOW_ROLLOVER_G : boolean := false;  -- If true, allow counter to rollover from 0xf..ff to 0.
    INITIAL_CNT_G    : integer := 0     -- Initial value of counter.
    );
  port (
    clk_i   : in  std_logic;            -- Input clock.
    reset_i : in  std_logic := NO;      -- Reset.
    a_i     : in  std_logic;            -- A phase of rotary encoder.
    b_i     : in  std_logic;            -- B phase of rotary encoder.
    cnt_o   : out std_logic_vector      -- counter output.
    );
end entity;

architecture arch of RotaryEncoderWithCounter is
  signal cnt_r       : signed(cnt_o'range) := TO_SIGNED(INITIAL_CNT_G, cnt_o'length);
  constant MAX_CNT_C : integer             := 2**(cnt_r'length-1) - 1;
  signal en_s        : std_logic;
  signal cw_s        : std_logic;
begin
  
  u0 : RotaryEncoder
    port map(
      clk_i => clk_i,
      a_i   => a_i,
      b_i   => b_i,
      en_o  => en_s,
      cw_o  => cw_s
      );

  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if reset_i = YES then
        cnt_r <= TO_SIGNED(INITIAL_CNT_G, cnt_o'length);
      elsif en_s = HI then              -- A rotational increment occured.
        if cw_s = YES then              -- A clockwise movement occured.
          if ALLOW_ROLLOVER_G or cnt_r /= MAX_CNT_C then
            cnt_r <= cnt_r + 1;
          end if;
        else                -- A counter-clockwise movement occured.
          if ALLOW_ROLLOVER_G or cnt_r /= 0 then
            cnt_r <= cnt_r - 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  cnt_o <= std_logic_vector(cnt_r);
  
end architecture;



