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


--*********************************************************************
-- Interface to HC-SR04 ultrasonic range finder.
--*********************************************************************


library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;

package Hcsr04Pckg is

--*********************************************************************
-- This module triggers the HC-SR04 and then times the echo pulse to
-- determine the distance to the object.
--*********************************************************************
  component Hcsr04 is
    generic (
      FREQ_G       : real := 12.0;      -- Operating frequency in MHz.
      SENSE_FREQ_G : real := 20.0  -- Number of times distance is sensed per second.
      );
    port (
      clk_i   : in  std_logic;          -- Input clock.
      trig_o  : out std_logic;
      echo_i  : in  std_logic;
      dist_o  : out std_logic_vector;
      clear_o : out std_logic;
      done_o  : out std_logic
      );
  end component;

end package;




--*********************************************************************
-- This module triggers the HC-SR04 and then times the echo pulse to
-- determine the distance to the object.
--*********************************************************************

library IEEE, UNISIM, XESS;
use IEEE.MATH_REAL.all;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use UNISIM.vcomponents.all;
use XESS.CommonPckg.all;
use XESS.Hcsr04Pckg.all;

entity Hcsr04 is
  generic (
    FREQ_G       : real := 12.0;        -- Operating frequency in MHz.
    SENSE_FREQ_G : real := 20.0  -- Number of times distance is sensed per second.
    );
  port (
    clk_i   : in  std_logic;            -- Input clock.
    trig_o  : out std_logic;
    echo_i  : in  std_logic;
    dist_o  : out std_logic_vector;
    clear_o : out std_logic;
    done_o  : out std_logic
    );
end entity;

architecture arch of Hcsr04 is
  constant MAX_ECHO_TIME_C : real    := 38.0;  -- Maximum echo pulse width when no obstructions (ms).
  constant TRIG_WIDTH_C    : real    := 20.0;  -- Trigger width (us).
  constant SENSE_PERIOD_C  : natural := natural(ceil(FREQ_G * 1.0E6 / SENSE_FREQ_G));
  signal timer_r, dist_r   : natural range 0 to SENSE_PERIOD_C;
begin

  process(clk_i)
  begin
    if rising_edge(clk_i) then
      timer_r <= timer_r + 1;
      trig_o  <= LO;
      dist_o  <= std_logic_vector(TO_UNSIGNED(dist_r, dist_o'length));
      clear_o <= NO;
      done_o  <= NO;
      if timer_r < integer(ceil(TRIG_WIDTH_C * FREQ_G)) then
        trig_o <= HI;
        dist_r <= 0;
      elsif echo_i = HI then
        dist_r <= dist_r + 1;
      end if;
      if timer_r = SENSE_PERIOD_C then
        timer_r <= 0;
        done_o  <= YES;
        if dist_r >= integer(ceil(MAX_ECHO_TIME_C * FREQ_G * 1000.0)) then
          clear_o <= YES;
          dist_o  <= (dist_o'range => ONE);
        else
          clear_o <= NO;
        end if;
      end if;
    end if;
  end process;
  
end architecture;




--**********************************************************************
-- HCSR04 Test Jig
--**********************************************************************

library IEEE, XESS;
use IEEE.STD_LOGIC_1164.ALL;
use XESS.CommonPckg.all;
use XESS.Hcsr04Pckg.all;
use XESS.LedDigitsPckg.all;
use work.XessBoardPckg.all;

entity hcsr04_test is
    Port ( clk_i : in  STD_LOGIC;
           trig_o : out  STD_LOGIC;
           echo_i : in  STD_LOGIC;
           s_o : out  STD_LOGIC_VECTOR (7 downto 0));
end hcsr04_test;

architecture Behavioral of hcsr04_test is
  signal dist_s : std_logic_vector(31 downto 0);
begin

-- Get the distance reading from the ultrasonic scanner.
u0 : Hcsr04
    generic map (
      FREQ_G => BASE_FREQ_C
      )
    port map (
      clk_i   => clk_i,
      trig_o  => trig_o,
      echo_i  => echo_i,
      dist_o  => dist_s,
      clear_o => open
      );
      
-- Display the distance reading on the 8-digit LED display.
u1 : LedHexDisplay
  generic map (
    FREQ_G        => BASE_FREQ_C
    )
  port map (
    clk_i          => clk_i,
    hexAllDigits_i => dist_s,
    ledDrivers_o   => s_o
    );

end Behavioral;

