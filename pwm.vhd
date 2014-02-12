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
-- Pulse-width modulation (PWM) module.
--*********************************************************************


library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;

package PwmPckg is

--*********************************************************************
-- PWM module.
--*********************************************************************
  component Pwm is
    port (
      clk_i  : in  std_logic;           -- Input clock.
      duty_i : in  std_logic_vector;    -- Duty-cycle input.
      pwm_o  : out std_logic            -- PWM output.
      );
  end component;

end package;




--*********************************************************************
-- PWM module.
--*********************************************************************

library IEEE, UNISIM, XESS;
use IEEE.MATH_REAL.all;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use UNISIM.vcomponents.all;
use XESS.CommonPckg.all;
use XESS.PwmPckg.all;

entity Pwm is
  port (
    clk_i  : in  std_logic;             -- Input clock.
    duty_i : in  std_logic_vector;      -- Duty-cycle input.
    pwm_o  : out std_logic              -- PWM output.
    );
end entity;

architecture arch of Pwm is
  constant MAX_DUTY_C : std_logic_vector(duty_i'range) := (duty_i'range => ONE);
  signal timer_r      : natural range 0 to 2**duty_i'length-1;
begin

  process(clk_i)
  begin
    if rising_edge(clk_i) then
      pwm_o   <= LO;
      timer_r <= timer_r + 1;
      if timer_r < TO_INTEGER(unsigned(duty_i)) then
        pwm_o <= HI;
      end if;
    end if;
  end process;
  
end architecture;
