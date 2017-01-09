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
-- Modules for generating repetitive or single pulses.
--*********************************************************************

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;

package PulsePckg is

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

--**********************************************************************
-- Generate a triggered pulse with a lockout interval after the trigger.
--**********************************************************************
  component PulseGen is
    generic (
      FREQ_G         : real;            -- Clock frequency (MHz).
      LOCKOUT_TIME_G : real;            -- Lockout time after trigger (us).
      PULSE_WIDTH_G  : real             -- Pulse duration (us).
      );
    port (
      clk_i     : in  std_logic;        -- Input clock.
      trigger_i : in  std_logic;        -- Pulse trigger.
      lockout_o : out std_logic;        -- Lockout indicator.
      pulse_o   : out std_logic         -- Output pulse.
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
use XESS.PulsePckg.all;

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



--**********************************************************************
-- Generate a triggered pulse with a lockout interval after the trigger.
--**********************************************************************

library IEEE, XESS;
use IEEE.MATH_REAL.all;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use XESS.CommonPckg.all;

entity PulseGen is
  generic (
    FREQ_G         : real;              -- Clock frequency (MHz).
    LOCKOUT_TIME_G : real;              -- Lockout time after trigger (us).
    PULSE_WIDTH_G  : real               -- Pulse duration (us).
    );
  port (
    clk_i     : in  std_logic;          -- Input clock.
    trigger_i : in  std_logic;          -- Pulse trigger.
    lockout_o : out std_logic;          -- Lockout indicator.
    pulse_o   : out std_logic           -- Output pulse.
    );
end entity;

architecture arch of PulseGen is
  signal prevTrigger_r  : std_logic;
begin
  process(clk_i)
    constant LOCKOUT_TIME_C : natural := integer(round(FREQ_G * LOCKOUT_TIME_G));
    constant PULSE_WIDTH_C  : natural := integer(round(FREQ_G * PULSE_WIDTH_G));
    variable lockoutCnt_v   : natural range 0 to LOCKOUT_TIME_C;
    variable pulseCnt_v     : natural range 0 to PULSE_WIDTH_C;
  begin
    if rising_edge(clk_i) then
      -- The pulse output is high as long as the pulse counter is
      -- non-zero. Once the pulse counter decrements to zero, the
      -- pulse output is lowered and the counter decrements stop.
      if pulseCnt_v /= 0 then
        pulse_o    <= HI;
        pulseCnt_v := pulseCnt_v - 1;
      else
        pulse_o <= LO;
      end if;
      -- The lockout output is active as long as the lockout counter
      -- is non-zero. Once the lockout counter decrements to zero, the
      -- lockout output is deactivated and the counter decrements stop.
      -- After the lockout expires, triggers will once again be accepted.
      -- A rising edge on the trigger will cause another lockout and will
      -- start the generation of an output pulse. 
      if lockoutCnt_v /= 0 then
        lockout_o    <= HI;
        lockoutCnt_v := lockoutCnt_v - 1;
      else
        lockout_o <= LO;
        if trigger_i = HI and prevTrigger_r = LO then
          lockoutCnt_v := LOCKOUT_TIME_C;
          pulseCnt_v   := PULSE_WIDTH_C;
          pulse_o      <= HI;
        end if;
      end if;
      prevTrigger_r <= trigger_i;
    end if;
  end process;
end architecture;
