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
-- Button debouncer module.
--*********************************************************************


library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;

package ButtonDebouncerPckg is

--*********************************************************************
-- Button  Debouncer.
--*********************************************************************

  component ButtonDebouncer is
    generic(
      FREQ_G      : real := 100.0;      -- Master clock frequency in MHz.
      SCAN_FREQ_G : real := 1.0  -- Desired frequency for scanning the button in KHz.
      );
    port(
      clk_i    : in  std_logic;         -- Master clock.
      button_i : in  std_logic;         -- Raw button input.
      button_o : out std_logic          -- Debounced button.
      );
  end component;

end package;




--*********************************************************************
-- Button debouncer.
--*********************************************************************

library IEEE, UNISIM, XESS;
use IEEE.MATH_REAL.all;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use UNISIM.vcomponents.all;
use XESS.CommonPckg.all;
use XESS.ButtonDebouncerPckg.all;

entity ButtonDebouncer is
  generic(
    FREQ_G      : real := 100.0;        -- Master clock frequency in MHz.
    SCAN_FREQ_G : real := 1.0  -- Desired frequency for scanning the button in KHz.
    );
  port(
    clk_i    : in  std_logic;           -- Master clock.
    button_i : in  std_logic;           -- Raw button input.
    button_o : out std_logic            -- Debounced button.
    );
end entity;

architecture arch of ButtonDebouncer is
begin

  process(clk_i)
    constant SCAN_PERIOD_C  : natural   := integer(ceil(FREQ_G * 1000.0 / (SCAN_FREQ_G)));
    variable scanTimer_v    : natural range 0 to SCAN_PERIOD_C;  -- Button scan timer.
    -- The button has to hold its value for a certain number of individual scans before being accepted.
    constant DEBOUNCE_CNT_C : natural   := 10;
    variable debounceCntr_v : natural range 0 to DEBOUNCE_CNT_C;
    variable buttonPrev_v   : std_logic := '0';  -- Previous state of the button.
  begin
    if rising_edge(clk_i) then
      
      if scanTimer_v /= 0 then  -- Wait until current button scan time has elapsed.
        scanTimer_v := scanTimer_v - 1;
        
      else  -- OK, now process the state of the currently-driven buttons.
        
        scanTimer_v := SCAN_PERIOD_C;   -- Reload the timer for the next scan.

        if button_i /= buttonPrev_v then
          -- If the button's state has changed from its previous value,
          -- then record its current value and reset the debounce counter.
          buttonPrev_v   := button_i;
          debounceCntr_v := DEBOUNCE_CNT_C;
        else
          -- If the button's state has not changed, then just decrement the debounce counter.
          debounceCntr_v := debounceCntr_v - 1;
          if debounceCntr_v = 0 then
            -- If the debounce counter has reached 0, then output the current state of the button.
            button_o       <= button_i;
            debounceCntr_v := DEBOUNCE_CNT_C;  -- Reset the debounce counter for the next scan.
          end if;
        end if;

      end if;
    end if;
  end process;

end architecture;
