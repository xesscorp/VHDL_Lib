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
-- Modules for delaying signals and buses by a given number of clocks.
--**********************************************************************

library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use XESS.CommonPckg.all;

package DelayPckg is

  --**********************************************************************
  -- Module for delaying a signal by a given number of clock cycles.
  --**********************************************************************
  component DelayLine is
    generic (
      NUM_DELAY_CYCLES_G : natural := 1
      );
    port (
      clk_i      : in  std_logic;         -- Master clock.
      cke_i      : in  std_logic := YES;  -- Clock-enable.
      a_i        : in  std_logic;         -- Signal to be delayed.
      aDelayed_o : out std_logic          -- Delayed version of signal.
      );
  end component;

  --**********************************************************************
  -- Module for delaying a bus by a given number of clock cycles.
  --**********************************************************************
  component DelayBus is
    generic (
      NUM_DELAY_CYCLES_G : natural := 1
      );
    port (
      clk_i        : in  std_logic;         -- Master clock.
      cke_i        : in  std_logic := YES;  -- Clock-enable.
      bus_i        : in  std_logic_vector;  -- Signal bus to be delayed.
      busDelayed_o : out std_logic_vector   -- Delayed version of signal bus.
      );
  end component;

end package;




--**********************************************************************
-- Module for delaying a signal by a given number of clock cycles.
--**********************************************************************

library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use XESS.CommonPckg.all;

entity DelayLine is
  generic (
    NUM_DELAY_CYCLES_G : natural := 1
    );
  port (
    clk_i      : in  std_logic;         -- Master clock.
    cke_i      : in  std_logic := YES;  -- Clock-enable.
    a_i        : in  std_logic;         -- Signal to be delayed.
    aDelayed_o : out std_logic          -- Delayed version of signal.
    );
end entity;

architecture arch of DelayLine is
  signal delay_r : std_logic_vector(NUM_DELAY_CYCLES_G downto 1);
begin
  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if cke_i = YES then
        delay_r(1) <= a_i;
        for d in 2 to NUM_DELAY_CYCLES_G loop
          delay_r(d) <= delay_r(d-1);
        end loop;
      end if;
    end if;
  end process;
  aDelayed_o <= delay_r(NUM_DELAY_CYCLES_G);
end architecture;




--**********************************************************************
-- Module for delaying a bus by a given number of clock cycles.
--**********************************************************************

library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use XESS.CommonPckg.all;
use XESS.DelayPckg.all;

entity DelayBus is
  generic (
    NUM_DELAY_CYCLES_G : natural := 1
    );
  port (
    clk_i        : in  std_logic;         -- Master clock.
    cke_i        : in  std_logic := YES;  -- Clock-enable.
    bus_i        : in  std_logic_vector;  -- Signal bus to be delayed.
    busDelayed_o : out std_logic_vector   -- Delayed version of signal bus.
    );
end entity;

architecture arch of DelayBus is
begin
  DlyLoop : for j in bus_i'range generate
  begin
    UDlyBit : DelayLine
      generic map(NUM_DELAY_CYCLES_G => NUM_DELAY_CYCLES_G)
      port map(
        clk_i      => clk_i,
        cke_i      => cke_i,
        a_i        => bus_i(j),
        aDelayed_o => busDelayed_o(j)
        );  
  end generate;
end architecture;
