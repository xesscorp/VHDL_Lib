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
-- Modules for passing bits into a clock domain.
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.all;

package SyncToClockPckg is

  -- Pass a bit into a clock domain.
  component SyncToClock is
    generic (
      NUM_SYNC_STAGES_G : natural := 2
      );
    port (
      clk_i      : in  std_logic;       -- Clock for the domain being entered.
      unsynced_i : in  std_logic;       -- Signal that is entering domain.
      synced_o   : out std_logic        -- Signal sync'ed to clock domain
      );
  end component;

  -- Pass a bus into a clock domain.
  component SyncBusToClock is
    generic (
      NUM_SYNC_STAGES_G : natural := 2
      );
    port (
      clk_i      : in  std_logic;       -- Clock for the domain being entered.
      unsynced_i : in  std_logic_vector;  -- Bus signal that is entering domain.
      synced_o   : out std_logic_vector   -- Bus signal sync'ed to clock domain
      );
  end component;

end package;



library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity SyncToClock is
  generic (
    NUM_SYNC_STAGES_G : natural := 2
    );
  port (
    clk_i      : in  std_logic;         -- Clock for the domain being entered.
    unsynced_i : in  std_logic;         -- Signal that is entering domain.
    synced_o   : out std_logic          -- Signal sync'ed to clock domain
    );
end entity;


architecture arch of SyncToClock is
  -- This is the sync'ing shift register.  The index indicates the number of clocked flip-flops the incoming signal
  -- has passed through, so sync_r(1) is one clk_i cycle stage, sync_r(2) is two cycles, etc.
  signal sync_r : std_logic_vector(NUM_SYNC_STAGES_G downto 1);
begin
  process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- Shift the unsync'ed signal into one end of the sync'ing register.
      sync_r <= sync_r(NUM_SYNC_STAGES_G-1 downto 1) & unsynced_i;
    end if;
  end process;
  -- Output the sync'ed signal from the other end of the shift register.
  synced_o <= sync_r(NUM_SYNC_STAGES_G);
end architecture;



library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use XESS.SyncToClockPckg.all;

entity SyncBusToClock is
  generic (
    NUM_SYNC_STAGES_G : natural := 2
    );
  port (
    clk_i      : in  std_logic;         -- Clock for the domain being entered.
    unsynced_i : in  std_logic_vector;  -- Bus signal that is entering domain.
    synced_o   : out std_logic_vector   -- Bus signal sync'ed to clock domain
    );
end entity;


architecture arch of SyncBusToClock is
begin
  SyncLoop : for i in unsynced_i'range generate
  begin
    USyncBit : SyncToClock
      generic map(NUM_SYNC_STAGES_G => NUM_SYNC_STAGES_G)
      port map(
        clk_i      => clk_i,
        unsynced_i => unsynced_i(i),
        synced_o   => synced_o(i)
        );  
  end generate;
end architecture;
