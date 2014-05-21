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


library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;

package MiscPckg is

  component ResetGenerator is
    generic (
      PULSE_DURATION_G : natural := 1
      );
    port (
      clk_i     : in  std_logic;        -- Master clock.
      trigger_i : in  std_logic := HI;  -- Drive high to trigger reset.
      reset_o   : out std_logic := LO;  -- Active-high reset.
      reset_bo  : out std_logic := HI   -- Active-low reset.
      );
  end component;

  component HandshakeIntfc is
    port (
      ctrl_i : in  std_logic;           -- Control signal from source.
      ctrl_o : out std_logic := LO;     -- Control signal to the destination.
      done_i : in  std_logic;           -- Op done signal from the destination.
      done_o : out std_logic := LO      -- Op done signal to the source.
      );
  end component;

end package;




--**********************************************************************
-- Reset generator.
--
-- Generates an N-clock reset pulse upon the rising edge of the trigger input.
--**********************************************************************

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;

entity ResetGenerator is
  generic (
    PULSE_DURATION_G : natural := 1 -- Reset pulse duration in # of clock cycles.
    );
  port (
    clk_i     : in  std_logic;          -- Master clock.
    trigger_i : in  std_logic := HI;    -- Rising edge of trigger initiates reset pulse.
    reset_o   : out std_logic := LO;    -- Active-high reset.
    reset_bo  : out std_logic := HI     -- Active-low reset.
    );
end entity;

architecture arch of ResetGenerator is
  signal prevTrigger_r : std_logic := LO; -- Stores previous value of trigger for edge detection.
  signal resetCntr_r   : natural range PULSE_DURATION_G-1 downto 0 := 0; -- Counts down reset pulse duration.
begin
  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if prevTrigger_r = LO and trigger_i = HI then
        -- Initiate reset when trigger goes from low to high.
        resetCntr_r <= PULSE_DURATION_G - 1; -- Initialize reset pulse duration counter.
        reset_o     <= HI; -- Initiate active-high reset pulse.
        reset_bo    <= LO; -- Initiate active-low reset pulse.
      elsif resetCntr_r /= 0 then
        -- Decrement pulse duration counter until it reaches zero.
        resetCntr_r <= resetCntr_r - 1;
      else
        -- Pulse duration counter has reached zero, so deactivate the resets.
        reset_o  <= LO;
        reset_bo <= HI;
      end if;
      prevTrigger_r <= trigger_i; -- Store the trigger level so we can detect an edge.
    end if;
  end process;
end architecture;




--*********************************************************************
-- Handshake interface circuit.
--
-- This circuit transforms a control signal into a handshake interface:
-- the source raises the control (ctrl_i), then waits for the done_o
-- to go high, then lowers the control signal, and then the done_o
-- signal goes low. Meanwhile, the control signal that goes to the
-- destination (ctrl_o) is sequenced so that there is no chance
-- of doing a double operation, and the one-cycle done_i signal
-- from the destination is held high by the handshake circuit so the
-- source has a chance to see it. 
--
--   ctrl_i   _____/--------------------------\_____ (From source...)
--   ctrl_o   _____/--------------\_________________ (to destination.)
--   done_i   ____________________/--\______________ (From destination...)
--   done_o   ____________________/-----------\_____ (to source.)
--*********************************************************************

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;
use work.XessBoardPckg.all;

entity HandshakeIntfc is
  port (
    ctrl_i : in  std_logic;             -- Control signal from source.
    ctrl_o : out std_logic := LO;       -- Control signal to the destination.
    done_i : in  std_logic;             -- Op done signal from the destination.
    done_o : out std_logic := LO        -- Op done signal to the source.
    );
end entity;

architecture arch of HandshakeIntfc is
  signal done_r : std_logic := LO;
begin

  process(ctrl_i, done_i, done_r)
  begin

    -- Raise the control to the destination as soon as the source asserts it,
    -- hold it until the destination finishes the operation, and then lower it.
    if ctrl_i = HI and done_i = LO and done_r = LO then
      ctrl_o <= HI;
    elsif done_i = HI or done_r = HI then
      ctrl_o <= LO;
    end if;

    -- Tell the source when the destination finishes doing the operation.
    -- Hold the done signal until the source lowers its control signal.
    if ctrl_i = LO then
      done_r <= LO;
    elsif done_i = HI then
      done_r <= HI;
    end if;

  end process;

  done_o <= done_r;
  
end architecture;
