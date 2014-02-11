--**********************************************************************
-- Copyright (c) 2013-2014 by XESS Corp <http://www.xess.com>.
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

package SpiPckg is

  component SpiMaster is
    generic (
      FREQ_G     : real      := 100.0;  -- Main clock frequency (MHz).
      SPI_FREQ_G : real      := 25.0;   -- SPI clock frequency (MHz).
      CPOL_G     : std_logic := LO;  -- SCK polarity (0=normally low, 1=normally high).
      CPHA_G     : std_logic := HI  -- SCK phase (0=sample on leading edge, 1=sample on trailing edge).
      );
    port (
      rst_i   : in  std_logic := NO;    -- Active-high reset input.
      clk_i   : in  std_logic;          -- Master clock input.
      addr_i  : in  std_logic_vector(1 downto 0);  -- Register address from PC.
      data_i  : in  std_logic_vector(7 downto 0);  -- Data from PC.
      data_o  : out std_logic_vector(7 downto 0);  -- Data to PC.
      wr_i    : in  std_logic := LO;    -- Write control from PC.
      rd_i    : in  std_logic := LO;    -- Read control from PC.
      begun_o : out std_logic;          -- True when R/W operation has begun.
      done_o  : out std_logic;          -- True when R/W operation completed.
      ssel_o  : out std_logic;          -- SPI chip-select line.
      sck_o   : out std_logic;          -- SPI clock line.
      mosi_o  : out std_logic;          -- SPI data from master to slave.
      miso_i  : in  std_logic := LO     -- SPI data from slave to master.
      );
  end component;

end package;



library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use XESS.CommonPckg.all;
--library unisim;
--use unisim.vcomponents.all;

entity SpiMaster is
  generic (
    FREQ_G     : real      := 100.0;    -- Main clock frequency (MHz).
    SPI_FREQ_G : real      := 25.0;     -- SPI clock frequency (MHz).
    CPOL_G     : std_logic := LO;  -- SCK polarity (0=normally low, 1=normally high).
    CPHA_G     : std_logic := HI  -- SCK phase (0=sample on leading edge, 1=sample on trailing edge).
    );
  port (
    rst_i   : in  std_logic := NO;      -- Active-high reset input.
    clk_i   : in  std_logic;            -- Master clock input.
    addr_i  : in  std_logic_vector(1 downto 0);  -- Register address from PC.
    data_i  : in  std_logic_vector(7 downto 0);  -- Data from PC.
    data_o  : out std_logic_vector(7 downto 0);  -- Data to PC.
    wr_i    : in  std_logic := LO;      -- Write control from PC.
    rd_i    : in  std_logic := LO;      -- Read control from PC.
    begun_o : out std_logic;            -- True when R/W operation has begun.
    done_o  : out std_logic;            -- True when R/W operation completed.
    ssel_o  : out std_logic;            -- SPI chip-select line.
    sck_o   : out std_logic;            -- SPI clock line.
    mosi_o  : out std_logic;            -- SPI data from master to slave.
    miso_i  : in  std_logic := LO       -- SPI data from slave to master.
    );
end entity;

architecture arch of SpiMaster is
  subtype regAddr_t is std_logic_vector(addr_i'range);
  constant RESET_ADDR            : regAddr_t := "00";
  constant SINGLE_XFER_ADDR      : regAddr_t := "01";
  constant MULTI_XFER_ADDR       : regAddr_t := "10";
  constant CLKS_PER_SCLK_PHASE_C : natural   := integer(round(0.5 * FREQ_G / SPI_FREQ_G));
  constant DISABLE_TIME_C        : natural   := 10;
  constant ENABLE_SETUP_TIME_C   : natural   := 5;
  constant ENABLE_HOLD_TIME_C    : natural   := 7;

  signal sck_r     : std_logic := CPOL_G;
  signal sr_r      : std_logic_vector(data_i'range);
  signal enabled_r : boolean;
begin

  process (clk_i)
    variable rst_v              : boolean := false;
    variable sclkPhaseTimer_v   : natural range 0 to CLKS_PER_SCLK_PHASE_C;
    variable sselTimer_v        : natural range 0 to DISABLE_TIME_C;
    variable disableAfterXfer_v : boolean;
    variable bitCntr_v          : natural range 0 to sr_r'length;
  begin
    if rising_edge(clk_i) then

      -- These acknowledgement signals are off by default.
      done_o  <= NO;
      begun_o <= NO;

      -- Reset the SPI module.
      if rst_i = YES or rst_v then
        rst_v              := false;
        sck_r              <= CPOL_G;
        sclkPhaseTimer_v   := 0;
        sselTimer_v        := 0;
        bitCntr_v          := 0;
        enabled_r          <= false;
        disableAfterXfer_v := false;

      -- Time the setup or hold for the chip-select pulse, or the
      -- duration of the deselect time between operations.
      elsif sselTimer_v /= 0 then
        sselTimer_v := sselTimer_v - 1;
        
      elsif enabled_r and bitCntr_v = 0 and disableAfterXfer_v then
        enabled_r          <= false;
        disableAfterXfer_v := false;
        sselTimer_v        := DISABLE_TIME_C - 1;

      -- If bit xfer is not active, then look for R/W operations from the host.
      elsif bitCntr_v = 0 then
        if wr_i = YES or rd_i = YES then
          if addr_i = RESET_ADDR then
            rst_v   := true;
            begun_o <= YES;
            done_o  <= YES;
          else
            sclkPhaseTimer_v   := CLKS_PER_SCLK_PHASE_C - 1;
            sr_r               <= data_i;
            begun_o            <= YES;
            enabled_r          <= true;
            sselTimer_v        := ENABLE_SETUP_TIME_C;
            disableAfterXfer_v := false;
            if addr_i = SINGLE_XFER_ADDR then
              disableAfterXfer_v := true;
            end if;
            bitCntr_v := sr_r'length;
          end if;
        end if;

      -- Wait until another transition of the SPI clock is needed.
      elsif sclkPhaseTimer_v /= 0 then
        sclkPhaseTimer_v := sclkPhaseTimer_v - 1;

      -- Generate SPI clock transition and handle bit xfer.
      else
        if sck_r = (CPOL_G xor CPHA_G) then
          -- Sample data from the slave on the leading edge of SCK when CPHA_G=0
          -- and the trailing edge when CPHA_G=1.
          sr_r <= sr_r(sr_r'high-1 downto 0) & miso_i;
        else
          -- Propagate data to the slave on the opposite edge of SCK.
          mosi_o <= sr_r(sr_r'high);
        end if;
        -- Decrement bit counter on the trailing edge of SCK.
        if sck_r = not CPOL_G then
          if bitCntr_v /= 0 then
            if bitCntr_v = 1 then
              done_o <= YES;
              if disableAfterXfer_v = true then
                sselTimer_v := ENABLE_HOLD_TIME_C - 1;
              end if;
            end if;
            bitCntr_v := bitCntr_v - 1;
          end if;
        end if;

        -- Transition the SPI clock and set the phase interval.
        sck_r            <= not sck_r;
        sclkPhaseTimer_v := CLKS_PER_SCLK_PHASE_C - 1;
      end if;
      
    end if;
  end process;

  ssel_o <= LO when enabled_r = true else HI;
  sck_o  <= sck_r;
  data_o <= sr_r;

end architecture;
