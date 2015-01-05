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


--**********************************************************************
-- A driver for an RGB LED. It takes the intensity of the red, green and blue
-- colors and creates PWM outputs to drive the red, green and blue LEDs.
-- A version with a Wishbone bus interface is also included.
--**********************************************************************



library IEEE;
use IEEE.std_logic_1164.all;

package RgbLedPckg is

  component RgbLed is
    port (
      clk_i    : in  std_logic;         -- Input clock.
      red_i    : in  std_logic_vector;  -- Red intensity.
      grn_i    : in  std_logic_vector;  -- Green intensity.
      blu_i    : in  std_logic_vector;  -- Blue intensity.
      redLed_o : out std_logic;         -- Red LED PWM output.
      grnLed_o : out std_logic;         -- Green LED PWM output.
      bluLed_o : out std_logic          -- Blue LED PWM output.
      );
  end component;

  component WbRgbLed is
    generic (
      VENDOR_ID_G  : std_logic_vector := x"FF";  -- Null vendor.
      PRODUCT_ID_G : std_logic_vector := x"FF"   -- Null device.
      );
    port (
      -- Wishbone interface.
      wb_clk_i  : in  std_logic;        -- Input clock.
      wb_rst_i  : in  std_logic;        -- Reset.
      wb_dat_o  : out std_logic_vector; -- Data bus from RGB LED driver.
      wb_dat_i  : in  std_logic_vector; -- Data bus to RGB LED driver.
      wb_adr_i  : in  std_logic_vector; -- Address bus to RGB LED driver.
      wb_we_i   : in  std_logic;        -- Write-enable.
      wb_cyc_i  : in  std_logic;        -- Valid cycle indicator.
      wb_stb_i  : in  std_logic;        -- Chip-enable.
      wb_ack_o  : out std_logic;        -- Acknowledge upon completion of operation.
      wb_inta_o : out std_logic;        -- Interrupt from RGB LED driver.
      id        : out std_logic_vector; -- Identifier for RGB LED driver.
      -- RGB LED drivers.
      redLed_o  : out std_logic;        -- Red LED PWM output.
      grnLed_o  : out std_logic;        -- Green LED PWM output.
      bluLed_o  : out std_logic         -- Blue LED PWM output.
      );
  end component;

end package;



--**********************************************************************
-- A driver for an RGB LED. It takes the intensity of the red, green and blue
-- colors and creates PWM outputs to drive the red, green and blue LEDs.
--**********************************************************************

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;
use XESS.PulsePckg.all;
use work.XessBoardPckg.all;

entity RgbLed is
  port (
    clk_i    : in  std_logic;           -- Input clock.
    red_i    : in  std_logic_vector;    -- Red intensity.
    grn_i    : in  std_logic_vector;    -- Green intensity.
    blu_i    : in  std_logic_vector;    -- Blue intensity.
    redLed_o : out std_logic;           -- Red LED PWM output.
    grnLed_o : out std_logic;           -- Green LED PWM output.
    bluLed_o : out std_logic            -- Blue LED PWM output.
    );
end entity;

architecture arch of RgbLed is
  signal redLed_s, grnLed_s, bluLed_s : std_logic;
begin

  -- Three PWMs drive the red, green and blue LED outputs.
  -- The duty cycle % of an LED is the intensity / 2**intensity'length.
  -- An LED is on when its driver is pulled low which happens when its PWM output is high.
  -- An LED is off when its driver is tristated which happens when its PWM output is low.
  
  uRedPwm : Pwm
    port map(
      clk_i  => clk_i,
      duty_i => red_i,
      pwm_o  => redLed_s
      );
  redLed_o <= LO when redLed_s = HI else HIZ;

  uGrnPwm : Pwm
    port map(
      clk_i  => clk_i,
      duty_i => grn_i,
      pwm_o  => grnLed_s
      );
  grnLed_o <= LO when grnLed_s = HI else HIZ;

  uBluPwm : Pwm
    port map(
      clk_i  => clk_i,
      duty_i => blu_i,
      pwm_o  => bluLed_s
      );
  bluLed_o <= LO when bluLed_s = HI else HIZ;

end architecture;



--**********************************************************************
-- A Wishbone-compliant driver for an RGB LED.
-- It contains a single 32-bit register at address 0 whose lower three 
-- bytes each control the duty cycle of a PWM attached to one of the red,
-- green or blue LEDs.
--**********************************************************************

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use XESS.CommonPckg.all;
use XESS.PulsePckg.all;
use XESS.RgbLedPckg.all;
use work.XessBoardPckg.all;

entity WbRgbLed is
  generic (
    VENDOR_ID_G  : std_logic_vector := x"FF";  -- Null vendor.
    PRODUCT_ID_G : std_logic_vector := x"FF"   -- Null device.
    );
  port (
    -- Wishbone interface.
    wb_clk_i  : in  std_logic;          -- Input clock.
    wb_rst_i  : in  std_logic;          -- Reset.
    wb_dat_o  : out std_logic_vector;   -- Data bus from RGB LED driver.
    wb_dat_i  : in  std_logic_vector;   -- Data bus to RGB LED driver.
    wb_adr_i  : in  std_logic_vector;   -- Address bus to RGB LED driver.
    wb_we_i   : in  std_logic;          -- Write-enable.
    wb_cyc_i  : in  std_logic;          -- Valid cycle indicator.
    wb_stb_i  : in  std_logic;          -- Chip-enable.
    wb_ack_o  : out std_logic;          -- Acknowledge upon completion of operation.
    wb_inta_o : out std_logic;          -- Interrupt from RGB LED driver.
    id        : out std_logic_vector;   -- Identifier for RGB LED driver.
    -- RGB LED drivers.
    redLed_o  : out std_logic;          -- Red LED PWM output.
    grnLed_o  : out std_logic;          -- Green LED PWM output.
    bluLed_o  : out std_logic           -- Blue LED PWM output.
    );
end entity;

architecture arch of WbRgbLed is
  signal wbActive_s          : std_logic;  -- True when this device is read/written over Wishbone bus.
  signal red_r, grn_r, blu_r : std_logic_vector(7 downto 0);  -- Red, green and blue intensity registers.
begin

  id <= VENDOR_ID_G & PRODUCT_ID_G;  -- Output the vendor and product IDs so the ZPUino can identify it.
  
  wb_inta_o <= NO;  -- No interrupts come from this module.

  wbActive_s <= wb_cyc_i and wb_stb_i;  -- True when this device is read/written over Wishbone bus.
  wb_ack_o   <= wbActive_s;  -- Immediately acknowledge any read or write operation.

  -- This process controls the reading/writing of the RGB intensity register.
  process(wb_clk_i)
  begin
    if rising_edge(wb_clk_i) then
      wb_dat_o <= (others => '0');  -- The default data output for this device is all zeroes.
      if wbActive_s = YES then  -- True when this device is being accessed over the Wishbone bus.
        if wb_we_i = YES then   -- True when this device is being written.
          case TO_INTEGER(unsigned(wb_adr_i(9 downto 2))) is
            when 0 =>  -- Writing address 0 sets the R, G, B intensities.
              red_r <= wb_dat_i(7 downto 0);
              grn_r <= wb_dat_i(15 downto 8);
              blu_r <= wb_dat_i(23 downto 16);
            when others =>  -- Writing other addresses does nothing.
              null;
          end case;
        else  -- This device is being read.
          case TO_INTEGER(unsigned(wb_adr_i(9 downto 2))) is
            when 0 =>  -- Reading address 0 gets the R, G, B intensities.
              wb_dat_o(23 downto 0) <= blu_r & grn_r & red_r;
            when others =>  -- Reading other addresses does nothing.
              null;
          end case;
        end if;
      end if;
    end if;
  end process;

  -- Apply the red, green and blue intensities from the register to the RGB LED driver module
  -- described previously.
  uRgbLed : RgbLed
    port map(
      clk_i    => wb_clk_i,
      red_i    => red_r,
      grn_i    => grn_r,
      blu_i    => blu_r,
      redLed_o => redLed_o,
      grnLed_o => grnLed_o,
      bluLed_o => bluLed_o
      );

end architecture;
