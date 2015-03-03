--**********************************************************************
-- Copyright (c) 2015 by XESS Corp <http://www.xess.com>.
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
-- Module for driving a 32x32 RGB LED panel. 
--**********************************************************************

library ieee, xess;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use xess.CommonPckg.all;

package LedPanelPckg is

  component LedPanelDriver is
    generic (
      FREQ_G         : real    := 12.0;    -- Master clock frequency (MHz).
      REFRESH_RATE_G : real    := 2000.0;  -- Total panel refresh rate (Hz).
      NPANELS_G      : natural := 1;    -- Number of concatenated 32x32 panels.
      PANEL_WIDTH_G  : natural := 32;   -- Width of panel in columns of pixels.
      PANEL_HEIGHT_G : natural := 32    -- Height of panel in rows of pixels.
      );
    port (
      -- Signals to/from FPGA fabric.
      clk_i   : in  std_logic;          -- Master clock input.
      rst_i   : in  std_logic := NO;    -- Active-high, synchronous reset.
      rd_i    : in  std_logic := NO;  -- Active-high read-enable for pixel RAM.
      wr_i    : in  std_logic := NO;  -- Active-high write-enable for pixel RAM.
      addr_i  : in  std_logic_vector;   -- Address for pixel read/write.
      pixel_i : in  std_logic_vector;   -- Input bus to pixel RAM.
      pixel_o : out std_logic_vector;   -- Output bus from pixel RAM.
      -- Outputs to LED panel.
      clk_o   : out std_logic;          -- Clock output.
      oe_bo   : out std_logic;          -- Active-low output-enable.
      latch_o : out std_logic;          -- Active-high pixel-row latch enable.
      row_o   : out std_logic_vector;   -- Active row of LED panel.
      red1_o  : out std_logic := LO;    -- Red component of upper pixel.
      grn1_o  : out std_logic := LO;    -- Green component of upper pixel.
      blu1_o  : out std_logic := LO;    -- Blue component of upper pixel.
      red2_o  : out std_logic := LO;    -- Red component of lower pixel.
      grn2_o  : out std_logic := LO;    -- Green component of lower pixel.
      blu2_o  : out std_logic := LO     -- Blue component of lower pixel.
      );
  end component;

  component WbLedPanelDriver is
    generic (
      FREQ_G         : real             := 96.0;  -- Master clock frequency (MHz).
      REFRESH_RATE_G : real             := 2000.0;  -- Total panel refresh rate (Hz).
      NPANELS_G      : natural          := 1;  -- Number of concatenated 32x32 panels.
      PANEL_WIDTH_G  : natural          := 32;  -- Width of panel in columns of pixels.
      PANEL_HEIGHT_G : natural          := 32;  -- Height of panel in rows of pixels.
      COLOR_WIDTH_G  : natural          := 8;  -- Bit width of R, G, B color component fields.
      VENDOR_ID_G    : std_logic_vector := x"FF";   -- Unknown.
      PRODUCT_ID_G   : std_logic_vector := x"FF"  -- Unknown.
      );
    port (
      -- Wishbone interface.
      wb_clk_i  : in  std_logic;
      wb_rst_i  : in  std_logic;
      wb_dat_o  : out std_logic_vector;
      wb_dat_i  : in  std_logic_vector;
      wb_adr_i  : in  std_logic_vector;
      wb_we_i   : in  std_logic;
      wb_cyc_i  : in  std_logic;
      wb_stb_i  : in  std_logic;
      wb_ack_o  : out std_logic;
      wb_inta_o : out std_logic;
      id        : out std_logic_vector;
      -- Outputs to LED panel.
      clk_o     : out std_logic;        -- Clock output.
      oe_bo     : out std_logic;        -- Active-low output-enable.
      latch_o   : out std_logic;        -- Active-high pixel-row latch enable.
      row_o     : out std_logic_vector;        -- Active row of LED panel.
      red1_o    : out std_logic := LO;  -- Red component of upper pixel.
      grn1_o    : out std_logic := LO;  -- Green component of upper pixel.
      blu1_o    : out std_logic := LO;  -- Blue component of upper pixel.
      red2_o    : out std_logic := LO;  -- Red component of lower pixel.
      grn2_o    : out std_logic := LO;  -- Green component of lower pixel.
      blu2_o    : out std_logic := LO   -- Blue component of lower pixel.
      );
  end component;

end package;




library ieee, xess;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use xess.CommonPckg.all;
use xess.RandPckg.all;
use xess.DelayPckg.all;
--use xess.ClkGenPckg.all;
--use xess.MiscPckg.all;
use work.XessBoardPckg.all;
--library unisim;
--use unisim.vcomponents.all;

entity LedPanelDriver is
  generic (
    FREQ_G         : real    := 12.0;   -- Master clock frequency (MHz).
    REFRESH_RATE_G : real    := 2000.0;  -- Total panel refresh rate (Hz).
    NPANELS_G      : natural := 1;      -- Number of concatenated 32x32 panels.
    PANEL_WIDTH_G  : natural := 32;     -- Width of panel in columns of pixels.
    PANEL_HEIGHT_G : natural := 32      -- Height of panel in rows of pixels.
    );
  port (
    -- Signals to/from FPGA fabric.
    clk_i   : in  std_logic;            -- Master clock input.
    rst_i   : in  std_logic := NO;      -- Active-high, synchronous reset.
    rd_i    : in  std_logic := NO;   -- Active-high read-enable for pixel RAM.
    wr_i    : in  std_logic := NO;   -- Active-high write-enable for pixel RAM.
    addr_i  : in  std_logic_vector;     -- Address for pixel read/write.
    pixel_i : in  std_logic_vector;     -- Input bus to pixel RAM.
    pixel_o : out std_logic_vector;     -- Output bus from pixel RAM.
    -- Outputs to LED panel.
    clk_o   : out std_logic;            -- Clock output.
    oe_bo   : out std_logic;            -- Active-low output-enable.
    latch_o : out std_logic;            -- Active-high pixel-row latch enable.
    row_o   : out std_logic_vector;     -- Active row of LED panel.
    red1_o  : out std_logic := LO;      -- Red component of upper pixel.
    grn1_o  : out std_logic := LO;      -- Green component of upper pixel.
    blu1_o  : out std_logic := LO;      -- Blue component of upper pixel.
    red2_o  : out std_logic := LO;      -- Red component of lower pixel.
    grn2_o  : out std_logic := LO;      -- Green component of lower pixel.
    blu2_o  : out std_logic := LO       -- Blue component of lower pixel.
    );
end entity;

architecture arch of LedPanelDriver is

  -- Constants and signals for the size of the LED panel in pixels.
  constant NCOLS_C     : natural := NPANELS_G * PANEL_WIDTH_G;  -- # cols wide.
  constant NROWS_C     : natural := PANEL_HEIGHT_G;  -- # of rows tall.
  constant NPIXELS_C   : natural := NCOLS_C * NROWS_C;
  signal col_r         : natural range 0 to NCOLS_C - 1;  -- Current column.
  signal row_r         : natural range 0 to NROWS_C - 1;  -- Active row.

  -- Compute clock for display circuitry based on number of pixels and the refresh rate.
  constant DISPLAY_FREQ_C : real := real(NPIXELS_C)/2.0 * REFRESH_RATE_G / 10.0**6;
  constant CLK_DIVISOR_C : natural := integer(ceil(FREQ_G / DISPLAY_FREQ_C));
  signal enbl_r: std_logic := NO; -- Clock-enable flag for dividing clock.

  -- Constants and signals for the pixel RAM.
  -- The pixel_i/pixel_o buses are a single pixel wide. The output display bus
  -- from the RAM is two pixels wide so the display circuitry can simultaneously
  -- fetch pixels for the upper and lower halves of the LED panel.
  signal addr_r               : natural range 0 to NPIXELS_C - 1;
  subtype Pixel_t is unsigned(pixel_i'range);  -- The pixel_i bus determines the width of pixels.
  signal pixel_r         : Pixel_t;
  type PixelRam_t is array(0 to NPIXELS_C - 1) of Pixel_t;
  signal pixelRam_r : PixelRam_t := (0 => X"FFFF", 33=>X"03ff", 64 => X"7c00", 128 => X"03e0", 192 => X"001F",
                                          256 => X"7C1F", 320 => X"7fe0", 384 => X"03ff",
                                          others => X"0000");  -- Storage for RGB pixels going to LED panel.

  -- Definitions and types for the color fields in each pixel.
  -- The pixel width is divided into three, equal-sized fields for the red,
  -- green and blue color components. In a 16-bit pixel, these
  -- would be arranged as 15-X(R4R3R2R1R0)(G4G3G2G1G0)(B4B3B2B1B0)-0.
  constant COLOR_FLD_SZ : natural := pixel_i'length / 3;
  subtype bluField_t is natural range COLOR_FLD_SZ*1-1 downto COLOR_FLD_SZ*0;
  subtype grnField_t is natural range COLOR_FLD_SZ*2-1 downto COLOR_FLD_SZ*1;
  subtype redField_t is natural range COLOR_FLD_SZ*3-1 downto COLOR_FLD_SZ*2;

  -- Constants and signals for controlling how many times each row is displayed
  -- before moving to the next row.
  constant NROW_REPEAT_C : natural := 8;
  signal rowRpt_r           : natural range 0 to NROW_REPEAT_C - 1;

  -- Threshold that determines whether a given value of a color field will cause
  -- the associated LED to turn on.
  signal thresh_s    : natural range 0 to 2**COLOR_FLD_SZ - 1;
  signal randNum_s   : std_logic_vector(31 downto 0);  -- Random number used to set threshold.
  signal newThresh_r : std_logic;  -- True when a new threshold value should be calculated.
  -- Size the threshold to the size of the color component fields.
  subtype threshField_t is natural range COLOR_FLD_SZ*1-1 downto 0;
  
  signal clk_r : std_logic;
  signal latch_r : std_logic;
  signal row_s : std_logic_vector(row_o'range);
  signal red_r, grn_r, blu_r, red1_r, grn1_r, blu1_r : std_logic;
  
begin

  -- This process lets the host read/write the dual-port pixel RAMs at full speed.
  process (clk_i)
  begin
    if rising_edge(clk_i) then
      if wr_i = YES then
        -- Write a single pixel to the appropriate half of the pixel RAM.
        pixelRam_r(TO_INTEGER(unsigned(addr_i))) <= unsigned(pixel_i);
      elsif rd_i = YES then
        -- Read a single pixel from the appropriate half of the pixel RAM.
        pixel_o <= std_logic_vector(pixelRam_r(TO_INTEGER(unsigned(addr_i))));
      end if;
    end if;
  end process;

  -- Generate a clock-enable for circuitry that displays the pixels on the LEDs.
  process(clk_i)
    variable enblCnt_v : natural range 0 to CLK_DIVISOR_C-1 := 0;
  begin
    if rising_edge(clk_i) then
      if enblCnt_v = 0 or rst_i = YES then
        enbl_r    <= YES;
        enblCnt_v := CLK_DIVISOR_C - 1;
      else
        enbl_r    <= NO;
        enblCnt_v := enblCnt_v - 1;
      end if;
    end if;
  end process;

  -- Generate random numbers for the threshold. Higher-intensity colors
  -- will exceed the threshold more often than low-intensity colors, and
  -- their LEDs will be on more often (thus increasing the intensity).
  uRandThresh : RandGen
    port map(
      clk_i  => clk_i,
      cke_i  => newThresh_r,
      ld_i   => rst_i,
      seed_i => X"FFFFFFFF",
      rand_o => randNum_s
      );
  --thresh_s <= TO_INTEGER(unsigned(randNum_s(threshField_t)));
  thresh_s <= 15;

  -- This process reads the pixel RAMs and computes the
  -- RGB bits for the LED array.
  process (clk_i)
  begin
    if rising_edge(clk_i) then
      if enbl_r = YES then
        -- Fetch two pixels from the double-wide port of the pixel RAM.
        -- The pixel at the even address is in one half of the panel,
        -- and the pixel at the odd address is in the other half.
        pixel_r <= pixelRam_r(addr_r);  -- Pixel for lower half of LED array.
        -- upperPixel_r <= upperPixelRam_r(addr_r);  -- Pixel for upper half of LED array.

        -- Compare the color fields of both pixels against the threshold 
        -- to determine which LEDs should be active or not.
        red_r <= ZERO;  -- Start off assuming all LEDs will be off.
        grn_r <= ZERO;
        blu_r <= ZERO;
        -- If this isn't the last time the row will be displayed, then compare
        -- the color fields against the threshold to see which LEDs are on.
        -- If this is the last time the row is displayed, then just leave all
        -- the LEDs off. (This helps prevent "ghosting" when the next row of
        -- pixels is enabled.)
        if rowRpt_r /= NROW_REPEAT_C - 1 then
          -- Compare the pixel field for the red color component to the threshold.
          if TO_INTEGER(pixel_r(redField_t)) > thresh_s then
            red_r <= ONE;
          end if;
          -- Same thing for the green field of the upper pixel.
          if TO_INTEGER(pixel_r(grnField_t)) > thresh_s then
            grn_r <= ONE;
          end if;
          -- Same thing for the blue field of the upper pixel.
          if TO_INTEGER(pixel_r(bluField_t)) > thresh_s then
            blu_r <= ONE;
          end if;
        end if;
        
        if clk_r = LO then
          red1_r <= red_r;
          grn1_r <= grn_r;
          blu1_r <= blu_r;
        else
          red1_o <= red1_r;
          grn1_o <= grn1_r;
          blu1_o <= blu1_r;
          red2_o <= red_r;
          grn2_o <= grn_r;
          blu2_o <= blu_r;
        end if;
      end if;
    end if;
  end process;
  
  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_i = YES then
        newThresh_r <= NO;
        addr_r <= 0;
        row_r <= 0;
        col_r <= 0;
        rowRpt_r <= 0;
        clk_r <= LO;
        latch_r <= LO;
      elsif enbl_r = YES then
        clk_r <= not clk_r; -- Toggle output clock at half of input clock frequency.
        newThresh_r <= NO;
        if clk_r = HI then -- Output clock is about to go low.
          latch_r <= LO;
          col_r <= col_r + 1;
          addr_r <= addr_r + 1;
          if col_r = NCOLS_C - 1 then
            newThresh_r <= YES;
            col_r <= 0;
            addr_r <= addr_r - 2*NCOLS_C + 1;
            rowRpt_r <= rowRpt_r + 1;
            if rowRpt_r = NROW_REPEAT_C - 1 then
              rowRpt_r <= 0;
              addr_r <= addr_r + 1;
              row_r <= row_r + 1;
              if row_r = NROWS_C - 1 then
                row_r <= 0;
                addr_r <= 0;
              end if;
            end if;
          end if;
        else -- Output clock is about to go high.
          addr_r <= addr_r + 1;
          latch_r <= LO;
          if col_r = NCOLS_C - 1 then
            latch_r <= HI;
          end if;
        end if;
      end if;
    end if;
  end process;
  
  -- Delay the clock, latch and row signals to the LED panel by two clock cycles
  -- so they line up with the RGB signals.
  
  clk_o <= clk_r;
  -- uClkDly: DelayLine
    -- generic map(NUM_DELAY_CYCLES_G => 2)
    -- port map(clk_i=>clk_i, cke_i=>enbl_r, a_i=>clk_r, aDelayed_o=>clk_o);
  
  -- latch_o <= latch_r;  
  uLtchDly: DelayLine
    generic map(NUM_DELAY_CYCLES_G => 4)
    port map(clk_i=>clk_i, cke_i=>enbl_r, a_i=>latch_r, aDelayed_o=>latch_o);
    
  -- row_o <= std_logic_vector(TO_UNSIGNED(row_r, row_o'length));  -- Output the active row to the display.
  row_s <= std_logic_vector(TO_UNSIGNED(row_r, row_o'length));
  uRowDly: DelayBus
    generic map(NUM_DELAY_CYCLES_G => 6)
    port map(clk_i=>clk_i, cke_i=>enbl_r, bus_i=>row_s, busDelayed_o=>row_o);

  oe_bo <= LO;                          -- Always keep the display enabled.
end architecture;




library ieee, xess;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use xess.CommonPckg.all;
use xess.LedPanelPckg.all;
--library unisim;
--use unisim.vcomponents.all;

entity WbLedPanelDriver is
  generic (
    FREQ_G         : real             := 96.0;  -- Master clock frequency (MHz).
    REFRESH_RATE_G : real             := 2000.0;  -- Total panel refresh rate (Hz).
    NPANELS_G      : natural          := 1;  -- Number of concatenated 32x32 panels.
    PANEL_WIDTH_G  : natural          := 32;  -- Width of panel in columns of pixels.
    PANEL_HEIGHT_G : natural          := 32;  -- Height of panel in rows of pixels.
    COLOR_WIDTH_G  : natural          := 8;  -- Bit width of R, G, B color component fields.
    VENDOR_ID_G    : std_logic_vector := x"FF";   -- Unknown.
    PRODUCT_ID_G   : std_logic_vector := x"FF"  -- Unknown.
    );
  port (
    -- Wishbone interface.
    wb_clk_i  : in  std_logic;
    wb_rst_i  : in  std_logic;
    wb_dat_o  : out std_logic_vector;
    wb_dat_i  : in  std_logic_vector;
    wb_adr_i  : in  std_logic_vector;
    wb_we_i   : in  std_logic;
    wb_cyc_i  : in  std_logic;
    wb_stb_i  : in  std_logic;
    wb_ack_o  : out std_logic;
    wb_inta_o : out std_logic;
    id        : out std_logic_vector;
    -- Outputs to LED panel.
    clk_o     : out std_logic;          -- Clock output.
    oe_bo     : out std_logic;          -- Active-low output-enable.
    latch_o   : out std_logic;          -- Active-high pixel-row latch enable.
    row_o     : out std_logic_vector;   -- Active row of LED panel.
    red1_o    : out std_logic := LO;    -- Red component of upper pixel.
    grn1_o    : out std_logic := LO;    -- Green component of upper pixel.
    blu1_o    : out std_logic := LO;    -- Blue component of upper pixel.
    red2_o    : out std_logic := LO;    -- Red component of lower pixel.
    grn2_o    : out std_logic := LO;    -- Green component of lower pixel.
    blu2_o    : out std_logic := LO     -- Blue component of lower pixel.
    );
end entity;

architecture arch of WbLedPanelDriver is
  constant PIXEL_WIDTH_C : natural := COLOR_WIDTH_G * 3;
  subtype pixelField_t is natural range PIXEL_WIDTH_C-1 downto 0;
  signal wbActive_s      : std_logic;  -- True when this device is read/written over Wishbone bus.
  signal wbWr_s          : std_logic;  -- True when this device is written to over the Wishbone bus.
  signal wbRd_s          : std_logic;  -- True when this device is read over the Wishbone bus.
begin

  id <= VENDOR_ID_G & PRODUCT_ID_G;  -- Output the vendor and product IDs so the ZPUino can identify it.

  wbActive_s                                   <= wb_cyc_i and wb_stb_i;  -- True when this device is read/written over Wishbone bus.
  wbWr_s                                       <= wbActive_s and wb_we_i;  -- True when this device is written to over the Wishbone bus.
  wbRd_s                                       <= wbActive_s and not wb_we_i;  -- True when this device is read over the Wishbone bus.
  wb_ack_o                                     <= wbActive_s;  -- Immediately acknowledge any read or write operation.
  wb_dat_o(wb_dat_o'high downto PIXEL_WIDTH_C) <= (others => ZERO);  -- Set default value for output data to Wishbone bus.

  wb_inta_o <= NO;                      -- No interrupts come from this module.

  uLedPanel : LedPanelDriver
    generic map(
      FREQ_G         => FREQ_G,
      REFRESH_RATE_G => REFRESH_RATE_G,
      NPANELS_G      => NPANELS_G,
      PANEL_WIDTH_G  => PANEL_WIDTH_G,
      PANEL_HEIGHT_G => PANEL_HEIGHT_G
      )
    port map(
      -- Signals to/from FPGA fabric.
      clk_i   => wb_clk_i,
      rst_i   => wb_rst_i,
      rd_i    => wbRd_s,
      wr_i    => wbWr_s,
      addr_i  => wb_adr_i,
      pixel_i => wb_dat_i(pixelField_t),
      pixel_o => wb_dat_o(pixelField_t),
      -- Outputs to LED panel.
      clk_o   => clk_o,
      oe_bo   => oe_bo,
      latch_o => latch_o,
      row_o   => row_o,
      red1_o  => red1_o,
      grn1_o  => grn1_o,
      blu1_o  => blu1_o,
      red2_o  => red2_o,
      grn2_o  => grn2_o,
      blu2_o  => blu2_o
      );
end architecture;




library ieee, xess;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use xess.CommonPckg.all;
use xess.LedPanelPckg.all;
use xess.MiscPckg.all;
use xess.ClkGenPckg.all;
--library unisim;
--use unisim.vcomponents.all;

entity LedPanelTest is
  port (
    clk_i   : in  std_logic;
    clk_o   : out std_logic;
    oe_bo   : out std_logic;
    latch_o : out std_logic;
    row_o   : out std_logic_vector(3 downto 0);
    red1_o  : out std_logic;
    grn1_o  : out std_logic;
    blu1_o  : out std_logic;
    red2_o  : out std_logic;
    grn2_o  : out std_logic;
    blu2_o  : out std_logic;
    gnd0    : out std_logic;
    gnd1    : out std_logic
    );
end entity;

architecture arch of LedPanelTest is
  signal clk_s      : std_logic;
  signal rst_s      : std_logic                     := NO;
  signal rd_s       : std_logic                     := NO;
  signal wr_s       : std_logic                     := NO;
  signal addr_s     : std_logic_vector(15 downto 0) := (others => ZERO);
  signal inPixel_s  : std_logic_vector(15 downto 0) := (others => ZERO);
  signal outPixel_s : std_logic_vector(15 downto 0);
begin

  gnd0 <= LO;
  gnd1 <= LO;

  uRst : ResetGenerator
    generic map(PULSE_DURATION_G => 10)
    port map(clk_i               => clk_i, trigger_i => YES, reset_o => rst_s);

  u0 : LedPanelDriver
    port map(
      clk_i   => clk_i,
      rst_i   => rst_s,
      rd_i    => rd_s,
      wr_i    => wr_s,
      addr_i  => addr_s,
      pixel_i => inPixel_s,
      pixel_o => outPixel_s,
      clk_o   => clk_o,
      oe_bo   => oe_bo,
      latch_o => latch_o,
      row_o   => row_o,
      red1_o  => red1_o,
      grn1_o  => grn1_o,
      blu1_o  => blu1_o,
      red2_o  => red2_o,
      grn2_o  => grn2_o,
      blu2_o  => blu2_o
      );

end architecture;
