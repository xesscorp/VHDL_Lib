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
      FREQ_G         : real    := 12.0;   -- Master clock frequency (MHz).
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
      VENDOR_ID_G    : std_logic_vector := x"FF";  -- Unknown.
      PRODUCT_ID_G   : std_logic_vector := x"FF"   -- Unknown.
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
use xess.ClkGenPckg.all;
use xess.MiscPckg.all;
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
    rd_i    : in  std_logic := NO;  -- Active-high read-enable for pixel RAM.
    wr_i    : in  std_logic := NO;  -- Active-high write-enable for pixel RAM.
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
  constant NPIXELS_C   : natural := NPANELS_G * PANEL_WIDTH_G * PANEL_HEIGHT_G;
  constant NCOLS_C     : natural := NPANELS_G * PANEL_WIDTH_G;  -- # cols wide.
  constant FINAL_COL_C : natural := NCOLS_C - 1;     -- Index of last column.
  constant NROWS_C     : natural := PANEL_HEIGHT_G;  -- # of rows tall.
  constant FINAL_ROW_C : natural := NROWS_C / 2 -1;  -- Index of last row in a half-panel.
  signal row_r         : natural range 0 to FINAL_ROW_C;  -- Active row.
  signal col_r         : natural range 0 to FINAL_COL_C;  -- Current column.

  -- Compute clock for display circuitry based on number of pixels and the refresh rate.
  constant DISPLAY_FREQ_C : real := real(NPIXELS_C)/2.0 * REFRESH_RATE_G / 10.0**6;
  signal displayClk_s     : std_logic;

  -- Constants and signals for the pixel RAM.
  -- The pixel_i/pixel_o buses are a single pixel wide. The output display bus
  -- from the RAM is two pixels wide so the display circuitry can simultaneously
  -- fetch pixels for the upper and lower halves of the LED panel.
  constant MAX_ADDR_C         : natural := NROWS_C * NCOLS_C - 1;
  constant DBLWIDE_MAX_ADDR_C : natural := NROWS_C * NCOLS_C / 2 - 1;
  signal addr_r               : natural range 0 to DBLWIDE_MAX_ADDR_C;
  subtype Pixel_t is unsigned(pixel_i'range);  -- The pixel_i bus determines the width of pixels.
  signal upperPixel_r         : Pixel_t;
  signal lowerPixel_r         : Pixel_t;
  type PixelRam_t is array(0 to DBLWIDE_MAX_ADDR_C) of Pixel_t;
  signal upperPixelRam_r : PixelRam_t;
  signal lowerPixelRam_r : PixelRam_t := (0      => X"FFFFFFFF", 1 => X"FFFFFFFF", 2 => X"FFFF0000", 3 => X"FFFF0000",
                                     40     => X"FFFF0000", 41 => X"FFFF0000", 96 => X"FFFFFFFF", 92 => X"FF0000FF",
                                     88     => X"0000FF00", 84 => X"FFFF0000", 80 => X"FFFFFF00", 76 => X"0000FFFF", 72 => X"FFFF00FF",
                                     200    => X"FFFF0000", 204 => X"FFFF0000", 208 => X"FFFFFFFF", 210 => X"000000FF",
                                     214    => X"0000FF00", 218 => X"0000FF00", 222 => X"FFFFFF00", 226 => X"0000FFFF", 230 => X"FFFF00FF",
                                     others => X"00000000");  -- Storage for RGB pixels going to LED panel.
  -- This is the subfield of the input address that can be applied to the pixel RAM.
  subtype addrField_t is natural range Log2(MAX_ADDR_C)-1+addr_i'low downto addr_i'low+1;

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
  constant NUM_ROW_REPEAT_C : natural := 32;
  constant MAX_ROW_REPEAT_C : natural := NUM_ROW_REPEAT_C - 1;
  signal rowRptCntr_r       : natural range 0 to MAX_ROW_REPEAT_C;

  -- Threshold that determines whether a given value of a color field will cause
  -- the associated LED to turn on.
  signal thresh_s    : natural range 0 to 2**COLOR_FLD_SZ - 1;
  signal randNum_s   : std_logic_vector(31 downto 0); -- Random number used to set threshold.
  signal newThresh_r : std_logic; -- True when a new threshold value should be calculated.
  -- Size the threshold to the size of the color component fields.
  subtype threshField_t is natural range COLOR_FLD_SZ*1-1 downto COLOR_FLD_SZ*0;

  -- Reset signals.
  signal rst_s     : std_logic := ZERO; -- Internal reset to the low-speed display logic.
  signal rstDone_r : std_logic := NO; -- True when display logic has been reset.
  
begin

  -- This process lets the host read/write the dual-port pixel RAMs at full speed.
  process (clk_i)
  begin
    if rising_edge(clk_i) then
      if wr_i = YES then
        -- Write a single pixel to the appropriate half of the pixel RAM.
        if addr_i(addr_i'low) = ZERO then
          lowerPixelRam_r(TO_INTEGER(unsigned(addr_i(addrField_t)))) <= unsigned(pixel_i);
        else
          upperPixelRam_r(TO_INTEGER(unsigned(addr_i(addrField_t)))) <= unsigned(pixel_i);
        end if;
      elsif rd_i = YES then
        -- Read a single pixel from the appropriate half of the pixel RAM.
        if addr_i(addr_i'low) = ZERO then
          pixel_o <= std_logic_vector(lowerPixelRam_r(TO_INTEGER(unsigned(addr_i(addrField_t)))));
        else
          pixel_o <= std_logic_vector(upperPixelRam_r(TO_INTEGER(unsigned(addr_i(addrField_t)))));
        end if;
      end if;
    end if;
  end process;

  -- Generate a slower clock for circuitry that displays the pixels on the LEDs.
  uClk : SlowClkGen
    generic map(
      INPUT_FREQ_G  => FREQ_G,
      OUTPUT_FREQ_G => DISPLAY_FREQ_C
      )
    port map(
      clk_i => clk_i,
      clk_o => displayClk_s
      );

  -- This catches any reset from the high-speed clock domain and holds it
  -- until the slower-speed display circuitry completes its reset.
  uReset: HandshakeIntfc
    port map(
      ctrl_i => rst_i,
      ctrl_o => rst_s,
      done_i => rstDone_r
    );

  -- Generate random numbers for the threshold. Higher-intensity colors
  -- will exceed the threshold more often than low-intensity colors, and
  -- their LEDs will be on more often (thus increasing the intensity).
  uRandThresh : RandGen
    port map(
      clk_i  => displayClk_s,
      cke_i  => newThresh_r,
      ld_i   => rst_s,
      seed_i => X"FFFFFFFF",
      rand_o => randNum_s
      );
  thresh_s <= TO_INTEGER(unsigned(randNum_s(threshField_t)));

  -- This process sequentially reads the pixel RAMs and computes the
  -- RGB bits and the control signal sequencing needed by the LED array.
  process (displayClk_s)
  begin
    if rising_edge(displayClk_s) then
      -- Fetch two pixels from the double-wide port of the pixel RAM.
      -- The pixel at the even address is in one half of the panel,
      -- and the pixel at the odd address is in the other half.
      lowerPixel_r <= lowerPixelRam_r(addr_r);  -- Pixel for lower half of LED array.
      upperPixel_r <= upperPixelRam_r(addr_r);  -- Pixel for upper half of LED array.

      -- Compare the color fields of both pixels against the threshold 
      -- to determine which LEDs should be active or not.
      red1_o <= ZERO;  -- Start off assuming all LEDs will be off.
      grn1_o <= ZERO;
      blu1_o <= ZERO;
      red2_o <= ZERO;
      grn2_o <= ZERO;
      blu2_o <= ZERO;
      -- If this isn't the last time the row will be displayed, then compare
      -- the color fields against the threshold to see which LEDs are on.
      -- If this is the last time the row is displayed, then just leave all
      -- the LEDs off. (This helps prevent "ghosting" when the next row of
      -- pixels is enabled.)
      if rowRptCntr_r > 0 then
        -- Compare the pixel field for the red color component to the threshold.
        if TO_INTEGER(upperPixel_r(redField_t)) > thresh_s then
          red1_o <= ONE;
        end if;
        -- Same thing for the green field of the upper pixel.
        if TO_INTEGER(upperPixel_r(grnField_t)) > thresh_s then
          grn1_o <= ONE;
        end if;
        -- Same thing for the blue field of the upper pixel.
        if TO_INTEGER(upperPixel_r(bluField_t)) > thresh_s then
          blu1_o <= ONE;
        end if;
        -- Same thing for the red field of the lower pixel.
        if TO_INTEGER(lowerPixel_r(redField_t)) > thresh_s then
          red2_o <= ONE;
        end if;
        -- Same thing for the green field of the lower pixel.
        if TO_INTEGER(lowerPixel_r(grnField_t)) > thresh_s then
          grn2_o <= ONE;
        end if;
        -- Same thing for the blue field of the lower pixel.
        if TO_INTEGER(lowerPixel_r(bluField_t)) > thresh_s then
          blu2_o <= ONE;
        end if;
      end if;

      -- Generate the control signals for the LED array.
      if rst_s = NO then  -- Normal (non-reset) operations go here.
      
        rstDone_r <= NO; -- Allow future resets.
      
        -- Latch the just-completed row of pixels into the panel's display
        -- register just before the first pixel of the new row is output..
        latch_o <= LO;
        if col_r = 0 then
          latch_o <= HI;                -- Latch new LED outputs for this row.
        end if;

        -- Switch the display row at the half-way point where the new row
        -- of pixels is being output. The pixel latch should be set to all-zero
        -- from the last scan of the previous row, so this will power the new
        -- row with all the LEDs initially off to reduce ghosting.
        if col_r = FINAL_COL_C-NCOLS_C/2 and rowRptCntr_r = MAX_ROW_REPEAT_C then
          if row_r < FINAL_ROW_C then
            row_r <= row_r + 1;
          else
            row_r <= 0;  -- This was the last row, so roll-over the counter.
          end if;
        end if;

        -- Increment the column counter until it rolls over at the end of the row.
        -- Activate the latch at the end of the at the end of the row and
        -- increment the row counter.
        if col_r < FINAL_COL_C then -- Currently within a row of LEDs.
          -- This is the normal incrementing through the pixel RAM and the
          -- LEDs of a row of the panel.
          addr_r      <= addr_r + 1;
          col_r       <= col_r + 1;
          newThresh_r <= NO;  -- Keep the same color threshold for an entire display row.
        else                            -- At the end of a row of LEDs.
          col_r       <= 0;   -- Go back to the start of a row of LEDs.
          newThresh_r <= YES;  -- Get a new color threshold for each row display.
          if rowRptCntr_r > 0 then
            -- Not done repeating this row, so go back to the starting 
            -- address for this row of pixels.
            addr_r       <= addr_r - NCOLS_C + 1;
            rowRptCntr_r <= rowRptCntr_r - 1;  -- Decrement the row-repeat counter.
          else
            -- This is the last time this row of pixels will be repeated,
            -- so let the RAM pointer move to the next row of pixels.
            if row_r < FINAL_ROW_C then
              -- The next row starts right after the end of the current row.
              addr_r <= addr_r + 1;
            else
              -- Reached the last row of pixels, so go back to the beginning.
              addr_r <= 0;
            end if;
            rowRptCntr_r <= MAX_ROW_REPEAT_C;  -- Roll-over the row-repeat counter.
          end if;
        end if;
        
      else                    -- Reset values for the control signals.
        addr_r       <= 0;    -- Start at the beginning of the pixel RAM.
        col_r        <= 0;              -- Start at the beginning of a row.
        row_r        <= FINAL_ROW_C;  -- Start at the final row because this will be rolled-over when the latch occurs.
        latch_o      <= LO;   -- No RGB bits output yet, so don't latch.
        newThresh_r  <= NO;
        rowRptCntr_r <= MAX_ROW_REPEAT_C;  -- Initialize row-repeat counter.
        rstDone_r    <= YES;  -- Reset completed, so clear the current reset flag.
      end if;
    end if;
  end process;

  clk_o <= displayClk_s;  -- Display clock is the same as the clock driving this logic.
  oe_bo <= LO;                          -- Always keep the display enabled.
  row_o <= std_logic_vector(TO_UNSIGNED(row_r, row_o'length));  -- Output the active row to the display.
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
    VENDOR_ID_G    : std_logic_vector := x"FF";  -- Unknown.
    PRODUCT_ID_G   : std_logic_vector := x"FF"   -- Unknown.
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
  signal inPixel_s  : std_logic_vector(31 downto 0) := (others => ZERO);
  signal outPixel_s : std_logic_vector(31 downto 0);
begin

  gnd0 <= LO;
  gnd1 <= LO;
  
  uRst: ResetGenerator
    generic map(PULSE_DURATION_G => 10)
    port map(clk_i=>clk_i, trigger_i=>YES, reset_o=>rst_s);

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
