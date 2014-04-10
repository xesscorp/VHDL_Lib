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
-- Modules for generating video signals for a VGA monitor.
----------------------------------------------------------------------------------


library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.math_real.all;
use XESS.CommonPckg.all;

package VgaPckg is

  component SyncGen is
    generic (
      FREQ_G    : real    := 50.0;      -- Master clock frequency (in MHz).
      PERIOD_G  : real    := 31.77;     -- Period of sync pulse (in us).
      START_G   : real    := 26.11;  -- Time sync pulse starts within the period (in us).
      WIDTH_G   : real    := 3.77;      -- Width of sync pulse (in us).
      VISIBLE_G : natural := 1024;  -- Number of visible pixels/line or lines/frame.
      OFFSET_G  : integer := 0  -- Number of pixels/lines to shift image right/down on screen.
      );
    port (
      rst_i   : in  std_logic;          -- Asynchronous reset.
      clk_i   : in  std_logic;          -- Master clock.
      cke_i   : in  std_logic;          -- Clock-enable.
      sync_bo : out std_logic;          -- Sync pulse.
      gate_o  : out std_logic;  -- Single-clock pulse at start of sync pulse.
      blank_o : out std_logic;          -- Blanking signal.
      cnt_o   : out std_logic_vector    -- Output the timing counter value.
      );
  end component;

  component HVSyncGen is
    generic (
      PIXEL_FREQ_G      : real    := 50.0;  -- Pixel clock frequency (in MHz).
      PIXELS_PER_LINE_G : natural := 1024;  -- Pixels per video scan line.
      LINES_PER_FRAME_G : natural := 512;   -- Scan lines per video frame.
      OFFSET_RIGHT_G    : integer := 0;  -- Offset of image to the right (pixels).
      OFFSET_DOWN_G     : integer := 0;  -- Offset of image downward (pixels).
      FIT_TO_SCREEN_G   : boolean := false  -- Fit width x length to monitor screen.
      );
    port (
      rst_i      : in  std_logic;       -- Asynchronous reset.
      clk_i      : in  std_logic;       -- Master clock.
      cke_i      : in  std_logic;  -- Clock enable (used to reduce master clock to pixel clock).
      eol_o      : out std_logic;       -- End of vga scan line.
      eof_o      : out std_logic;       -- End of vga frame.
      pixelCnt_o : out std_logic_vector(Log2(PIXELS_PER_LINE_G)-1 downto 0);  -- Current pixel within line.
      lineCnt_o  : out std_logic_vector(Log2(LINES_PER_FRAME_G)-1 downto 0);  -- Current video line within frame.
      hSync_bo   : out std_logic;       -- Horizontal sync pulse.
      vSync_bo   : out std_logic;       -- Vertical sync pulse.
      hBlank_o   : out std_logic;       -- Horizontal blanking signal.
      vBlank_o   : out std_logic        -- Vertical blanking signal.
      );
  end component;

  component PixelVga is
    generic (
      FREQ_G            : real    := 50.0;  -- Master clock frequency (in MHz).
      CLK_DIV_G         : natural := 1;  -- FREQ_G / CLK_DIV_G = pixel clock.
      PIXELS_PER_LINE_G : natural := 1024;  -- Pixels per video scan line.
      LINES_PER_FRAME_G : natural := 512;   -- Scan lines per video frame.
      OFFSET_RIGHT_G    : integer := 0;  -- Offset of image to the right (pixels).
      OFFSET_DOWN_G     : integer := 0;  -- Offset of image downward (pixels).
      FIT_TO_SCREEN_G   : boolean := false  -- Fit width x length to monitor screen.
      );
    port (
      rst_i         : in  std_logic;    -- Asynchronous reset.
      clk_i         : in  std_logic;    -- Master clock.
      wr_i          : in  std_logic;    -- Write-enable for pixel buffer.
      pixelDataIn_i : in  std_logic_vector(15 downto 0);  -- Input databus to pixel buffer.
      full_o        : out std_logic;    -- Pixel buffer full.
      eof_o         : out std_logic;    -- End of vga frame.
      hSync_bo      : out std_logic;    -- Horizontal sync pulse.
      vSync_bo      : out std_logic;    -- Vertical sync pulse.
      rgb_o         : out std_logic_vector  -- Red, green, blue color output bus.
      );
  end component;

  component TextCounter is
    generic (
      CHAR_SIZE_G     : natural := 8;  -- Width or height of a single character.
      TEXT_SIZE_G     : natural := 80;  -- Number of characters on a text line or the entire screen.
      POSITION_INCR_G : natural := 1  -- Amount to increment position after each character or line.
      );
    port (
      rst_i         : in  std_logic;    -- Synchronous reset.
      clk_i         : in  std_logic;    -- Master clock.
      cke_i         : in  std_logic;    -- Clock enable.
      position_o    : out std_logic_vector(Log2(TEXT_SIZE_G)-1 downto 0);  -- On-screen position of character being displayed.
      subPosition_o : out std_logic_vector(Log2(CHAR_SIZE_G)-1 downto 0);  -- Current pixel position within character being displayed.
      newPosition_o : out std_logic  -- Single-clock pulse when position changes.
      );
  end component;

  component TextVga is
    generic (
      FREQ_G          : real    := 50.0;  -- Master clock frequency (in MHz).
      CLK_DIV_G       : natural := 1;   -- FREQ_G / CLK_DIV_G = pixel clock.
      CHAR_HEIGHT_G   : natural := 8;   -- Character height in lines.
      NUM_TEXT_COLS_G : natural := 80;  -- Width of text screen in characters.
      NUM_TEXT_ROWS_G : natural := 25;  -- Height of text screen in characters.
      OFFSET_RIGHT_G  : integer := 0;  -- Offset of image to the right in pixels.
      OFFSET_DOWN_G   : integer := 0;   -- Offset of image downward in pixels.
      FIT_TO_SCREEN_G : boolean := false  -- Fit width x length to monitor screen.
      );
    port (
      rst_i            : in  std_logic;   -- Reset.
      clk_i            : in  std_logic;   -- Master clock.
      charAddr_o       : out std_logic_vector;  -- Address to video RAM containing text char codes.
      charCode_i       : in  std_logic_vector;  -- Char code from video RAM.
      charBitmapAddr_o : out std_logic_vector;  -- Address to character generator ROM.
      charBitmapRow_i  : in  std_logic_vector;  -- Row of character pixels from ROM.
      hSync_bo         : out std_logic;   -- Horizontal sync pulse.
      vSync_bo         : out std_logic;   -- Vertical sync pulse.
      rgb_o            : out std_logic_vector  -- Red, green, blue color output bus.
      );
  end component;

end package;



--
-- Generate a sync pulse within a waveform PERIOD_G.
-- Also output the value of the counter used for timing so that
-- it can be used in generating an address for a video RAM.
--

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use IEEE.std_logic_arith.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use XESS.CommonPckg.all;

entity SyncGen is
  generic (
    FREQ_G    : real    := 50.0;        -- Master clock frequency (in MHz).
    PERIOD_G  : real    := 31.77;       -- Period of sync pulse (in us).
    START_G   : real    := 26.11;  -- Time sync pulse starts within the period (in us).
    WIDTH_G   : real    := 3.77;        -- Width of sync pulse (in us).
    VISIBLE_G : natural := 1024;  -- Number of visible pixels/line or lines/frame.
    OFFSET_G  : integer := 0  -- Number of pixels/lines to shift image right/down on screen.
    );
  port (
    rst_i   : in  std_logic;            -- Asynchronous reset.
    clk_i   : in  std_logic;            -- Master clock.
    cke_i   : in  std_logic;            -- Clock-enable.
    sync_bo : out std_logic;            -- Sync pulse.
    gate_o  : out std_logic;  -- Single-clock pulse at start of sync pulse.
    blank_o : out std_logic;            -- Blanking signal.
    cnt_o   : out std_logic_vector      -- Output the timing counter value.
    );
end entity;


architecture arch of SyncGen is
  constant CYC_PERIOD_C : natural := natural(ceil(PERIOD_G * FREQ_G));  -- Sync wave PERIOD in clock cycles.
  constant CYC_START_C  : natural := natural(ceil(START_G * FREQ_G));  -- Sync pulse START in cycles.
  constant CYC_END_C    : natural := natural(ceil((START_G+WIDTH_G) * FREQ_G));  -- Sync pulse end in cycles.
  signal cnt_r          : natural range 0 to CYC_PERIOD_C-1;  -- Counter for timing sync pulse waveform.
begin

  process(rst_i, clk_i)
  begin
    if rst_i = YES then
      cnt_r   <= 0;
      sync_bo <= HI;
      gate_o  <= NO;
      blank_o <= NO;

    elsif rising_edge(clk_i) then

      gate_o <= NO;  -- This insures gate_o is only a single clock cycle in duration.

      if cke_i = YES then

        -- Increment counter and wrap around to zero at end of period.
        if cnt_r = CYC_PERIOD_C-1 then
          cnt_r <= 0;
        else
          cnt_r <= cnt_r + 1;
        end if;

        -- Generate sync pulse within waveform period.
        -- The sync signal can be retarded or advanced to offset the displayed pixels on the screen.
        if cnt_r = (CYC_START_C-OFFSET_G-1) mod CYC_PERIOD_C then
          sync_bo <= LO;
          gate_o  <= YES;  -- Generate a single-cycle gate signal at start of sync pulse.
        elsif cnt_r = (CYC_END_C-OFFSET_G-1) mod CYC_PERIOD_C then
          sync_bo <= HI;
        end if;

        -- Generate blank_o signal after initial visible period.
        if cnt_r = VISIBLE_G - 1 then
          blank_o <= YES;
        elsif cnt_r = CYC_PERIOD_C-1 then
          blank_o <= NO;
        end if;

      end if;
    end if;
  end process;

-- output counter value
  cnt_o <= CONV_STD_LOGIC_VECTOR(cnt_r, cnt_o'length);

end architecture;



--
-- Generate both horizontal and vertical sync, blanking and EOL/EOF
-- pulses for a display of a given size.
--

library IEEE, UNISIM, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.std_logic_unsigned.all;
use UNISIM.vcomponents.all;
use XESS.CommonPckg.all;
use XESS.VgaPckg.all;

entity HVSyncGen is
  generic (
    PIXEL_FREQ_G      : real    := 50.0;  -- Pixel clock frequency (in MHz).
    PIXELS_PER_LINE_G : natural := 1024;  -- Pixels per video scan line.
    LINES_PER_FRAME_G : natural := 512;   -- Scan lines per video frame.
    OFFSET_RIGHT_G    : integer := 0;  -- Offset of image to the right (pixels).
    OFFSET_DOWN_G     : integer := 0;   -- Offset of image downward (pixels).
    FIT_TO_SCREEN_G   : boolean := false  -- Fit width x length to monitor screen.
    );
  port (
    rst_i      : in  std_logic;         -- Asynchronous reset.
    clk_i      : in  std_logic;         -- Master clock.
    cke_i      : in  std_logic;  -- Clock enable (used to reduce master clock to pixel clock).
    eol_o      : out std_logic;         -- End of vga scan line.
    eof_o      : out std_logic;         -- End of vga frame.
    pixelCnt_o : out std_logic_vector(Log2(PIXELS_PER_LINE_G)-1 downto 0);  -- Current pixel within line.
    lineCnt_o  : out std_logic_vector(Log2(LINES_PER_FRAME_G)-1 downto 0);  -- Current video line within frame.
    hSync_bo   : out std_logic;         -- Horizontal sync pulse.
    vSync_bo   : out std_logic;         -- Vertical sync pulse.
    hBlank_o   : out std_logic;         -- Horizontal blanking signal.
    vBlank_o   : out std_logic          -- Vertical blanking signal.
    );
end entity;


architecture arch of HVSyncGen is
  -- Video timing for fit-to-screen or 31 KHz horizontal, 60 Hz vertical screen refresh.
  constant HSYNC_START_C  : real := RealSelect(FIT_TO_SCREEN_G, real(PIXELS_PER_LINE_G)/PIXEL_FREQ_G, 26.11);  -- Start of horiz. sync pulse with a scanline (us).
  constant HSYNC_WIDTH_C  : real := 3.77;  -- Width of horiz. sync pulse (us).
  constant HSYNC_PERIOD_C : real := HSYNC_START_C + 5.66;  -- Horizontal scanline period (us).
  constant HSYNC_FREQ_C   : real := 1.0 / HSYNC_PERIOD_C;  -- Scanline frequency (MHz).
  constant VSYNC_START_C  : real := RealSelect(FIT_TO_SCREEN_G, HSYNC_PERIOD_C * real(LINES_PER_FRAME_G) + 340.0, 15700.0);  -- start of vert. sync pulse within a frame (us)
  constant VSYNC_PERIOD_C : real := VSYNC_START_C + 1084.0;  -- video frame period (us)
  constant VSYNC_WIDTH_C  : real := 64.0;  -- width of vert. sync pulse (us)
  signal eol_s            : std_logic;  -- Pulses on end-of-line.
begin

  -- Generate the horizontal and vertical sync pulses.
  UHSync : SyncGen
    generic map (
      FREQ_G    => PIXEL_FREQ_G,        -- Master pixel-clock frequency.
      PERIOD_G  => HSYNC_PERIOD_C,      -- Scanline period.
      START_G   => HSYNC_START_C,  -- Start of horizontal sync pulse in scan line.
      WIDTH_G   => HSYNC_WIDTH_C,       -- Width of horizontal sync pulse.
      VISIBLE_G => PIXELS_PER_LINE_G,  -- Number of pixels shown on each scan line.
      OFFSET_G  => OFFSET_RIGHT_G       -- Rightward image offset.
      )
    port map (
      rst_i   => rst_i,
      clk_i   => clk_i,                 -- Master clock.
      cke_i   => cke_i,  -- A new pixel is output whenever the clock is enabled.
      sync_bo => hSync_bo,              -- Horizontal sync.
      gate_o  => eol_s,  -- Send gate signal to increment vertical sync pulse generator once per scan line.
      blank_o => hBlank_o,              -- Blanking signal within a scan line.
      cnt_o   => pixelCnt_o             -- Current pixel within the scan line.
      );

  UVSync : SyncGen
    generic map (
      FREQ_G    => HSYNC_FREQ_C,        -- Scanline frequency.
      PERIOD_G  => VSYNC_PERIOD_C,      -- Image frame period.
      START_G   => VSYNC_START_C,  -- Start of vertical sync pulse in frame.
      WIDTH_G   => VSYNC_WIDTH_C,       -- Width of vertical sync pulse.
      VISIBLE_G => LINES_PER_FRAME_G,  -- Number of scan lines visible per frame.
      OFFSET_G  => OFFSET_DOWN_G        -- Downward image offset.
      )
    port map (
      rst_i   => rst_i,
      clk_i   => clk_i,                 -- Master clock.
      cke_i   => eol_s,  -- Enable clock once per horizontal scan line.
      sync_bo => vSync_bo,              -- Vertical sync.
      gate_o  => eof_o,  -- Indicate the end of a complete frame.
      blank_o => vBlank_o,              -- Blanking signal within a frame.
      cnt_o   => lineCnt_o              -- Current scan line within a frame.
      );

  eol_o <= eol_s;

end architecture;



--
-- Module for displaying individual pixels on a VGA display.
--

library IEEE, UNISIM, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;
use UNISIM.vcomponents.all;
use XESS.CommonPckg.all;
use XESS.FifoPckg.all;
use XESS.VgaPckg.all;

entity PixelVga is
  generic (
    FREQ_G            : real    := 50.0;  -- Master clock frequency (in MHz).
    CLK_DIV_G         : natural := 1;   -- FREQ_G / CLK_DIV_G = pixel clock.
    PIXELS_PER_LINE_G : natural := 1024;  -- Pixels per video scan line.
    LINES_PER_FRAME_G : natural := 512;   -- Scan lines per video frame.
    OFFSET_RIGHT_G    : integer := 0;  -- Offset of image to the right (pixels).
    OFFSET_DOWN_G     : integer := 0;   -- Offset of image downward (pixels).
    FIT_TO_SCREEN_G   : boolean := false  -- Fit width x length to monitor screen.
    );
  port (
    rst_i         : in  std_logic;      -- Asynchronous reset.
    clk_i         : in  std_logic;      -- Master clock.
    wr_i          : in  std_logic;      -- Write-enable for pixel buffer.
    pixelDataIn_i : in  std_logic_vector(15 downto 0);  -- Input databus to pixel buffer.
    full_o        : out std_logic;      -- Pixel buffer full.
    eof_o         : out std_logic;      -- End of vga frame.
    hSync_bo      : out std_logic;      -- Horizontal sync pulse.
    vSync_bo      : out std_logic;      -- Vertical sync pulse.
    rgb_o         : out std_logic_vector  -- Red, green, blue color output bus.
    );
end entity;


architecture arch of PixelVga is
  constant COLOR_DEPTH_C          : natural := rgb_o'length;
  constant PIXEL_WIDTH_C          : natural := 2**Log2(COLOR_DEPTH_C);
  constant PIXEL_FREQ_C           : real    := FREQ_G / real(CLK_DIV_G);  -- Pixel drawing rate.
  signal clkDivCnt_r              : natural range 0 to CLK_DIV_G;  -- Clock divider counter.
  signal cke_r                    : std_logic;  -- Clock enable pulses once every CLK_DIV_G clocks.
  signal pixelCnt_s               : std_logic_vector(Log2(PIXELS_PER_LINE_G)-1 downto 0);  -- Current pixel within line.
  signal lineCnt_s                : std_logic_vector(Log2(LINES_PER_FRAME_G)-1 downto 0);  -- Current video line.
  signal fifoRst_s                : std_logic;  -- Reset for pixel buffer.
  signal fifoLevel_s              : std_logic_vector(7 downto 0);  -- Number of entries in pixel buffer.
  signal eof_s                    : std_logic;  -- Pulses on end-of-frame.
  signal hBlank_s                 : std_logic;  -- High during horizontal blanking interval.
  signal vBlank_s                 : std_logic;  -- High during vertical blanking interval.
  signal visible_s                : std_logic;  -- High during visible portion of screen display.
  constant PIX_PROC_DELAY_C       : natural := 2;  -- Time delay to read a pixel from the FIFO and colormap it.
  signal hSync_x, hSync_r         : std_logic_vector(PIX_PROC_DELAY_C downto 1);  -- Horizontal sync delay line.
  signal blank_x, blank_r         : std_logic_vector(PIX_PROC_DELAY_C downto 1);  -- Blanking delay line.
  signal rd_x, rd_r               : std_logic;  -- Pixel buffer read control signal.
  signal ckeRd_s                  : std_logic;  -- Clock-gated pixel buffer control read signal.
  signal pixelDataOut_s           : std_logic_vector(pixelDataIn_i'range);  -- Output data from pixel buffer.
  signal pixelData_x, pixelData_r : std_logic_vector(pixelDataIn_i'range);  -- Shift register for pixel buffer data.
  signal rgb_x, rgb_r             : std_logic_vector(rgb_o'range);  -- Pixel output register.
begin

  -- Clock divider for reducing the pixel clock rate.
  process(clk_i, rst_i)
  begin
    if rst_i = YES then
      clkDivCnt_r <= 0;
      cke_r       <= YES;
    elsif rising_edge(clk_i) then
      if clkDivCnt_r = CLK_DIV_G-1 then
        clkDivCnt_r <= 0;
        cke_r       <= YES;  -- Pulse clock enable once every CLK_DIV_G cycles.
      else
        clkDivCnt_r <= clkDivCnt_r + 1;
        cke_r       <= NO;
      end if;
    end if;
  end process;

  -- Pixel data buffer (FIFO).
  ckeRd_s <= rd_x and cke_r;
  fifo : Fifo255x16cc
    port map (
      clk_i     => clk_i,
      rd_i      => ckeRd_s,
      wr_i      => wr_i,
      data_i    => pixelDataIn_i,
      rst_i     => fifoRst_s,
      data_o    => pixelDataOut_s,
      full_o    => open,
      empty_o   => open,
      level_o   => fifoLevel_s
      );
  full_o <= YES when fifoLevel_s(7 downto 3) = "11111" else NO;

  -- Horizontal & vertical sync generator.
  UHVSyncGen : HVSyncGen
    generic map(
      PIXEL_FREQ_G      => PIXEL_FREQ_C,
      PIXELS_PER_LINE_G => PIXELS_PER_LINE_G,
      LINES_PER_FRAME_G => LINES_PER_FRAME_G,
      OFFSET_RIGHT_G    => OFFSET_RIGHT_G,
      OFFSET_DOWN_G     => OFFSET_DOWN_G,
      FIT_TO_SCREEN_G   => FIT_TO_SCREEN_G
      )
    port map(
      rst_i      => rst_i,
      clk_i      => clk_i,
      cke_i      => cke_r,
      eof_o      => eof_s,
      pixelCnt_o => pixelCnt_s,
      hSync_bo   => hSync_x(1),         -- Send pulse through delay line.
      vSync_bo   => vSync_bo,
      vBlank_o   => vBlank_s,
      hBlank_o   => hBlank_s
      );
  eof_o      <= eof_s;
  blank_x(1) <= vBlank_s or hBlank_s;  -- Send blanking signal through delay line.
  visible_s  <= not vBlank_s and not hBlank_s;

  fifoRst_s <= eof_s or rst_i;  -- Clear the contents of the pixel buffer at the end of every frame.

  -- Pass the horiz. sync and blanking signal through delay lines to compensate for the
  -- processing delays incurred by the pixel data.
  hSync_x(hSync_x'high downto 2) <= hSync_r(hSync_r'high-1 downto 1);
  hSync_bo                       <= hSync_r(hSync_r'high);
  blank_x(blank_x'high downto 2) <= blank_r(blank_r'high-1 downto 1);

  -- Get the current pixel from the word of pixel data or read more pixel data from the buffer.
  process(visible_s, pixelDataOut_s, pixelData_r, rd_r, pixelCnt_s)
    variable cntMask_v : std_logic_vector(pixelCnt_s'range);
  begin

    -- Create the mask for the lower bits of the pixel row counter based on the width of the pixel.
    cntMask_v := CONV_STD_LOGIC_VECTOR(pixelData_r'length/PIXEL_WIDTH_C - 1, cntMask_v'length);

    -- Read new pixel data from buffer whenever pixel shift register is empty during visible portion of scan line.
    if visible_s = YES and (pixelCnt_s and cntMask_v) = 0 then
      rd_x <= YES;  -- Read new pixel data whenever lower bits of pixel counter are all zero (shift reg is empty).
    else
      rd_x <= NO;  -- Otherwise, don't read next word of pixel data from the buffer (shift reg is not empty).
    end if;

    -- Get new pixel data from buffer or shift existing pixel data.
    if rd_r = YES then
      -- A read operation was initiated in the previous cycle, so store data from buffer into shift register.
      pixelData_x <= pixelDataOut_s;
    else
      -- Shift pixel data depending on its width so the next pixel is in the LSBs of the shift register.
      pixelData_x(pixelData_x'high-PIXEL_WIDTH_C downto 0) <= pixelData_r(pixelData_x'high downto PIXEL_WIDTH_C);
    end if;

    -- The current pixel is in the lower bits of the pixel data shift register. Zero it if it's in the blanking period.
    if blank_r(blank_r'high) = YES then
      rgb_x <= (others => ZERO);        -- Blank the pixel.
    else
      rgb_x <= pixelData_r(rgb_x'range);  -- Otherwise, store the pixel in an output register.
    end if;
  end process;

  -- Update registers.
  process(rst_i, clk_i)
  begin
    if rst_i = YES then
      rd_r        <= NO;
      hSync_r     <= (others => '1');
      blank_r     <= (others => '0');
      pixelData_r <= (others => '0');
      rgb_r       <= (others => '0');
    elsif rising_edge(clk_i) then
      if cke_r = YES then
        rd_r        <= rd_x;
        hSync_r     <= hSync_x;
        blank_r     <= blank_x;
        pixelData_r <= pixelData_x;
        rgb_r       <= rgb_x;
      end if;
    end if;
  end process;

  -- Output the contents of the pixel output register.
  rgb_o <= rgb_r;

end architecture;



--
-- Keep track of the current column or the beginning address of a row of text being drawn.
--

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use XESS.VgaPckg.all;
use XESS.CommonPCkg.all;

entity TextCounter is
  generic (
    CHAR_SIZE_G     : natural := 8;   -- Width or height of a single character.
    TEXT_SIZE_G     : natural := 80;  -- Number of characters on a text line or the entire screen.
    POSITION_INCR_G : natural := 1  -- Amount to increment position after each character or line.
    );
  port (
    rst_i         : in  std_logic;      -- Synchronous reset.
    clk_i         : in  std_logic;      -- Master clock.
    cke_i         : in  std_logic;      -- Clock enable.
    position_o    : out std_logic_vector(Log2(TEXT_SIZE_G)-1 downto 0);  -- On-screen position of character being displayed.
    subPosition_o : out std_logic_vector(Log2(CHAR_SIZE_G)-1 downto 0);  -- Current pixel position within character being displayed.
    newPosition_o : out std_logic  -- Single-clock pulse when position changes.
    );
end entity;

architecture arch of TextCounter is
  signal position_r    : natural range 0 to TEXT_SIZE_G - 1;
  signal subPosition_r : natural range 0 to CHAR_SIZE_G-1;
begin

  process(clk_i)
  begin
    if rising_edge(clk_i) then
      newPosition_o <= NO;  -- This insures the pulse is only a single clock cycle wide.
      if rst_i = YES then  -- Reset is driven by horizontal or vertical blanking, so positions go to zero at that time.
        position_r    <= 0;
        subPosition_r <= 0;
      elsif cke_i = YES then  -- Enable position update on pixel clock or end-of-line.
        if subPosition_r = CHAR_SIZE_G-1 then  -- An entire character width or text line height has been reached.
          subPosition_r <= 0;  -- Reset once an entire character or text line height is complete and the next one starts.
          position_r    <= position_r + POSITION_INCR_G;  -- Also increment position to next char or line starting position.
          newPosition_o <= YES;         -- Indicate position has been updated.
        else
          subPosition_r <= subPosition_r + 1;  -- Keep incrementing subposition until it hits its maximum.
        end if;
      end if;
    end if;
  end process;

  position_o    <= CONV_STD_LOGIC_VECTOR(position_r, position_o'length);
  subPosition_o <= CONV_STD_LOGIC_VECTOR(subPosition_r, subPosition_o'length);

end architecture;



-- A character generator that can display a page of R x C characters
-- each of which is W x H pixels in black/white. It reads a character
-- code from a dual-port RAM and uses this code to extract the bitmap
-- for the character from a ROM.

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;
use XESS.VgaPckg.all;
use XESS.CommonPckg.all;

entity TextVga is
  generic (
    FREQ_G          : real    := 50.0;  -- Master clock frequency (in MHz).
    CLK_DIV_G       : natural := 1;     -- FREQ_G / CLK_DIV_G = pixel clock.
    CHAR_HEIGHT_G   : natural := 8;     -- Character height in lines.
    NUM_TEXT_COLS_G : natural := 80;    -- Width of text screen in characters.
    NUM_TEXT_ROWS_G : natural := 25;    -- Height of text screen in characters.
    OFFSET_RIGHT_G  : integer := 0;  -- Offset of image to the right in pixels.
    OFFSET_DOWN_G   : integer := 0;     -- Offset of image downward in pixels.
    FIT_TO_SCREEN_G : boolean := false  -- Fit width x length to monitor screen.
    );
  port (
    rst_i            : in  std_logic;   -- Reset.
    clk_i            : in  std_logic;   -- Master clock.
    charAddr_o       : out std_logic_vector;  -- Address to video RAM containing text char codes.
    charCode_i       : in  std_logic_vector;  -- Char code from video RAM.
    charBitmapAddr_o : out std_logic_vector;  -- Address to character generator ROM.
    charBitmapRow_i  : in  std_logic_vector;  -- Row of character pixels from ROM.
    hSync_bo         : out std_logic;   -- Horizontal sync pulse.
    vSync_bo         : out std_logic;   -- Vertical sync pulse.
    rgb_o            : out std_logic_vector  -- Red, green, blue color output bus.
    );
end entity;

architecture arch of TextVga is
  constant CHAR_WIDTH_C : natural := charBitmapRow_i'length;  -- Character width in pixels.
  constant PIXEL_FREQ_C : real    := FREQ_G / real(CLK_DIV_G);  -- Pixel drawing rate.
  signal pixelGate_r    : std_logic;  -- Pulses once every time a new pixel is drawn to the screen.
  signal eol_s          : std_logic;  -- Pulses once on end of each line of text.
  signal eof_s          : std_logic;  -- Pulses once on end of a frame of text.
  signal hBlank_s       : std_logic;  -- Goes high during horizontal blanking interval.
  signal vBlank_s       : std_logic;  -- Goes high during vertical blanking interval.
  signal rowStartAddr_s : std_logic_vector(Log2(NUM_TEXT_COLS_G * NUM_TEXT_ROWS_G)-1 downto 0);  -- Address of first char in a row of text.
  signal textCol_s      : std_logic_vector(Log2(NUM_TEXT_COLS_G)-1 downto 0);  -- Current text column of char being drawn.
  signal newTextCol_s   : std_logic;    -- Pulses when text column increments.
  signal charRow_s      : std_logic_vector(Log2(CHAR_HEIGHT_G)-1 downto 0);  -- Index of row of character being drawn.
  signal pixelReg_r     : std_logic_vector(charBitmapRow_i'range);  -- Pixels for output to display.
  signal rgb_s          : std_logic_vector(rgb_o'range);
begin

  -- Clock divider for generating the pixel clock rate from the master clock.
  process(clk_i, rst_i)
    variable clkDivCnt_v : natural range 0 to CLK_DIV_G;
  begin
    if rst_i = YES then
      clkDivCnt_v := 0;
      pixelGate_r <= YES;
    elsif rising_edge(clk_i) then
      if clkDivCnt_v = CLK_DIV_G-1 then
        clkDivCnt_v := 0;
        pixelGate_r <= YES;  -- Pixel gate pulses once every CLK_DIV_G cycles.
      else
        clkDivCnt_v := clkDivCnt_v + 1;
        pixelGate_r <= NO;
      end if;
    end if;
  end process;

  -- Horizontal & vertical sync generator.
  UHVSyncGen : HVSyncGen
    generic map(
      PIXEL_FREQ_G      => PIXEL_FREQ_C,
      PIXELS_PER_LINE_G => NUM_TEXT_COLS_G * CHAR_WIDTH_C,
      LINES_PER_FRAME_G => NUM_TEXT_ROWS_G * CHAR_HEIGHT_G,
      OFFSET_RIGHT_G    => OFFSET_RIGHT_G,
      OFFSET_DOWN_G     => OFFSET_DOWN_G,
      FIT_TO_SCREEN_G   => FIT_TO_SCREEN_G
      )
    port map(
      rst_i    => rst_i,
      clk_i    => clk_i,
      cke_i    => pixelGate_r,
      eol_o    => eol_s,
      eof_o    => eof_s,
      hSync_bo => hSync_bo,
      vSync_bo => vSync_bo,
      hBlank_o => hBlank_s,
      vBlank_o => vBlank_s
      );

  -- Keep track of the current column of text being displayed.
  UColTextCounter : TextCounter
    generic map (
      CHAR_SIZE_G     => CHAR_WIDTH_C,
      TEXT_SIZE_G     => NUM_TEXT_COLS_G,
      POSITION_INCR_G => 1
      )
    port map (
      rst_i         => hBlank_s,        -- Horizontal blank resets column to 0.
      clk_i         => clk_i,
      cke_i         => pixelGate_r,
      position_o    => textCol_s,
      newPosition_o => newTextCol_s  -- Pulses when new column of text begins.
      );

  -- Keep track of the starting address of the current text row being displayed.
  URowTextCounter : TextCounter
    generic map (
      CHAR_SIZE_G     => CHAR_HEIGHT_G,
      TEXT_SIZE_G     => NUM_TEXT_COLS_G * NUM_TEXT_ROWS_G,
      POSITION_INCR_G => NUM_TEXT_COLS_G
      )
    port map (
      rst_i         => vBlank_s,        -- Vertical blank resets address to 0.
      clk_i         => clk_i,
      cke_i         => eol_s,  -- Update address after a complete line of text characters is drawn.
      position_o    => rowStartAddr_s,
      subPosition_o => charRow_s  -- Current row of pixels being drawn in the line of text.
      );

  -- Address of character to display is starting address of the row of text + column of the current character being drawn.
  charAddr_o <= rowStartAddr_s + textCol_s;

  -- Use the character display code and the current pixel row index to get the current row of pixels for the character bitmap.
  charBitmapAddr_o <= CONV_STD_LOGIC_VECTOR(CONV_INTEGER(charCode_i) * CHAR_HEIGHT_G + CONV_INTEGER(charRow_s), charBitmapAddr_o'length);

  -- Shift the row of character pixels out to the display.
  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if hBlank_s = YES or vBlank_s = YES then
        pixelReg_r <= (others => '0');  -- Zero the pixels during blanking intervals.
      elsif newTextCol_s = YES then
        pixelReg_r <= charBitmapRow_i;  -- Load new row of pixels every time text column changes.
      elsif pixelGate_r = YES then
        pixelReg_r <= pixelReg_r(pixelReg_r'high-1 downto 0) & ZERO;  -- Shift out pixels to display.
      end if;
    end if;
  end process;

  -- Output black/white pixels (R, G and B all the same value).
  rgb_s <= (others => pixelReg_r(pixelReg_r'high));
  rgb_o <= rgb_s;

end architecture;



------------------------------------------------------------------------------------------
-- This is a test design that reads an image from SDRAM and displays it on a VGA monitor.
------------------------------------------------------------------------------------------

library IEEE, XESS;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_ARITH.all;
use IEEE.STD_LOGIC_UNSIGNED.all;
use XESS.VgaPckg.all;
use XESS.ClkgenPckg.all;
use XESS.SdramCntlPckg.all;
use XESS.CommonPckg.all;
use work.XessBoardPckg.all;


entity PixelVgaTest is
  generic(
    FREQ_G            : real    := 100.0;  -- Master clock frequency (Mhz).
    CLK_DIV_G         : natural := 2;   -- Pixel clock = FREQ / CLK_DIV.
    NUM_RGB_BITS_G    : natural := 5;  -- #Bits in each R,G,B component of a pixel.
    PIXELS_PER_LINE_G : natural := 800;    -- Width of image in pixels.
    LINES_PER_FRAME_G : natural := 600  -- Height of image in scanlines.
    );
  port(
    clk_i     : in    std_logic;        -- 12 MHz XuLA clock.
    hSync_bo  : out   std_logic;        -- VGA horizontal sync.
    vSync_bo  : out   std_logic;        -- VGA vertical sync.
    red_o     : out   std_logic_vector(NUM_RGB_BITS_G-1 downto 1);  -- VGA red signals (only 4 bits).
    green_o   : out   std_logic_vector(NUM_RGB_BITS_G-1 downto 0);  -- VGA green signals.
    blue_o    : out   std_logic_vector(NUM_RGB_BITS_G-1 downto 0);  -- VGA blue signals.
    -- SDRAM I/O.
    sdCke_o   : out   std_logic;        -- SDRAM clock enable.
    sdClk_o   : out   std_logic;        -- Clock to SDRAM.
    sdClkFb_i : in    std_logic;  -- Clock from SDRAM after I/O and PCB delays.
    sdCe_bo   : out   std_logic;        -- SDRAM chip-enable.
    sdRas_bo  : out   std_logic;        -- SDRAM row address strobe.
    sdCas_bo  : out   std_logic;        -- SDRAM column address strobe.
    sdWe_bo   : out   std_logic;        -- SDRAM write enable.
    sdDqml_o  : out   std_logic;        -- SDRAM data bus lower-byte qualifier.
    sdDqmh_o  : out   std_logic;        -- SDRAM data bus upper-byte qualifier.
    sdBs_o    : out   std_logic_vector(1 downto 0);        -- SDRAM bank address bits.
    sdAddr_o  : out   std_logic_vector(SDRAM_SADDR_WIDTH_C-1 downto 0);  -- SDRAM row/column address.
    sdData_io : inout std_logic_vector(SDRAM_DATA_WIDTH_C-1 downto 0)  -- SDRAM in/out databus.
    );
end entity;


architecture arch of PixelVgaTest is

  signal clk_s           : std_logic;  -- 100 MHz clock generated from 12 MHz XuLA clock.
  signal rst_r           : std_logic;   -- reset signal.
  signal eof_s           : std_logic;  -- end-of-frame signal from VGA controller.
  signal earlyOpBegun_s  : std_logic;  -- indicates when an SDRAM read operation has begun.
  signal rdDone_s        : std_logic;  -- indicates when data read from the SDRAM is available.
  signal full_s, full_bs : std_logic;  -- indicates when the VGA pixel buffer is full.
  signal address_r       : std_logic_vector(SDRAM_HADDR_WIDTH_C-1 downto 0);  -- SDRAM address counter .
  signal pixel_s         : std_logic_vector(SDRAM_DATA_WIDTH_C-1 downto 0);  -- pixel values from SDRAM.
  signal rgb_s           : std_logic_vector(14 downto 0);  -- RGB color bus.

begin

  -- Generate a 100 MHz clock from the 12 MHz clock on the XuLA board.
  UClkGen : ClkGen
    generic map (BASE_FREQ_G => BASE_FREQ_C, CLK_MUL_G => 25, CLK_DIV_G => 3)
    port map (I              => clk_i, clkToLogic_o => clk_s);
  
  sdClk_o <= clk_s;  -- Clock SDRAM at 100 MHz. This signal feeds-back to sdClkFb_i.

  -- Generate a reset pulse at the start of operations.
  process(sdClkFb_i)
    variable rstCntr_r : natural range 0 to 15 := 15;
  begin
    if rising_edge(sdClkFb_i) then
      if rstCntr_r /= 0 then
        rstCntr_r := rstCntr_r - 1;
        rst_r     <= YES;
      else
        rst_r <= NO;  --- Release reset after initial interval..
      end if;
    end if;
  end process;

  -- Update the SDRAM address counter.
  process(sdClkFb_i)
  begin
    if rising_edge(sdClkFb_i) then
      if eof_s = YES or rst_r = YES then
        address_r <= (others => ZERO);  -- Reset the address at the end of a video frame.
      elsif earlyOpBegun_s = YES then
        address_r <= address_r + 1;  -- Go to the next address once the read of the current address has begun.
      end if;
    end if;
  end process;

  -- SDRAM controller used to get pixel data from the external SDRAM.
  USdramCntl : SdramCntl
    generic map(
      FREQ_G        => FREQ_G,
      PIPE_EN_G     => true
      )
    port map(
      clk_i          => sdClkFb_i,      -- Master clock fed back from SDRAM.
      rst_i          => rst_r,
      rd_i           => full_bs,  -- Initiate a read when the VGA pixel buffer is not full.
      addr_i         => address_r,  -- The address to read from is stored in the address counter.
      earlyOpBegun_o => earlyOpBegun_s,  -- Indicate when the read operation has actually begun.
      rdDone_o       => rdDone_s,  -- Indicate when the data from the read operation is available.
      data_o         => pixel_s,  -- This is the pixel data that was read from the SDRAM.
      sdCke_o        => sdCke_o,        -- SDRAM clock enable.
      sdCe_bo        => sdCe_bo,        -- SDRAM chip-enable.
      sdRas_bo       => sdRas_bo,       -- SDRAM RAS.
      sdCas_bo       => sdCas_bo,       -- SDRAM CAS.
      sdWe_bo        => sdWe_bo,        -- SDRAM write-enable.
      sdDqml_o       => sdDqml_o,       -- SDRAM data bus lower-byte qualifier.
      sdDqmh_o       => sdDqmh_o,       -- SDRAM data bus upper-byte qualifier.
      sdBs_o         => sdBs_o,         -- SDRAM bank address.
      sdAddr_o       => sdAddr_o,       -- SDRAM address.
      sdData_io      => sdData_io       -- Data to/from SDRAM.
      );

  -- Pixel-mode VGA generator.
  UPixelVga : PixelVga
    generic map (
      FREQ_G            => FREQ_G,
      CLK_DIV_G         => CLK_DIV_G,
      PIXELS_PER_LINE_G => PIXELS_PER_LINE_G,
      LINES_PER_FRAME_G => LINES_PER_FRAME_G,
      FIT_TO_SCREEN_G   => true
      )
    port map (
      rst_i         => rst_r,
      clk_i         => sdClkFb_i,  -- Use the fed-back SDRAM clock so VGA generator is in sync with SDRAM.
      wr_i          => rdDone_s,  -- Write to pixel buffer when the data read from SDRAM is available.
      pixelDataIn_i => pixel_s,         -- Pixel data from SDRAM.
      full_o        => full_s,    -- Indicates when the pixel buffer is full.
      eof_o         => eof_s,  -- Indicates when the VGA generator has finished a video frame.
      hSync_bo      => hSync_bo,        -- Horizontal sync.
      vSync_bo      => vSync_bo,        -- Vertical sync.
      rgb_o         => rgb_s            -- Red, green, blue color bus.
      );

  -- Split the RGB color bus into its individual components.
  red_o   <= rgb_s(14 downto 11); -- Ignore the LSB red bit.
  green_o <= rgb_s(9 downto 5);
  blue_o  <= rgb_s(4 downto 0);

  full_bs <= not full_s;  -- Negate the full signal for use in controlling the SDRAM read operation.

end architecture;
