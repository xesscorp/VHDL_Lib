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
use work.XessBoardPckg.all;

package AdcInterfacesPckg is

  component Adc_088S_108S_128S_Intfc is
    generic (
      FREQ_G        : real := 96.0;     -- Master clock frequency in MHz.
      SAMPLE_FREQ_G : real := 1.0       -- Sample freq in MHz.
      );
    port (
      clk_i         : in  std_logic;    -- Master clock input.
      -- Sampling setup.
      numChans_i    : in  std_logic_vector;  -- Number of channels to sample: 1,2,3,4,5,6,7, or 8.
      analogChans_i : in  std_logic_vector;  -- Concatenated 3-bit indices of channels to sample.
      startAddr_i   : in  std_logic_vector;  -- Start address for storing samples.
      numSamples_i  : in  std_logic_vector;  --# of samples to store.
      -- Sampling control and status.
      run_i         : in  std_logic := NO;  -- When true, sampling is enabled.
      busy_o        : out std_logic := NO;  -- When true, sampling is occurring.
      done_o        : out std_logic := NO;  -- When true, sampling run has completed.
      -- RAM interface for storing samples.
      wr_o          : out std_logic := NO;  -- Write strobe to RAM.
      sampleAddr_o  : out std_logic_vector;  -- Current sample RAM address.
      sampleData_o  : out std_logic_vector;  -- Current sample value for RAM.
      wrDone_i      : in  std_logic := NO;  -- True when sample write to RAM has completed.
      -- Interface signals to ADC chip.
      cs_bo         : out std_logic := HI;  -- Active-low ADC chip-select.
      sclk_o        : out std_logic := HI;  -- ADC clock.
      mosi_o        : out std_logic;    -- Output to ADC serial input.
      miso_i        : in  std_logic     -- Input from ADC serial output.
      );
  end component;

end package;




--****************************************************************************
-- Interface to TI ADC088S, ADC108S, and ADC128S analog-to-digital converters.
--
-- This interface grabs a number of samples from the ADC chip and stores them
-- into RAM.
--
-- HOW TO USE:
--   1. Apply a clock input that can be easily divided down to 16 MHz.
--   2. Apply the number of channels to sample: 1, 2, ..., 7 or 8. (Never 0!)
--   3. Concatenate and apply the 3-bit codes for the analog channels you want to sample.
--      (For example, apply the code "101001" to sample channels 5 and 1.) 
--   4. Apply the lower RAM address where the sample storage will begin.
--   5. Apply the number of samples you want to collect across all channels.
--   6. Raise the run input.
--   7. Wait until the done output is true. The samples are now interleaved in RAM.
--   8. Once you have processed the samples in RAM, lower the run input.
--   9. To do another sample run, go to step #2.
--****************************************************************************

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use XESS.CommonPckg.all;
use work.XessBoardPckg.all;

entity Adc_088S_108S_128S_Intfc is
  generic (
    FREQ_G        : real := 96.0;       -- Master clock frequency in MHz.
    SAMPLE_FREQ_G : real := 1.0         -- Sample freq in MHz.
    );
  port (
    clk_i         : in  std_logic;      -- Master clock input.
    -- Sampling setup.
    numChans_i    : in  std_logic_vector;  -- Number of channels to sample: 1,2,3,4,5,6,7, or 8.
    analogChans_i : in  std_logic_vector;  -- Concatenated 3-bit indices of channels to sample.
    startAddr_i   : in  std_logic_vector;  -- Start address for storing samples.
    numSamples_i  : in  std_logic_vector;  --# of samples to store.
    -- Sampling control and status.
    run_i         : in  std_logic := NO;  -- When true, sampling is enabled.
    busy_o        : out std_logic := NO;  -- When true, sampling is occurring.
    done_o        : out std_logic := NO;  -- When true, sampling run has completed.
    -- RAM interface for storing samples.
    wr_o          : out std_logic := NO;  -- Write strobe to RAM.
    sampleAddr_o  : out std_logic_vector;  -- Current sample RAM address.
    sampleData_o  : out std_logic_vector;  -- Current sample value for RAM.
    wrDone_i      : in  std_logic := NO;  -- True when sample write to RAM has completed.
    -- Interface signals to ADC chip.
    cs_bo         : out std_logic := HI;  -- Active-low ADC chip-select.
    sclk_o        : out std_logic := HI;  -- ADC clock.
    mosi_o        : out std_logic;      -- Output to ADC serial input.
    miso_i        : in  std_logic       -- Input from ADC serial output.
    );
end entity;

architecture arch of Adc_088S_108S_128S_Intfc is
  constant MAX_ADDR_C       : natural   := 2**sampleAddr_o'length - 1;  -- Highest possible RAM address.
  constant BITS_PER_FRAME_C : natural   := 16;  -- # bits in ADC conversion frame.
  constant CLK_DIVISOR_C    : natural   := integer(round(FREQ_G / (SAMPLE_FREQ_G * real(BITS_PER_FRAME_C))));
  constant CLK_LO_CNT_C     : natural   := CLK_DIVISOR_C / 2;
  constant CLK_HI_CNT_C     : natural   := CLK_DIVISOR_C - CLK_LO_CNT_C;
  signal clkDivider_r       : natural range 0 to IntMax(CLK_LO_CNT_C, CLK_HI_CNT_C) - 1 := 0;
  signal adcClk_r           : std_logic := HI;  -- ADC clock signal generated from master clock.
  signal busy_r             : std_logic := NO;  -- True when gathering samples from ADC.
  signal wr_r               : std_logic := HI;  -- True when writing sample to RAM.
  signal done_s             : std_logic := NO;  -- True when all samples are gathered.
  signal shiftReg_r         : unsigned(BITS_PER_FRAME_C-1 downto 0);  -- Bits shifted in and out of the ADC.
  signal bitCntr_r          : natural range 0 to BITS_PER_FRAME_C-1;  -- Shift reg bit counter.
  subtype Address_t is natural range 0 to MAX_ADDR_C;  -- Address sub type.
  signal sampleCntr_r       : Address_t := 1;  -- Holds # of samples that still need to be taken.
  signal sampleAddr_r       : Address_t;  -- Holds RAM address where next sample will be stored.
  signal addrInc_r          : Address_t;  -- Amount to increment sample address after each sample.
  signal sampleData_r       : unsigned(sampleData_o'range);  -- Sample from ADC chip.
  signal chanShiftReg_r     : unsigned(analogChans_i'high downto 0);  -- Holds sequence of channels to sample.
  signal chanCntr_r         : natural range 0 to 2**numChans_i'length; -- Stores number of channels left to sample.
begin

  -- The sample run is done when the sample counter reaches 0.
  done_s <= YES when sampleCntr_r = 0 else NO;

  process(clk_i)
  begin
    if rising_edge(clk_i) then

      -- Release the write strobe and inc address once a sample has been written into RAM. 
      -- This is done at the master clock rate so we don't miss it.
      if wr_r = YES and wrDone_i = YES then
        wr_r         <= NO;
        sampleAddr_r <= sampleAddr_r + addrInc_r;
        addrInc_r    <= 1;  -- 1st sample is garbage so doesn't increment, but each sample thereafter does increment.
      end if;

      -- All the other operations happen at the ADC bit-rate clock.
      if clkDivider_r /= 0 then         -- No edge on the ADC clock, yet.
        clkDivider_r <= clkDivider_r - 1;
        
      else  -- clkDivider_r = 0, so generate an edge on the ADC clock.

        if adcClk_r = LO then  -- ADC clock is currently low, so create a rising edge on ADC clock.
          adcClk_r     <= HI;
          clkDivider_r <= CLK_HI_CNT_C - 1;  -- Reload master clock divider with high duration of ADC clock.

        else  -- ADC clock is currently high, so create a falling edge on ADC clock.
          adcClk_r     <= LO;
          clkDivider_r <= CLK_LO_CNT_C - 1;  -- Reload master clock divider with low duration of ADC clock.

          -- All the ADC interface operations happen on the falling edge of the ADC clock.

          -- By default, shift out bits to ADC and shift in bits from ADC.
          shiftReg_r <= shiftReg_r(shiftReg_r'high-1 downto 0) & miso_i;
          bitCntr_r  <= bitCntr_r + 1;

          -- If the run input is not asserted, then clear the busy
          -- and done flags in preparation for when the next sampling run does begin.
          if run_i = NO then            -- run=NO, busy=XXX, done=XXX.
            busy_r         <= NO;
            sampleCntr_r   <= TO_INTEGER(unsigned(numSamples_i));  -- This clears the done flag.
            sampleAddr_r   <= TO_INTEGER(unsigned(startAddr_i));
            addrInc_r      <= 0;  -- First sample is garbage, so this will overwrite it on the second sample.
            chanShiftReg_r <= unsigned(analogChans_i);
            chanCntr_r     <= TO_INTEGER(unsigned(numChans_i));

          -- The run input is asserted, but sampling isn't occurring.
          elsif busy_r = NO then        -- run=YES, busy=NO, done=XXX.
            if done_s = NO then         -- run=YES, busy=NO, done=NO.
              -- A sampling run has not been completed, so get ready to start one.
              busy_r     <= YES;  -- Sampling is occurring, ADC is enabled.
              shiftReg_r <= "00" & chanShiftReg_r(2 downto 0) & "00000000000";  -- Init shift register.
              bitCntr_r  <= 1;  -- First bit of shift register is being output.
              if chanCntr_r /= 1 then  -- Rotate to the next index of channel to sample.
                chanShiftReg_r <= rotate_right(chanShiftReg_r, 3);
                chanCntr_r     <= chanCntr_r - 1;
              else -- Reload to sample the list of channels again.
                chanShiftReg_r <= unsigned(analogChans_i);
                chanCntr_r     <= TO_INTEGER(unsigned(numChans_i));
              end if;
            else                        -- run=YES, busy=NO, done=YES.
              -- A sampling run has completed, so just hold still.
              null;
            end if;

          -- Sampling is occurring but all the samples haven't been collected.
          elsif done_s = NO then        -- run=YES, busy=YES, done=NO.
            if bitCntr_r = 0 then
              -- Output the sample taken in the previous 16-cycle interval.
              sampleData_r <= "0000" & shiftReg_r(10 downto 0) & miso_i;
              wr_r         <= YES;
              sampleCntr_r <= sampleCntr_r - 1;  -- Got another sample, so dec the sample counter.
              shiftReg_r   <= "00" & chanShiftReg_r(2 downto 0) & "00000000000";  -- Init shift register for the next sample.
              if chanCntr_r /= 1 then  -- Rotate to the next index of channel to sample.
                chanShiftReg_r <= rotate_right(chanShiftReg_r, 3);
                chanCntr_r     <= chanCntr_r - 1;
              else -- Reload to sample the list of channels again.
                chanShiftReg_r <= unsigned(analogChans_i);
                chanCntr_r     <= TO_INTEGER(unsigned(numChans_i));
              end if;
            end if;

          -- Sampling run has completed, but don't shut off the ADC chip-select until
          -- all the bits in the current frame are clocked out.
          else                          -- run=YES, busy=YES, done=YES.
            if bitCntr_r = BITS_PER_FRAME_C - 1 then
              busy_r <= NO;
            end if;
          end if;
          
        end if;  -- ADC clock is currently high.
      end if;  -- CLock divider = 0.
    end if;  -- Rising edge of master clock.
  end process;

  -- Output control and data signal to the ADC chip.
  sclk_o <= adcClk_r;    -- The ADC clock input is always toggling.
  cs_bo  <= not busy_r;  -- The ADC is enabled when the interface is busy sampling.
  mosi_o <= shiftReg_r(shiftReg_r'high);  -- MSB of shift register goes to ADC DIN pin.

  -- Output address, data and write signals for storing the sample into RAM.
  sampleAddr_o <= std_logic_vector(TO_UNSIGNED(sampleAddr_r, sampleAddr_o'length));
  sampleData_o <= std_logic_vector(sampleData_r);
  wr_o         <= wr_r;

  -- Output the signals that show the status of the sampling run.
  busy_o <= busy_r;
  done_o <= done_s;

end architecture;

