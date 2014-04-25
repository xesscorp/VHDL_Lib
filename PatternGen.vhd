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


library IEEE;
use IEEE.STD_LOGIC_1164.all;

package PatternGenPckg is

--**********************************************************************
-- Module for outputting a sequence of values from a ROM.
--**********************************************************************
  component PatternGen is
    generic (
      PATTERN_FILE_NAME_G : string      -- MIF file where patterns are stored.
      );
    port (
      clk_i       : in  std_logic;  -- Input clock that determines rate of sample outputs.
      load_i      : in  std_logic;      -- Load starting address for pattern.
      startAddr_i : in  std_logic_vector;  -- Starting address for pattern.
      length_i    : in  std_logic_vector;  -- # of samples in pattern. MUST NOT BE 0!
      sample_o    : out std_logic_vector;  -- Sample from pattern, one sample per clock cycle.
      busy_o      : out std_logic       -- High while samples are being output.
      );
  end component;

  component SineGen is
    generic (
      NUM_SAMPLES_G : natural := 256  -- Number of samples in quarter wave of sinusoid.
      );
    port (
      clk_i       : in  std_logic;  -- Input clock that determines rate of sample outputs.
      freq_i      : in  std_logic_vector;  -- Larger value -> sine wave of higher frequency.
      amplitude_i : in  std_logic_vector;  -- Amplitude of the sinusoid.
      offset_i    : in  std_logic_vector;  -- Offset of the sinusoid from 0.
      sine_o      : out std_logic_vector   -- Unipolar, sinusoidal value.
      );
  end component;

end package;




--**********************************************************************
-- Module for outputting a sequence of values from a ROM.
--**********************************************************************

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use std.textio.all;

library XESS;
use XESS.CommonPckg.all;
use XESS.PatternGenPckg.all;

entity PatternGen is
  generic (
    PATTERN_FILE_NAME_G : string        -- MIF file where patterns are stored.
    );
  port (
    clk_i       : in  std_logic;  -- Input clock that determines rate of sample outputs.
    load_i      : in  std_logic;        -- Load starting address for pattern.
    startAddr_i : in  std_logic_vector;  -- Starting address for pattern.
    length_i    : in  std_logic_vector;  -- # of samples in pattern. MUST NOT BE 0!
    sample_o    : out std_logic_vector;  -- Sample from pattern, one sample per clock cycle.
    busy_o      : out std_logic         -- High while samples are being output.
    );
end entity;

architecture arch of PatternGen is

  constant ROM_LENGTH_C : natural := 2 ** startAddr_i'length;  -- # address bits determines ROM size.
  subtype RomWord_t is std_logic_vector(sample_o'range);   -- ROM word type.
  type Rom_t is array (0 to ROM_LENGTH_C-1) of RomWord_t;  -- ROM array type.

  --**********************************************************************
  -- This function reads the bit vectors from the MIF file and initializes
  -- the pattern ROM with them.
  --**********************************************************************
  impure function InitRom(mifFileName_i : in string) return Rom_t is
    file mifFile_f     : text open read_mode is mifFileName_i;
    variable mifLine_v : line;
    variable tempBv_v  : bit_vector(sample_o'length-1 downto 0);
    variable tempRom_v : Rom_t;
  begin
    for i in Rom_t'range loop
      readline(mifFile_f, mifLine_v);
      read(mifLine_v, tempBv_v);
      tempRom_v(i) := to_stdlogicvector(tempBv_v);
    end loop;
    return tempRom_v;
  end function;

  constant patternRom_r : Rom_t := InitRom(PATTERN_FILE_NAME_G);  -- ROM declaration & initialization from file.

  signal addr_r : natural range 0 to ROM_LENGTH_C-1 := 0;  -- Address of next sample in pattern ROM.
  signal cnt_r  : natural range 0 to ROM_LENGTH_C-1 := 0;  -- Number of samples left in pattern.
  
begin
  -- Read the pattern ROM and output the current sample every clock cycle. Increment the address to 
  -- the next sample as long as the number of samples left to output is not zero. Once the entire 
  -- pattern is output, the starting address and length of the next pattern can be loaded.
  -- Output a busy signal as long as a pattern is being output.
  process(clk_i)
  begin
    if rising_edge(clk_i) then
      sample_o <= patternRom_r(addr_r);
      if cnt_r /= 0 then
        addr_r <= addr_r + 1;
        cnt_r  <= cnt_r - 1;
      elsif load_i = HI then
        addr_r <= TO_INTEGER(unsigned(startAddr_i));
        cnt_r  <= TO_INTEGER(unsigned(length_i)) - 1;
      end if;
    end if;
  end process;
  busy_o <= HI when cnt_r /= 0 else LO;
end architecture;




--**********************************************************************
-- Module for outputting a sinusoidal sequence.
--**********************************************************************

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.math_real.all;
use IEEE.NUMERIC_STD.all;

library XESS;
use XESS.CommonPckg.all;

entity SineGen is
  generic (
    NUM_SAMPLES_G : natural := 512  -- Number of samples in quarter wave of sinusoid.
    );
  port (
    clk_i       : in  std_logic;  -- Input clock that determines rate of sample outputs.
    freq_i      : in  std_logic_vector;  -- Larger value -> sine wave of higher frequency.
    amplitude_i : in  std_logic_vector;  -- Amplitude of the sinusoid.
    offset_i    : in  std_logic_vector;  -- Offset of the sinusoid from 0.
    sine_o      : out std_logic_vector  -- Unipolar, sinusoidal value.
    );
end entity;

architecture arch of SineGen is

  constant SAMPLE_WIDTH_C      : natural := sine_o'length;
  constant MAX_SAMPLE_C        : natural := 2 ** SAMPLE_WIDTH_C - 1;
  constant ADDR_WIDTH_C        : natural := integer(ceil(log2(real(NUM_SAMPLES_G))));
  constant NUM_SAMPLES_C       : natural := 2 ** ADDR_WIDTH_C;
  constant PHASE_ACCUM_WIDTH_C : natural := freq_i'length;
  signal phase_accum_r         : unsigned(PHASE_ACCUM_WIDTH_C-1 downto 0) := (others=>'0');
  alias half_r is phase_accum_r(phase_accum_r'high);
  alias addr_r is phase_accum_r(phase_accum_r'high-1 downto phase_accum_r'high-1-ADDR_WIDTH_C+1);
  alias remainder_r is phase_accum_r(phase_accum_r'high-ADDR_WIDTH_C-1 downto 0);

  subtype RomWord_t is unsigned(17 downto 0);              -- ROM word type.
  type Rom_t is array (0 to NUM_SAMPLES_C-1) of RomWord_t;  -- ROM array type.

  --**********************************************************************
  -- This function loads the pattern ROM with sinusoidal samples from [0,PI).
  --**********************************************************************
  impure function InitRom(amplitude_i : in natural) return Rom_t is
    variable tempRom_v   : Rom_t;
    constant angleStep_c : real := MATH_PI / real(Rom_t'length);
  begin
    for i in Rom_t'range loop
      tempRom_v(i) := TO_UNSIGNED(integer(floor(real(amplitude_i) * sin(real(i) * angleStep_c) + 0.5)), RomWord_t'length);
    end loop;
    return tempRom_v;
  end function;

  constant patternRom_r : Rom_t := InitRom(2**RomWord_t'length-1);  -- ROM declaration & initialization.

  constant PROD_HI_C : natural := amplitude_i'length + RomWord_t'length - 1;
  constant PROD_LO_C : natural := PROD_HI_C - sine_o'length + 1;
  signal product_s   : unsigned(PROD_HI_C downto PROD_LO_C);  -- Just for delimiting the range of multiplier product.
  signal ac_r        : unsigned(PROD_HI_C downto 0);
  signal sine_r      : unsigned(sine_o'range);
  constant ROUND_C   : unsigned(ac_r'range) := (PROD_LO_C-1=>ONE, others=>ZERO);

begin

  -- Read the sinusoid ROM and output the current sample every clock cycle. 
  -- Increment the accumulator by the frequency input. 
  process(clk_i)
    variable addr_v : natural range 0 to NUM_SAMPLES_C-1;
  begin
    if rising_edge(clk_i) then
      -- The upper bit of the accumulator determines whether sinusoid is positive or negative. 
      -- The middle bits determine the location within the sinusoid lookup table.
      -- The lower bits store the remainder.
      phase_accum_r <= phase_accum_r + unsigned(freq_i);
      addr_v := TO_INTEGER(addr_r);
      ac_r   <= unsigned(amplitude_i) * patternRom_r(addr_v) + ROUND_C;
      if half_r = ZERO then
          sine_r <= unsigned(offset_i) + ac_r(product_s'range);
      else
          sine_r <= unsigned(offset_i) - ac_r(product_s'range);
      end if;
    end if;
  end process;

  sine_o <= std_logic_vector(sine_r);
  
end architecture;
