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
