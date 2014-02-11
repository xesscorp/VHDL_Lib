--**********************************************************************
-- Copyright (c) 1997-2014 by XESS Corp <http://www.xess.com>.
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

--------------------------------------------------------------------
--    Loadable LFSR random number generator.
--------------------------------------------------------------------


library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use IEEE.numeric_std.all;
use XESS.CommonPckg.all;


package RandPckg is

  component RandGen is
    port(
      clk_i  : in  std_logic;           -- Main clock input.
      cke_i  : in  std_logic;           -- Clock enable.
      ld_i   : in  std_logic;           -- Load enable for seed_i.
      seed_i : in  std_logic_vector;    -- Random number seed_i value.
      rand_o : out std_logic_vector     -- Output for random number.
      );
  end component;

end package;



library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use IEEE.numeric_std.all;
use XESS.CommonPckg.all;


entity RandGen is
  port(
    clk_i  : in  std_logic;             -- Main clock input.
    cke_i  : in  std_logic;             -- Clock enable.
    ld_i   : in  std_logic;             -- Load enable for seed_i.
    seed_i : in  std_logic_vector;      -- Random number seed_i value.
    rand_o : out std_logic_vector       -- Output for random number.
    );
end entity;


architecture arch of RandGen is
  signal r_r      : std_logic_vector(rand_o'range);  -- random number shift register
  signal newBit_s : std_logic;          -- feedback bit into LSB of LFSR
begin

  -- Use the length parameter to select the bits in the shift register
  -- which will be XOR'ed to compute the bit fed back into the 
  -- least significant bit of the shift register.
  newBit_s <=
    r_r(1) xor r_r(0)                           when r_r'length = 2 else
    r_r(2) xor r_r(1)                           when r_r'length = 3 else
    r_r(3) xor r_r(2)                           when r_r'length = 4 else
    r_r(4) xor r_r(2)                           when r_r'length = 5 else
    r_r(5) xor r_r(4)                           when r_r'length = 6 else
    r_r(6) xor r_r(3)                           when r_r'length = 7 else
    r_r(7) xor r_r(5) xor r_r(4) xor r_r(3)     when r_r'length = 8 else
    r_r(11) xor r_r(10) xor r_r(7) xor r_r(5)   when r_r'length = 12 else
    r_r(15) xor r_r(12) xor r_r(11) xor r_r(10) when r_r'length = 16 else
    r_r(19) xor r_r(16)                         when r_r'length = 20 else
    r_r(23) xor r_r(22) xor r_r(21) xor r_r(16) when r_r'length = 24 else
    r_r(27) xor r_r(24)                         when r_r'length = 28 else
    r_r(31) xor r_r(30) xor r_r(29) xor r_r(9)  when r_r'length = 32 else
--    r_r(31) xor r_r(29) xor r_r(6) xor r_r(3)  when r_r'length = 32 else
    r_r(r_r'length-1);

  process(clk_i)
  begin
    if rising_edge(clk_i) then  -- Update shift register on rising clock edge.
      if ld_i = YES then
        r_r <= seed_i;                  -- Load with seed value.
      elsif cke_i = YES then
        r_r <= r_r(r_r'high-1 downto 0) & newBit_s;  -- Generate new random value.
      end if;
    end if;
  end process;

  rand_o <= r_r;  -- Output the random number in the shift register.

end architecture;
