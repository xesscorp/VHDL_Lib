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

----------------------------------------------------------------------------------
-- Commonly-used functions and constants.
----------------------------------------------------------------------------------


library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

package CommonPckg is

  constant YES  : std_logic := '1';
  constant NO   : std_logic := '0';
  constant HI   : std_logic := '1';
  constant LO   : std_logic := '0';
  constant ONE  : std_logic := '1';
  constant ZERO : std_logic := '0';
  constant HIZ  : std_logic := 'Z';

  -- FPGA chip families.
  type FpgaFamily_t is (SPARTAN3A_E, SPARTAN6_E);

  -- XESS FPGA boards.
  type XessBoard_t is (XULA_E, XULA2_E);

  -- Convert a Boolean to a std_logic.
  function BooleanToStdLogic(b : in boolean) return std_logic;

  -- Find the base-2 logarithm of a number.
  function Log2(v : in natural) return natural;

  -- Select one of two integers based on a Boolean.
  function IntSelect(s : in boolean; a : in integer; b : in integer) return integer;

  -- Select one of two reals based on a Boolean.
  function RealSelect(s : in boolean; a : in real; b : in real) return real;

  -- Convert a binary number to a graycode number.
  function BinaryToGray(b : in std_logic_vector) return std_logic_vector;

  -- Convert a graycode number to a binary number.
  function GrayToBinary(g : in std_logic_vector) return std_logic_vector;

  -- Find the maximum of two integers.
  function IntMax(a : in integer; b : in integer) return integer;

end package;



library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;


package body CommonPckg is

  -- Convert a Boolean to a std_logic.
  function BooleanToStdLogic(b : in boolean) return std_logic is
    variable s : std_logic;
  begin
    if b then
      s := '1';
    else
      s := '0';
    end if;
    return s;
  end function BooleanToStdLogic;

  -- Find the base 2 logarithm of a number.
  function Log2(v : in natural) return natural is
    variable n    : natural;
    variable logn : natural;
  begin
    n := 1;
    for i in 0 to 128 loop
      logn := i;
      exit when (n >= v);
      n    := n * 2;
    end loop;
    return logn;
  end function Log2;

  -- Select one of two integers based on a Boolean.
  function IntSelect(s : in boolean; a : in integer; b : in integer) return integer is
  begin
    if s then
      return a;
    else
      return b;
    end if;
    return a;
  end function IntSelect;

  -- Select one of two reals based on a Boolean.
  function RealSelect(s : in boolean; a : in real; b : in real) return real is
  begin
    if s then
      return a;
    else
      return b;
    end if;
    return a;
  end function RealSelect;

  -- Convert a binary number to a graycode number.
  function BinaryToGray(b : in std_logic_vector) return std_logic_vector is
    variable g : std_logic_vector(b'range);
  begin
    for i in b'low to b'high-1 loop
      g(i) := b(i) xor b(i+1);
    end loop;
    g(b'high) := b(b'high);
    return g;
  end function BinaryToGray;

  -- Convert a graycode number to a binary number.
  function GrayToBinary(g : in std_logic_vector) return std_logic_vector is
    variable b : std_logic_vector(g'range);
  begin
    b(b'high) := g(b'high);
    for i in g'high-1 downto g'low loop
      b(i) := b(i+1) xor g(i);
    end loop;
    return b;
  end function GrayToBinary;

  -- Find the maximum of two integers.
  function IntMax(a : in integer; b : in integer) return integer is
  begin
    if a > b then
      return a;
    else
      return b;
    end if;
    return a;
  end function IntMax;

end package body;
