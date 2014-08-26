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

library IEEE, XESS;
use IEEE.math_real.all;
use IEEE.std_logic_1164.all;
use XESS.CommonPckg.all;

package FifoPckg is

  -- 255 x 16 FIFO with common read and write clock.
  component Fifo255x16cc is
    port (
      clk_i   : in  std_logic;          -- master clock
      rst_i   : in  std_logic                     := NO;  -- reset
      rd_i    : in  std_logic                     := NO;  -- read fifo control
      wr_i    : in  std_logic                     := NO;  -- write fifo control
      data_i  : in  std_logic_vector(15 downto 0) := (others => ZERO);  -- input data bus
      data_o  : out std_logic_vector(15 downto 0);        -- output data bus
      full_o  : out std_logic;          -- fifo-full_o status
      empty_o : out std_logic;          -- fifo-empty_o status
      level_o : out std_logic_vector(7 downto 0)          -- fifo level_o
      );
  end component;

  -- 255 x 16 FIFO with independent read and write clocks.
  component Fifo255x16ic is
    port (
      rdClk_i : in  std_logic;          -- clock for reading from the FIFO
      wrClk_i : in  std_logic;          -- clock for writing to the FIFO
      rst_i   : in  std_logic                     := NO;  -- reset
      rd_i    : in  std_logic                     := NO;  -- read fifo control
      wr_i    : in  std_logic                     := NO;  -- write fifo control
      data_i  : in  std_logic_vector(15 downto 0) := (others => ZERO);  -- input data bus
      data_o  : out std_logic_vector(15 downto 0);        -- output data bus
      full_o  : out std_logic;          -- fifo-full_o status
      empty_o : out std_logic;          -- fifo-empty_o status
      level_o : out std_logic_vector(7 downto 0)          -- fifo level_o
      );
  end component;

  -- Adjustable FIFO with common read and write clock.
  component FifoCc is
    generic (
      WIDTH_G  : natural := 8;          -- FIFO word width.
      LENGTH_G : natural := 16          -- Number of words in the FIFO.
      );
    port (
      rst_i   : in  std_logic                            := NO;  -- Active-high reset.
      clk_i   : in  std_logic;          -- Master clock.
      add_i   : in  std_logic                            := NO;  -- Remove data from the front of the FIFO.
      rmv_i   : in  std_logic                            := NO;  -- Add data to the back of the FIFO.
      data_i  : in  std_logic_vector(WIDTH_G-1 downto 0) := (others => ZERO);  -- Input data to FIFO.
      data_o  : out std_logic_vector(WIDTH_G-1 downto 0);  -- Output data from FIFO.
      empty_o : out std_logic;          -- True when the FIFO is empty.
      full_o  : out std_logic;          -- True when the FIFO is full.
      level_o : out std_logic_vector(natural(ceil(log2(real(LENGTH_G+1))))-1 downto 0)  -- # of data words currently in FIFO.
      );
  end component;

end package;



library IEEE, UNISIM, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
--use UNISIM.vcomponents.all;
use XESS.CommonPckg.all;

--**********************************************************************
-- 255 x 16 FIFO with common read and write clock.
--**********************************************************************
entity Fifo255x16cc is
  port (
    clk_i   : in  std_logic;            -- master clock
    rst_i   : in  std_logic                     := NO;  -- reset
    rd_i    : in  std_logic                     := NO;  -- read fifo control
    wr_i    : in  std_logic                     := NO;  -- write fifo control
    data_i  : in  std_logic_vector(15 downto 0) := (others => ZERO);  -- input data bus
    data_o  : out std_logic_vector(15 downto 0);        -- output data bus
    full_o  : out std_logic;            -- fifo-full_o status
    empty_o : out std_logic;            -- fifo-empty_o status
    level_o : out std_logic_vector(7 downto 0)          -- fifo level_o
    );
end entity;

architecture arch of Fifo255x16cc is
  signal full_s    : std_logic;
  signal empty_s   : std_logic;
  subtype Address_t is integer range 0 to 255;
  signal rdAddr_r  : Address_t := 0;
  signal wrAddr_r  : Address_t := 0;
  signal level_s   : Address_t := 0;
  signal rdAllow_s : std_logic;
  signal wrAllow_s : std_logic;
  subtype RamWord_t is std_logic_vector(data_i'range);  -- RAM word type.
  type Ram_t is array (0 to 255) of RamWord_t;  -- array of RAM words type.
  signal ram_r     : Ram_t;             -- RAM declaration.
begin

  -- Inferred dual-port RAM.  
  process (clk_i)
  begin
    if rising_edge(clk_i) then
      if wrAllow_s = YES then
        ram_r(wrAddr_r) <= data_i;
      end if;
      data_o <= ram_r(rdAddr_r);
    end if;
  end process;

  rdAllow_s <= rd_i and not empty_s;
  wrAllow_s <= wr_i and not full_s;

  process (clk_i, rst_i)
  begin
    if rst_i = '1' then
      rdAddr_r <= 0;
      wrAddr_r <= 0;
      level_s  <= 0;
    elsif rising_edge(clk_i) then
      if rdAllow_s = YES then
        rdAddr_r <= rdAddr_r + 1;
      end if;
      if wrAllow_s = YES then
        wrAddr_r <= wrAddr_r + 1;
      end if;
      if (wrAllow_s and not rdAllow_s and not full_s) = YES then
        level_s <= level_s + 1;
      elsif (rdAllow_s and not wrAllow_s and not empty_s) = YES then
        level_s <= level_s - 1;
      end if;
    end if;
  end process;

  full_s  <= YES when level_s = Address_t'high else NO;
  full_o  <= full_s;
  empty_s <= YES when level_s = 0              else NO;
  empty_o <= empty_s;
  level_o <= std_logic_vector(TO_UNSIGNED(level_s, level_o'length));

end architecture;



library IEEE, UNISIM, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use UNISIM.vcomponents.all;
use XESS.CommonPckg.all;
use XESS.FifoPckg.all;

--**********************************************************************
-- 255 x 16 FIFO with independent read and write clocks.
--**********************************************************************
entity Fifo255x16ic is
  port (
    rdClk_i : in  std_logic;            -- clock for reading from the FIFO
    wrClk_i : in  std_logic;            -- clock for writing to the FIFO
    rst_i   : in  std_logic                     := NO;  -- reset
    rd_i    : in  std_logic                     := NO;  -- read fifo control
    wr_i    : in  std_logic                     := NO;  -- write fifo control
    data_i  : in  std_logic_vector(15 downto 0) := (others => ZERO);  -- input data bus
    data_o  : out std_logic_vector(15 downto 0);        -- output data bus
    full_o  : out std_logic;            -- fifo-full_o status
    empty_o : out std_logic;            -- fifo-empty_o status
    level_o : out std_logic_vector(7 downto 0)          -- fifo level_o
    );
end entity;

architecture arch of Fifo255x16ic is
  signal full_s                 : std_logic;
  signal empty_s                : std_logic;
  signal rdAddr_r               : std_logic_vector(7 downto 0) := "00000000";
  signal grayRdAddr_r           : std_logic_vector(7 downto 0);
  signal prevGrayRdAddr_r       : std_logic_vector(7 downto 0);
  signal prevGrayRdAddrWrSide_r : std_logic_vector(7 downto 0);
  signal wrAddr_r               : std_logic_vector(7 downto 0) := "00000000";
  signal grayWrAddr_r           : std_logic_vector(7 downto 0);
  signal grayWrAddrRdSide_r     : std_logic_vector(7 downto 0);
  signal level_s                : std_logic_vector(7 downto 0) := "00000000";
  signal rdAllow_s              : std_logic;
  signal wrAllow_s              : std_logic;
begin

  bram1 : RAMB4_S16_S16 port map (addra => rdAddr_r, addrb => wrAddr_r,
                                  dia   => (others => '0'), dib => data_i, wea => '0', web => '1',
                                  clka  => rdClk_i, clkb => wrClk_i, rsta => '0', rstb => '0',
                                  ena   => rdAllow_s, enb => wrAllow_s, doa => data_o);

  rdAllow_s <= rd_i and not empty_s;
  wrAllow_s <= wr_i and not full_s;

  process (rdClk_i, rst_i)
  begin
    if rst_i = YES then
      rdAddr_r           <= (others => '0');
      prevGrayRdAddr_r   <= binaryTogray("11111111");
      grayRdAddr_r       <= binaryTogray("00000000");
      grayWrAddrRdSide_r <= binaryTogray("00000000");
    elsif rising_edge(rdClk_i) then
      grayWrAddrRdSide_r <= grayWrAddr_r;
      if rdAllow_s = YES then
        rdAddr_r         <= rdAddr_r + '1';
        prevGrayRdAddr_r <= grayRdAddr_r;
        grayRdAddr_r     <= binaryTogray(rdAddr_r + 1);
      end if;
    end if;
  end process;

  process (wrClk_i, rst_i)
  begin
    if rst_i = YES then
      wrAddr_r               <= (others => '0');
      grayWrAddr_r           <= BinaryToGray("00000000");
      prevGrayRdAddrWrSide_r <= BinaryToGray("11111111");
    elsif rising_edge(wrClk_i) then
      prevGrayRdAddrWrSide_r <= prevGrayRdAddr_r;
      if wrAllow_s = YES then
        wrAddr_r     <= wrAddr_r + '1';
        grayWrAddr_r <= BinaryToGray(wrAddr_r + '1');
      end if;
    end if;
  end process;

  full_s  <= YES when prevGrayRdAddrWrSide_r = grayWrAddr_r else NO;
  full_o  <= full_s;
  empty_s <= YES when grayRdAddr_r = grayWrAddrRdSide_r     else NO;
  empty_o <= empty_s;
  level_s <= wrAddr_r - GrayToBinary(grayRdAddr_r);
  level_o <= level_s;

end architecture;




library IEEE, XESS;
use IEEE.math_real.all;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use XESS.CommonPckg.all;

--**********************************************************************
-- Adjustable FIFO with common read and write clock.
--**********************************************************************
entity FifoCc is
  generic (
    WIDTH_G  : natural := 8;            -- FIFO word width.
    LENGTH_G : natural := 16            -- Number of words in the FIFO.
    );
  port (
    rst_i   : in  std_logic                            := NO;  -- Active-high reset.
    clk_i   : in  std_logic;            -- Master clock.
    add_i   : in  std_logic                            := NO;  -- Remove data from the front of the FIFO.
    rmv_i   : in  std_logic                            := NO;  -- Add data to the back of the FIFO.
    data_i  : in  std_logic_vector(WIDTH_G-1 downto 0) := (others => ZERO);  -- Input data to FIFO.
    data_o  : out std_logic_vector(WIDTH_G-1 downto 0);  -- Output data from FIFO.
    empty_o : out std_logic;            -- True when the FIFO is empty.
    full_o  : out std_logic;            -- True when the FIFO is full.
    level_o : out std_logic_vector(natural(ceil(log2(real(LENGTH_G+1))))-1 downto 0)  -- # of data words currently in FIFO.
    );
end entity;

architecture arch of FifoCc is
  signal full_s     : std_logic;
  signal empty_s    : std_logic;
  subtype Address_t is natural range 0 to LENGTH_G-1;
  signal rmvAddr_r  : Address_t := 0;
  signal addAddr_r  : Address_t := 0;
  subtype Level_t is natural range 0 to LENGTH_G;
  signal level_r    : Level_t   := 0;
  signal rmvAllow_s : std_logic;
  signal addAllow_s : std_logic;
  subtype RamWord_t is std_logic_vector(data_i'range);  -- RAM word type.
  type Ram_t is array (0 to LENGTH_G-1) of RamWord_t;  -- array of RAM words type.
  signal ram_r      : Ram_t;            -- RAM declaration.
begin

  full_s     <= YES when level_r = Level_t'high else NO;
  empty_s    <= YES when level_r = 0            else NO;
  rmvAllow_s <= rmv_i and not empty_s;
  addAllow_s <= add_i and not full_s;

  process (rst_i, clk_i)
  begin
    if rst_i = '1' then
      rmvAddr_r <= 0;
      addAddr_r <= 0;
      level_r   <= 0;
    elsif rising_edge(clk_i) then
      if rmvAllow_s = YES then
        rmvAddr_r <= rmvAddr_r + 1;
        data_o <= ram_r(rmvAddr_r + 1);  -- Data at the front of the FIFO is always available on the output.
      else
        data_o <= ram_r(rmvAddr_r);  -- Data at the front of the FIFO is always available on the output.
      end if;
      if addAllow_s = YES then
        ram_r(addAddr_r) <= data_i;
        addAddr_r        <= addAddr_r + 1;
      end if;
      if (addAllow_s and not rmvAllow_s) = YES then
        level_r <= level_r + 1;
      elsif (rmvAllow_s and not addAllow_s) = YES then
        level_r <= level_r - 1;
      end if;
    end if;
  end process;

  full_o  <= full_s;
  empty_o <= empty_s;
  level_o <= std_logic_vector(TO_UNSIGNED(level_r, level_o'length));

end architecture;
