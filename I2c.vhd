--**********************************************************************
-- The first three modules in this package are from the Opencores.org
-- I2C communications module. The last module was built by XESS to give
-- the I2C module an interface more friendly to what we are doing.
-- 
-- Each module is governed by its own copyright statement.
--**********************************************************************



library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use XESS.CommonPckg.all;

package I2cPckg is

  component i2c_master_bit_ctrl is
    port (
      clk    : in std_logic;
      rst    : in std_logic;
      nReset : in std_logic;
      ena    : in std_logic;            -- core enable signal

      clk_cnt : in unsigned(15 downto 0);  -- clock prescale value

      cmd     : in  std_logic_vector(3 downto 0);
      cmd_ack : out std_logic;          -- command completed
      busy    : out std_logic;          -- i2c bus busy
      al      : out std_logic;          -- arbitration lost

      din  : in  std_logic;
      dout : out std_logic;

      -- i2c lines
      scl_i   : in  std_logic;          -- i2c clock line input
      scl_o   : out std_logic;          -- i2c clock line output
      scl_oen : out std_logic;  -- i2c clock line output enable, active low
      sda_i   : in  std_logic;          -- i2c data line input
      sda_o   : out std_logic;          -- i2c data line output
      sda_oen : out std_logic   -- i2c data line output enable, active low
      );
  end component;

  component i2c_master_byte_ctrl is
    port (
      clk    : in std_logic;
      rst    : in std_logic;  -- synchronous active high reset (WISHBONE compatible)
      nReset : in std_logic;  -- asynchornous active low reset (FPGA compatible)
      ena    : in std_logic;            -- core enable signal

      clk_cnt : in unsigned(15 downto 0);  -- 4x SCL

      -- input signals
      start,
      stop,
      read,
      write,
      ack_in :    std_logic;
      din    : in std_logic_vector(7 downto 0);

      -- output signals
      cmd_ack  : out std_logic;         -- command done
      ack_out  : out std_logic;
      i2c_busy : out std_logic;         -- arbitration lost
      i2c_al   : out std_logic;         -- i2c bus busy
      dout     : out std_logic_vector(7 downto 0);

      -- i2c lines
      scl_i   : in  std_logic;          -- i2c clock line input
      scl_o   : out std_logic;          -- i2c clock line output
      scl_oen : out std_logic;  -- i2c clock line output enable, active low
      sda_i   : in  std_logic;          -- i2c data line input
      sda_o   : out std_logic;          -- i2c data line output
      sda_oen : out std_logic   -- i2c data line output enable, active low
      );
  end component;

  component i2c_master_top is
    generic(
      ARST_LVL   : std_logic := '0';    -- asynchronous reset level
      FREQ_G     : real      := 100.0;  -- Master clock frequency (MHz).
      I2C_FREQ_G : real      := 0.1     -- I2C clock frequency (MHz).
      );
    port (
      -- wishbone signals
      wb_clk_i  : in  std_logic;        -- master clock input
      wb_rst_i  : in  std_logic := '0';  -- synchronous active high reset
      arst_i    : in  std_logic := not ARST_LVL;     -- asynchronous reset
      wb_adr_i  : in  std_logic_vector(2 downto 0);  -- lower address bits
      wb_dat_i  : in  std_logic_vector(7 downto 0);  -- Databus input
      wb_dat_o  : out std_logic_vector(7 downto 0);  -- Databus output
      wb_we_i   : in  std_logic;        -- Write enable input
      wb_stb_i  : in  std_logic;        -- Strobe signals / core select signal
      wb_cyc_i  : in  std_logic;        -- Valid bus cycle input
      wb_ack_o  : out std_logic;        -- Bus cycle acknowledge output
      wb_inta_o : out std_logic;        -- interrupt request output signal

      -- i2c lines
      scl_pad_i    : in  std_logic;     -- i2c clock line input
      scl_pad_o    : out std_logic;     -- i2c clock line output
      scl_padoen_o : out std_logic;  -- i2c clock line output enable, active low
      sda_pad_i    : in  std_logic;     -- i2c data line input
      sda_pad_o    : out std_logic;     -- i2c data line output
      sda_padoen_o : out std_logic   -- i2c data line output enable, active low
      );
  end component;

  component I2c is
    generic(
      FREQ_G     : real := 100.0;       -- Main clock frequency (MHz).
      I2C_FREQ_G : real := 0.1          -- I2C clock frequency (MHz).
      );
    port (
      clk_i  : in    std_logic;         -- Master clock input.
      rst_i  : in    std_logic := LO;   -- Synchronous active high reset.
      addr_i : in    std_logic_vector(2 downto 0);  -- Lower address bits.
      data_i : in    std_logic_vector(7 downto 0);  -- Databus input.
      data_o : out   std_logic_vector(7 downto 0);  -- Databus output.
      wr_i   : in    std_logic;         -- Write enable input.
      rd_i   : in    std_logic;         -- Read enable input.
      done_o : out   std_logic;         -- True when R/W operation is done.
      scl_io : inout std_logic;         -- Tristateable I2C clock line.
      sda_io : inout std_logic          -- Tristateable I2C data line.
      );
  end component;

end package;



---------------------------------------------------------------------
----                                                             ----
----  WISHBONE revB2 I2C Master Core; bit-controller             ----
----                                                             ----
----                                                             ----
----  Author: Richard Herveille                                  ----
----          richard@asics.ws                                   ----
----          www.asics.ws                                       ----
----                                                             ----
----  Downloaded from: http://www.opencores.org/projects/i2c/    ----
----                                                             ----
---------------------------------------------------------------------
----                                                             ----
---- Copyright (C) 2000 Richard Herveille                        ----
----                    richard@asics.ws                         ----
----                                                             ----
---- This source file may be used and distributed without        ----
---- restriction provided that this copyright statement is not   ----
---- removed from the file and that any derivative XESS contains ----
---- the original copyright notice and the associated disclaimer.----
----                                                             ----
----     THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ----
---- EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ----
---- TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ----
---- FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ----
---- OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ----
---- INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ----
---- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ----
---- GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ----
---- BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ----
---- LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ----
---- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ----
---- OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ----
---- POSSIBILITY OF SUCH DAMAGE.                                 ----
----                                                             ----
---------------------------------------------------------------------

-------------------------------------
-- Bit controller section
------------------------------------
--
-- Translate simple commands into SCL/SDA transitions
-- Each command has 5 states, A/B/C/D/idle
--
-- start:    SCL  ~~~~~~~~~~~~~~\____
--           SDA  XX/~~~~~~~\______
--                x | A | B | C | D | i
--
-- repstart  SCL  ______/~~~~~~~\___
--           SDA  __/~~~~~~~\______
--                x | A | B | C | D | i
--
-- stop      SCL  _______/~~~~~~~~~~~
--           SDA  ==\___________/~~~~~
--                x | A | B | C | D | i
--
--- write    SCL  ______/~~~~~~~\____
--           SDA  XXX===============XX
--                x | A | B | C | D | i
--
--- read     SCL  ______/~~~~~~~\____
--           SDA  XXXXXXX=XXXXXXXXXXX
--                x | A | B | C | D | i
--

-- Timing:      Normal mode     Fast mode
-----------------------------------------------------------------
-- Fscl         100KHz          400KHz
-- Th_scl       4.0us           0.6us   High period of SCL
-- Tl_scl       4.7us           1.3us   Low period of SCL
-- Tsu:sta      4.7us           0.6us   setup time for a repeated start condition
-- Tsu:sto      4.0us           0.6us   setup time for a stop conditon
-- Tbuf         4.7us           1.3us   Bus free time between a stop and start condition
--

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity i2c_master_bit_ctrl is
  port (
    clk    : in std_logic;
    rst    : in std_logic;
    nReset : in std_logic;
    ena    : in std_logic;              -- core enable signal

    clk_cnt : in unsigned(15 downto 0);  -- clock prescale value

    cmd     : in  std_logic_vector(3 downto 0);
    cmd_ack : out std_logic;            -- command completed
    busy    : out std_logic;            -- i2c bus busy
    al      : out std_logic;            -- arbitration lost

    din  : in  std_logic;
    dout : out std_logic;

    -- i2c lines
    scl_i   : in  std_logic;            -- i2c clock line input
    scl_o   : out std_logic;            -- i2c clock line output
    scl_oen : out std_logic;  -- i2c clock line output enable, active low
    sda_i   : in  std_logic;            -- i2c data line input
    sda_o   : out std_logic;            -- i2c data line output
    sda_oen : out std_logic   -- i2c data line output enable, active low
    );
end entity i2c_master_bit_ctrl;

architecture structural of i2c_master_bit_ctrl is
  constant I2C_CMD_NOP   : std_logic_vector(3 downto 0) := "0000";
  constant I2C_CMD_START : std_logic_vector(3 downto 0) := "0001";
  constant I2C_CMD_STOP  : std_logic_vector(3 downto 0) := "0010";
  constant I2C_CMD_READ  : std_logic_vector(3 downto 0) := "0100";
  constant I2C_CMD_WRITE : std_logic_vector(3 downto 0) := "1000";

  type states is (idle, start_a, start_b, start_c, start_d, start_e,
                  stop_a, stop_b, stop_c, stop_d, rd_a, rd_b, rd_c, rd_d, wr_a, wr_b, wr_c, wr_d);
  signal c_state : states;

  signal iscl_oen, isda_oen   : std_logic;  -- internal I2C lines
  signal sda_chk              : std_logic;  -- check SDA status (multi-master arbitration)
  signal dscl_oen             : std_logic;  -- delayed scl_oen signals
  signal sSCL, sSDA           : std_logic;  -- synchronized SCL and SDA inputs
  signal dSCL, dSDA           : std_logic;  -- delayed versions ofsSCL and sSDA
  signal clk_en               : std_logic;  -- statemachine clock enable
  signal scl_sync, slave_wait : std_logic;  -- clock generation signals
  signal ial                  : std_logic;  -- internal arbitration lost signal
  signal cnt                  : unsigned(15 downto 0);  -- clock divider counter (synthesis)

begin
  -- whenever the slave is not ready it can delay the cycle by pulling SCL low
  -- delay scl_oen
  process (clk, nReset)
  begin
    if (nReset = '0') then
      dscl_oen <= '0';
    elsif (clk'event and clk = '1') then
      dscl_oen <= iscl_oen;
    end if;
  end process;

  -- slave_wait is asserted when master wants to drive SCL high, but the slave pulls it low
  -- slave_wait remains asserted until the slave releases SCL
  process (clk, nReset)
  begin
    if (nReset = '0') then
      slave_wait <= '0';
    elsif (clk'event and clk = '1') then
      slave_wait <= (iscl_oen and not dscl_oen and not sSCL) or (slave_wait and not sSCL);
    end if;
  end process;

  -- master drives SCL high, but another master pulls it low
  -- master start counting down its low cycle now (clock synchronization)
  scl_sync <= dSCL and not sSCL and iscl_oen;

  -- generate clk enable signal
  gen_clken : process(clk, nReset)
  begin
    if (nReset = '0') then
      cnt    <= (others => '0');
      clk_en <= '1';
    elsif (clk'event and clk = '1') then
      if ((rst = '1') or (cnt = 0) or (ena = '0') or (scl_sync = '1')) then
        cnt    <= clk_cnt;
        clk_en <= '1';
      elsif (slave_wait = '1') then
        cnt    <= cnt;
        clk_en <= '0';
      else
        cnt    <= cnt -1;
        clk_en <= '0';
      end if;
    end if;
  end process gen_clken;


  -- generate bus status controller
  bus_status_ctrl : block
    signal cSCL, cSDA    : std_logic_vector(1 downto 0);  -- capture SDA and SCL
    signal fSCL, fSDA    : std_logic_vector(2 downto 0);  -- filter inputs for SCL and SDA
    signal filter_cnt    : unsigned(13 downto 0);  -- clock divider for filter
    signal sta_condition : std_logic;   -- start detected
    signal sto_condition : std_logic;   -- stop detected
    signal cmd_stop      : std_logic;   -- STOP command
    signal ibusy         : std_logic;   -- internal busy signal
  begin
    -- capture SCL and SDA
    capture_scl_sda : process(clk, nReset)
    begin
      if (nReset = '0') then
        cSCL <= "00";
        cSDA <= "00";
      elsif (clk'event and clk = '1') then
        if (rst = '1') then
          cSCL <= "00";
          cSDA <= "00";
        else
          cSCL <= (cSCL(0) & scl_i);
          cSDA <= (cSDA(0) & sda_i);
        end if;
      end if;
    end process capture_scl_sda;

    -- filter SCL and SDA; (attempt to) remove glitches
    filter_divider : process(clk, nReset)
    begin
      if (nReset = '0') then
        filter_cnt <= (others => '0');
      elsif (clk'event and clk = '1') then
        if ((rst = '1') or (ena = '0')) then
          filter_cnt <= (others => '0');
        elsif (filter_cnt = 0) then
          filter_cnt <= clk_cnt(15 downto 2);
        else
          filter_cnt <= filter_cnt -1;
        end if;
      end if;
    end process filter_divider;

    filter_scl_sda : process(clk, nReset)
    begin
      if (nReset = '0') then
        fSCL <= (others => '1');
        fSDA <= (others => '1');
      elsif (clk'event and clk = '1') then
        if (rst = '1') then
          fSCL <= (others => '1');
          fSDA <= (others => '1');
        elsif (filter_cnt = 0) then
          fSCL <= (fSCL(1 downto 0) & cSCL(1));
          fSDA <= (fSDA(1 downto 0) & cSDA(1));
        end if;
      end if;
    end process filter_scl_sda;

    -- generate filtered SCL and SDA signals
    scl_sda : process(clk, nReset)
    begin
      if (nReset = '0') then
        sSCL <= '1';
        sSDA <= '1';

        dSCL <= '1';
        dSDA <= '1';
      elsif (clk'event and clk = '1') then
        if (rst = '1') then
          sSCL <= '1';
          sSDA <= '1';

          dSCL <= '1';
          dSDA <= '1';
        else
          sSCL <= (fSCL(2) and fSCL(1)) or
                  (fSCL(2) and fSCL(0)) or
                  (fSCL(1) and fSCL(0));
          sSDA <= (fSDA(2) and fSDA(1)) or
                  (fSDA(2) and fSDA(0)) or
                  (fSDA(1) and fSDA(0));

          dSCL <= sSCL;
          dSDA <= sSDA;
        end if;
      end if;
    end process scl_sda;


    -- detect start condition => detect falling edge on SDA while SCL is high
    -- detect stop condition  => detect rising edge on SDA while SCL is high
    detect_sta_sto : process(clk, nReset)
    begin
      if (nReset = '0') then
        sta_condition <= '0';
        sto_condition <= '0';
      elsif (clk'event and clk = '1') then
        if (rst = '1') then
          sta_condition <= '0';
          sto_condition <= '0';
        else
          sta_condition <= (not sSDA and dSDA) and sSCL;
          sto_condition <= (sSDA and not dSDA) and sSCL;
        end if;
      end if;
    end process detect_sta_sto;


    -- generate i2c-bus busy signal
    gen_busy : process(clk, nReset)
    begin
      if (nReset = '0') then
        ibusy <= '0';
      elsif (clk'event and clk = '1') then
        if (rst = '1') then
          ibusy <= '0';
        else
          ibusy <= (sta_condition or ibusy) and not sto_condition;
        end if;
      end if;
    end process gen_busy;
    busy <= ibusy;


    -- generate arbitration lost signal
    -- aribitration lost when:
    -- 1) master drives SDA high, but the i2c bus is low
    -- 2) stop detected while not requested (detect during 'idle' state)
    gen_al : process(clk, nReset)
    begin
      if (nReset = '0') then
        cmd_stop <= '0';
        ial      <= '0';
      elsif (clk'event and clk = '1') then
        if (rst = '1') then
          cmd_stop <= '0';
          ial      <= '0';
        else
          if (clk_en = '1') then
            if (cmd = I2C_CMD_STOP) then
              cmd_stop <= '1';
            else
              cmd_stop <= '0';
            end if;
          end if;

          if (c_state = idle) then
            ial <= (sda_chk and not sSDA and isda_oen) or (sto_condition and not cmd_stop);
          else
            ial <= (sda_chk and not sSDA and isda_oen);
          end if;
        end if;
      end if;
    end process gen_al;
    al <= ial;


    -- generate dout signal, store dout on rising edge of SCL
    gen_dout : process(clk, nReset)
    begin
      if (nReset = '0') then
        dout <= '0';
      elsif (clk'event and clk = '1') then
        if (sSCL = '1' and dSCL = '0') then
          dout <= sSDA;
        end if;
      end if;
    end process gen_dout;
  end block bus_status_ctrl;


  -- generate statemachine
  nxt_state_decoder : process (clk, nReset)
  begin
    if (nReset = '0') then
      c_state  <= idle;
      cmd_ack  <= '0';
      iscl_oen <= '1';
      isda_oen <= '1';
      sda_chk  <= '0';
    elsif (clk'event and clk = '1') then
      if (rst = '1' or ial = '1') then
        c_state  <= idle;
        cmd_ack  <= '0';
        iscl_oen <= '1';
        isda_oen <= '1';
        sda_chk  <= '0';
      else
        cmd_ack <= '0';                 -- default no acknowledge

        if (clk_en = '1') then
          case (c_state) is
            -- idle
            when idle =>
              case cmd is
                when I2C_CMD_START => c_state <= start_a;
                when I2C_CMD_STOP  => c_state <= stop_a;
                when I2C_CMD_WRITE => c_state <= wr_a;
                when I2C_CMD_READ  => c_state <= rd_a;
                when others        => c_state <= idle;  -- NOP command
              end case;

              iscl_oen <= iscl_oen;     -- keep SCL in same state
              isda_oen <= isda_oen;     -- keep SDA in same state
              sda_chk  <= '0';          -- don't check SDA

            -- start
            when start_a =>
              c_state  <= start_b;
              iscl_oen <= iscl_oen;  -- keep SCL in same state (for repeated start)
              isda_oen <= '1';          -- set SDA high
              sda_chk  <= '0';          -- don't check SDA

            when start_b =>
              c_state  <= start_c;
              iscl_oen <= '1';          -- set SCL high
              isda_oen <= '1';          -- keep SDA high
              sda_chk  <= '0';          -- don't check SDA

            when start_c =>
              c_state  <= start_d;
              iscl_oen <= '1';          -- keep SCL high
              isda_oen <= '0';          -- set SDA low
              sda_chk  <= '0';          -- don't check SDA

            when start_d =>
              c_state  <= start_e;
              iscl_oen <= '1';          -- keep SCL high
              isda_oen <= '0';          -- keep SDA low
              sda_chk  <= '0';          -- don't check SDA

            when start_e =>
              c_state  <= idle;
              cmd_ack  <= '1';          -- command completed
              iscl_oen <= '0';          -- set SCL low
              isda_oen <= '0';          -- keep SDA low
              sda_chk  <= '0';          -- don't check SDA

            -- stop
            when stop_a =>
              c_state  <= stop_b;
              iscl_oen <= '0';          -- keep SCL low
              isda_oen <= '0';          -- set SDA low
              sda_chk  <= '0';          -- don't check SDA

            when stop_b =>
              c_state  <= stop_c;
              iscl_oen <= '1';          -- set SCL high
              isda_oen <= '0';          -- keep SDA low
              sda_chk  <= '0';          -- don't check SDA

            when stop_c =>
              c_state  <= stop_d;
              iscl_oen <= '1';          -- keep SCL high
              isda_oen <= '0';          -- keep SDA low
              sda_chk  <= '0';          -- don't check SDA

            when stop_d =>
              c_state  <= idle;
              cmd_ack  <= '1';          -- command completed
              iscl_oen <= '1';          -- keep SCL high
              isda_oen <= '1';          -- set SDA high
              sda_chk  <= '0';          -- don't check SDA

            -- read
            when rd_a =>
              c_state  <= rd_b;
              iscl_oen <= '0';          -- keep SCL low
              isda_oen <= '1';          -- tri-state SDA
              sda_chk  <= '0';          -- don't check SDA

            when rd_b =>
              c_state  <= rd_c;
              iscl_oen <= '1';          -- set SCL high
              isda_oen <= '1';          -- tri-state SDA
              sda_chk  <= '0';          -- don't check SDA

            when rd_c =>
              c_state  <= rd_d;
              iscl_oen <= '1';          -- keep SCL high
              isda_oen <= '1';          -- tri-state SDA
              sda_chk  <= '0';          -- don't check SDA

            when rd_d =>
              c_state  <= idle;
              cmd_ack  <= '1';          -- command completed
              iscl_oen <= '0';          -- set SCL low
              isda_oen <= '1';          -- tri-state SDA
              sda_chk  <= '0';          -- don't check SDA

            -- write
            when wr_a =>
              c_state  <= wr_b;
              iscl_oen <= '0';          -- keep SCL low
              isda_oen <= din;          -- set SDA
              sda_chk  <= '0';          -- don't check SDA (SCL low)

            when wr_b =>
              c_state  <= wr_c;
              iscl_oen <= '1';          -- set SCL high
              isda_oen <= din;          -- keep SDA
              sda_chk  <= '0';          -- don't check SDA yet
                                        -- Allow some more time for SDA and SCL to settle

            when wr_c =>
              c_state  <= wr_d;
              iscl_oen <= '1';          -- keep SCL high
              isda_oen <= din;          -- keep SDA
              sda_chk  <= '1';          -- check SDA

            when wr_d =>
              c_state  <= idle;
              cmd_ack  <= '1';          -- command completed
              iscl_oen <= '0';          -- set SCL low
              isda_oen <= din;          -- keep SDA
              sda_chk  <= '0';          -- don't check SDA (SCL low)

            when others =>

          end case;
        end if;
      end if;
    end if;
  end process nxt_state_decoder;


  -- assign outputs
  scl_o   <= '0';
  scl_oen <= iscl_oen;
  sda_o   <= '0';
  sda_oen <= isda_oen;
end architecture structural;



---------------------------------------------------------------------
----                                                             ----
----  WISHBONE revB2 compl. I2C Master Core; byte-controller     ----
----                                                             ----
----                                                             ----
----  Author: Richard Herveille                                  ----
----          richard@asics.ws                                   ----
----          www.asics.ws                                       ----
----                                                             ----
----  Downloaded from: http://www.opencores.org/projects/i2c/    ----
----                                                             ----
---------------------------------------------------------------------
----                                                             ----
---- Copyright (C) 2000 Richard Herveille                        ----
----                    richard@asics.ws                         ----
----                                                             ----
---- This source file may be used and distributed without        ----
---- restriction provided that this copyright statement is not   ----
---- removed from the file and that any derivative XESS contains ----
---- the original copyright notice and the associated disclaimer.----
----                                                             ----
----     THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ----
---- EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ----
---- TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ----
---- FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ----
---- OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ----
---- INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ----
---- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ----
---- GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ----
---- BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ----
---- LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ----
---- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ----
---- OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ----
---- POSSIBILITY OF SUCH DAMAGE.                                 ----
----                                                             ----
---------------------------------------------------------------------

------------------------------------------
-- Byte controller section
------------------------------------------
--
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity i2c_master_byte_ctrl is
  port (
    clk    : in std_logic;
    rst    : in std_logic;  -- synchronous active high reset (WISHBONE compatible)
    nReset : in std_logic;  -- asynchornous active low reset (FPGA compatible)
    ena    : in std_logic;              -- core enable signal

    clk_cnt : in unsigned(15 downto 0);  -- 4x SCL

    -- input signals
    start,
    stop,
    read,
    write,
    ack_in :    std_logic;
    din    : in std_logic_vector(7 downto 0);

    -- output signals
    cmd_ack  : out std_logic;           -- command done
    ack_out  : out std_logic;
    i2c_busy : out std_logic;           -- arbitration lost
    i2c_al   : out std_logic;           -- i2c bus busy
    dout     : out std_logic_vector(7 downto 0);

    -- i2c lines
    scl_i   : in  std_logic;            -- i2c clock line input
    scl_o   : out std_logic;            -- i2c clock line output
    scl_oen : out std_logic;  -- i2c clock line output enable, active low
    sda_i   : in  std_logic;            -- i2c data line input
    sda_o   : out std_logic;            -- i2c data line output
    sda_oen : out std_logic   -- i2c data line output enable, active low
    );
end entity i2c_master_byte_ctrl;

architecture structural of i2c_master_byte_ctrl is
  component i2c_master_bit_ctrl is
    port (
      clk    : in std_logic;
      rst    : in std_logic;
      nReset : in std_logic;
      ena    : in std_logic;            -- core enable signal

      clk_cnt : in unsigned(15 downto 0);  -- clock prescale value

      cmd     : in  std_logic_vector(3 downto 0);
      cmd_ack : out std_logic;          -- command done
      busy    : out std_logic;          -- i2c bus busy
      al      : out std_logic;          -- arbitration lost

      din  : in  std_logic;
      dout : out std_logic;

      -- i2c lines
      scl_i   : in  std_logic;          -- i2c clock line input
      scl_o   : out std_logic;          -- i2c clock line output
      scl_oen : out std_logic;  -- i2c clock line output enable, active low
      sda_i   : in  std_logic;          -- i2c data line input
      sda_o   : out std_logic;          -- i2c data line output
      sda_oen : out std_logic   -- i2c data line output enable, active low
      );
  end component i2c_master_bit_ctrl;

  -- commands for bit_controller block
  constant I2C_CMD_NOP   : std_logic_vector(3 downto 0) := "0000";
  constant I2C_CMD_START : std_logic_vector(3 downto 0) := "0001";
  constant I2C_CMD_STOP  : std_logic_vector(3 downto 0) := "0010";
  constant I2C_CMD_READ  : std_logic_vector(3 downto 0) := "0100";
  constant I2C_CMD_WRITE : std_logic_vector(3 downto 0) := "1000";

  -- signals for bit_controller
  signal core_cmd                     : std_logic_vector(3 downto 0);
  signal core_ack, core_txd, core_rxd : std_logic;
  signal al                           : std_logic;

  -- signals for shift register
  signal sr        : std_logic_vector(7 downto 0);  -- 8bit shift register
  signal shift, ld : std_logic;

  -- signals for state machine
  signal go, host_ack : std_logic;
  signal dcnt         : unsigned(2 downto 0);  -- data counter
  signal cnt_done     : std_logic;

begin
  -- hookup bit_controller
  bit_ctrl : i2c_master_bit_ctrl port map(
    clk     => clk,
    rst     => rst,
    nReset  => nReset,
    ena     => ena,
    clk_cnt => clk_cnt,
    cmd     => core_cmd,
    cmd_ack => core_ack,
    busy    => i2c_busy,
    al      => al,
    din     => core_txd,
    dout    => core_rxd,
    scl_i   => scl_i,
    scl_o   => scl_o,
    scl_oen => scl_oen,
    sda_i   => sda_i,
    sda_o   => sda_o,
    sda_oen => sda_oen
    );
  i2c_al <= al;

  -- generate host-command-acknowledge
  cmd_ack <= host_ack;

  -- generate go-signal
  go <= (read or write or stop) and not host_ack;

  -- assign Dout output to shift-register
  dout <= sr;

  -- generate shift register
  shift_register : process(clk, nReset)
  begin
    if (nReset = '0') then
      sr <= (others => '0');
    elsif (clk'event and clk = '1') then
      if (rst = '1') then
        sr <= (others => '0');
      elsif (ld = '1') then
        sr <= din;
      elsif (shift = '1') then
        sr <= (sr(6 downto 0) & core_rxd);
      end if;
    end if;
  end process shift_register;

  -- generate data-counter
  data_cnt : process(clk, nReset)
  begin
    if (nReset = '0') then
      dcnt <= (others => '0');
    elsif (clk'event and clk = '1') then
      if (rst = '1') then
        dcnt <= (others => '0');
      elsif (ld = '1') then
        dcnt <= (others => '1');        -- load counter with 7
      elsif (shift = '1') then
        dcnt <= dcnt -1;
      end if;
    end if;
  end process data_cnt;

  cnt_done <= '1' when (dcnt = 0) else '0';

  --
  -- state machine
  --
  statemachine : block
    type states is (st_idle, st_start, st_read, st_write, st_ack, st_stop);
    signal c_state : states;
  begin
    --
    -- command interpreter, translate complex commands into simpler I2C commands
    --
    nxt_state_decoder : process(clk, nReset)
    begin
      if (nReset = '0') then
        core_cmd <= I2C_CMD_NOP;
        core_txd <= '0';
        shift    <= '0';
        ld       <= '0';
        host_ack <= '0';
        c_state  <= st_idle;
        ack_out  <= '0';
      elsif (clk'event and clk = '1') then
        if (rst = '1' or al = '1') then
          core_cmd <= I2C_CMD_NOP;
          core_txd <= '0';
          shift    <= '0';
          ld       <= '0';
          host_ack <= '0';
          c_state  <= st_idle;
          ack_out  <= '0';
        else
          -- initialy reset all signal
          core_txd <= sr(7);
          shift    <= '0';
          ld       <= '0';
          host_ack <= '0';

          case c_state is
            when st_idle =>
              if (go = '1') then
                if (start = '1') then
                  c_state  <= st_start;
                  core_cmd <= I2C_CMD_START;
                elsif (read = '1') then
                  c_state  <= st_read;
                  core_cmd <= I2C_CMD_READ;
                elsif (write = '1') then
                  c_state  <= st_write;
                  core_cmd <= I2C_CMD_WRITE;
                else                    -- stop
                  c_state  <= st_stop;
                  core_cmd <= I2C_CMD_STOP;
                end if;

                ld <= '1';
              end if;

            when st_start =>
              if (core_ack = '1') then
                if (read = '1') then
                  c_state  <= st_read;
                  core_cmd <= I2C_CMD_READ;
                else
                  c_state  <= st_write;
                  core_cmd <= I2C_CMD_WRITE;
                end if;

                ld <= '1';
              end if;

            when st_write =>
              if (core_ack = '1') then
                if (cnt_done = '1') then
                  c_state  <= st_ack;
                  core_cmd <= I2C_CMD_READ;
                else
                  c_state  <= st_write;       -- stay in same state
                  core_cmd <= I2C_CMD_WRITE;  -- write next bit
                  shift    <= '1';
                end if;
              end if;

            when st_read =>
              if (core_ack = '1') then
                if (cnt_done = '1') then
                  c_state  <= st_ack;
                  core_cmd <= I2C_CMD_WRITE;
                else
                  c_state  <= st_read;       -- stay in same state
                  core_cmd <= I2C_CMD_READ;  -- read next bit
                end if;

                shift    <= '1';
                core_txd <= ack_in;
              end if;

            when st_ack =>
              if (core_ack = '1') then
                -- check for stop; Should a STOP command be generated ?
                if (stop = '1') then
                  c_state  <= st_stop;
                  core_cmd <= I2C_CMD_STOP;
                else
                  c_state  <= st_idle;
                  core_cmd <= I2C_CMD_NOP;

                  -- generate command acknowledge signal
                  host_ack <= '1';
                end if;

                -- assign ack_out output to core_rxd (contains last received bit)
                ack_out <= core_rxd;

                core_txd <= '1';
              else
                core_txd <= ack_in;
              end if;

            when st_stop =>
              if (core_ack = '1') then
                c_state  <= st_idle;
                core_cmd <= I2C_CMD_NOP;

                -- generate command acknowledge signal
                host_ack <= '1';
              end if;

            when others =>              -- illegal states
              c_state  <= st_idle;
              core_cmd <= I2C_CMD_NOP;
              report ("Byte controller entered illegal state.");

          end case;

        end if;
      end if;
    end process nxt_state_decoder;

  end block statemachine;

end architecture structural;



---------------------------------------------------------------------
----                                                             ----
----  WISHBONE revB2 compl. I2C Master Core; top level           ----
----                                                             ----
----                                                             ----
----  Author: Richard Herveille                                  ----
----          richard@asics.ws                                   ----
----          www.asics.ws                                       ----
----                                                             ----
----  Downloaded from: http://www.opencores.org/projects/i2c/    ----
----                                                             ----
---------------------------------------------------------------------
----                                                             ----
---- Copyright (C) 2000 Richard Herveille                        ----
----                    richard@asics.ws                         ----
----                                                             ----
---- This source file may be used and distributed without        ----
---- restriction provided that this copyright statement is not   ----
---- removed from the file and that any derivative XESS contains ----
---- the original copyright notice and the associated disclaimer.----
----                                                             ----
----     THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ----
---- EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ----
---- TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ----
---- FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ----
---- OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ----
---- INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ----
---- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ----
---- GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ----
---- BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ----
---- LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ----
---- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ----
---- OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ----
---- POSSIBILITY OF SUCH DAMAGE.                                 ----
----                                                             ----
---------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity i2c_master_top is
  generic(
    ARST_LVL   : std_logic := '0';      -- asynchronous reset level
    FREQ_G     : real      := 100.0;    -- Master clock frequency (MHz).
    I2C_FREQ_G : real      := 0.1       -- I2C clock frequency (MHz).
    );
  port (
    -- wishbone signals
    wb_clk_i  : in  std_logic;          -- master clock input
    wb_rst_i  : in  std_logic := '0';   -- synchronous active high reset
    arst_i    : in  std_logic := not ARST_LVL;     -- asynchronous reset
    wb_adr_i  : in  std_logic_vector(2 downto 0);  -- lower address bits
    wb_dat_i  : in  std_logic_vector(7 downto 0);  -- Databus input
    wb_dat_o  : out std_logic_vector(7 downto 0);  -- Databus output
    wb_we_i   : in  std_logic;          -- Write enable input
    wb_stb_i  : in  std_logic;          -- Strobe signals / core select signal
    wb_cyc_i  : in  std_logic;          -- Valid bus cycle input
    wb_ack_o  : out std_logic;          -- Bus cycle acknowledge output
    wb_inta_o : out std_logic;          -- interrupt request output signal

    -- i2c lines
    scl_pad_i    : in  std_logic;       -- i2c clock line input
    scl_pad_o    : out std_logic;       -- i2c clock line output
    scl_padoen_o : out std_logic;  -- i2c clock line output enable, active low
    sda_pad_i    : in  std_logic;       -- i2c data line input
    sda_pad_o    : out std_logic;       -- i2c data line output
    sda_padoen_o : out std_logic   -- i2c data line output enable, active low
    );
end entity i2c_master_top;

architecture structural of i2c_master_top is
  component i2c_master_byte_ctrl is
    port (
      clk    : in std_logic;
      rst    : in std_logic;  -- synchronous active high reset (WISHBONE compatible)
      nReset : in std_logic;  -- asynchornous active low reset (FPGA compatible)
      ena    : in std_logic;            -- core enable signal

      clk_cnt : in unsigned(15 downto 0);  -- 4x SCL

      -- input signals
      start,
      stop,
      read,
      write,
      ack_in :    std_logic;
      din    : in std_logic_vector(7 downto 0);

      -- output signals
      cmd_ack  : out std_logic;
      ack_out  : out std_logic;
      i2c_busy : out std_logic;
      i2c_al   : out std_logic;
      dout     : out std_logic_vector(7 downto 0);

      -- i2c lines
      scl_i   : in  std_logic;          -- i2c clock line input
      scl_o   : out std_logic;          -- i2c clock line output
      scl_oen : out std_logic;  -- i2c clock line output enable, active low
      sda_i   : in  std_logic;          -- i2c data line input
      sda_o   : out std_logic;          -- i2c data line output
      sda_oen : out std_logic   -- i2c data line output enable, active low
      );
  end component i2c_master_byte_ctrl;

  -- registers
  constant PRESCALE_C : unsigned(15 downto 0) := TO_UNSIGNED(integer(FREQ_G/(5.0*I2C_FREQ_G)) - 1, 16);
  signal prer         : unsigned(15 downto 0) := PRESCALE_C;  -- clock prescale register
  signal ctr          : std_logic_vector(7 downto 0);  -- control register
  signal txr          : std_logic_vector(7 downto 0);  -- transmit register
  signal rxr          : std_logic_vector(7 downto 0);  -- receive register
  signal cr           : std_logic_vector(7 downto 0);  -- command register
  signal sr           : std_logic_vector(7 downto 0);  -- status register

  -- internal reset signal
  signal rst_i : std_logic;

  -- wishbone write access
  signal wb_wacc : std_logic;

  -- internal acknowledge signal
  signal iack_o : std_logic;

  -- done signal: command completed, clear command register
  signal done : std_logic;

  -- command register signals
  signal sta, sto, rd, wr, ack, iack : std_logic;

  signal core_en : std_logic;           -- core enable signal
  signal ien     : std_logic;           -- interrupt enable signal

  -- status register signals
  signal irxack, rxack : std_logic;     -- received aknowledge from slave
  signal tip           : std_logic;     -- transfer in progress
  signal irq_flag      : std_logic;     -- interrupt pending flag
  signal i2c_busy      : std_logic;     -- i2c bus busy (start signal detected)
  signal i2c_al, al    : std_logic;     -- arbitration lost

begin
  -- generate internal reset signal
  rst_i <= arst_i xor ARST_LVL;

  -- generate acknowledge output signal
  gen_ack_o : process(wb_clk_i)
  begin
    if (wb_clk_i'event and wb_clk_i = '1') then
      iack_o <= wb_cyc_i and wb_stb_i and not iack_o;  -- because timing is always honored
    end if;
  end process gen_ack_o;
  wb_ack_o <= iack_o;

  -- generate wishbone write access signal
  wb_wacc <= wb_we_i and iack_o;

  -- assign wb_dat_o
  assign_dato : process(wb_clk_i)
  begin
    if (wb_clk_i'event and wb_clk_i = '1') then
      case wb_adr_i is
        when "000" => wb_dat_o <= std_logic_vector(prer(7 downto 0));
        when "001" => wb_dat_o <= std_logic_vector(prer(15 downto 8));
        when "010" => wb_dat_o <= ctr;
        when "011" => wb_dat_o <= rxr;  -- write is transmit register TxR
        when "100" => wb_dat_o <= sr;   -- write is command register CR

        -- Debugging registers:
        -- These registers are not documented.
        -- Functionality could change in future releases
        when "101"  => wb_dat_o <= txr;
        when "110"  => wb_dat_o <= cr;
        when "111"  => wb_dat_o <= (others => '0');
        when others => wb_dat_o <= (others => 'X');  -- for simulation only
      end case;
    end if;
  end process assign_dato;


  -- generate registers (CR, SR see below)
  gen_regs : process(rst_i, wb_clk_i)
  begin
    if (rst_i = '0') then
      prer <= PRESCALE_C;
      ctr  <= (others => '0');
      txr  <= (others => '0');
    elsif (wb_clk_i'event and wb_clk_i = '1') then
      if (wb_rst_i = '1') then
        prer <= PRESCALE_C;
        ctr  <= (others => '0');
        txr  <= (others => '0');
      elsif (wb_wacc = '1') then
        case wb_adr_i is
          when "000" => prer(7 downto 0)  <= unsigned(wb_dat_i);
          when "001" => prer(15 downto 8) <= unsigned(wb_dat_i);
          when "010" => ctr               <= wb_dat_i;
          when "011" => txr               <= wb_dat_i;
          when "100" => null;  --write to CR, avoid executing the others clause

          -- illegal cases, for simulation only
          when others =>
            report ("Illegal write address, setting all registers to unknown.");
            prer <= (others => 'X');
            ctr  <= (others => 'X');
            txr  <= (others => 'X');
        end case;
      end if;
    end if;
  end process gen_regs;


  -- generate command register
  gen_cr : process(rst_i, wb_clk_i)
  begin
    if (rst_i = '0') then
      cr <= (others => '0');
    elsif (wb_clk_i'event and wb_clk_i = '1') then
      if (wb_rst_i = '1') then
        cr <= (others => '0');
      elsif (wb_wacc = '1') then
        if ((core_en = '1') and (wb_adr_i = "100")) then
          -- only take new commands when i2c core enabled
          -- pending commands are finished
          cr <= wb_dat_i;
        end if;
      else
        if (done = '1' or i2c_al = '1') then
          cr(7 downto 4) <= (others => '0');  -- clear command bits when command done or arbitration lost
        end if;

        cr(2 downto 1) <= (others => '0');  -- reserved bits, always '0'
        cr(0)          <= '0';              -- clear IRQ_ACK bit
      end if;
    end if;
  end process gen_cr;

  -- decode command register
  sta  <= cr(7);
  sto  <= cr(6);
  rd   <= cr(5);
  wr   <= cr(4);
  ack  <= cr(3);
  iack <= cr(0);

  -- decode control register
  core_en <= ctr(7);
  ien     <= ctr(6);

  -- hookup byte controller block
  byte_ctrl : i2c_master_byte_ctrl
    port map (
      clk      => wb_clk_i,
      rst      => wb_rst_i,
      nReset   => rst_i,
      ena      => core_en,
      clk_cnt  => prer,
      start    => sta,
      stop     => sto,
      read     => rd,
      write    => wr,
      ack_in   => ack,
      i2c_busy => i2c_busy,
      i2c_al   => i2c_al,
      din      => txr,
      cmd_ack  => done,
      ack_out  => irxack,
      dout     => rxr,
      scl_i    => scl_pad_i,
      scl_o    => scl_pad_o,
      scl_oen  => scl_padoen_o,
      sda_i    => sda_pad_i,
      sda_o    => sda_pad_o,
      sda_oen  => sda_padoen_o
      );


  -- status register block + interrupt request signal
  st_irq_block : block
  begin
    -- generate status register bits
    gen_sr_bits : process (wb_clk_i, rst_i)
    begin
      if (rst_i = '0') then
        al       <= '0';
        rxack    <= '0';
        tip      <= '0';
        irq_flag <= '0';
      elsif (wb_clk_i'event and wb_clk_i = '1') then
        if (wb_rst_i = '1') then
          al       <= '0';
          rxack    <= '0';
          tip      <= '0';
          irq_flag <= '0';
        else
          al    <= i2c_al or (al and not sta);
          rxack <= irxack;
          tip   <= (rd or wr);

          -- interrupt request flag is always generated
          irq_flag <= (done or i2c_al or irq_flag) and not iack;
        end if;
      end if;
    end process gen_sr_bits;

    -- generate interrupt request signals
    gen_irq : process (wb_clk_i, rst_i)
    begin
      if (rst_i = '0') then
        wb_inta_o <= '0';
      elsif (wb_clk_i'event and wb_clk_i = '1') then
        if (wb_rst_i = '1') then
          wb_inta_o <= '0';
        else
          -- interrupt signal is only generated when IEN (interrupt enable bit) is set
          wb_inta_o <= irq_flag and ien;
        end if;
      end if;
    end process gen_irq;

    -- assign status register bits
    sr(7)          <= rxack;
    sr(6)          <= i2c_busy;
    sr(5)          <= al;
    sr(4 downto 2) <= (others => '0');  -- reserved
    sr(1)          <= tip;
    sr(0)          <= irq_flag;
  end block;

end architecture structural;



--**********************************************************************
-- Copyright (c) 2012-2014 by XESS Corp <http://www.xess.com>.
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
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use XESS.CommonPckg.all;
use XESS.I2CPckg.all;

entity I2c is
  generic(
    FREQ_G     : real := 100.0;         -- Main clock frequency (MHz).
    I2C_FREQ_G : real := 0.1            -- I2C clock frequency (MHz).
    );
  port (
    clk_i  : in    std_logic;           -- Master clock input.
    rst_i  : in    std_logic := LO;     -- Synchronous active high reset.
    addr_i : in    std_logic_vector(2 downto 0);  -- Lower address bits.
    data_i : in    std_logic_vector(7 downto 0);  -- Databus input.
    data_o : out   std_logic_vector(7 downto 0);  -- Databus output.
    wr_i   : in    std_logic;           -- Write enable input.
    rd_i   : in    std_logic;           -- Read enable input.
    done_o : out   std_logic;           -- True when R/W operation is done.
    scl_io : inout std_logic;           -- Tristateable I2C clock line.
    sda_io : inout std_logic            -- Tristateable I2C data line.
    );
end entity;

architecture arch of I2c is
  signal rd_or_wr_s              : std_logic;  -- True when either read or write occurs.
  signal scl_pad_s, scl_padoen_s : std_logic;
  signal sda_pad_s, sda_padoen_s : std_logic;
begin
  rd_or_wr_s <= rd_i or wr_i;  -- Initiate a Wishbone bus cycle whenever read or write occurs.

  u0 : i2c_master_top
    generic map(
      FREQ_G     => FREQ_G,
      I2C_FREQ_G => I2C_FREQ_G
      )
    port map (
      wb_clk_i     => clk_i,
      wb_rst_i     => rst_i,
      wb_adr_i     => addr_i,
      wb_dat_i     => data_i,
      wb_dat_o     => data_o,
      wb_we_i      => wr_i,
      wb_stb_i     => rd_or_wr_s,  -- R/W operation activates the Wishbone interface.
      wb_cyc_i     => rd_or_wr_s,  -- R/W operation activates the Wishbone interface.
      wb_ack_o     => done_o,  -- Wishbone acknowledgement asserted when R/W is done.
      scl_pad_i    => scl_io,
      scl_pad_o    => scl_pad_s,
      scl_padoen_o => scl_padoen_s,
      sda_pad_i    => sda_io,
      sda_pad_o    => sda_pad_s,
      sda_padoen_o => sda_padoen_s
      );

  -- Tristate drivers for the I2C clock and data lines.
  scl_io <= scl_pad_s when scl_padoen_s = LO else HIZ;
  sda_io <= sda_pad_s when sda_padoen_s = LO else HIZ;
  
end architecture;
