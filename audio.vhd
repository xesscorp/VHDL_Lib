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

--**********************************************************************
--  A simple interface to the AK4565 stereo audio codec.
--**********************************************************************



library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use XESS.CommonPckg.all;

package AudioPckg is

  constant MIC_INPUT_C  : natural := 16#02#;
  constant LINE_INPUT_C : natural := 16#08#;

  component AudioInit is
    generic(
      FREQ_G         : real    := 100.0;  -- Master clock frequency (in MHz).
      INPUT_SOURCE_G : natural := LINE_INPUT_C;  -- Audio input source, either line-in or microphone.
      INPUT_GAIN_G   : real    := 0.0  -- Gain (in dB) of amplifier preceding codec ADC.
      );
    port(
      rst_i  : in  std_logic := LO;     -- Active-high reset.
      clk_i  : in  std_logic;           -- Master clock.
      csn_o  : out std_logic;  -- Active-low codec control port chip-select.
      cclk_o : out std_logic;           -- Codec control port clock.
      cdti_o : out std_logic;           -- Serial data to codec control port.
      done_o : out std_logic   -- True/high after initialization is complete.
      );
  end component;

  component AudioStream is
    generic(
      FREQ_G       : real    := 100.0;  -- Master clock frequency (in MHz).
      RESOLUTION_G : natural := 20      -- # bits in ADC/DAC.
      );
    port(
      rst_i : in std_logic := NO;       -- Active-high reset.
      clk_i : in std_logic;             -- Master clock.

      -- Host side.
      adc_o      : out std_logic_vector(RESOLUTION_G-1 downto 0);  -- Digitized output from analog->digital converters.
      dac_i      : in  std_logic_vector(RESOLUTION_G-1 downto 0) := (others => '0');  -- Input to codec digital->analog converters. 
      xfer_o     : out std_logic;  -- High when ADC transfers data and DAC accepts data.
      leftChan_o : out std_logic;  -- High when left channel is active; low when right channel is active.

      -- Codec side.
      mclk_o : out std_logic;  -- Master clock to codec (lower freq than master clock clk_i).
      sclk_o : out std_logic;           -- Serial bit clock = mclk / 4.
      lrck_o : out std_logic;           -- Left-right clock = sclk / 64.
      sdti_o : out std_logic;           -- Serial data to codec DAC.
      sdto_i : in  std_logic := LO      -- Serial data from codec ADC.
      );
  end component;

  component Audio is
    generic(
      FREQ_G         : real    := 100.0;  -- Master clock frequency (in MHz).
      RESOLUTION_G   : natural := 20;   -- # bits in ADC/DAC.
      INPUT_SOURCE_G : natural := LINE_INPUT_C;  -- Audio input source, either line-in or microphone.
      INPUT_GAIN_G   : real    := 0.0  -- Gain (in dB) of amplifier preceding codec ADC.
      );
    port(
      rst_i : in std_logic := NO;       -- Active-high reset.
      clk_i : in std_logic;             -- Master clock.

      -- Host side.
      leftAdc_o  : out std_logic_vector(RESOLUTION_G-1 downto 0);  -- Digitized output from analog->digital converters.
      rightAdc_o : out std_logic_vector(RESOLUTION_G-1 downto 0);  -- Digitized output from analog->digital converters.
      leftDac_i  : in  std_logic_vector(RESOLUTION_G-1 downto 0) := (others => '0');  -- Input to codec digital->analog converters. 
      rightDac_i : in  std_logic_vector(RESOLUTION_G-1 downto 0) := (others => '0');  -- Input to codec digital->analog converters. 
      xfer_o     : out std_logic;  -- High when ADC transfers data and DAC accepts data.

      -- Codec side.
      mclk_o : out std_logic;  -- Master clock to codec (lower freq than master clock clk_i).
      sclk_o : out std_logic;           -- Serial bit clock = mclk / 4.
      lrck_o : out std_logic;           -- Left-right clock = sclk / 64.
      sdti_o : out std_logic;  -- Serial data to codec DAC and also the control port (during initialization).
      sdto_i : in  std_logic := LO;     -- Serial data from codec ADC.
      csn_o  : out std_logic;  -- Active-low codec control-port chip-select.
      cclk_o : out std_logic            -- Codec control-port clock.
      );
  end component;

  component AudioRamIntfc is
    port (
      clk_i            : in  std_logic;  -- Master clock.
      rcrdStartAddr_i  : in  std_logic_vector;  -- Starting address for recording stereo audio samples.
      numRcrdSamples_i : in  std_logic_vector;  -- Number of left/right channel samples to record.
      rcrdSampleCntr_o : out std_logic_vector;  -- Outputs the number of recorded samples during a run.
      playStartAddr_i  : in  std_logic_vector;  -- Starting address for stereo audio playback.
      numPlaySamples_i : in  std_logic_vector;  -- Number of left/right channel samples to playback.
      playSampleCntr_o : out std_logic_vector;  -- Outputs the number of samples played during a run.
      run_i            : in  std_logic;  -- Raise to initiate record/playback.
      rcrdDone_o       : out std_logic;  -- True when recording is done.
      playDone_o       : out std_logic;  -- True when playback is done.
      xfer_i           : in  std_logic;  -- True when samples are transferred in/out of the audio codec.
      rcrdLeft_i       : in  std_logic_vector;  -- Input for samples from the codec left ADC channel.
      rcrdRight_i      : in  std_logic_vector;  -- Input for samples from the codec right ADC channel.
      playLeft_o       : out std_logic_vector;  -- Output for samples to the codec left DAC channel.
      playRight_o      : out std_logic_vector;  -- Output for samples to the codec right DAC channel.
      ramAddr_o        : out std_logic_vector;  -- RAM address bus.
      ramData_i        : in  std_logic_vector;  -- Data from RAM.
      ramData_o        : out std_logic_vector;  -- Data to RAM.
      ramRd_o          : out std_logic;  -- RAM read control.
      ramWr_o          : out std_logic;  -- RAM write control.
      ramDone_i        : in  std_logic  -- Ram R/W operation complete when true.
      );
  end component;

end package;



--**********************************************************************
-- Initialize the control registers of the AK4565 audio codec.
--**********************************************************************
library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.std_logic_arith.all;
use IEEE.MATH_REAL.all;
use XESS.CommonPckg.all;
use XESS.AudioPckg.all;

entity AudioInit is
  generic(
    FREQ_G         : real    := 100.0;  -- Master clock frequency (in MHz).
    INPUT_SOURCE_G : natural := LINE_INPUT_C;  -- Audio input source, either line-in or microphone.
    INPUT_GAIN_G   : real    := 0.0  -- Gain (in dB) of amplifier preceding codec ADC.
    );
  port(
    rst_i  : in  std_logic := LO;       -- Active-high reset.
    clk_i  : in  std_logic;             -- Master clock.
    csn_o  : out std_logic;  -- Active-low codec control port chip-select.
    cclk_o : out std_logic;             -- Codec control port clock.
    cdti_o : out std_logic;             -- Serial data to codec control port.
    done_o : out std_logic   -- True/high after initialization is complete.
    );
end entity;

architecture arch of AudioInit is
  constant C_FREQ_C        : real    := 3.0;  -- Control port frequency in MHz.
  constant NUM_REGS_C      : natural := 8;  -- # of control registers in codec.
  constant OPCODE_LEN_C    : natural := 3;  -- Length of control port opcode (read/write op).
  constant ADDR_LEN_C      : natural := 5;  -- Length of control register address.
  constant REG_LEN_C       : natural := 8;  -- Length of control register.
  constant TOTAL_REG_LEN_C : natural := OPCODE_LEN_C + ADDR_LEN_C + REG_LEN_C;

  -- Function to convert an input gain into the correct code to load into the audio codec register.
  function GainToAK4565Code(gain : in real; src : in natural) return natural is
    variable adjustedGain_v : real;
    type GainCodeTable_t is array(0 to 16#60#) of real;
    constant GAIN_CODE_TABLE_C : GainCodeTable_t := (-54.0, -50.0, -46.0, -42.0,
                 -39.0, -37.0, -35.0, -33.0, -31.0, -29.0, -27.0, -25.0, -23.0, -21.0, -19.0, -17.0,
                 -15.5, -14.5, -13.5, -12.5, -11.5, -10.5, -9.5, -8.5,
                 -7.75, -7.25, -6.75, -6.25, -5.75, -5.25, -4.75, -4.25, -3.75, -3.25, -2.75, -2.25,
                 -1.75, -1.25, -0.75, -0.25, 0.25, 0.75, 1.25, 1.75, 2.25, 2.75, 3.25, 3.75, 4.25, 4.75,
                 5.25, 5.75, 6.25, 6.75, 7.25, 7.75, 8.25, 8.75, 9.25, 9.75, 10.25, 10.75, 11.25, 11.75,
                 12.25, 12.75, 13.25, 13.75, 14.25, 14.75, 15.25, 15.75, 16.25, 16.75, 17.25, 17.75,
                 18.25, 18.75, 19.25, 19.75, 20.25, 20.75, 21.25, 21.75, 22.25, 22.75, 23.25, 23.75,
                 24.25, 24.75, 25.25, 25.75, 26.25, 26.75, 27.25, 27.75, 100000.0);
  begin
    if src = MIC_INPUT_C then
      adjustedGain_v := gain;
    else
      adjustedGain_v := gain + 22.0;
    end if;
    for gainCode in GAIN_CODE_TABLE_C'range loop
      if adjustedGain_v < GAIN_CODE_TABLE_C(gainCode) then
        return gainCode;
      end if;
    end loop;
    return GAIN_CODE_TABLE_C'high;
  end function;

  -- Function for initializing the bit string that will be loaded into codec Input PGA Control register (page 29 of AK4565 datasheet).
  function RegInit(reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7 : in natural) return std_logic_vector is
    constant WRITE_OPCODE_C : std_logic_vector := "111";  -- Write opcode for codec control port.
    variable regInitBits_v  : std_logic_vector(NUM_REGS_C * TOTAL_REG_LEN_C - 1 downto 0);
  begin
    regInitBits_v(15 downto 0)    := CONV_STD_LOGIC_VECTOR(reg0, REG_LEN_C) & "00000" & WRITE_OPCODE_C;
    regInitBits_v(31 downto 16)   := CONV_STD_LOGIC_VECTOR(reg1, REG_LEN_C) & "00001" & WRITE_OPCODE_C;
    regInitBits_v(47 downto 32)   := CONV_STD_LOGIC_VECTOR(reg2, REG_LEN_C) & "00010" & WRITE_OPCODE_C;
    regInitBits_v(63 downto 48)   := CONV_STD_LOGIC_VECTOR(reg3, REG_LEN_C) & "00011" & WRITE_OPCODE_C;
    regInitBits_v(79 downto 64)   := CONV_STD_LOGIC_VECTOR(reg4, REG_LEN_C) & "00100" & WRITE_OPCODE_C;
    regInitBits_v(95 downto 80)   := CONV_STD_LOGIC_VECTOR(reg5, REG_LEN_C) & "00101" & WRITE_OPCODE_C;
    regInitBits_v(111 downto 96)  := CONV_STD_LOGIC_VECTOR(reg6, REG_LEN_C) & "00110" & WRITE_OPCODE_C;
    regInitBits_v(127 downto 112) := CONV_STD_LOGIC_VECTOR(reg7, REG_LEN_C) & "00111" & WRITE_OPCODE_C;
    return regInitBits_v;
  end function;

  -- Bit string that contains the initialization data to load into codec control registers.
  --   R0: Input Select     - 0x08 = Line-in; 0x02 = Microphone. (Change this to change ADC input between line and microphone.)
  --   R1: Power Mgmt       - 0x15 = Everything ON; 0x00 = Everything OFF.
  --   R2: Mode Ctrl        - 0x19 = No de-emphasis, 20-bit MSB-justified serial data, 48 KHz data sampling, muting OFF.
  --   R3: Timer Select     - 0x01 = 125 us ALC limiter period, 8 ms ALC recovery period, 8 ms zero-crossing timeout, 24 ms fading period.
  --   R4: ALC Mode Ctrl #1 - 0x00 (Not using ALC, so who cares?)
  --   R5: ALC Mode Ctrl #2 - 0x28 (Not using ALC, so who cares?)
  --   R6: Operation Mode   - 0x00 = ALC disabled, fade in/out disabled.
  --   R7: Input Gain Ctrl  - 0x28 = -22 dB in line mode, 0 dB in microphone mode.
  signal regInitBits_r : std_logic_vector(NUM_REGS_C * TOTAL_REG_LEN_C-1 downto 0)
    := RegInit(INPUT_SOURCE_G, 16#15#, 16#19#, 16#01#, 16#00#, 16#28#, 16#00#, GainToAK4565Code(INPUT_GAIN_G, INPUT_SOURCE_G));

  signal shiftEn_s : std_logic := NO;  -- Enable shifting of bits into control registers.
  signal cclk_s    : std_logic := LO;  -- Internal version of control port clock. 
  signal cclkEn_s  : std_logic := NO;  -- When true, enable external control-port clock.
begin

  -- Use master clock to generate control-port clock and shift-enable pulse.
  process(clk_i)
    constant CLK_DIVISOR_C : integer                            := integer(ceil(FREQ_G / C_FREQ_C));
    variable shiftCnt_v    : integer range 0 to CLK_DIVISOR_C-1 := 0;
  begin
    if rising_edge(clk_i) then
      -- Scale-down master clock to generate control-port clock.
      if shiftCnt_v < CLK_DIVISOR_C / 2 then
        cclk_s <= LO;
      else
        cclk_s <= HI;
      end if;
      -- Create shift-enable pulse once at end of each control port clock cycle.
      if shiftCnt_v = CLK_DIVISOR_C - 1 then
        shiftCnt_v := 0;    -- Scaling interval ends; counter rolls over.
        shiftEn_s  <= YES;              -- Generate shift pulse.
      else
        shiftCnt_v := shiftCnt_v + 1;
        shiftEn_s  <= NO;
      end if;
    end if;
  end process;

  -- Shift the register initialization bit string into the codec control port.
  process(rst_i, clk_i, shiftEn_s)
    type StateType_t is (STARTUP_STATE, SELECT_STATE, SHIFT_STATE, STOP_STATE);  -- FSM states.
    variable state_v         : StateType_t                          := STARTUP_STATE;  -- FSM state register.
    constant STARTUP_DLY_F_C : real                                 := 86.0;  -- Codec startup delay in ms.
    constant STARTUP_DLY_C   : integer                              := integer(ceil(STARTUP_DLY_F_C * C_FREQ_C * 1000.0));
    variable startupTimer_v  : integer range 0 to STARTUP_DLY_C-1   := STARTUP_DLY_C-1;  -- Timer for startup interval.
    variable bitCnt_v        : integer range 0 to TOTAL_REG_LEN_C-1 := 0;  -- Current control register bit counter.
    variable regCnt_v        : integer range 0 to NUM_REGS_C-1      := 0;  -- Current control register address.
    variable reset_v         : boolean                              := false;  -- Store reset input.
  begin
    if rising_edge(clk_i) then
      if rst_i = YES then
        -- Save the reset input and only act upon it when in the STOP state so the shift register
        -- of register control bits doesn't get out of sync.
        reset_v := true;
      end if;
      if shiftEn_s = YES then
        case state_v is
          when STARTUP_STATE =>
            -- Wait for the codec startup delay to pass.
            done_o   <= NO;             -- Codec initialization is not done.
            csn_o    <= HI;             -- Disable codec control port.
            cclkEn_s <= NO;             -- Disable clock to codec control port.
            regCnt_v := 0;     -- Start initializing with control register 0.
            -- Increment timer and exit this state when startup interval ends.
            if startupTimer_v = 0 then
              state_v := SELECT_STATE;
            end if;
            startupTimer_v := startupTimer_v - 1;
          when SELECT_STATE =>
            -- Select the control port and get ready to load another register.
            csn_o    <= LO;             -- Enable codec control port.
            cclkEn_s <= YES;            -- Enable codec control port clock.
            bitCnt_v := 0;              -- Start at bit 0 in current register.
            state_v  := SHIFT_STATE;
          when SHIFT_STATE =>
            -- Shift register bit into control port.
            regInitBits_r <= regInitBits_r(0) & regInitBits_r(regInitBits_r'high downto 1);
            -- Repeat this state until all the bits for this register are sent to control port.
            if bitCnt_v = TOTAL_REG_LEN_C-1 then
              -- All bits for current register are output at this point.
              csn_o    <= HI;  -- Deselect the control port to write value into register.
              cclkEn_s <= NO;           -- Disable the control port clock.
              if regCnt_v = NUM_REGS_C-1 then
                state_v := STOP_STATE;  -- All register initialized, so stop.
              else
                state_v := SELECT_STATE;  -- Otherwise, init the next register.
              end if;
              regCnt_v := regCnt_v + 1;   -- Go to the next register.
            end if;
            bitCnt_v := bitCnt_v + 1;   -- Go to the next bit.
          when STOP_STATE =>
            if reset_v then
              -- If reset occured, re-do the entire initialization. 
              startupTimer_v := STARTUP_DLY_C-1;  -- Re-do the startup delay (probably not necessary).
              state_v        := STARTUP_STATE;    -- Re-do the initialization.
              reset_v        := false;  -- Clear the reset flag.
            else
              -- Otherwise, initialization has benn completed.
              done_o <= YES;
            end if;
          when others =>
        end case;
      end if;
    end if;
  end process;

  -- Attach internal signals to control port.
  cclk_o <= cclk_s when cclkEn_s = YES else ZERO;  -- Attach internal clock to control-port clock.
  cdti_o <= regInitBits_r(0);  -- Attach shift register to control-port data pin.

end architecture;



--**********************************************************************
-- Shift data into the audio codec DAC and out of the ADC.
--**********************************************************************
library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.MATH_REAL.all;
use XESS.CommonPckg.all;

entity AudioStream is
  generic(
    FREQ_G       : real    := 100.0;    -- Master clock frequency (in MHz).
    RESOLUTION_G : natural := 20        -- # bits in ADC/DAC.
    );
  port(
    rst_i : in std_logic := NO;         -- Active-high reset.
    clk_i : in std_logic;               -- Master clock.

    -- Host side.
    adc_o      : out std_logic_vector(RESOLUTION_G-1 downto 0);  -- Digitized output from analog->digital converters.
    dac_i      : in  std_logic_vector(RESOLUTION_G-1 downto 0) := (others => '0');  -- Input to codec digital->analog converters. 
    xfer_o     : out std_logic;  -- High when ADC transfers data and DAC accepts data.
    leftChan_o : out std_logic;  -- High when left channel is active; low when right channel is active.

    -- Codec side.
    mclk_o : out std_logic;  -- Master clock to codec (lower freq than master clock clk_i).
    sclk_o : out std_logic;             -- Serial bit clock = mclk / 4.
    lrck_o : out std_logic;             -- Left-right clock = sclk / 64.
    sdti_o : out std_logic;             -- Serial data to codec DAC.
    sdto_i : in  std_logic := LO        -- Serial data from codec ADC.
    );
end entity;


architecture arch of AudioStream is
  constant MCLK_FREQ           : real    := 12.0;  -- Frequency of MCLK signal to codec.
  constant DIVISOR_C           : natural := 2**Log2(integer(FREQ_G / MCLK_FREQ));
  constant MCLK_SCLK_RATIO_C   : natural := 4;  -- Four mclks per sclk.
  constant CHAN_FRAME_LEN_C    : natural := 32;  -- Number of sclks in each left/right serial channel frame.
  constant NUM_CHANNELS_C      : natural := 2;  -- Number of codec channels (left & right).
  constant MCLK_BIT_C          : natural := Log2(DIVISOR_C)-1;  -- Position of mclk output in counter.
  constant PHASE_CNT_LEN_C     : natural := Log2(DIVISOR_C * MCLK_SCLK_RATIO_C);  -- Num bits in phase portion of counter.
  constant BIT_CNT_LEN_C       : natural := Log2(CHAN_FRAME_LEN_C);  -- Number of bits in bit counter portion of counter.
  constant CHAN_CNT_LEN_C      : natural := Log2(NUM_CHANNELS_C);  -- Number of bits in left/right channel portion of counter.
  constant CNT_LEN_C           : natural := PHASE_CNT_LEN_C + BIT_CNT_LEN_C + CHAN_CNT_LEN_C;  -- Counter = left/right & bit counter & phase.
  constant SCLK_RISING_EDGE_C  : natural := (2**(PHASE_CNT_LEN_C-1))-1;  -- Value of phase when sclk has a rising edge.
  constant SCLK_FALLING_EDGE_C : natural := (2**PHASE_CNT_LEN_C)-1;  -- Value of phase when sclk has a falling edge.

  -- Timing counter is divided into three fields: (MSB)|<- channel frame ->|<- bit slot within frame ->|<- phase within bit ->| (LSB)
  signal cnt_r        : unsigned(CNT_LEN_C-1 downto 0)            := (others => ZERO);  -- Counter for generating all the codec timing signals.
  -- LSBits of the counter that indicate the phase within each serial bit interval.
  alias phase_r       : unsigned(PHASE_CNT_LEN_C-1 downto 0) is cnt_r(PHASE_CNT_LEN_C-1 downto 0);
  -- Upper bits of counter that indicate the left/right channel frame and the bit slot.
  alias leftChanCnt_r : unsigned(BIT_CNT_LEN_C + CHAN_CNT_LEN_C - 1 downto 0) is cnt_r(cnt_r'high downto PHASE_CNT_LEN_C);
  -- Upper bits of the counter that indicate the bit slot within a frame.
  alias bitCnt_r      : unsigned(BIT_CNT_LEN_C-1 downto 0) is cnt_r(cnt_r'high-CHAN_CNT_LEN_C downto PHASE_CNT_LEN_C);
  signal adc_r        : std_logic_vector(RESOLUTION_G-1 downto 0) := (others => ZERO);  -- Shift-reg for receiving bits from the ADCs.
  signal dac_r        : std_logic_vector(RESOLUTION_G-1 downto 0) := (others => ZERO);  -- Shift-reg for sending bits to the DACs.
  signal xfer_s       : std_logic;
begin

  process(rst_i, clk_i)
  begin
    if rst_i = YES then
      cnt_r <= (others => ZERO);
      adc_r <= (others => ZERO);
      dac_r <= (others => ZERO);
    elsif rising_edge(clk_i) then
      cnt_r <= cnt_r + 1;
      if bitCnt_r < RESOLUTION_G then   -- Shift ADC & DAC bits.
        if phase_r = SCLK_RISING_EDGE_C then
          -- SDTO changes on the falling edge of SCLK, so we gather the bits on the rising edge when they are stable.
          adc_r <= adc_r(RESOLUTION_G-2 downto 0) & sdto_i;  -- Gather the bits from the codec ADC into the ADC shift register.
        end if;
        if phase_r = SCLK_FALLING_EDGE_C then
          -- SDTI accepts bits on the rising edge of SCLK, so we output on the falling edge to make sure the bits are stable.
          dac_r <= dac_r(RESOLUTION_G-2 downto 0) & ZERO;  -- Shift the bits out of the DAC shift register and into the codec DAC.
        end if;
      elsif xfer_s = YES then
        dac_r <= dac_i;  -- Else, parallel load the DAC shift register with a new value from the host.
      end if;
    end if;
  end process;

  -- Generate the codec clocks from the appropriate counter bits.
  mclk_o <= cnt_r(MCLK_BIT_C);
  sclk_o <= phase_r(phase_r'high);
  lrck_o <= leftChanCnt_r(leftChanCnt_r'high);

  -- Tell the host when the ADC value is available and the DAC needs to be reloaded.
  xfer_s <= YES when bitcnt_r = RESOLUTION_G and phase_r = 0 else NO;
  xfer_o <= xfer_s;                     -- Output to the host.

  leftChan_o <= leftChanCnt_r(leftChanCnt_r'high);  -- Tell the host which stereo channel is active.
  adc_o      <= adc_r;  -- Connect the parallel output from the ADC shift register to the host.
  sdti_o     <= dac_r(RESOLUTION_G-1);  -- Connect the MSBit of the DAC shift register to the codec SDTI pin.

end arch;



--**********************************************************************
-- Complete audio codec interface containing initialization and streaming modules.
--**********************************************************************
library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_signed.all;
use IEEE.numeric_std.all;
use XESS.AudioPckg.all;
use XESS.ClkGenPckg.all;
use XESS.CommonPckg.all;

entity Audio is
  generic(
    FREQ_G         : real    := 100.0;  -- Master clock frequency (in MHz).
    RESOLUTION_G   : natural := 20;     -- # bits in ADC/DAC.
    INPUT_SOURCE_G : natural := LINE_INPUT_C;  -- Audio input source, either line-in or microphone.
    INPUT_GAIN_G   : real    := 0.0  -- Gain (in dB) of amplifier preceding codec ADC.
    );
  port(
    rst_i : in std_logic := NO;         -- Active-high reset.
    clk_i : in std_logic;               -- Master clock.

    -- Host side.
    leftAdc_o  : out std_logic_vector(RESOLUTION_G-1 downto 0);  -- Digitized output from analog->digital converters.
    rightAdc_o : out std_logic_vector(RESOLUTION_G-1 downto 0);  -- Digitized output from analog->digital converters.
    leftDac_i  : in  std_logic_vector(RESOLUTION_G-1 downto 0) := (others => '0');  -- Input to codec digital->analog converters. 
    rightDac_i : in  std_logic_vector(RESOLUTION_G-1 downto 0) := (others => '0');  -- Input to codec digital->analog converters. 
    xfer_o     : out std_logic;  -- High when ADC transfers data and DAC accepts data.

    -- Codec side.
    mclk_o : out std_logic;  -- Master clock to codec (lower freq than master clock clk_i).
    sclk_o : out std_logic;             -- Serial bit clock = mclk / 4.
    lrck_o : out std_logic;             -- Left-right clock = sclk / 64.
    sdti_o : out std_logic;  -- Serial data to codec DAC and also the control port (during initialization).
    sdto_i : in  std_logic := LO;       -- Serial data from codec ADC.
    csn_o  : out std_logic;  -- Active-low codec control-port chip-select.
    cclk_o : out std_logic              -- Codec control-port clock.
    );
end entity;


architecture arch of Audio is
  signal adc_s      : std_logic_vector(RESOLUTION_G-1 downto 0);  -- Output from active codec ADC.
  signal dac_s      : std_logic_vector(RESOLUTION_G-1 downto 0);  -- Input to active codec DAC.
  signal leftChan_s : std_logic;  -- True when left codec channel is active, false when right channel is active.
  signal initDone_s : std_logic;  -- True when codec initialization has completed.
  signal xfer_s     : std_logic;  -- True (for one clock cycle) when ADC output is available and DAC input is accepted.
  signal sdti_s     : std_logic;  -- Serial bitstream output to codec DACs.
  signal cdti_s     : std_logic;  -- Serial bitstream output to codec control port.
begin

  -- Instantiate the codec initialization module.
  u1 : AudioInit
    generic map(
      FREQ_G         => FREQ_G,
      INPUT_SOURCE_G => INPUT_SOURCE_G,
      INPUT_GAIN_G   => INPUT_GAIN_G
      )
    port map(
      rst_i  => rst_i,
      clk_i  => clk_i,
      csn_o  => csn_o,
      cclk_o => cclk_o,
      cdti_o => cdti_s,
      done_o => initDone_s  -- This output enabless the AudioStream module once initialization is complete.
      );

  -- Instantiate the codec interface.
  u0 : AudioStream
    generic map(
      FREQ_G => FREQ_G
      )
    port map(
      rst_i      => not initDone_s,
      clk_i      => clk_i,
      adc_o      => adc_s,
      dac_i      => dac_s,
      xfer_o     => xfer_s,
      leftChan_o => leftChan_s,
      mclk_o     => mclk_o,
      sclk_o     => sclk_o,
      lrck_o     => lrck_o,
      sdti_o     => sdti_s,
      sdto_i     => sdto_i
      );

  -- Clock the ADC output from the codec into the left or right
  -- ADC output depending upon which codec channel is active.
  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if xfer_s = YES then
        if leftChan_s = YES then
          leftAdc_o <= adc_s;  -- Store the current ADC output from the left channel.
        else
          rightAdc_o <= adc_s;  -- Store the current ADC output from the right channel.
        end if;
      end if;
    end if;
  end process;

  -- Indicate when both left and right channel ADC samples are available.
  xfer_o <= xfer_s and not leftChan_s;

  -- Connect the appropriate input to the DAC depending upon whether the left or right channel is active.
  -- (The left input has to enter before the left channel becomes active, so the selection looks like
  -- it's opposite from what it should be. Same with the right channel.)
  dac_s <= leftDac_i when leftChan_s = NO else rightDac_i;  -- Input enters BEFORE the channel becomes active.

  -- The AudioStream module pipes serial data to the codec DACs after AudioInit module has
  -- finished using the same pin to initialize the codec.
  sdti_o <= sdti_s when initDone_s = YES else cdti_s;
end architecture;




--**********************************************************************
-- Interface for recording/playing stereo audio to/from RAM.
--
-- This finite state machine (FSM) takes left/right samples from the audio codec ADC and stores them
-- in RAM, and transfers left/right samples from the RAM to the audio codec DAC for playback.

-- Whenever the run control input is inactive, the FSM latches new values for the starting addresses
-- and the number of samples for recording and playback.

-- When the run control input is active, the FSM transfers values from the codec ADC, stores
-- the left-channel value at address N, stores the right channel address at address N+1, and decrements
-- the number of samples that remain to be recorded. Then it transfers RAM values at addresses M and M+1
-- to the left and right codec DACs, respectively, and decrements the number of samples that remain
-- to be played back.
--**********************************************************************
library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_signed.all;
use IEEE.numeric_std.all;
use XESS.CommonPckg.all;

entity AudioRamIntfc is
  port (
    clk_i            : in  std_logic;   -- Master clock.
    rcrdStartAddr_i  : in  std_logic_vector;  -- Starting address for recording stereo audio samples.
    numRcrdSamples_i : in  std_logic_vector;  -- Number of left/right channel samples to record.
    rcrdSampleCntr_o : out std_logic_vector;  -- Outputs the number of recorded samples during a run.
    playStartAddr_i  : in  std_logic_vector;  -- Starting address for stereo audio playback.
    numPlaySamples_i : in  std_logic_vector;  -- Number of left/right channel samples to playback.
    playSampleCntr_o : out std_logic_vector;  -- Outputs the number of samples played during a run.
    run_i            : in  std_logic;   -- Raise to initiate record/playback.
    rcrdDone_o       : out std_logic;   -- True when recording is done.
    playDone_o       : out std_logic;   -- True when playback is done.
    xfer_i           : in  std_logic;  -- True when samples are transferred in/out of the audio codec.
    rcrdLeft_i       : in  std_logic_vector;  -- Input for samples from the codec left ADC channel.
    rcrdRight_i      : in  std_logic_vector;  -- Input for samples from the codec right ADC channel.
    playLeft_o       : out std_logic_vector;  -- Output for samples to the codec left DAC channel.
    playRight_o      : out std_logic_vector;  -- Output for samples to the codec right DAC channel.
    ramAddr_o        : out std_logic_vector;  -- RAM address bus.
    ramData_i        : in  std_logic_vector;  -- Data from RAM.
    ramData_o        : out std_logic_vector;  -- Data to RAM.
    ramRd_o          : out std_logic;   -- RAM read control.
    ramWr_o          : out std_logic;   -- RAM write control.
    ramDone_i        : in  std_logic   -- Ram R/W operation complete when true.
    );
end entity;

architecture arch of AudioRamIntfc is
  type FsmState_t is (WAIT_FOR_XFER,
                      CHECK_RECORDING_ACTIVE, RECORD_LEFT, RECORD_RIGHT,
                      CHECK_PLAYBACK_ACTIVE, PLAY_LEFT, PLAY_RIGHT);
  signal fsmState_r       : FsmState_t := WAIT_FOR_XFER;
  signal rcrdLeft_r       : std_logic_vector(rcrdLeft_i'range);  -- Holds the current sample from the codec left ADC channel.
  signal rcrdRight_r      : std_logic_vector(rcrdRight_i'range);  -- Holds the current sample from the codec right ADC channel.
  signal numRcrdSamples_r : unsigned(numRcrdSamples_i'range);  -- Holds the number of samples to be recorded during a run.
  signal rcrdAddr_r       : unsigned(rcrdStartAddr_i'range);  -- Holds the address where the current sample will be recorded.
  signal rcrdSampleCntr_r : unsigned(rcrdSampleCntr_o'range);  -- Holds the current number of samples that have been recorded.
  signal numPlaySamples_r : unsigned(numPlaySamples_i'range);  --  Holds the number of samples to be played during a run.
  signal playAddr_r       : unsigned(playStartAddr_i'range);  -- Holds the address of the current sample to be played.
  signal playSampleCntr_r : unsigned(playSampleCntr_o'range);  -- Holds the current number of samples that have been played.
begin

  process(clk_i)
  begin
    if rising_edge(clk_i) then
      case fsmState_r is
        when WAIT_FOR_XFER =>  -- Waiting for transfer of samples to/from the audio codec.
          ramWr_o <= NO;  -- Make sure reads/writes of RAM are turned off while waiting.
          ramRd_o <= NO;
          if run_i = YES then
            -- If playback/record is active, then wait for the codec to accept/provide samples.
            if xfer_i = YES then
              -- Transfer is active, so latch the recorded samples from the codec.
              rcrdLeft_r  <= rcrdLeft_i;
              rcrdRight_r <= rcrdRight_i;
              fsmState_r  <= CHECK_RECORDING_ACTIVE;  -- Start/continue with recording and playback.
            end if;
          else
            -- Playback/record is not active, so just continually latch new values for the
            -- playback/record starting addresses and number of samples.
            numRcrdSamples_r <= unsigned(numRcrdSamples_i);
            rcrdSampleCntr_r <= (others => ZERO);
            rcrdAddr_r       <= unsigned(rcrdStartAddr_i);
            numPlaySamples_r <= unsigned(numPlaySamples_i);
            playSampleCntr_r <= (others => ZERO);
            playAddr_r       <= unsigned(playStartAddr_i);
            -- Zero the output after playback is done so charge doesn't build up in the DC-blocking capacitors.
            playLeft_o       <= (others => ZERO);
            playRight_o      <= (others => ZERO);
          end if;
        when CHECK_RECORDING_ACTIVE =>  -- Check to see if any samples remain to be recorded.
          
          if rcrdSampleCntr_r /= numRcrdSamples_r then
            rcrdDone_o <= NO;  -- More samples needed, so indicate the recording is not done.
            fsmState_r <= RECORD_LEFT;  -- Collect samples from the left and right channels.
          else
            rcrdDone_o <= YES;  -- No more samples needed, so indicate that the recording is done.
            fsmState_r <= CHECK_PLAYBACK_ACTIVE;  -- Go to playback portion of this FSM.
          end if;
        when RECORD_LEFT =>  -- Sample the left codec channel and store it in RAM.
          -- Output the current left-channel sample value and storage address for writing to RAM.
          ramData_o <= rcrdLeft_r(rcrdLeft_r'high downto rcrdLeft_r'high-ramData_o'length+1);
          ramAddr_o <= std_logic_vector(rcrdAddr_r);
          ramWr_o   <= YES;
          -- Hold the RAM data, address and write-control until the RAM indicates the write is completed.
          if ramDone_i = YES then
            -- Write of sample to RAM is done, so deactivate the write-control, advance write address
            -- to the next location, and move to the next state.
            ramWr_o    <= NO;
            rcrdAddr_r <= rcrdAddr_r + 1;
            fsmState_r <= RECORD_RIGHT;
          end if;
        when RECORD_RIGHT =>  -- Sample the right codec channel and store it in RAM.
          -- Output the current right-channel sample value and storage address for writing to RAM.
          ramData_o <= rcrdRight_r(rcrdRight_r'high downto rcrdRight_r'high-ramData_o'length+1);
          ramAddr_o <= std_logic_vector(rcrdAddr_r);
          ramWr_o   <= YES;
          -- Hold the RAM data, address and write-control until the RAM indicates the write is completed.
          if ramDone_i = YES then
            -- Write of sample to RAM is done, so deactivate the write-control, advance write address
            -- to the next location, increment the number of samples already collected,
            -- and move to the next state.
            ramWr_o          <= NO;
            rcrdAddr_r       <= rcrdAddr_r + 1;
            rcrdSampleCntr_r <= rcrdSampleCntr_r + 1;
            rcrdSampleCntr_o <= std_logic_vector(rcrdSampleCntr_r); -- Register the current number of samples collected on the output bus.
            fsmState_r       <= CHECK_PLAYBACK_ACTIVE;
          end if;
        when CHECK_PLAYBACK_ACTIVE =>  -- Check to see if any samples remain to be played back.
          if playSampleCntr_r /= numPlaySamples_r then
            playDone_o <= NO;  -- More samples needed, so indicate the playback is not done.
            fsmState_r <= PLAY_LEFT;  -- Playback samples from the left and right channels.
          else
            playDone_o <= YES;  -- No more samples needed, so indicate that the playback is done.
            fsmState_r <= WAIT_FOR_XFER;  -- Go back and wait for another sample from the codec.
          end if;
        when PLAY_LEFT =>  -- Read the next sample from RAM and send it to the left codec channel.
          -- Output the current left-channel sample storage address for reading from RAM.
          ramAddr_o <= std_logic_vector(playAddr_r);
          ramRd_o   <= YES;
          -- Hold the RAM address and read-control until the RAM delivers the sample value.
          if ramDone_i = YES then
            -- Register and output the left sample value so the codec can grab it during the next transfer.
            playLeft_o <= ramData_i & "0000";
            -- Read of sample from RAM is done, so deactivate the read-control, advance read address
            -- to the next location, and move to the next state.
            ramRd_o    <= NO;
            playAddr_r <= playAddr_r + 1;
            fsmState_r <= PLAY_RIGHT;
          end if;
        when PLAY_RIGHT =>  -- Read the next sample from RAM and send it to the right codec channel.
          -- Output the current right-channel sample storage address for reading from RAM.
          ramAddr_o <= std_logic_vector(playAddr_r);
          ramRd_o   <= YES;
          -- Hold the RAM address and read-control until the RAM delivers the sample value.
          if ramDone_i = YES then
            -- Register and output the right sample value so the codec can grab it during the next transfer.
            playRight_o      <= ramData_i & "0000";
            -- Read of sample from RAM is done, so deactivate the read-control, advance read address
            -- to the next location, increment the number of samples that have been played,
            -- and move to the next state.
            ramRd_o          <= NO;
            playAddr_r       <= playAddr_r + 1;
            playSampleCntr_r <= playSampleCntr_r + 1;
            playSampleCntr_o <= std_logic_vector(playSampleCntr_r);  -- Register the current number of samples played on the output bus.
            fsmState_r       <= WAIT_FOR_XFER;  -- Go back and wait for another sample from the codec.
          end if;
      end case;
    end if;
  end process;
end architecture;




--**********************************************************************
-- A simple loopback test.
--**********************************************************************

library IEEE, XESS;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_signed.all;
use IEEE.numeric_std.all;
use XESS.AudioPckg.all;
use XESS.ClkGenPckg.all;
use XESS.CommonPckg.all;

entity AudioLoopbackTest is
  port(
    clk_i  : in  std_logic;             -- 12 MHz clock.
    mclk_o : out std_logic;
    sclk_o : out std_logic;
    lrck_o : out std_logic;
    sdti_o : out std_logic;
    sdto_i : in  std_logic;
    csn_o  : out std_logic;
    cclk_o : out std_logic
    );
end entity;


architecture arch of AudioLoopbackTest is
  signal clk_s      : std_logic;
  signal leftAdc_s  : std_logic_vector(19 downto 0);
  signal rightAdc_s : std_logic_vector(19 downto 0);
begin

  -- Generate a 100 MHz clock from the 12 MHz input clock.
  uClkgen : ClkGen
    generic map (CLK_MUL_G => 25, CLK_DIV_G => 3)
    port map(I             => clk_i, O => clk_s);

  -- Loop the ADC codec outputs back to the DAC inputs.
  uAudio : Audio
    port map(
      clk_i      => clk_s,
      leftAdc_o  => leftAdc_s,
      rightAdc_o => rightAdc_s,
      leftDac_i  => leftAdc_s,
      rightDac_i => rightAdc_s,
      mclk_o     => mclk_o,
      sclk_o     => sclk_o,
      lrck_o     => lrck_o,
      sdti_o     => sdti_o,
      sdto_i     => sdto_i,
      csn_o      => csn_o,
      cclk_o     => cclk_o
      );

end architecture;
