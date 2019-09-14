--------------------------------------------------------------------------------
-- Copyright (c) 1995-2003 Xilinx, Inc.
-- All Right Reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 9.1i
--  \   \         Application : ISE
--  /   /         Filename : ca_code_nco_tbw.vhw
-- /___/   /\     Timestamp : Thu May 10 15:48:17 2007
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: 
--Design Name: ca_code_nco_tbw
--Device: Xilinx
--

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
USE IEEE.STD_LOGIC_TEXTIO.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE STD.TEXTIO.ALL;

ENTITY ca_code_nco_tbw IS
END ca_code_nco_tbw;

ARCHITECTURE testbench_arch OF ca_code_nco_tbw IS
    FILE RESULTS: TEXT OPEN WRITE_MODE IS "results.txt";

    COMPONENT ca_code_nco
        PORT (
            BUS_IN_FREQ : In std_logic_vector (31 DownTo 0);
            BUS_IN_PHASE : In std_logic_vector (10 DownTo 0);
            CLOCK : In std_logic;
            CODE_EPOCH : In std_logic;
            DAV_FREQ : In std_logic;
            DAV_PHASE : In std_logic;
            RESET : In std_logic;
            DCLK : Out std_logic;
            FREQ_INC_READ : Out std_logic;
            INC : Out std_logic;
            PHASE_OUT : Out std_logic_vector (15 DownTo 0);
            PHASE_SHIFT_APPLIED : Out std_logic
        );
    END COMPONENT;

    SIGNAL BUS_IN_FREQ : std_logic_vector (31 DownTo 0) := "00000010000000000000000000000001";
    SIGNAL BUS_IN_PHASE : std_logic_vector (10 DownTo 0) := "00000000010";
    SIGNAL CLOCK : std_logic := '0';
    SIGNAL CODE_EPOCH : std_logic := '0';
    SIGNAL DAV_FREQ : std_logic := '0';
    SIGNAL DAV_PHASE : std_logic := '0';
    SIGNAL RESET : std_logic := '0';
    SIGNAL DCLK : std_logic := '0';
    SIGNAL FREQ_INC_READ : std_logic := '0';
    SIGNAL INC : std_logic := '0';
    SIGNAL PHASE_OUT : std_logic_vector (15 DownTo 0) := "0000000000000000";
    SIGNAL PHASE_SHIFT_APPLIED : std_logic := '0';

    constant PERIOD : time := 10 ns;
    constant DUTY_CYCLE : real := 0.5;
    constant OFFSET : time := 0 ns;

    BEGIN
        UUT : ca_code_nco
        PORT MAP (
            BUS_IN_FREQ => BUS_IN_FREQ,
            BUS_IN_PHASE => BUS_IN_PHASE,
            CLOCK => CLOCK,
            CODE_EPOCH => CODE_EPOCH,
            DAV_FREQ => DAV_FREQ,
            DAV_PHASE => DAV_PHASE,
            RESET => RESET,
            DCLK => DCLK,
            FREQ_INC_READ => FREQ_INC_READ,
            INC => INC,
            PHASE_OUT => PHASE_OUT,
            PHASE_SHIFT_APPLIED => PHASE_SHIFT_APPLIED
        );

        PROCESS    -- clock process for CLOCK
        BEGIN
            WAIT for OFFSET;
            CLOCK_LOOP : LOOP
                CLOCK <= '0';
                WAIT FOR (PERIOD - (PERIOD * DUTY_CYCLE));
                CLOCK <= '1';
                WAIT FOR (PERIOD * DUTY_CYCLE);
            END LOOP CLOCK_LOOP;
        END PROCESS;

        PROCESS
            BEGIN
                -- -------------  Current Time:  14ns
                WAIT FOR 14 ns;
                RESET <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  24ns
                WAIT FOR 10 ns;
                CODE_EPOCH <= '1';
                DAV_FREQ <= '1';
                RESET <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  34ns
                WAIT FOR 10 ns;
                CODE_EPOCH <= '0';
                DAV_FREQ <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  2764ns
                WAIT FOR 2730 ns;
                BUS_IN_FREQ <= "00000000000000000000000000000000";
                -- -------------------------------------
                -- -------------  Current Time:  2784ns
                WAIT FOR 20 ns;
                CODE_EPOCH <= '1';
                DAV_FREQ <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  2794ns
                WAIT FOR 10 ns;
                CODE_EPOCH <= '0';
                DAV_FREQ <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  2904ns
                WAIT FOR 110 ns;
                DAV_PHASE <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  2914ns
                WAIT FOR 10 ns;
                DAV_PHASE <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  2934ns
                WAIT FOR 20 ns;
                CODE_EPOCH <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  2944ns
                WAIT FOR 10 ns;
                CODE_EPOCH <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  3074ns
                WAIT FOR 130 ns;
                BUS_IN_PHASE <= "01000000010";
                -- -------------------------------------
                -- -------------  Current Time:  3084ns
                WAIT FOR 10 ns;
                DAV_PHASE <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  3094ns
                WAIT FOR 10 ns;
                DAV_PHASE <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  3104ns
                WAIT FOR 10 ns;
                CODE_EPOCH <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  3114ns
                WAIT FOR 10 ns;
                CODE_EPOCH <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  3304ns
                WAIT FOR 190 ns;
                BUS_IN_PHASE <= "10000000010";
                -- -------------------------------------
                -- -------------  Current Time:  3324ns
                WAIT FOR 20 ns;
                DAV_PHASE <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  3334ns
                WAIT FOR 10 ns;
                DAV_PHASE <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  3554ns
                WAIT FOR 220 ns;
                BUS_IN_PHASE <= "11000000010";
                -- -------------------------------------
                -- -------------  Current Time:  3574ns
                WAIT FOR 20 ns;
                DAV_PHASE <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  3584ns
                WAIT FOR 10 ns;
                DAV_PHASE <= '0';
                -- -------------------------------------
                WAIT FOR 16426 ns;

            END PROCESS;

    END testbench_arch;
