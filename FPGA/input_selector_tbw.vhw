--------------------------------------------------------------------------------
-- Copyright (c) 1995-2003 Xilinx, Inc.
-- All Right Reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 9.1i
--  \   \         Application : ISE
--  /   /         Filename : input_selector_tbw.vhw
-- /___/   /\     Timestamp : Thu Apr 19 21:51:59 2007
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: 
--Design Name: input_selector_tbw
--Device: Xilinx
--

library IEEE;
use IEEE.std_logic_1164.all;
USE IEEE.STD_LOGIC_TEXTIO.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
USE STD.TEXTIO.ALL;

ENTITY input_selector_tbw IS
END input_selector_tbw;

ARCHITECTURE testbench_arch OF input_selector_tbw IS
    FILE RESULTS: TEXT OPEN WRITE_MODE IS "results.txt";

    COMPONENT INPUT_SELECTOR
        PORT (
            CLOCK : In std_logic;
            I0 : In std_logic_vector (1 DownTo 0);
            I1 : In std_logic_vector (1 DownTo 0);
            I2 : In std_logic_vector (1 DownTo 0);
            I3 : In std_logic_vector (1 DownTo 0);
            I4 : In std_logic_vector (1 DownTo 0);
            I5 : In std_logic_vector (1 DownTo 0);
            I6 : In std_logic_vector (1 DownTo 0);
            I7 : In std_logic_vector (1 DownTo 0);
            INVQ_L1 : In std_logic;
            INVQ_L2 : In std_logic;
            INVI_L2 : In std_logic;
            Q0 : In std_logic_vector (1 DownTo 0);
            Q1 : In std_logic_vector (1 DownTo 0);
            Q2 : In std_logic_vector (1 DownTo 0);
            Q3 : In std_logic_vector (1 DownTo 0);
            Q4 : In std_logic_vector (1 DownTo 0);
            Q5 : In std_logic_vector (1 DownTo 0);
            Q6 : In std_logic_vector (1 DownTo 0);
            Q7 : In std_logic_vector (1 DownTo 0);
            SEL0_3 : In std_logic_vector (1 DownTo 0);
            SEL4_7 : In std_logic_vector (1 DownTo 0);
            SEL4_7NOT0_3 : In std_logic;
            IOUT_L1 : Out std_logic_vector (1 DownTo 0);
            IOUT_L2 : Out std_logic_vector (1 DownTo 0);
            QOUT_L1 : Out std_logic_vector (1 DownTo 0);
            QOUT_L2 : Out std_logic_vector (1 DownTo 0)
        );
    END COMPONENT;

    SIGNAL CLOCK : std_logic := '0';
    SIGNAL I0 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL I1 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL I2 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL I3 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL I4 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL I5 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL I6 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL I7 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL INVQ_L1 : std_logic := '0';
    SIGNAL INVQ_L2 : std_logic := '0';
    SIGNAL INVI_L2 : std_logic := '0';
    SIGNAL Q0 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL Q1 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL Q2 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL Q3 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL Q4 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL Q5 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL Q6 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL Q7 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL SEL0_3 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL SEL4_7 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL SEL4_7NOT0_3 : std_logic := '0';
    SIGNAL IOUT_L1 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL IOUT_L2 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL QOUT_L1 : std_logic_vector (1 DownTo 0) := "00";
    SIGNAL QOUT_L2 : std_logic_vector (1 DownTo 0) := "00";

    constant PERIOD : time := 10 ns;
    constant DUTY_CYCLE : real := 0.5;
    constant OFFSET : time := 0 ns;

    BEGIN
        UUT : INPUT_SELECTOR
        PORT MAP (
            CLOCK => CLOCK,
            I0 => I0,
            I1 => I1,
            I2 => I2,
            I3 => I3,
            I4 => I4,
            I5 => I5,
            I6 => I6,
            I7 => I7,
            INVQ_L1 => INVQ_L1,
            INVQ_L2 => INVQ_L2,
            INVI_L2 => INVI_L2,
            Q0 => Q0,
            Q1 => Q1,
            Q2 => Q2,
            Q3 => Q3,
            Q4 => Q4,
            Q5 => Q5,
            Q6 => Q6,
            Q7 => Q7,
            SEL0_3 => SEL0_3,
            SEL4_7 => SEL4_7,
            SEL4_7NOT0_3 => SEL4_7NOT0_3,
            IOUT_L1 => IOUT_L1,
            IOUT_L2 => IOUT_L2,
            QOUT_L1 => QOUT_L1,
            QOUT_L2 => QOUT_L2
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
                -- -------------  Current Time:  4ns
                WAIT FOR 4 ns;
                SEL0_3 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  14ns
                WAIT FOR 10 ns;
                SEL0_3 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  24ns
                WAIT FOR 10 ns;
                SEL0_3 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  34ns
                WAIT FOR 10 ns;
                SEL0_3 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  1754ns
                WAIT FOR 1720 ns;
                INVI_L2 <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  2554ns
                WAIT FOR 800 ns;
                INVI_L2 <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  3054ns
                WAIT FOR 500 ns;
                INVQ_L1 <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  4124ns
                WAIT FOR 1070 ns;
                INVQ_L2 <= '1';
                -- -------------------------------------

            END PROCESS;

    END testbench_arch;

