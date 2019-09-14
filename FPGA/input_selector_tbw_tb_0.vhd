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
-- /___/   /\     Timestamp : Sat Apr 07 18:35:23 2007
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: 
--Design Name: input_selector_tbw_tb_0
--Device: Xilinx
--

library IEEE;
use IEEE.std_logic_1164.all;
USE IEEE.STD_LOGIC_TEXTIO.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
USE STD.TEXTIO.ALL;

ENTITY input_selector_tbw_tb_0 IS
END input_selector_tbw_tb_0;

ARCHITECTURE testbench_arch OF input_selector_tbw_tb_0 IS
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

    constant PERIOD : time := 100 ns;
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
                -- -------------  Current Time:  40ns
                WAIT FOR 40 ns;
                I0 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  840ns
                WAIT FOR 800 ns;
                I0 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  940ns
                WAIT FOR 100 ns;
                I0 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  1040ns
                WAIT FOR 100 ns;
                I0 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  1140ns
                WAIT FOR 100 ns;
                I0 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  1240ns
                WAIT FOR 100 ns;
                I0 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  1340ns
                WAIT FOR 100 ns;
                I0 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  1540ns
                WAIT FOR 200 ns;
                I0 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  1640ns
                WAIT FOR 100 ns;
                I0 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  1740ns
                WAIT FOR 100 ns;
                I0 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  1940ns
                WAIT FOR 200 ns;
                I0 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  2040ns
                WAIT FOR 100 ns;
                I0 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  2140ns
                WAIT FOR 100 ns;
                I0 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  2240ns
                WAIT FOR 100 ns;
                I0 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  2340ns
                WAIT FOR 100 ns;
                I0 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  2440ns
                WAIT FOR 100 ns;
                I0 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  2540ns
                WAIT FOR 100 ns;
                I0 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  2740ns
                WAIT FOR 200 ns;
                I0 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  2840ns
                WAIT FOR 100 ns;
                I0 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  3040ns
                WAIT FOR 200 ns;
                I0 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  3140ns
                WAIT FOR 100 ns;
                I0 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  3240ns
                WAIT FOR 100 ns;
                I0 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  3440ns
                WAIT FOR 200 ns;
                I0 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  3540ns
                WAIT FOR 100 ns;
                I0 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  3640ns
                WAIT FOR 100 ns;
                I0 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  3740ns
                WAIT FOR 100 ns;
                I0 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  3840ns
                WAIT FOR 100 ns;
                I0 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  3940ns
                WAIT FOR 100 ns;
                I0 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  4040ns
                WAIT FOR 100 ns;
                I0 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  4240ns
                WAIT FOR 200 ns;
                I0 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  4340ns
                WAIT FOR 100 ns;
                I0 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  4440ns
                WAIT FOR 100 ns;
                I0 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  4540ns
                WAIT FOR 100 ns;
                I0 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  4640ns
                WAIT FOR 100 ns;
                I0 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  4740ns
                WAIT FOR 100 ns;
                I0 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  4940ns
                WAIT FOR 200 ns;
                I0 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  5040ns
                WAIT FOR 100 ns;
                I0 <= "01";
                -- -------------------------------------
                WAIT FOR 1060 ns;

            END PROCESS;

    END testbench_arch;

