--------------------------------------------------------------------------------
-- Copyright (c) 1995-2003 Xilinx, Inc.
-- All Right Reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 9.1i
--  \   \         Application : ISE
--  /   /         Filename : input_selector_tbw_selfcheck.vhw
-- /___/   /\     Timestamp : Sat Apr 07 15:28:11 2007
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: 
--Design Name: input_selector_tbw_selfcheck
--Device: Xilinx
--

library IEEE;
use IEEE.std_logic_1164.all;
USE IEEE.STD_LOGIC_TEXTIO.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
USE STD.TEXTIO.ALL;

ENTITY input_selector_tbw_selfcheck IS
END input_selector_tbw_selfcheck;

ARCHITECTURE testbench_arch OF input_selector_tbw_selfcheck IS
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
    SIGNAL I0 : std_logic_vector (1 DownTo 0) := "01";
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
    SIGNAL IOUT_L1 : std_logic_vector (1 DownTo 0) := "UU";
    SIGNAL IOUT_L2 : std_logic_vector (1 DownTo 0) := "UU";
    SIGNAL QOUT_L1 : std_logic_vector (1 DownTo 0) := "UU";
    SIGNAL QOUT_L2 : std_logic_vector (1 DownTo 0) := "UU";

    SHARED VARIABLE TX_ERROR : INTEGER := 0;
    SHARED VARIABLE TX_OUT : LINE;

    constant PERIOD : time := 200 ns;
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
            PROCEDURE CHECK_IOUT_L1(
                next_IOUT_L1 : std_logic_vector (1 DownTo 0);
                TX_TIME : INTEGER
            ) IS
                VARIABLE TX_STR : String(1 to 4096);
                VARIABLE TX_LOC : LINE;
                BEGIN
                IF (IOUT_L1 /= next_IOUT_L1) THEN
                    STD.TEXTIO.write(TX_LOC, string'("Error at time="));
                    STD.TEXTIO.write(TX_LOC, TX_TIME);
                    STD.TEXTIO.write(TX_LOC, string'("ns IOUT_L1="));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, IOUT_L1);
                    STD.TEXTIO.write(TX_LOC, string'(", Expected = "));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, next_IOUT_L1);
                    STD.TEXTIO.write(TX_LOC, string'(" "));
                    TX_STR(TX_LOC.all'range) := TX_LOC.all;
                    STD.TEXTIO.Deallocate(TX_LOC);
                    ASSERT (FALSE) REPORT TX_STR SEVERITY ERROR;
                    TX_ERROR := TX_ERROR + 1;
                END IF;
            END;
            PROCEDURE CHECK_IOUT_L2(
                next_IOUT_L2 : std_logic_vector (1 DownTo 0);
                TX_TIME : INTEGER
            ) IS
                VARIABLE TX_STR : String(1 to 4096);
                VARIABLE TX_LOC : LINE;
                BEGIN
                IF (IOUT_L2 /= next_IOUT_L2) THEN
                    STD.TEXTIO.write(TX_LOC, string'("Error at time="));
                    STD.TEXTIO.write(TX_LOC, TX_TIME);
                    STD.TEXTIO.write(TX_LOC, string'("ns IOUT_L2="));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, IOUT_L2);
                    STD.TEXTIO.write(TX_LOC, string'(", Expected = "));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, next_IOUT_L2);
                    STD.TEXTIO.write(TX_LOC, string'(" "));
                    TX_STR(TX_LOC.all'range) := TX_LOC.all;
                    STD.TEXTIO.Deallocate(TX_LOC);
                    ASSERT (FALSE) REPORT TX_STR SEVERITY ERROR;
                    TX_ERROR := TX_ERROR + 1;
                END IF;
            END;
            PROCEDURE CHECK_QOUT_L1(
                next_QOUT_L1 : std_logic_vector (1 DownTo 0);
                TX_TIME : INTEGER
            ) IS
                VARIABLE TX_STR : String(1 to 4096);
                VARIABLE TX_LOC : LINE;
                BEGIN
                IF (QOUT_L1 /= next_QOUT_L1) THEN
                    STD.TEXTIO.write(TX_LOC, string'("Error at time="));
                    STD.TEXTIO.write(TX_LOC, TX_TIME);
                    STD.TEXTIO.write(TX_LOC, string'("ns QOUT_L1="));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, QOUT_L1);
                    STD.TEXTIO.write(TX_LOC, string'(", Expected = "));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, next_QOUT_L1);
                    STD.TEXTIO.write(TX_LOC, string'(" "));
                    TX_STR(TX_LOC.all'range) := TX_LOC.all;
                    STD.TEXTIO.Deallocate(TX_LOC);
                    ASSERT (FALSE) REPORT TX_STR SEVERITY ERROR;
                    TX_ERROR := TX_ERROR + 1;
                END IF;
            END;
            PROCEDURE CHECK_QOUT_L2(
                next_QOUT_L2 : std_logic_vector (1 DownTo 0);
                TX_TIME : INTEGER
            ) IS
                VARIABLE TX_STR : String(1 to 4096);
                VARIABLE TX_LOC : LINE;
                BEGIN
                IF (QOUT_L2 /= next_QOUT_L2) THEN
                    STD.TEXTIO.write(TX_LOC, string'("Error at time="));
                    STD.TEXTIO.write(TX_LOC, TX_TIME);
                    STD.TEXTIO.write(TX_LOC, string'("ns QOUT_L2="));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, QOUT_L2);
                    STD.TEXTIO.write(TX_LOC, string'(", Expected = "));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, next_QOUT_L2);
                    STD.TEXTIO.write(TX_LOC, string'(" "));
                    TX_STR(TX_LOC.all'range) := TX_LOC.all;
                    STD.TEXTIO.Deallocate(TX_LOC);
                    ASSERT (FALSE) REPORT TX_STR SEVERITY ERROR;
                    TX_ERROR := TX_ERROR + 1;
                END IF;
            END;
            BEGIN
                -- -------------  Current Time:  85ns
                WAIT FOR 85 ns;
                I0 <= "11";
                I1 <= "01";
                I2 <= "01";
                I3 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  115ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("11", 115);
                CHECK_IOUT_L2("11", 115);
                CHECK_QOUT_L1("00", 115);
                CHECK_QOUT_L2("00", 115);
                -- -------------------------------------
                -- -------------  Current Time:  285ns
                WAIT FOR 170 ns;
                I0 <= "10";
                I1 <= "11";
                I2 <= "11";
                I3 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  315ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("10", 315);
                CHECK_IOUT_L2("10", 315);
                -- -------------------------------------
                -- -------------  Current Time:  485ns
                WAIT FOR 170 ns;
                I0 <= "00";
                I1 <= "10";
                I2 <= "10";
                I3 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  515ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("00", 515);
                CHECK_IOUT_L2("00", 515);
                -- -------------------------------------
                -- -------------  Current Time:  685ns
                WAIT FOR 170 ns;
                I0 <= "11";
                I1 <= "00";
                I2 <= "00";
                I3 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  715ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("11", 715);
                CHECK_IOUT_L2("11", 715);
                -- -------------------------------------
                -- -------------  Current Time:  885ns
                WAIT FOR 170 ns;
                I0 <= "01";
                I1 <= "11";
                I2 <= "11";
                I3 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  915ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("01", 915);
                CHECK_IOUT_L2("01", 915);
                -- -------------------------------------
                -- -------------  Current Time:  1085ns
                WAIT FOR 170 ns;
                I1 <= "01";
                I2 <= "01";
                I3 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  1285ns
                WAIT FOR 200 ns;
                I0 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  1315ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("10", 1315);
                CHECK_IOUT_L2("10", 1315);
                -- -------------------------------------
                -- -------------  Current Time:  1485ns
                WAIT FOR 170 ns;
                I0 <= "00";
                I1 <= "10";
                I2 <= "10";
                I3 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  1515ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("00", 1515);
                CHECK_IOUT_L2("00", 1515);
                -- -------------------------------------
                -- -------------  Current Time:  1685ns
                WAIT FOR 170 ns;
                I0 <= "01";
                I1 <= "00";
                I2 <= "00";
                I3 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  1715ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("01", 1715);
                CHECK_IOUT_L2("01", 1715);
                -- -------------------------------------
                -- -------------  Current Time:  1885ns
                WAIT FOR 170 ns;
                I0 <= "11";
                I1 <= "01";
                I2 <= "01";
                I3 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  1915ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("11", 1915);
                CHECK_IOUT_L2("11", 1915);
                -- -------------------------------------
                -- -------------  Current Time:  2085ns
                WAIT FOR 170 ns;
                I0 <= "00";
                I1 <= "11";
                I2 <= "11";
                I3 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  2115ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("00", 2115);
                CHECK_IOUT_L2("00", 2115);
                -- -------------------------------------
                -- -------------  Current Time:  2285ns
                WAIT FOR 170 ns;
                I0 <= "10";
                I1 <= "00";
                I2 <= "00";
                I3 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  2315ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("10", 2315);
                CHECK_IOUT_L2("10", 2315);
                -- -------------------------------------
                -- -------------  Current Time:  2485ns
                WAIT FOR 170 ns;
                I1 <= "10";
                I2 <= "10";
                I3 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  2685ns
                WAIT FOR 200 ns;
                I0 <= "01";
                I4 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  2715ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("01", 2715);
                CHECK_IOUT_L2("01", 2715);
                -- -------------------------------------
                -- -------------  Current Time:  2885ns
                WAIT FOR 170 ns;
                I0 <= "11";
                I1 <= "01";
                I2 <= "01";
                I3 <= "01";
                I4 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  2915ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("11", 2915);
                CHECK_IOUT_L2("11", 2915);
                -- -------------------------------------
                -- -------------  Current Time:  3085ns
                WAIT FOR 170 ns;
                I0 <= "00";
                I1 <= "11";
                I2 <= "11";
                I3 <= "11";
                I4 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  3115ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("00", 3115);
                CHECK_IOUT_L2("00", 3115);
                -- -------------------------------------
                -- -------------  Current Time:  3285ns
                WAIT FOR 170 ns;
                I1 <= "00";
                I2 <= "00";
                I3 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  3485ns
                WAIT FOR 200 ns;
                I0 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  3515ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("10", 3515);
                CHECK_IOUT_L2("10", 3515);
                -- -------------------------------------
                -- -------------  Current Time:  3685ns
                WAIT FOR 170 ns;
                I0 <= "01";
                I1 <= "10";
                I2 <= "10";
                I3 <= "10";
                I5 <= "01";
                -- -------------------------------------
                -- -------------  Current Time:  3715ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("01", 3715);
                CHECK_IOUT_L2("01", 3715);
                -- -------------------------------------
                -- -------------  Current Time:  3885ns
                WAIT FOR 170 ns;
                I0 <= "11";
                I1 <= "01";
                I2 <= "01";
                I3 <= "01";
                I5 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  3915ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("11", 3915);
                CHECK_IOUT_L2("11", 3915);
                -- -------------------------------------
                -- -------------  Current Time:  4085ns
                WAIT FOR 170 ns;
                I0 <= "10";
                I1 <= "11";
                I2 <= "11";
                I3 <= "11";
                I5 <= "11";
                -- -------------------------------------
                -- -------------  Current Time:  4115ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("10", 4115);
                CHECK_IOUT_L2("10", 4115);
                -- -------------------------------------
                -- -------------  Current Time:  4285ns
                WAIT FOR 170 ns;
                I0 <= "00";
                I1 <= "10";
                I2 <= "10";
                I3 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  4315ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("00", 4315);
                CHECK_IOUT_L2("00", 4315);
                -- -------------------------------------
                -- -------------  Current Time:  4485ns
                WAIT FOR 170 ns;
                I0 <= "10";
                I1 <= "00";
                I2 <= "00";
                I3 <= "00";
                -- -------------------------------------
                -- -------------  Current Time:  4515ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("10", 4515);
                CHECK_IOUT_L2("10", 4515);
                -- -------------------------------------
                -- -------------  Current Time:  4685ns
                WAIT FOR 170 ns;
                I0 <= "01";
                I1 <= "10";
                I2 <= "10";
                I3 <= "10";
                -- -------------------------------------
                -- -------------  Current Time:  4715ns
                WAIT FOR 30 ns;
                CHECK_IOUT_L1("01", 4715);
                CHECK_IOUT_L2("01", 4715);
                -- -------------------------------------
                -- -------------  Current Time:  4885ns
                WAIT FOR 170 ns;
                I1 <= "01";
                I2 <= "01";
                I3 <= "01";
                -- -------------------------------------
                WAIT FOR 1315 ns;

                IF (TX_ERROR = 0) THEN
                    STD.TEXTIO.write(TX_OUT, string'("No errors or warnings"));
                    ASSERT (FALSE) REPORT
                      "Simulation successful (not a failure).  No problems detected."
                      SEVERITY FAILURE;
                ELSE
                    STD.TEXTIO.write(TX_OUT, TX_ERROR);
                    STD.TEXTIO.write(TX_OUT,
                        string'(" errors found in simulation"));
                    ASSERT (FALSE) REPORT "Errors found during simulation"
                         SEVERITY FAILURE;
                END IF;
            END PROCESS;

    END testbench_arch;

