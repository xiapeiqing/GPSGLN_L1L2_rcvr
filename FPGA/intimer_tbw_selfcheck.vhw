--------------------------------------------------------------------------------
-- Copyright (c) 1995-2003 Xilinx, Inc.
-- All Right Reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 9.1i
--  \   \         Application : ISE
--  /   /         Filename : intimer_tbw_selfcheck.vhw
-- /___/   /\     Timestamp : Sun Apr 08 16:08:19 2007
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: 
--Design Name: intimer_tbw_selfcheck
--Device: Xilinx
--

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
USE IEEE.STD_LOGIC_TEXTIO.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE STD.TEXTIO.ALL;

ENTITY intimer_tbw_selfcheck IS
END intimer_tbw_selfcheck;

ARCHITECTURE testbench_arch OF intimer_tbw_selfcheck IS
    COMPONENT intimer
        PORT (
            CA_CLOCK : In std_logic;
            CA_EPOCH : In std_logic;
            CLOCK : In std_logic;
            DAV : In std_logic;
            DIN : In std_logic_vector (9 DownTo 0);
            INTEG : In std_logic;
            INT : Out std_logic;
            INTENDS : Out std_logic
        );
    END COMPONENT;

    SIGNAL CA_CLOCK : std_logic := '0';
    SIGNAL CA_EPOCH : std_logic := '0';
    SIGNAL CLOCK : std_logic := '0';
    SIGNAL DAV : std_logic := '0';
    SIGNAL DIN : std_logic_vector (9 DownTo 0) := "0000000000";
    SIGNAL INTEG : std_logic := '1';
    SIGNAL INT : std_logic := 'U';
    SIGNAL INTENDS : std_logic := 'U';

    SHARED VARIABLE TX_ERROR : INTEGER := 0;
    SHARED VARIABLE TX_OUT : LINE;

    constant PERIOD : time := 20 ns;
    constant DUTY_CYCLE : real := 0.5;
    constant OFFSET : time := 0 ns;

    BEGIN
        UUT : intimer
        PORT MAP (
            CA_CLOCK => CA_CLOCK,
            CA_EPOCH => CA_EPOCH,
            CLOCK => CLOCK,
            DAV => DAV,
            DIN => DIN,
            INTEG => INTEG,
            INT => INT,
            INTENDS => INTENDS
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
            PROCEDURE CHECK_INT(
                next_INT : std_logic;
                TX_TIME : INTEGER
            ) IS
                VARIABLE TX_STR : String(1 to 4096);
                VARIABLE TX_LOC : LINE;
                BEGIN
                IF (INT /= next_INT) THEN
                    STD.TEXTIO.write(TX_LOC, string'("Error at time="));
                    STD.TEXTIO.write(TX_LOC, TX_TIME);
                    STD.TEXTIO.write(TX_LOC, string'("ns INT="));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, INT);
                    STD.TEXTIO.write(TX_LOC, string'(", Expected = "));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, next_INT);
                    STD.TEXTIO.write(TX_LOC, string'(" "));
                    TX_STR(TX_LOC.all'range) := TX_LOC.all;
                    STD.TEXTIO.Deallocate(TX_LOC);
                    ASSERT (FALSE) REPORT TX_STR SEVERITY ERROR;
                    TX_ERROR := TX_ERROR + 1;
                END IF;
            END;
            PROCEDURE CHECK_INTENDS(
                next_INTENDS : std_logic;
                TX_TIME : INTEGER
            ) IS
                VARIABLE TX_STR : String(1 to 4096);
                VARIABLE TX_LOC : LINE;
                BEGIN
                IF (INTENDS /= next_INTENDS) THEN
                    STD.TEXTIO.write(TX_LOC, string'("Error at time="));
                    STD.TEXTIO.write(TX_LOC, TX_TIME);
                    STD.TEXTIO.write(TX_LOC, string'("ns INTENDS="));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, INTENDS);
                    STD.TEXTIO.write(TX_LOC, string'(", Expected = "));
                    IEEE.STD_LOGIC_TEXTIO.write(TX_LOC, next_INTENDS);
                    STD.TEXTIO.write(TX_LOC, string'(" "));
                    TX_STR(TX_LOC.all'range) := TX_LOC.all;
                    STD.TEXTIO.Deallocate(TX_LOC);
                    ASSERT (FALSE) REPORT TX_STR SEVERITY ERROR;
                    TX_ERROR := TX_ERROR + 1;
                END IF;
            END;
            BEGIN
                -- -------------  Current Time:  8ns
                WAIT FOR 8 ns;
                CA_CLOCK <= '1';
                CA_EPOCH <= '1';
                DAV <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  28ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                DAV <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  32ns
                WAIT FOR 4 ns;
                CHECK_INT('0', 32);
                CHECK_INTENDS('0', 32);
                -- -------------------------------------
                -- -------------  Current Time:  48ns
                WAIT FOR 16 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  68ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  88ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  108ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                CA_EPOCH <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  128ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  148ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  168ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  188ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  208ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                CA_EPOCH <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  228ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  248ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  268ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  288ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  308ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                CA_EPOCH <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  328ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  348ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  368ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  388ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  408ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                CA_EPOCH <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  428ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  448ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  468ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  488ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  508ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                CA_EPOCH <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  528ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  548ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  568ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  588ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  608ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                CA_EPOCH <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  628ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  648ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  668ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  688ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  708ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  728ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  748ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  768ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  788ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  808ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  828ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  848ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  868ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  888ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  908ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  928ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  948ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                -- -------------  Current Time:  968ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '1';
                -- -------------------------------------
                -- -------------  Current Time:  988ns
                WAIT FOR 20 ns;
                CA_CLOCK <= '0';
                -- -------------------------------------
                WAIT FOR 32 ns;

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
