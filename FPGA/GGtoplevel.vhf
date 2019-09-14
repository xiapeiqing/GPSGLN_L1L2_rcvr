--------------------------------------------------------------------------------
-- Copyright (c) 1995-2007 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 9.1i
--  \   \         Application : sch2vhdl
--  /   /         Filename : GGtoplevel.vhf
-- /___/   /\     Timestamp : 05/10/2007 10:32:30
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: C:\Xilinx91i\bin\nt\sch2vhdl.exe -intstyle ise -family xa9500xl -flat -suppress -w C:/peiqing/XilinxProjectPQ/GG/GGtoplevel.sch GGtoplevel.vhf
--Design Name: GGtoplevel
--Device: xa9500xl
--Purpose:
--    This vhdl netlist is translated from an ECS schematic. It can be 
--    synthesis and simulted, but it should not be modified. 
--

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity GGtoplevel is
   port ( clock    : in    std_logic; 
          Inv_Q_L1 : in    std_logic; 
          Iout_L1  : out   std_logic_vector (1 downto 0));
end GGtoplevel;

architecture BEHAVIORAL of GGtoplevel is
   signal XLXI_1_INVI_L2_openSignal      : std_logic;
   signal XLXI_1_INVQ_L2_openSignal      : std_logic;
   signal XLXI_1_I0_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_I1_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_I2_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_I3_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_I4_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_I5_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_I6_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_I7_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_Q0_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_Q1_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_Q2_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_Q3_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_Q4_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_Q5_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_Q6_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_Q7_openSignal           : std_logic_vector (1 downto 0);
   signal XLXI_1_SEL0_3_openSignal       : std_logic_vector (1 downto 0);
   signal XLXI_1_SEL4_7_openSignal       : std_logic_vector (1 downto 0);
   signal XLXI_1_SEL4_7NOT0_3_openSignal : std_logic;
   component INPUT_SELECTOR
      port ( CLOCK        : in    std_logic; 
             INVQ_L1      : in    std_logic; 
             INVQ_L2      : in    std_logic; 
             INVI_L2      : in    std_logic; 
             SEL4_7NOT0_3 : in    std_logic; 
             I0           : in    std_logic_vector (1 downto 0); 
             I1           : in    std_logic_vector (1 downto 0); 
             I2           : in    std_logic_vector (1 downto 0); 
             I3           : in    std_logic_vector (1 downto 0); 
             I4           : in    std_logic_vector (1 downto 0); 
             I5           : in    std_logic_vector (1 downto 0); 
             I6           : in    std_logic_vector (1 downto 0); 
             I7           : in    std_logic_vector (1 downto 0); 
             Q0           : in    std_logic_vector (1 downto 0); 
             Q1           : in    std_logic_vector (1 downto 0); 
             Q2           : in    std_logic_vector (1 downto 0); 
             Q3           : in    std_logic_vector (1 downto 0); 
             Q4           : in    std_logic_vector (1 downto 0); 
             Q5           : in    std_logic_vector (1 downto 0); 
             Q6           : in    std_logic_vector (1 downto 0); 
             Q7           : in    std_logic_vector (1 downto 0); 
             SEL0_3       : in    std_logic_vector (1 downto 0); 
             SEL4_7       : in    std_logic_vector (1 downto 0); 
             IOUT_L1      : out   std_logic_vector (1 downto 0); 
             IOUT_L2      : out   std_logic_vector (1 downto 0); 
             QOUT_L1      : out   std_logic_vector (1 downto 0); 
             QOUT_L2      : out   std_logic_vector (1 downto 0));
   end component;
   
begin
   XLXI_1 : INPUT_SELECTOR
      port map (CLOCK=>clock,
                INVI_L2=>XLXI_1_INVI_L2_openSignal,
                INVQ_L1=>Inv_Q_L1,
                INVQ_L2=>XLXI_1_INVQ_L2_openSignal,
                I0(1 downto 0)=>XLXI_1_I0_openSignal(1 downto 0),
                I1(1 downto 0)=>XLXI_1_I1_openSignal(1 downto 0),
                I2(1 downto 0)=>XLXI_1_I2_openSignal(1 downto 0),
                I3(1 downto 0)=>XLXI_1_I3_openSignal(1 downto 0),
                I4(1 downto 0)=>XLXI_1_I4_openSignal(1 downto 0),
                I5(1 downto 0)=>XLXI_1_I5_openSignal(1 downto 0),
                I6(1 downto 0)=>XLXI_1_I6_openSignal(1 downto 0),
                I7(1 downto 0)=>XLXI_1_I7_openSignal(1 downto 0),
                Q0(1 downto 0)=>XLXI_1_Q0_openSignal(1 downto 0),
                Q1(1 downto 0)=>XLXI_1_Q1_openSignal(1 downto 0),
                Q2(1 downto 0)=>XLXI_1_Q2_openSignal(1 downto 0),
                Q3(1 downto 0)=>XLXI_1_Q3_openSignal(1 downto 0),
                Q4(1 downto 0)=>XLXI_1_Q4_openSignal(1 downto 0),
                Q5(1 downto 0)=>XLXI_1_Q5_openSignal(1 downto 0),
                Q6(1 downto 0)=>XLXI_1_Q6_openSignal(1 downto 0),
                Q7(1 downto 0)=>XLXI_1_Q7_openSignal(1 downto 0),
                SEL0_3(1 downto 0)=>XLXI_1_SEL0_3_openSignal(1 downto 0),
                SEL4_7(1 downto 0)=>XLXI_1_SEL4_7_openSignal(1 downto 0),
                SEL4_7NOT0_3=>XLXI_1_SEL4_7NOT0_3_openSignal,
                IOUT_L1(1 downto 0)=>Iout_L1(1 downto 0),
                IOUT_L2=>open,
                QOUT_L1=>open,
                QOUT_L2=>open);
   
end BEHAVIORAL;


