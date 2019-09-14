VERSION 6
BEGIN SCHEMATIC
    BEGIN ATTR DeviceFamilyName "xa9500xl"
        DELETE all:0
        EDITNAME all:0
        EDITTRAIT all:0
    END ATTR
    BEGIN NETLIST
        SIGNAL clock
        SIGNAL Inv_Q_L1
        SIGNAL Iout_L1(1:0)
        PORT Input clock
        PORT Input Inv_Q_L1
        PORT Output Iout_L1(1:0)
        BEGIN BLOCKDEF INPUT_SELECTOR
            TIMESTAMP 2007 3 31 21 36 24
            RECTANGLE N 64 -1472 400 0 
            LINE N 64 -1440 0 -1440 
            LINE N 64 -1376 0 -1376 
            LINE N 64 -1312 0 -1312 
            LINE N 64 -1248 0 -1248 
            LINE N 64 -1184 0 -1184 
            RECTANGLE N 0 -1132 64 -1108 
            LINE N 64 -1120 0 -1120 
            RECTANGLE N 0 -1068 64 -1044 
            LINE N 64 -1056 0 -1056 
            RECTANGLE N 0 -1004 64 -980 
            LINE N 64 -992 0 -992 
            RECTANGLE N 0 -940 64 -916 
            LINE N 64 -928 0 -928 
            RECTANGLE N 0 -876 64 -852 
            LINE N 64 -864 0 -864 
            RECTANGLE N 0 -812 64 -788 
            LINE N 64 -800 0 -800 
            RECTANGLE N 0 -748 64 -724 
            LINE N 64 -736 0 -736 
            RECTANGLE N 0 -684 64 -660 
            LINE N 64 -672 0 -672 
            RECTANGLE N 0 -620 64 -596 
            LINE N 64 -608 0 -608 
            RECTANGLE N 0 -556 64 -532 
            LINE N 64 -544 0 -544 
            RECTANGLE N 0 -492 64 -468 
            LINE N 64 -480 0 -480 
            RECTANGLE N 0 -428 64 -404 
            LINE N 64 -416 0 -416 
            RECTANGLE N 0 -364 64 -340 
            LINE N 64 -352 0 -352 
            RECTANGLE N 0 -300 64 -276 
            LINE N 64 -288 0 -288 
            RECTANGLE N 0 -236 64 -212 
            LINE N 64 -224 0 -224 
            RECTANGLE N 0 -172 64 -148 
            LINE N 64 -160 0 -160 
            RECTANGLE N 0 -108 64 -84 
            LINE N 64 -96 0 -96 
            RECTANGLE N 0 -44 64 -20 
            LINE N 64 -32 0 -32 
            RECTANGLE N 400 -1452 464 -1428 
            LINE N 400 -1440 464 -1440 
            RECTANGLE N 400 -988 464 -964 
            LINE N 400 -976 464 -976 
            RECTANGLE N 400 -524 464 -500 
            LINE N 400 -512 464 -512 
            RECTANGLE N 400 -60 464 -36 
            LINE N 400 -48 464 -48 
        END BLOCKDEF
        BEGIN BLOCK XLXI_1 INPUT_SELECTOR
            PIN CLOCK clock
            PIN INVQ_L1 Inv_Q_L1
            PIN INVQ_L2
            PIN INVI_L2
            PIN SEL4_7NOT0_3
            PIN I0(1:0)
            PIN I1(1:0)
            PIN I2(1:0)
            PIN I3(1:0)
            PIN I4(1:0)
            PIN I5(1:0)
            PIN I6(1:0)
            PIN I7(1:0)
            PIN Q0(1:0)
            PIN Q1(1:0)
            PIN Q2(1:0)
            PIN Q3(1:0)
            PIN Q4(1:0)
            PIN Q5(1:0)
            PIN Q6(1:0)
            PIN Q7(1:0)
            PIN SEL0_3(1:0)
            PIN SEL4_7(1:0)
            PIN IOUT_L1(1:0) Iout_L1(1:0)
            PIN IOUT_L2(1:0)
            PIN QOUT_L1(1:0)
            PIN QOUT_L2(1:0)
        END BLOCK
    END NETLIST
    BEGIN SHEET 1 5382 3801
        ATTR LengthUnitName "CM"
        ATTR GridsPerUnit "4"
        BEGIN INSTANCE XLXI_1 384 2560 R0
        END INSTANCE
        BEGIN BRANCH clock
            WIRE 256 1120 384 1120
        END BRANCH
        BEGIN BRANCH Inv_Q_L1
            WIRE 256 1184 384 1184
        END BRANCH
        BEGIN BRANCH Iout_L1(1:0)
            WIRE 848 1120 960 1120
        END BRANCH
        IOMARKER 256 1120 clock R180 28
        IOMARKER 256 1184 Inv_Q_L1 R180 28
        IOMARKER 960 1120 Iout_L1(1:0) R0 28
    END SHEET
END SCHEMATIC
