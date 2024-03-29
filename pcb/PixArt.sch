EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:pixart
LIBS:PixArt-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "PixArt Adapter Board"
Date ""
Rev ""
Comp "Bart Trzynadlowski"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L PAJ7025 U1
U 1 1 595DBEC3
P 3600 3150
F 0 "U1" H 4050 2450 60  0000 C CNN
F 1 "PAJ7025" H 3300 3700 60  0000 C CNN
F 2 "PixArt:PAJ7025" H 3450 3350 60  0001 C CNN
F 3 "" H 3450 3350 60  0001 C CNN
	1    3600 3150
	1    0    0    -1  
$EndComp
NoConn ~ 4350 2850
NoConn ~ 4350 2950
NoConn ~ 2900 3450
NoConn ~ 2900 3550
$Comp
L C C2
U 1 1 595DC255
P 5400 3450
F 0 "C2" H 5425 3550 50  0000 L CNN
F 1 "0.1uF" H 5425 3350 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D3.8mm_W2.6mm_P2.50mm" H 5438 3300 50  0001 C CNN
F 3 "" H 5400 3450 50  0001 C CNN
	1    5400 3450
	1    0    0    -1  
$EndComp
$Comp
L CP C1
U 1 1 595DC2E4
P 4850 3450
F 0 "C1" H 4875 3550 50  0000 L CNN
F 1 "10uF" H 4875 3350 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P2.50mm" H 4888 3300 50  0001 C CNN
F 3 "" H 4850 3450 50  0001 C CNN
	1    4850 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 3050 5150 3050
Wire Wire Line
	5150 2750 5150 3200
Wire Wire Line
	4850 3200 5400 3200
Wire Wire Line
	4850 3200 4850 3300
Wire Wire Line
	5400 3200 5400 3300
Connection ~ 5150 3200
Wire Wire Line
	4850 3600 4850 3700
Wire Wire Line
	4850 3700 5400 3700
Wire Wire Line
	5400 3700 5400 3600
$Comp
L GND #PWR01
U 1 1 595DC41A
P 5150 3850
F 0 "#PWR01" H 5150 3600 50  0001 C CNN
F 1 "GND" H 5150 3700 50  0000 C CNN
F 2 "" H 5150 3850 50  0001 C CNN
F 3 "" H 5150 3850 50  0001 C CNN
	1    5150 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 3350 4650 3350
Wire Wire Line
	4650 2750 4650 3800
Wire Wire Line
	4650 3800 5150 3800
Wire Wire Line
	5150 3700 5150 3850
Connection ~ 5150 3700
Connection ~ 5150 3800
Wire Wire Line
	4350 2750 4650 2750
Connection ~ 4650 3350
$Comp
L VCC #PWR02
U 1 1 595F07C7
P 5150 2750
F 0 "#PWR02" H 5150 2600 50  0001 C CNN
F 1 "VCC" H 5150 2900 50  0000 C CNN
F 2 "" H 5150 2750 50  0001 C CNN
F 3 "" H 5150 2750 50  0001 C CNN
	1    5150 2750
	1    0    0    -1  
$EndComp
Connection ~ 5150 3050
NoConn ~ 2900 3350
NoConn ~ 2900 3250
NoConn ~ 2900 3150
NoConn ~ 2900 3050
NoConn ~ 2900 2750
NoConn ~ 2900 2850
NoConn ~ 2900 2950
NoConn ~ 4350 3150
NoConn ~ 4350 3250
$Comp
L CONN_01X06 J1
U 1 1 595F0D95
P 5150 4700
F 0 "J1" H 5150 5050 50  0000 C CNN
F 1 "PIN HEADER" V 5250 4700 50  0000 C CNN
F 2 "Molex:Molex_KK-6410-06_06x2.54mm_Straight" H 5150 4700 50  0001 C CNN
F 3 "" H 5150 4700 50  0001 C CNN
	1    5150 4700
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR03
U 1 1 595F0E38
P 4850 4350
F 0 "#PWR03" H 4850 4200 50  0001 C CNN
F 1 "VCC" H 4850 4500 50  0000 C CNN
F 2 "" H 4850 4350 50  0001 C CNN
F 3 "" H 4850 4350 50  0001 C CNN
	1    4850 4350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 595F0E4F
P 4850 5050
F 0 "#PWR04" H 4850 4800 50  0001 C CNN
F 1 "GND" H 4850 4900 50  0000 C CNN
F 2 "" H 4850 5050 50  0001 C CNN
F 3 "" H 4850 5050 50  0001 C CNN
	1    4850 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 3650 2750 3650
Wire Wire Line
	2750 3650 2750 4550
Wire Wire Line
	2750 4550 4950 4550
Wire Wire Line
	4350 3650 4450 3650
Wire Wire Line
	4450 3650 4450 4650
Wire Wire Line
	4450 4650 4950 4650
Wire Wire Line
	4350 3550 4500 3550
Wire Wire Line
	4500 3550 4500 4750
Wire Wire Line
	4500 4750 4950 4750
Wire Wire Line
	4350 3450 4550 3450
Wire Wire Line
	4550 3450 4550 4850
Wire Wire Line
	4550 4850 4950 4850
Wire Wire Line
	4850 5050 4850 4950
Wire Wire Line
	4850 4950 4950 4950
Wire Wire Line
	4850 4350 4850 4450
Wire Wire Line
	4850 4450 4950 4450
$Comp
L VCC #PWR05
U 1 1 595F100E
P 7050 2750
F 0 "#PWR05" H 7050 2600 50  0001 C CNN
F 1 "VCC" H 7050 2900 50  0000 C CNN
F 2 "" H 7050 2750 50  0001 C CNN
F 3 "" H 7050 2750 50  0001 C CNN
	1    7050 2750
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR06
U 1 1 595F1025
P 7550 2750
F 0 "#PWR06" H 7550 2500 50  0001 C CNN
F 1 "GND" H 7550 2600 50  0000 C CNN
F 2 "" H 7550 2750 50  0001 C CNN
F 3 "" H 7550 2750 50  0001 C CNN
	1    7550 2750
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG07
U 1 1 595F1056
P 7050 2650
F 0 "#FLG07" H 7050 2725 50  0001 C CNN
F 1 "PWR_FLAG" H 7050 2800 50  0000 C CNN
F 2 "" H 7050 2650 50  0001 C CNN
F 3 "" H 7050 2650 50  0001 C CNN
	1    7050 2650
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG08
U 1 1 595F1076
P 7550 2650
F 0 "#FLG08" H 7550 2725 50  0001 C CNN
F 1 "PWR_FLAG" H 7550 2800 50  0000 C CNN
F 2 "" H 7550 2650 50  0001 C CNN
F 3 "" H 7550 2650 50  0001 C CNN
	1    7550 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2650 7050 2750
Wire Wire Line
	7550 2650 7550 2750
$EndSCHEMATC
