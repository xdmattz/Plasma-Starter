EESchema Schematic File Version 5
EELAYER 30 0
EELAYER END
$Descr B 17000 11000
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
Comment5 ""
Comment6 ""
Comment7 ""
Comment8 ""
Comment9 ""
$EndDescr
$Comp
L Plasma_Pilot4-rescue:ATTINY861A-S-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue IC1
U 1 1 56A54715
P 2650 4850
F 0 "IC1" H 1750 5800 50  0000 C CNN
F 1 "ATTINY861A-S" H 3400 3900 50  0000 C CNN
F 2 "w_smd_dil:soic-20" H 2650 4850 50  0000 C CIN
F 3 "" H 2650 4850 50  0000 C CNN
	1    2650 4850
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:BSS138-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue Q2
U 1 1 56A547A8
P 7500 5200
F 0 "Q2" H 7300 5400 50  0000 L CNN
F 1 "BSS131" H 7150 5300 50  0000 L CNN
F 2 "t_sot:sot-23" H 7700 5125 50  0001 L CIN
F 3 "" H 7500 5200 50  0000 L CNN
	1    7500 5200
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:CP1-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue C1
U 1 1 56A54808
P 7100 4350
F 0 "C1" H 7250 4400 50  0000 L CNN
F 1 "10uF" H 7250 4300 50  0000 L CNN
F 2 "t_passive_smd:cap_1210" H 7100 4350 50  0001 C CNN
F 3 "" H 7100 4350 50  0000 C CNN
	1    7100 4350
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR01
U 1 1 56A54BA8
P 7100 4600
F 0 "#PWR01" H 7100 4600 30  0001 C CNN
F 1 "GND" H 7100 4530 30  0001 C CNN
F 2 "" H 7100 4600 60  0000 C CNN
F 3 "" H 7100 4600 60  0000 C CNN
	1    7100 4600
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:C-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue C7
U 1 1 56A565E5
P 10150 6700
F 0 "C7" H 10300 6750 50  0000 L CNN
F 1 "1000pF" H 10300 6650 50  0000 L CNN
F 2 "t_passive_smd:cap_0805" H 10188 6550 50  0001 C CNN
F 3 "" H 10150 6700 50  0000 C CNN
	1    10150 6700
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:Rx-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue R9
U 1 1 56A56F5F
P 11200 6700
F 0 "R9" H 11350 6750 50  0000 C CNN
F 1 "4.7K" H 11400 6650 50  0000 C CNN
F 2 "t_passive_smd:res_0805" H 11200 6700 60  0001 C CNN
F 3 "" H 11200 6700 60  0000 C CNN
	1    11200 6700
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue TP5
U 1 1 56A57847
P 13350 5150
F 0 "TP5" V 13300 5300 60  0000 C CNN
F 1 "TP" V 13400 5300 60  0000 C CNN
F 2 "t_misc:TP100" H 13350 5150 60  0001 C CNN
F 3 "" H 13350 5150 60  0000 C CNN
	1    13350 5150
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 4650 8800 4650
Wire Wire Line
	7100 4100 7600 4100
Wire Wire Line
	7100 4150 7100 4100
Wire Wire Line
	10150 7050 10150 6900
Wire Wire Line
	11200 7050 11200 6950
Wire Wire Line
	12550 7050 12550 6550
Connection ~ 11200 7050
Connection ~ 12550 7050
Wire Wire Line
	10150 6500 10150 6350
Wire Wire Line
	9600 4650 9100 4650
$Comp
L Plasma_Pilot4-rescue:C-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue C10
U 1 1 56A59B2E
P 13050 6350
F 0 "C10" H 13200 6400 50  0000 L CNN
F 1 "0.022uF" H 13200 6300 50  0000 L CNN
F 2 "t_passive_thd:Cap_Rad_13x6mm" H 13088 6200 50  0001 C CNN
F 3 "" H 13050 6350 50  0000 C CNN
	1    13050 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	13050 6550 13050 6750
Connection ~ 13050 7050
Wire Wire Line
	9100 4450 9500 4450
$Comp
L Plasma_Pilot4-rescue:Q_NIGBT_GEC-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue Q3
U 1 1 56A5C6FA
P 12450 6350
F 0 "Q3" H 12700 6250 50  0000 L CNN
F 1 "Q_NIGBT_GEC" H 12650 6150 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-247-3_Vertical" H 12650 6450 50  0001 C CNN
F 3 "" H 12450 6350 50  0000 C CNN
	1    12450 6350
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR03
U 1 1 56A5CFCB
P 7000 5700
F 0 "#PWR03" H 7000 5700 30  0001 C CNN
F 1 "GND" H 7000 5630 30  0001 C CNN
F 2 "" H 7000 5700 60  0000 C CNN
F 3 "" H 7000 5700 60  0000 C CNN
	1    7000 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 5650 7000 5700
Wire Wire Line
	7100 4550 7100 4600
Wire Notes Line
	8950 4100 11200 4100
Wire Notes Line
	8950 7250 13650 7250
Wire Notes Line
	11200 4100 11200 5000
Wire Notes Line
	13650 5000 13650 7250
$Comp
L Plasma_Pilot4-rescue:BSS138-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue Q1
U 1 1 56A61052
P 7200 9150
F 0 "Q1" H 7400 9225 50  0000 L CNN
F 1 "BSS138" H 7400 9150 50  0000 L CNN
F 2 "t_sot:sot-23" H 7400 9075 50  0001 L CIN
F 3 "" H 7200 9150 50  0000 L CNN
	1    7200 9150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 9200 6850 9200
Wire Wire Line
	6850 9200 6850 9300
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR07
U 1 1 56A61B55
P 6850 9700
F 0 "#PWR07" H 6850 9700 30  0001 C CNN
F 1 "GND" H 6850 9630 30  0001 C CNN
F 2 "" H 6850 9700 60  0000 C CNN
F 3 "" H 6850 9700 60  0000 C CNN
	1    6850 9700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 9600 6850 9700
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR08
U 1 1 56A61C7C
P 7300 9450
F 0 "#PWR08" H 7300 9450 30  0001 C CNN
F 1 "GND" H 7300 9380 30  0001 C CNN
F 2 "" H 7300 9450 60  0000 C CNN
F 3 "" H 7300 9450 60  0000 C CNN
	1    7300 9450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 9350 7300 9450
Connection ~ 6850 9200
$Comp
L Plasma_Pilot4-rescue:CONN_02X03-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue P1
U 1 1 56A65893
P 6750 1800
F 0 "P1" H 6750 2000 50  0000 C CNN
F 1 "CONN_02X03" H 6750 1600 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" H 6750 600 50  0001 C CNN
F 3 "" H 6750 600 50  0000 C CNN
	1    6750 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 1900 7200 1900
Wire Wire Line
	7200 1900 7200 2000
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR09
U 1 1 56A666F9
P 7200 2000
F 0 "#PWR09" H 7200 2000 30  0001 C CNN
F 1 "GND" H 7200 1930 30  0001 C CNN
F 2 "" H 7200 2000 60  0000 C CNN
F 3 "" H 7200 2000 60  0000 C CNN
	1    7200 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 1700 7000 1700
Wire Wire Line
	7200 1000 7200 1700
Wire Wire Line
	7450 1800 7000 1800
Wire Wire Line
	5000 5050 5000 1700
Wire Wire Line
	5000 1700 6500 1700
Wire Wire Line
	5900 1900 6500 1900
Wire Wire Line
	3750 5150 5100 5150
Wire Wire Line
	5100 1800 6500 1800
Connection ~ 5900 1900
Wire Wire Line
	5900 950  5900 1000
Wire Wire Line
	5900 1000 7200 1000
Connection ~ 5900 1000
$Comp
L power:VDD #PWR010
U 1 1 56A67E23
P 5900 950
F 0 "#PWR010" H 5900 1050 30  0001 C CNN
F 1 "VDD" H 5900 1100 30  0000 C CNN
F 2 "" H 5900 950 60  0000 C CNN
F 3 "" H 5900 950 60  0000 C CNN
	1    5900 950 
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR011
U 1 1 56A68293
P 7600 3150
F 0 "#PWR011" H 7600 3100 20  0001 C CNN
F 1 "+12V" H 7615 3307 30  0000 C CNN
F 2 "" H 7600 3150 60  0000 C CNN
F 3 "" H 7600 3150 60  0000 C CNN
	1    7600 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 3150 7600 3250
Wire Wire Line
	9600 4650 9600 7050
$Comp
L Plasma_Pilot4-rescue:CP1-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue C2
U 1 1 56A70A08
P 8050 3550
F 0 "C2" H 8200 3600 50  0000 L CNN
F 1 "10uF" H 8200 3500 50  0000 L CNN
F 2 "t_passive_smd:cap_1210" H 8050 3550 50  0001 C CNN
F 3 "" H 8050 3550 50  0000 C CNN
	1    8050 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5550 4350 5550
Wire Wire Line
	4350 5550 4350 6200
Wire Wire Line
	4350 6200 3800 6200
Wire Wire Line
	6000 5350 3750 5350
Wire Wire Line
	8050 3350 8050 3250
Wire Wire Line
	7600 3250 8050 3250
Connection ~ 7600 3250
Text Label 7100 1800 0    60   ~ 0
MOSI
Text Label 6100 1900 0    60   ~ 0
~XRESET
Text Label 6100 1800 0    60   ~ 0
SCK
Text Label 6100 1700 0    60   ~ 0
MISO
Wire Wire Line
	15600 2850 15600 4000
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR015
U 1 1 56A7BDBC
P 8050 3850
F 0 "#PWR015" H 8050 3850 30  0001 C CNN
F 1 "GND" H 8050 3780 30  0001 C CNN
F 2 "" H 8050 3850 60  0000 C CNN
F 3 "" H 8050 3850 60  0000 C CNN
	1    8050 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 3750 8050 3850
$Comp
L lt1129cst-5:LT1129CST-5 U1
U 1 1 56A8192D
P 9050 3300
F 0 "U1" H 8800 3500 40  0000 C CNN
F 1 "LT1129CST-5" H 9200 3500 40  0000 C CNN
F 2 "t_sot:SOT-223" H 9050 3400 35  0000 C CIN
F 3 "" H 9050 3300 60  0000 C CNN
	1    9050 3300
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:CP1-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue C4
U 1 1 56A81A63
P 9600 3500
F 0 "C4" H 9750 3550 50  0000 L CNN
F 1 "10uF" H 9750 3450 50  0000 L CNN
F 2 "t_passive_smd:cap_1210" H 9600 3500 50  0001 C CNN
F 3 "" H 9600 3500 50  0000 C CNN
	1    9600 3500
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:C-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue C5
U 1 1 56A81B25
P 10050 3500
F 0 "C5" H 10200 3550 50  0000 L CNN
F 1 "0.1uF" H 10200 3450 50  0000 L CNN
F 2 "t_passive_smd:cap_0603" H 10088 3350 50  0001 C CNN
F 3 "" H 10050 3500 50  0000 C CNN
	1    10050 3500
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:C-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue C6
U 1 1 56A81C24
P 10550 3500
F 0 "C6" H 10700 3550 50  0000 L CNN
F 1 "0.1uF" H 10700 3450 50  0000 L CNN
F 2 "t_passive_smd:cap_0603" H 10588 3350 50  0001 C CNN
F 3 "" H 10550 3500 50  0000 C CNN
	1    10550 3500
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:C-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue C8
U 1 1 56A81CBE
P 11050 3500
F 0 "C8" H 11200 3550 50  0000 L CNN
F 1 "0.1uF" H 11200 3450 50  0000 L CNN
F 2 "t_passive_smd:cap_0805" H 11088 3350 50  0001 C CNN
F 3 "" H 11050 3500 50  0000 C CNN
	1    11050 3500
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:C-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue C9
U 1 1 56A82229
P 11550 3500
F 0 "C9" H 11700 3550 50  0000 L CNN
F 1 "0.1uF" H 11700 3450 50  0000 L CNN
F 2 "t_passive_smd:cap_0805" H 11588 3350 50  0001 C CNN
F 3 "" H 11550 3500 50  0000 C CNN
	1    11550 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 3250 9600 3250
Wire Wire Line
	9600 3250 9600 3300
Wire Wire Line
	10050 3250 10050 3300
Connection ~ 9600 3250
Wire Wire Line
	10550 3250 10550 3300
Connection ~ 10050 3250
Wire Wire Line
	11050 3250 11050 3300
Connection ~ 10550 3250
Wire Wire Line
	11550 3200 11550 3250
Connection ~ 11050 3250
Wire Wire Line
	8950 3750 9150 3750
Wire Wire Line
	9600 3750 9600 3700
Wire Wire Line
	10050 3700 10050 3750
Connection ~ 10050 3750
Wire Wire Line
	10550 3700 10550 3750
Connection ~ 10550 3750
Wire Wire Line
	11050 3700 11050 3750
Connection ~ 11050 3750
Wire Wire Line
	9150 3600 9150 3750
Connection ~ 9600 3750
Wire Wire Line
	8950 3600 8950 3750
Connection ~ 9150 3750
Connection ~ 8050 3250
$Comp
L power:VDD #PWR016
U 1 1 56A837A1
P 11550 3200
F 0 "#PWR016" H 11550 3300 30  0001 C CNN
F 1 "VDD" H 11567 3357 30  0000 C CNN
F 2 "" H 11550 3200 60  0000 C CNN
F 3 "" H 11550 3200 60  0000 C CNN
	1    11550 3200
	1    0    0    -1  
$EndComp
Connection ~ 11550 3250
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR017
U 1 1 56A84C88
P 6800 3600
F 0 "#PWR017" H 6800 3600 30  0001 C CNN
F 1 "GND" H 6800 3530 30  0001 C CNN
F 2 "" H 6800 3600 60  0000 C CNN
F 3 "" H 6800 3600 60  0000 C CNN
	1    6800 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 3450 6800 3450
Wire Wire Line
	6800 3450 6800 3600
Text Notes 11150 6150 0    60   ~ 0
DNI
$Comp
L Plasma_Pilot4-rescue:CONN_01X02-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue P3
U 1 1 56A65842
P 6550 3400
F 0 "P3" H 6550 3650 50  0000 C CNN
F 1 "CONN_01X02" H 6500 3550 50  0000 C CNN
F 2 "t_terminal:TerminalBlock5_08mm_x2" H 6550 3400 50  0001 C CNN
F 3 "" H 6550 3400 50  0000 C CNN
	1    6550 3400
	-1   0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:CONN_01X02-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue P2
U 1 1 56A66CCF
P 2800 7800
F 0 "P2" H 2800 8050 50  0000 C CNN
F 1 "CONN_01X02" H 2750 7950 50  0000 C CNN
F 2 "t_terminal:TerminalBlock5_08mm_x2" H 2800 7800 50  0001 C CNN
F 3 "" H 2800 7800 50  0000 C CNN
	1    2800 7800
	-1   0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:CONN_01X02-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue P6
U 1 1 56A67434
P 11950 4700
F 0 "P6" H 11950 4950 50  0000 C CNN
F 1 "CONN_01X02" H 11900 4850 50  0000 C CNN
F 2 "t_terminal:TerminalBlock5_08mm_x2" H 11950 4700 50  0001 C CNN
F 3 "" H 11950 4700 50  0000 C CNN
	1    11950 4700
	-1   0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:CONN_01X02-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue P5
U 1 1 56A67985
P 11800 6750
F 0 "P5" H 11800 7000 50  0000 C CNN
F 1 "CONN_01X02" H 11750 6900 50  0000 C CNN
F 2 "t_terminal:TerminalBlock5_08mm_x2" H 11800 6750 50  0001 C CNN
F 3 "" H 11800 6750 50  0000 C CNN
	1    11800 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	12150 4650 12550 4650
Wire Wire Line
	12550 4650 12550 3900
Wire Wire Line
	11600 6700 11500 6700
Wire Wire Line
	11500 6700 11500 6350
Wire Wire Line
	11600 6800 11500 6800
Wire Wire Line
	11500 6800 11500 7050
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue TP2
U 1 1 56A6B103
P 13350 7050
F 0 "TP2" V 13300 7200 60  0000 C CNN
F 1 "TP" V 13400 7200 60  0000 C CNN
F 2 "t_misc:TP160" H 13350 7050 60  0001 C CNN
F 3 "" H 13350 7050 60  0000 C CNN
	1    13350 7050
	0    1    1    0   
$EndComp
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue TP3
U 1 1 56A63761
P 12100 6850
F 0 "TP3" H 12250 6850 60  0000 C CNN
F 1 "TP" H 12250 6750 60  0000 C CNN
F 2 "t_misc:TP80" H 12100 6850 60  0001 C CNN
F 3 "" H 12100 6850 60  0000 C CNN
	1    12100 6850
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue TP1
U 1 1 56A63830
P 12100 6550
F 0 "TP1" H 11950 6500 60  0000 C CNN
F 1 "TP" H 11950 6600 60  0000 C CNN
F 2 "t_misc:TP80" H 12100 6550 60  0001 C CNN
F 3 "" H 12100 6550 60  0000 C CNN
	1    12100 6550
	-1   0    0    1   
$EndComp
Wire Wire Line
	12100 6400 12100 6350
Wire Wire Line
	12100 7000 12100 7050
Text Label 4300 6200 2    60   ~ 0
START
Text Notes 1650 8400 0    60   ~ 0
CT520 - MODE Connects to the Plasma/Tig switch \nPlasma side (Black wire). Should be +24V when\nin Plasma mode, Open otherwise (needs pull down)
Text Notes 800  7500 0    60   ~ 0
CT520 - START Connects Trigger/Foot Pedal switch\nDon't connect to the +24V side of the switch! \n(Con 11 Red?) \nAbout 10V when switch is open , \n+24V when trigger is pressed\n\nCT6000 no voltage when open, +24V when \nTrigger Pressed (Red Wire)
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR020
U 1 1 56A74B1E
P 3500 6700
F 0 "#PWR020" H 3500 6700 30  0001 C CNN
F 1 "GND" H 3500 6630 30  0001 C CNN
F 2 "" H 3500 6700 60  0000 C CNN
F 3 "" H 3500 6700 60  0000 C CNN
	1    3500 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6600 3500 6700
Wire Wire Line
	3500 6300 3500 6200
Text Notes 10050 8950 2    60   ~ 0
CT520 - LIMIT Connects to the current POT \ncenter tap, after the Machine/Pedal switch\nis OK too. This should limit the output \ncurrent to about 12A when activated
Text Notes 6200 5150 0    60   ~ 0
275 KHz 1.0us \nPulse width
Wire Wire Line
	10150 7050 11200 7050
Wire Wire Line
	11200 7050 11500 7050
Wire Wire Line
	12550 7050 13050 7050
Wire Wire Line
	13050 7050 13200 7050
Wire Wire Line
	7000 5250 7250 5250
Wire Wire Line
	6850 9200 7000 9200
Wire Wire Line
	5900 1900 5900 5650
Wire Wire Line
	5900 1000 5900 1100
Wire Wire Line
	9600 3250 10050 3250
Wire Wire Line
	10050 3250 10550 3250
Wire Wire Line
	10550 3250 11050 3250
Wire Wire Line
	11050 3250 11550 3250
Wire Wire Line
	10050 3750 10550 3750
Wire Wire Line
	10550 3750 11050 3750
Wire Wire Line
	9600 3750 10050 3750
Wire Wire Line
	9150 3750 9600 3750
Wire Wire Line
	8050 3250 8600 3250
Wire Wire Line
	11550 3250 11550 3300
Wire Wire Line
	12100 6350 12250 6350
Wire Wire Line
	12100 7050 12550 7050
Wire Wire Line
	3500 6200 3800 6200
Wire Wire Line
	15600 4100 15600 4000
Connection ~ 15600 4000
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR025
U 1 1 5DD1836A
P 15600 4400
F 0 "#PWR025" H 15600 4400 30  0001 C CNN
F 1 "GND" H 15600 4330 30  0001 C CNN
F 2 "" H 15600 4400 60  0000 C CNN
F 3 "" H 15600 4400 60  0000 C CNN
	1    15600 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	15600 4300 15600 4400
Wire Wire Line
	14850 4000 15100 4000
Wire Wire Line
	14000 4750 14150 4750
$Comp
L power:VDD #PWR012
U 1 1 56A68BF9
P 14300 3300
F 0 "#PWR012" H 14300 3400 30  0001 C CNN
F 1 "VDD" H 14317 3457 30  0000 C CNN
F 2 "" H 14300 3300 60  0000 C CNN
F 3 "" H 14300 3300 60  0000 C CNN
	1    14300 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	14300 4300 14300 4350
Wire Wire Line
	14700 4000 14850 4000
Wire Wire Line
	14850 4750 14850 4000
Wire Wire Line
	13850 4750 14000 4750
Wire Wire Line
	14000 4100 14000 4750
Wire Wire Line
	14100 4100 14000 4100
Connection ~ 14850 4000
Connection ~ 14000 4750
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR06
U 1 1 56A5F8EA
P 14300 4350
F 0 "#PWR06" H 14300 4350 30  0001 C CNN
F 1 "GND" H 14300 4280 30  0001 C CNN
F 2 "" H 14300 4350 60  0000 C CNN
F 3 "" H 14300 4350 60  0000 C CNN
	1    14300 4350
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:LMC7101-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue U3
U 1 1 56A5EDEF
P 14400 4000
F 0 "U3" H 14550 4250 50  0000 L CNN
F 1 "LMC7101" H 14400 4150 50  0000 L CNN
F 2 "t_sot:sot-23-5" H 14450 4200 50  0001 C CNN
F 3 "" H 14400 4250 50  0000 C CNN
	1    14400 4000
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR024
U 1 1 5DD38153
P 13100 4400
F 0 "#PWR024" H 13100 4400 30  0001 C CNN
F 1 "GND" H 13100 4330 30  0001 C CNN
F 2 "" H 13100 4400 60  0000 C CNN
F 3 "" H 13100 4400 60  0000 C CNN
	1    13100 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	13100 4300 13100 4400
Text Notes 11850 3800 0    50   ~ 0
Connection to Welder GND
Wire Wire Line
	3800 7600 3800 7750
Wire Wire Line
	3800 6200 3800 6550
$Comp
L t_Devices:Res6 R18
U 1 1 5DD66CAD
P 6850 8650
F 0 "R18" H 6930 8696 50  0000 L CNN
F 1 "10K" H 6930 8605 50  0000 L CNN
F 2 "t_passive_smd:res_0603" H 6930 8559 60  0001 L CNN
F 3 "" H 6850 8660 60  0000 C CNN
	1    6850 8650
	1    0    0    -1  
$EndComp
$Comp
L t_Devices:Res6 R17
U 1 1 5DD69463
P 6850 8250
F 0 "R17" H 6930 8296 50  0000 L CNN
F 1 "15K" H 6930 8205 50  0000 L CNN
F 2 "t_passive_smd:res_0603" H 6930 8159 60  0001 L CNN
F 3 "" H 6850 8260 60  0000 C CNN
	1    6850 8250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 8650 7300 8850
Wire Wire Line
	7300 8850 7300 8950
Connection ~ 7300 8850
Connection ~ 3800 6200
Wire Wire Line
	3000 7750 3800 7750
Wire Wire Line
	3750 4250 6200 4250
Wire Wire Line
	3750 5050 5000 5050
Wire Wire Line
	5100 1800 5100 5150
Wire Wire Line
	1450 5250 1550 5250
Wire Wire Line
	1450 4050 1550 4050
Wire Wire Line
	1450 4450 1550 4450
Wire Wire Line
	1450 5650 1550 5650
Wire Wire Line
	1450 5650 1450 5850
Wire Wire Line
	1450 4050 1450 4450
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR014
U 1 1 56A69CA8
P 1450 5850
F 0 "#PWR014" H 1450 5850 30  0001 C CNN
F 1 "GND" H 1450 5780 30  0001 C CNN
F 2 "" H 1450 5850 60  0000 C CNN
F 3 "" H 1450 5850 60  0000 C CNN
	1    1450 5850
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR013
U 1 1 56A695C2
P 1450 3800
F 0 "#PWR013" H 1450 3900 30  0001 C CNN
F 1 "VDD" H 1450 3910 30  0000 C CNN
F 2 "" H 1450 3800 60  0000 C CNN
F 3 "" H 1450 3800 60  0000 C CNN
	1    1450 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 5250 1450 5650
Wire Wire Line
	1450 3800 1450 4050
Connection ~ 1450 5650
Connection ~ 1450 4050
Wire Wire Line
	3750 4550 4250 4550
Wire Wire Line
	3750 4750 4750 4750
$Comp
L t_Devices:Res6 R13
U 1 1 5DD8C86A
P 15250 4000
F 0 "R13" V 15033 4000 50  0000 C CNN
F 1 "1.0K" V 15124 4000 50  0000 C CNN
F 2 "t_passive_smd:res_0603" H 15330 3909 60  0001 L CNN
F 3 "" H 15250 4010 60  0000 C CNN
	1    15250 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	15400 4000 15600 4000
$Comp
L t_Devices:Res6 R20
U 1 1 5DD8F297
P 13100 4150
F 0 "R20" H 13020 4104 50  0000 R CNN
F 1 "100" H 13020 4195 50  0000 R CNN
F 2 "t_passive_smd:res_0805" H 13180 4059 60  0001 L CNN
F 3 "" H 13100 4160 60  0000 C CNN
	1    13100 4150
	-1   0    0    1   
$EndComp
Wire Wire Line
	12550 5250 12550 5150
Connection ~ 12550 5150
Wire Wire Line
	12550 6100 12550 6150
$Comp
L t_Devices:Res6 R11
U 1 1 5DD97FDB
P 13700 4750
F 0 "R11" V 13483 4750 50  0000 C CNN
F 1 "6.8K" V 13574 4750 50  0000 C CNN
F 2 "t_passive_smd:res_0603" H 13780 4659 60  0001 L CNN
F 3 "" H 13700 4760 60  0000 C CNN
	1    13700 4750
	0    1    1    0   
$EndComp
$Comp
L t_Devices:Res6 R12
U 1 1 5DD99932
P 14300 4750
F 0 "R12" V 14083 4750 50  0000 C CNN
F 1 "6.8K" V 14174 4750 50  0000 C CNN
F 2 "t_passive_smd:res_0603" H 14380 4659 60  0001 L CNN
F 3 "" H 14300 4760 60  0000 C CNN
	1    14300 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	14450 4750 14850 4750
Wire Wire Line
	12550 6100 12300 6100
Wire Wire Line
	12300 6100 12300 4750
Wire Wire Line
	12300 4750 13550 4750
Connection ~ 12550 6100
Wire Wire Line
	12550 6100 12550 5850
$Comp
L Plasma_Pilot4-rescue:BAV99-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue D1
U 1 1 5DD7196A
P 4000 6850
F 0 "D1" V 3750 6950 50  0000 L CNN
F 1 "BAV99" V 3850 6950 50  0000 L CNN
F 2 "t_sot:sot-23" H 3900 6850 60  0001 C CNN
F 3 "" H 3900 6850 60  0000 C CNN
	1    4000 6850
	0    1    1    0   
$EndComp
$Comp
L Plasma_Pilot4-rescue:BAV99-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue D2
U 1 1 5DDA8E3E
P 4500 6850
F 0 "D2" V 4454 6770 50  0000 R CNN
F 1 "BAV99" V 4545 6770 50  0000 R CNN
F 2 "t_sot:sot-23" H 4400 6850 60  0001 C CNN
F 3 "" H 4400 6850 60  0000 C CNN
	1    4500 6850
	0    -1   1    0   
$EndComp
Wire Wire Line
	4000 6600 4000 6550
Wire Wire Line
	4000 6550 3800 6550
Wire Wire Line
	4500 6600 4500 6200
Wire Wire Line
	4350 6850 4250 6850
Connection ~ 4250 6850
Wire Wire Line
	4250 6850 4150 6850
$Comp
L power:VDD #PWR023
U 1 1 5DDB1BDE
P 4250 7100
F 0 "#PWR023" H 4250 7200 30  0001 C CNN
F 1 "VDD" H 4250 7250 30  0000 C CNN
F 2 "" H 4250 7100 60  0000 C CNN
F 3 "" H 4250 7100 60  0000 C CNN
	1    4250 7100
	1    0    0    1   
$EndComp
Wire Wire Line
	4250 6850 4250 7100
Wire Wire Line
	12550 4650 12550 5150
Connection ~ 12550 4650
Wire Wire Line
	12300 4750 12150 4750
Connection ~ 12300 4750
Wire Notes Line
	11200 5000 13650 5000
Wire Wire Line
	3750 4450 4100 4450
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue TP6
U 1 1 5DDBB0C5
P 4100 2950
F 0 "TP6" H 4100 3050 60  0000 C CNN
F 1 "TP" H 4100 3150 60  0001 C CNN
F 2 "t_misc:TP50" H 4100 2950 60  0001 C CNN
F 3 "" H 4100 2950 60  0000 C CNN
	1    4100 2950
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue TP7
U 1 1 5DDC0057
P 4250 3100
F 0 "TP7" H 4250 3200 60  0000 C CNN
F 1 "TP" H 4250 3300 60  0001 C CNN
F 2 "t_misc:TP50" H 4250 3100 60  0001 C CNN
F 3 "" H 4250 3100 60  0000 C CNN
	1    4250 3100
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue TP8
U 1 1 5DDC1086
P 4600 2950
F 0 "TP8" H 4600 3050 60  0000 C CNN
F 1 "TP" H 4600 3150 60  0001 C CNN
F 2 "t_misc:TP50" H 4600 2950 60  0001 C CNN
F 3 "" H 4600 2950 60  0000 C CNN
	1    4600 2950
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue TP9
U 1 1 5DDC19A7
P 4750 3050
F 0 "TP9" H 4750 3150 60  0000 C CNN
F 1 "TP" H 4750 3250 60  0001 C CNN
F 2 "t_misc:TP50" H 4750 3050 60  0001 C CNN
F 3 "" H 4750 3050 60  0000 C CNN
	1    4750 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3100 4100 3300
Wire Wire Line
	4250 3250 4250 3400
Wire Wire Line
	4600 3100 4600 4650
Wire Wire Line
	4750 3200 4750 4750
Wire Wire Line
	10150 7050 9600 7050
Connection ~ 10150 7050
Wire Wire Line
	12550 5150 13200 5150
Wire Wire Line
	13050 6150 13050 6050
Wire Wire Line
	13050 5850 12550 5850
Connection ~ 12550 5850
Wire Wire Line
	12550 5850 12550 5750
Wire Notes Line
	8950 4100 8950 7250
Wire Wire Line
	11000 6350 11200 6350
Wire Wire Line
	11200 6350 11500 6350
Wire Wire Line
	10150 6350 10500 6350
Wire Wire Line
	11200 6450 11200 6350
Connection ~ 11200 6350
$Comp
L Plasma_Pilot4-rescue:Rx-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue R8
U 1 1 56A566C5
P 10750 6350
F 0 "R8" V 10967 6350 50  0000 C CNN
F 1 "10" V 10876 6350 50  0000 C CNN
F 2 "t_passive_smd:res_0805" H 10750 6350 60  0001 C CNN
F 3 "" H 10750 6350 60  0000 C CNN
	1    10750 6350
	0    -1   -1   0   
$EndComp
Connection ~ 10150 6350
Wire Wire Line
	7600 4900 7600 5000
Connection ~ 7600 4900
Wire Wire Line
	7600 4650 7600 4900
$Comp
L Plasma_Pilot4-rescue:C-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue C3
U 1 1 56A54883
P 8600 5250
F 0 "C3" H 8625 5350 50  0000 L CNN
F 1 "1000pF" H 8625 5150 50  0000 L CNN
F 2 "t_passive_smd:cap_0805" H 8638 5100 50  0001 C CNN
F 3 "" H 8600 5250 50  0000 C CNN
	1    8600 5250
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:BSS138-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue Q4
U 1 1 5DD4B335
P 8000 5200
F 0 "Q4" H 8250 5350 50  0000 L CNN
F 1 "BSS131" H 8250 5250 50  0000 L CNN
F 2 "t_sot:sot-23" H 8200 5125 50  0001 L CIN
F 3 "" H 8000 5200 50  0000 L CNN
	1    8000 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 5250 7750 5250
Wire Wire Line
	7750 5250 7750 5500
Wire Wire Line
	7750 5500 7250 5500
Wire Wire Line
	7250 5500 7250 5250
Connection ~ 7250 5250
Wire Wire Line
	7250 5250 7300 5250
Wire Wire Line
	8100 5000 8100 4900
Connection ~ 8100 4900
Wire Wire Line
	8100 4900 7600 4900
$Comp
L t_Devices:Res6 R14
U 1 1 5DD4F24B
P 6600 5250
F 0 "R14" V 6725 5250 50  0000 C CNN
F 1 "10" V 6816 5250 50  0000 C CNN
F 2 "t_passive_smd:res_0603" H 6680 5159 60  0001 L CNN
F 3 "" H 6600 5260 60  0000 C CNN
	1    6600 5250
	0    1    1    0   
$EndComp
Wire Wire Line
	6750 5250 7000 5250
Connection ~ 7000 5250
Wire Wire Line
	6450 5250 3750 5250
$Comp
L t_Devices:Res6 R16
U 1 1 5DD546B7
P 7000 5500
F 0 "R16" H 7079 5454 50  0000 L CNN
F 1 "10K" H 7079 5545 50  0000 L CNN
F 2 "t_passive_smd:res_0603" H 7080 5409 60  0001 L CNN
F 3 "" H 7000 5510 60  0000 C CNN
	1    7000 5500
	-1   0    0    1   
$EndComp
Wire Wire Line
	7000 5250 7000 5350
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue TP10
U 1 1 5DD57000
P 13350 5850
F 0 "TP10" V 13300 6000 60  0000 C CNN
F 1 "TP" V 13400 6000 60  0000 C CNN
F 2 "t_misc:TP160" H 13350 5850 60  0001 C CNN
F 3 "" H 13350 5850 60  0000 C CNN
	1    13350 5850
	0    1    1    0   
$EndComp
Wire Wire Line
	13200 5850 13050 5850
Connection ~ 13050 5850
$Comp
L t_Diodes:BAV3004W D3
U 1 1 5DD58ADF
P 9650 4450
F 0 "D3" H 9650 4639 40  0000 C CNN
F 1 "BAV3004W" H 9650 4565 40  0000 C CNN
F 2 "Diode_SMD:D_SOD-123F" H 9650 4581 60  0001 C CNN
F 3 "http://www.diodes.com/_files/datasheets/ds30371.pdf" H 9650 4581 60  0001 C CNN
	1    9650 4450
	-1   0    0    1   
$EndComp
Wire Wire Line
	10150 4450 9800 4450
Wire Wire Line
	10150 4450 10150 6350
$Comp
L Plasma_Pilot4-rescue:CP1-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue C16
U 1 1 5DD60CFE
P 7100 3600
F 0 "C16" H 7250 3650 50  0000 L CNN
F 1 "47uF" H 7250 3550 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x6.9" H 7100 3600 50  0001 C CNN
F 3 "" H 7100 3600 50  0000 C CNN
	1    7100 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 4450 7600 4450
Wire Wire Line
	7600 4450 7600 4100
Connection ~ 7600 4100
Wire Wire Line
	7100 3350 7100 3400
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR028
U 1 1 5DD67B24
P 7100 3850
F 0 "#PWR028" H 7100 3850 30  0001 C CNN
F 1 "GND" H 7100 3780 30  0001 C CNN
F 2 "" H 7100 3850 60  0000 C CNN
F 3 "" H 7100 3850 60  0000 C CNN
	1    7100 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 3800 7100 3850
$Comp
L t_Devices:Res6 R22
U 1 1 5DD68788
P 7600 3700
F 0 "R22" H 7520 3654 50  0000 R CNN
F 1 "10" H 7520 3745 50  0000 R CNN
F 2 "t_passive_smd:res_1206" H 7680 3609 60  0001 L CNN
F 3 "" H 7600 3710 60  0000 C CNN
	1    7600 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	7600 3350 7600 3550
Wire Wire Line
	7600 3250 7600 3350
Connection ~ 7600 3350
Wire Wire Line
	6750 3350 7100 3350
Wire Wire Line
	7600 3850 7600 4100
Wire Wire Line
	11550 3750 11550 3850
Wire Wire Line
	11050 3750 11550 3750
Connection ~ 11550 3750
Wire Wire Line
	11550 3750 11550 3700
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR05
U 1 1 56A5F672
P 11550 3850
F 0 "#PWR05" H 11550 3850 30  0001 C CNN
F 1 "GND" H 11550 3780 30  0001 C CNN
F 2 "" H 11550 3850 60  0000 C CNN
F 3 "" H 11550 3850 60  0000 C CNN
	1    11550 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 8250 7300 8050
Connection ~ 7300 8050
Wire Wire Line
	8200 8150 8200 8250
Wire Wire Line
	8250 8150 8200 8150
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR018
U 1 1 56A69B26
P 8200 8250
F 0 "#PWR018" H 8200 8250 30  0001 C CNN
F 1 "GND" H 8200 8180 30  0001 C CNN
F 2 "" H 8200 8250 60  0000 C CNN
F 3 "" H 8200 8250 60  0000 C CNN
	1    8200 8250
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:CONN_01X02-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue P4
U 1 1 56A6967B
P 8450 8100
F 0 "P4" H 8450 8350 50  0000 C CNN
F 1 "CONN_01X02" H 8400 8250 50  0000 C CNN
F 2 "t_terminal:TerminalBlock5_08mm_x2" H 8450 8100 50  0001 C CNN
F 3 "" H 8450 8100 50  0000 C CNN
	1    8450 8100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 8050 8250 8050
Wire Wire Line
	6650 8850 6850 8850
Wire Wire Line
	6650 8550 6650 8850
Wire Wire Line
	6650 8050 6650 8350
Wire Wire Line
	6850 8050 6650 8050
Wire Wire Line
	6850 8050 6850 8100
Wire Wire Line
	7300 8050 6850 8050
Wire Wire Line
	6850 8400 6850 8450
Wire Wire Line
	6850 8450 6850 8500
Wire Wire Line
	6850 8850 6850 8800
Wire Wire Line
	7300 8850 6850 8850
$Comp
L t_Devices:C_small C11
U 1 1 5DD6AACB
P 6650 8450
F 0 "C11" H 6563 8496 50  0000 R CNN
F 1 "0.1uF" H 6563 8405 50  0000 R CNN
F 2 "t_passive_smd:cap_0603" H 6688 8300 50  0001 C CNN
F 3 "" H 6650 8450 50  0001 C CNN
	1    6650 8450
	1    0    0    -1  
$EndComp
Connection ~ 6850 8850
Connection ~ 6850 8050
Connection ~ 6850 8450
Wire Wire Line
	6850 8450 7000 8450
$Comp
L t_Devices:C_small C13
U 1 1 5DD85FE1
P 3250 6400
F 0 "C13" H 3163 6446 50  0000 R CNN
F 1 "0.1uF" H 3163 6355 50  0000 R CNN
F 2 "t_passive_smd:cap_0603" H 3288 6250 50  0001 C CNN
F 3 "" H 3250 6400 50  0001 C CNN
	1    3250 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6200 3250 6200
Wire Wire Line
	3250 6200 3250 6300
Connection ~ 3500 6200
Wire Wire Line
	3250 6500 3250 6600
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR022
U 1 1 5DD8E0EA
P 3250 6600
F 0 "#PWR022" H 3250 6600 30  0001 C CNN
F 1 "GND" H 3250 6530 30  0001 C CNN
F 2 "" H 3250 6600 60  0000 C CNN
F 3 "" H 3250 6600 60  0000 C CNN
	1    3250 6600
	1    0    0    -1  
$EndComp
$Comp
L t_Devices:Res6 R4
U 1 1 5DD8F247
P 3500 6450
F 0 "R4" H 3580 6496 50  0000 L CNN
F 1 "10K" H 3580 6405 50  0000 L CNN
F 2 "t_passive_smd:res_0603" H 3580 6359 60  0001 L CNN
F 3 "" H 3500 6460 60  0000 C CNN
	1    3500 6450
	1    0    0    -1  
$EndComp
$Comp
L t_Devices:Res6 R5
U 1 1 5DD99BEC
P 3800 7450
F 0 "R5" H 3880 7496 50  0000 L CNN
F 1 "10K" H 3880 7405 50  0000 L CNN
F 2 "t_passive_smd:res_1206" H 3880 7359 60  0001 L CNN
F 3 "" H 3800 7460 60  0000 C CNN
	1    3800 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 5650 3750 5650
Wire Wire Line
	5050 6550 5050 6600
Wire Wire Line
	5050 6350 5050 6200
$Comp
L t_Devices:Res6 R7
U 1 1 5DDE7649
P 5450 6450
F 0 "R7" H 5530 6496 50  0000 L CNN
F 1 "10K" H 5530 6405 50  0000 L CNN
F 2 "t_passive_smd:res_0603" H 5530 6359 60  0001 L CNN
F 3 "" H 5450 6460 60  0000 C CNN
	1    5450 6450
	1    0    0    -1  
$EndComp
$Comp
L t_Devices:C_small C14
U 1 1 5DD8A14A
P 5050 6450
F 0 "C14" H 5138 6496 50  0000 L CNN
F 1 "0.1uF" H 5138 6405 50  0000 L CNN
F 2 "t_passive_smd:cap_0603" H 5088 6300 50  0001 C CNN
F 3 "" H 5050 6450 50  0001 C CNN
	1    5050 6450
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR026
U 1 1 5DD8D591
P 5050 6600
F 0 "#PWR026" H 5050 6600 30  0001 C CNN
F 1 "GND" H 5050 6530 30  0001 C CNN
F 2 "" H 5050 6600 60  0000 C CNN
F 3 "" H 5050 6600 60  0000 C CNN
	1    5050 6600
	1    0    0    -1  
$EndComp
Connection ~ 5050 6200
Text Label 5050 6200 2    60   ~ 0
MODE
$Comp
L t_Devices:Res6 R15
U 1 1 5DDA318A
P 6850 9450
F 0 "R15" H 6930 9496 50  0000 L CNN
F 1 "10K" H 6930 9405 50  0000 L CNN
F 2 "t_passive_smd:res_0603" H 6930 9359 60  0001 L CNN
F 3 "" H 6850 9460 60  0000 C CNN
	1    6850 9450
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:LMC7101-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue U2
U 1 1 5DDA522F
P 12800 1800
F 0 "U2" H 12950 2050 50  0000 L CNN
F 1 "LMC7101" H 12800 1950 50  0000 L CNN
F 2 "t_sot:sot-23-5" H 12850 2000 50  0001 C CNN
F 3 "" H 12800 2050 50  0000 C CNN
	1    12800 1800
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR031
U 1 1 5DDA868E
P 12200 1800
F 0 "#PWR031" H 12200 1800 30  0001 C CNN
F 1 "GND" H 12200 1730 30  0001 C CNN
F 2 "" H 12200 1800 60  0000 C CNN
F 3 "" H 12200 1800 60  0000 C CNN
	1    12200 1800
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR033
U 1 1 5DDA97A3
P 12700 2200
F 0 "#PWR033" H 12700 2200 30  0001 C CNN
F 1 "GND" H 12700 2130 30  0001 C CNN
F 2 "" H 12700 2200 60  0000 C CNN
F 3 "" H 12700 2200 60  0000 C CNN
	1    12700 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	12700 2100 12700 2200
Wire Wire Line
	12500 1700 12200 1700
Wire Wire Line
	12200 1700 12200 1800
$Comp
L t_Devices:Res6 R24
U 1 1 5DDAA32C
P 12100 2400
F 0 "R24" V 11883 2400 50  0000 C CNN
F 1 "50K" V 11974 2400 50  0000 C CNN
F 2 "t_passive_smd:res_0603" H 12180 2309 60  0001 L CNN
F 3 "" H 12100 2410 60  0000 C CNN
	1    12100 2400
	0    1    1    0   
$EndComp
$Comp
L t_Devices:Res6 R25
U 1 1 5DDAC7D5
P 12900 2400
F 0 "R25" V 12683 2400 50  0000 C CNN
F 1 "10K" V 12774 2400 50  0000 C CNN
F 2 "t_passive_smd:res_0603" H 12980 2309 60  0001 L CNN
F 3 "" H 12900 2410 60  0000 C CNN
	1    12900 2400
	0    1    1    0   
$EndComp
$Comp
L power:VDD #PWR032
U 1 1 5DDACE7E
P 12700 1400
F 0 "#PWR032" H 12700 1500 30  0001 C CNN
F 1 "VDD" H 12717 1557 30  0000 C CNN
F 2 "" H 12700 1400 60  0000 C CNN
F 3 "" H 12700 1400 60  0000 C CNN
	1    12700 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	12700 1400 12700 1500
$Comp
L t_Devices:Res6 R26
U 1 1 5DDB0AC4
P 13450 1800
F 0 "R26" V 13233 1800 50  0000 C CNN
F 1 "1.0K" V 13324 1800 50  0000 C CNN
F 2 "t_passive_smd:res_0603" H 13530 1709 60  0001 L CNN
F 3 "" H 13450 1810 60  0000 C CNN
	1    13450 1800
	0    1    1    0   
$EndComp
$Comp
L t_Devices:C_small C18
U 1 1 5DDB18CB
P 13800 2000
F 0 "C18" H 13888 2046 50  0000 L CNN
F 1 "1000pF" H 13888 1955 50  0000 L CNN
F 2 "t_passive_smd:cap_0603" H 13838 1850 50  0001 C CNN
F 3 "" H 13800 2000 50  0001 C CNN
	1    13800 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	13800 1900 13800 1800
Wire Wire Line
	13800 1800 13600 1800
Wire Wire Line
	13300 1800 13200 1800
Wire Wire Line
	13050 2400 13200 2400
Wire Wire Line
	13200 2400 13200 1800
Connection ~ 13200 1800
Wire Wire Line
	13200 1800 13100 1800
Wire Wire Line
	12400 1900 12500 1900
Wire Wire Line
	12750 2400 12400 2400
Wire Wire Line
	12400 2400 12400 1900
Wire Wire Line
	12250 2400 12400 2400
Connection ~ 12400 2400
Wire Wire Line
	11950 2400 11850 2400
Wire Wire Line
	11850 2050 11850 2400
Connection ~ 11850 2400
Wire Wire Line
	11850 2400 11750 2400
Wire Wire Line
	12200 1700 11850 1700
Wire Wire Line
	11850 1700 11850 1850
Connection ~ 12200 1700
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR034
U 1 1 5DDBBA60
P 13800 2150
F 0 "#PWR034" H 13800 2150 30  0001 C CNN
F 1 "GND" H 13800 2080 30  0001 C CNN
F 2 "" H 13800 2150 60  0000 C CNN
F 3 "" H 13800 2150 60  0000 C CNN
	1    13800 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	13800 2100 13800 2150
Wire Wire Line
	6000 5350 6000 9200
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue TP4
U 1 1 5DDBFE89
P 11100 2400
F 0 "TP4" V 11050 2550 60  0000 C CNN
F 1 "TP" V 11150 2550 60  0000 C CNN
F 2 "t_misc:TP80" H 11100 2400 60  0001 C CNN
F 3 "" H 11100 2400 60  0000 C CNN
	1    11100 2400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	11250 2400 11450 2400
Wire Wire Line
	14450 2700 14450 1800
Wire Wire Line
	14450 1800 13800 1800
Connection ~ 13800 1800
Text Notes 14900 5000 0    50   ~ 0
Pilot Current\n
Text Notes 14050 1500 0    50   ~ 0
Torch Voltage\n
$Comp
L t_Connectors:CONN_01X03 P8
U 1 1 5DDC3A01
P 8700 2300
F 0 "P8" H 8778 2341 50  0000 L CNN
F 1 "CONN_01X03" H 8778 2250 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-3-5.08_1x03_P5.08mm_Horizontal" H 8778 2204 60  0001 L CNN
F 3 "" H 8700 2300 60  0000 C CNN
	1    8700 2300
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR029
U 1 1 5DDC57C0
P 8400 2050
F 0 "#PWR029" H 8400 2150 30  0001 C CNN
F 1 "VDD" H 8400 2200 30  0000 C CNN
F 2 "" H 8400 2050 60  0000 C CNN
F 3 "" H 8400 2050 60  0000 C CNN
	1    8400 2050
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR030
U 1 1 5DDC8DE7
P 8400 2500
F 0 "#PWR030" H 8400 2500 30  0001 C CNN
F 1 "GND" H 8400 2430 30  0001 C CNN
F 2 "" H 8400 2500 60  0000 C CNN
F 3 "" H 8400 2500 60  0000 C CNN
	1    8400 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 2400 8400 2400
Wire Wire Line
	8400 2400 8400 2500
Wire Wire Line
	8500 2200 8400 2200
Wire Wire Line
	8400 2200 8400 2050
Wire Wire Line
	6200 2300 6200 4250
Text Notes 8650 2000 0    50   ~ 0
Main Current - Hall Effect Sensor\n
$Comp
L t_Devices:C_small C15
U 1 1 5DDCABD5
P 6200 4600
F 0 "C15" H 6288 4646 50  0000 L CNN
F 1 "0.1uF" H 6288 4555 50  0000 L CNN
F 2 "t_passive_smd:cap_0603" H 6238 4450 50  0001 C CNN
F 3 "" H 6200 4600 50  0001 C CNN
	1    6200 4600
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR027
U 1 1 5DDCD8B5
P 6200 4750
F 0 "#PWR027" H 6200 4750 30  0001 C CNN
F 1 "GND" H 6200 4680 30  0001 C CNN
F 2 "" H 6200 4750 60  0000 C CNN
F 3 "" H 6200 4750 60  0000 C CNN
	1    6200 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 4750 6200 4700
Wire Wire Line
	6200 4500 6200 4350
Wire Wire Line
	6200 4350 3750 4350
$Comp
L t_Connectors:CONN_01X03 P7
U 1 1 5DDCF17C
P 3450 2250
F 0 "P7" H 3528 2291 50  0000 L CNN
F 1 "CONN_01X03" H 3528 2200 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-3-5.08_1x03_P5.08mm_Horizontal" H 3450 2250 60  0001 C CNN
F 3 "" H 3450 2250 60  0000 C CNN
	1    3450 2250
	1    0    0    -1  
$EndComp
$Comp
L t_Devices:Res6 R2
U 1 1 5DDD01F3
P 3050 1800
F 0 "R2" H 3130 1846 50  0000 L CNN
F 1 "10K" H 3130 1755 50  0000 L CNN
F 2 "t_passive_smd:res_0603" H 3130 1709 60  0001 L CNN
F 3 "" H 3050 1810 60  0000 C CNN
	1    3050 1800
	1    0    0    -1  
$EndComp
$Comp
L t_Devices:Res6 R1
U 1 1 5DDD2912
P 2700 1800
F 0 "R1" H 2780 1846 50  0000 L CNN
F 1 "10K" H 2780 1755 50  0000 L CNN
F 2 "t_passive_smd:res_0603" H 2780 1709 60  0001 L CNN
F 3 "" H 2700 1810 60  0000 C CNN
	1    2700 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 1950 3050 2150
Wire Wire Line
	3050 2150 3250 2150
Wire Wire Line
	3250 2350 2700 2350
Wire Wire Line
	2700 2350 2700 1950
Wire Wire Line
	3050 1650 3050 1550
Wire Wire Line
	3050 1550 2700 1550
Wire Wire Line
	2700 1550 2700 1650
$Comp
L power:VDD #PWR04
U 1 1 5DDD331A
P 3050 1450
F 0 "#PWR04" H 3050 1550 30  0001 C CNN
F 1 "VDD" H 3050 1600 30  0000 C CNN
F 2 "" H 3050 1450 60  0000 C CNN
F 3 "" H 3050 1450 60  0000 C CNN
	1    3050 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 1450 3050 1550
Connection ~ 3050 1550
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR021
U 1 1 5DDD4989
P 3200 2500
F 0 "#PWR021" H 3200 2500 30  0001 C CNN
F 1 "GND" H 3200 2430 30  0001 C CNN
F 2 "" H 3200 2500 60  0000 C CNN
F 3 "" H 3200 2500 60  0000 C CNN
	1    3200 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2250 3200 2250
Wire Wire Line
	3200 2250 3200 2500
Wire Wire Line
	4100 3300 3050 3300
Wire Wire Line
	3050 3300 3050 2150
Connection ~ 4100 3300
Wire Wire Line
	4100 3300 4100 4450
Connection ~ 3050 2150
Wire Wire Line
	4250 3400 2700 3400
Wire Wire Line
	2700 3400 2700 2350
Connection ~ 4250 3400
Wire Wire Line
	4250 3400 4250 4550
Connection ~ 2700 2350
Wire Wire Line
	3750 4650 4600 4650
Wire Wire Line
	6200 2300 8500 2300
Wire Wire Line
	3750 4950 5200 4950
Wire Wire Line
	7450 2150 7450 1800
Wire Wire Line
	5200 2150 7450 2150
Wire Wire Line
	5200 4950 5200 2150
Wire Wire Line
	5450 6200 5450 6300
Wire Wire Line
	3750 5450 4500 5450
Wire Wire Line
	5450 6200 5050 6200
Wire Wire Line
	4500 6200 4500 5450
Connection ~ 4500 6200
Wire Wire Line
	8600 4900 8100 4900
Wire Wire Line
	8600 5050 8600 4900
Wire Wire Line
	8100 5400 8100 5600
Wire Wire Line
	8100 5600 7600 5600
Connection ~ 8100 5600
Wire Wire Line
	7600 5600 7600 5750
Wire Wire Line
	7600 5400 7600 5600
Wire Wire Line
	8600 5600 8100 5600
Connection ~ 7600 5600
Wire Wire Line
	8600 5450 8600 5600
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR02
U 1 1 56A54BE7
P 7600 5750
F 0 "#PWR02" H 7600 5750 30  0001 C CNN
F 1 "GND" H 7600 5680 30  0001 C CNN
F 2 "" H 7600 5750 60  0000 C CNN
F 3 "" H 7600 5750 60  0000 C CNN
	1    7600 5750
	1    0    0    -1  
$EndComp
$Comp
L t_Devices:MOV Z1
U 1 1 5DDF9B21
P 13550 6400
F 0 "Z1" V 13504 6528 50  0000 L CNN
F 1 "MOV" V 13595 6528 50  0000 L CNN
F 2 "t_passive_thd:MOV_20mm" H 13550 6400 50  0001 C CNN
F 3 "" H 13550 6400 50  0001 C CNN
	1    13550 6400
	0    1    1    0   
$EndComp
Wire Wire Line
	13550 6150 13550 6050
Wire Wire Line
	13550 6050 13050 6050
Connection ~ 13050 6050
Wire Wire Line
	13050 6050 13050 5850
Wire Wire Line
	13550 6650 13550 6750
Wire Wire Line
	13550 6750 13050 6750
Connection ~ 13050 6750
Wire Wire Line
	13050 6750 13050 7050
Connection ~ 7100 3350
Wire Wire Line
	7100 3350 7600 3350
$Comp
L t_Devices:C_small C12
U 1 1 5DD56026
P 15600 4200
F 0 "C12" H 15688 4246 50  0000 L CNN
F 1 "1000pF" H 15688 4155 50  0000 L CNN
F 2 "t_passive_smd:cap_0603" H 15638 4050 50  0001 C CNN
F 3 "" H 15600 4200 50  0001 C CNN
	1    15600 4200
	1    0    0    -1  
$EndComp
$Comp
L t_Devices:C_small C17
U 1 1 5DDB96D4
P 11850 1950
F 0 "C17" H 11938 1996 50  0000 L CNN
F 1 "0.1uF" H 11938 1905 50  0000 L CNN
F 2 "t_passive_smd:cap_0603" H 11888 1800 50  0001 C CNN
F 3 "" H 11850 1950 50  0001 C CNN
	1    11850 1950
	1    0    0    -1  
$EndComp
$Comp
L t_Devices:Res6 R3
U 1 1 5DD5B4D2
P 5900 1250
F 0 "R3" H 5980 1296 50  0000 L CNN
F 1 "10K" H 5980 1205 50  0000 L CNN
F 2 "t_passive_smd:res_0603" H 5980 1159 60  0001 L CNN
F 3 "" H 5900 1260 60  0000 C CNN
	1    5900 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 1400 5900 1900
Text Notes 12400 5600 0    70   ~ 0
0.1 Ohm Sense\nResistor goes here
$Comp
L Plasma_Pilot4-rescue:Q_NPN_BEC-device-Plasma_Pilot4-rescue Q5
U 1 1 5DD7F9C6
P 7200 8450
F 0 "Q5" H 7390 8496 50  0000 L CNN
F 1 "FMMT3904" H 7390 8405 50  0000 L CNN
F 2 "t_smd_transistors:SOT-23" H 7400 8550 50  0001 C CNN
F 3 "~" H 7200 8450 50  0001 C CNN
	1    7200 8450
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue MTG1
U 1 1 5DDA3659
P 1700 9250
F 0 "MTG1" H 1600 9350 60  0000 L CNN
F 1 "TP" H 1778 9139 60  0001 L CNN
F 2 "t_MtgHoles:MTG_200_125D" H 1700 9250 60  0001 C CNN
F 3 "" H 1700 9250 60  0000 C CNN
	1    1700 9250
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue MTG2
U 1 1 5DD92B51
P 1950 9250
F 0 "MTG2" H 1850 9350 60  0000 L CNN
F 1 "TP" H 2028 9139 60  0001 L CNN
F 2 "t_MtgHoles:MTG_200_125D" H 1950 9250 60  0001 C CNN
F 3 "" H 1950 9250 60  0000 C CNN
	1    1950 9250
	1    0    0    -1  
$EndComp
$Comp
L Plasma_Pilot4-rescue:TEST_POINT-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue MTG3
U 1 1 5DD9BFE8
P 2200 9250
F 0 "MTG3" H 2100 9350 60  0000 L CNN
F 1 "TP" H 2278 9139 60  0001 L CNN
F 2 "t_MtgHoles:MTG_200_125D" H 2200 9250 60  0001 C CNN
F 3 "" H 2200 9250 60  0000 C CNN
	1    2200 9250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 9400 2200 9500
Wire Wire Line
	2200 9500 1950 9500
Wire Wire Line
	1700 9500 1700 9400
Wire Wire Line
	1950 9400 1950 9500
Connection ~ 1950 9500
Wire Wire Line
	1950 9500 1700 9500
Wire Wire Line
	2200 9500 2200 9550
Connection ~ 2200 9500
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR035
U 1 1 5DD9F000
P 2200 9550
F 0 "#PWR035" H 2200 9550 30  0001 C CNN
F 1 "GND" H 2200 9480 30  0001 C CNN
F 2 "" H 2200 9550 60  0000 C CNN
F 3 "" H 2200 9550 60  0000 C CNN
	1    2200 9550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 2700 14450 2700
Wire Wire Line
	5550 2850 15600 2850
Wire Wire Line
	5550 4150 5550 2850
Wire Wire Line
	3750 4150 5550 4150
Wire Wire Line
	3750 4050 5450 4050
Wire Wire Line
	5450 2700 5450 4050
Wire Wire Line
	4750 7600 4750 7850
Wire Wire Line
	3000 7850 4750 7850
$Comp
L t_Devices:Res6 R6
U 1 1 5DD95E85
P 4750 7450
F 0 "R6" H 4830 7496 50  0000 L CNN
F 1 "10K" H 4830 7405 50  0000 L CNN
F 2 "t_passive_smd:res_1206" H 4830 7359 60  0001 L CNN
F 3 "" H 4750 7460 60  0000 C CNN
	1    4750 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 6600 5450 6700
$Comp
L Plasma_Pilot4-rescue:GND-Plasma_Pilot4-rescue-Plasma_Pilot4-rescue #PWR019
U 1 1 56A743EF
P 5450 6700
F 0 "#PWR019" H 5450 6700 30  0001 C CNN
F 1 "GND" H 5450 6630 30  0001 C CNN
F 2 "" H 5450 6700 60  0000 C CNN
F 3 "" H 5450 6700 60  0000 C CNN
	1    5450 6700
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener_ALT D?
U 1 1 5DE8E4E4
P 4750 7100
F 0 "D?" V 4796 7021 50  0000 R CNN
F 1 "15V Zener" V 4705 7021 50  0000 R CNN
F 2 "" H 4750 7100 50  0001 C CNN
F 3 "~" H 4750 7100 50  0001 C CNN
	1    4750 7100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4500 6200 4750 6200
$Comp
L Device:D_Zener_ALT D?
U 1 1 5DE9435C
P 3800 7100
F 0 "D?" V 3846 7179 50  0000 L CNN
F 1 "15V Zener" V 3755 7179 50  0000 L CNN
F 2 "" H 3800 7100 50  0001 C CNN
F 3 "~" H 3800 7100 50  0001 C CNN
	1    3800 7100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4750 6950 4750 6200
Connection ~ 4750 6200
Wire Wire Line
	4750 6200 5050 6200
Wire Wire Line
	4750 7250 4750 7300
Wire Wire Line
	3800 7250 3800 7300
Wire Wire Line
	3800 6950 3800 6550
Connection ~ 3800 6550
$Comp
L t_Coilcraft:EP10_GDT_2 T?
U 1 1 5E02FDE9
P 8950 4550
F 0 "T?" H 8950 4855 60  0000 C CNN
F 1 "EP10_GDT_2" H 8950 4748 60  0000 C CNN
F 2 "t_coilcraft:EP10_SMT" H 8950 4747 60  0001 C CNN
F 3 "" H 8950 4800 60  0000 C CNN
	1    8950 4550
	1    0    0    -1  
$EndComp
Text Notes 11300 7150 0    50   ~ 0
Gate
Text Notes 11250 6300 0    50   ~ 0
Source
Text Notes 8700 4800 0    50   ~ 0
36:24
Text Notes 8700 5000 0    30   ~ 0
Primary L 250uH\nGluing with JB Weld \ncauses slight gap
Wire Wire Line
	14300 3300 14300 3700
Wire Wire Line
	13850 3900 14100 3900
Wire Wire Line
	12550 3900 13100 3900
$Comp
L t_Devices:Res6 R19
U 1 1 5E03A264
P 13700 3900
F 0 "R19" V 13483 3900 50  0000 C CNN
F 1 "1.0K" V 13574 3900 50  0000 C CNN
F 2 "t_passive_smd:res_0603" H 13780 3809 60  0001 L CNN
F 3 "" H 13700 3910 60  0000 C CNN
	1    13700 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	13100 3900 13100 4000
Connection ~ 13100 3900
Wire Wire Line
	13100 3900 13550 3900
$Comp
L t_Devices:Res6 R23
U 1 1 5DDB882D
P 11600 2400
F 0 "R23" V 11383 2400 50  0000 C CNN
F 1 "392K" V 11474 2400 50  0000 C CNN
F 2 "t_passive_smd:res_1206" H 11680 2309 60  0001 L CNN
F 3 "" H 11600 2410 60  0000 C CNN
	1    11600 2400
	0    1    1    0   
$EndComp
Text Notes 6400 7750 0    50   ~ 0
R17:\n1.0K on CT6000\n15K on CT520D
Text Notes 10200 1550 0    50   ~ 0
CT6000 Open Circuit Voltage -280V\nNeed another 100K in series
$EndSCHEMATC
