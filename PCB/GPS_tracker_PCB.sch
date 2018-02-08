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
LIBS:switches
LIBS:dp3t
LIBS:linearTechCombined
LIBS:open-project
LIBS:SamacSys_Parts
LIBS:MMA8652FCR1
LIBS:cp2102
LIBS:DM3AT-SF-PEJM5
LIBS:LTC4150CMSPBF
LIBS:MCP73831T-2ATI_OT
LIBS:DMP1045U
LIBS:GPS_tracker_PCB-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
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
$EndDescr
$Comp
L Crystal Crystal1
U 1 1 59A80FAE
P 8450 2475
F 0 "Crystal1" H 8450 2625 50  0000 C CNN
F 1 "8MHz" H 8450 2325 50  0000 C CNN
F 2 "Crystals:Crystal_HC49-4H_Vertical" H 8450 2475 50  0001 C CNN
F 3 "" H 8450 2475 50  0001 C CNN
	1    8450 2475
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 59A81175
P 8725 2700
F 0 "C12" H 8750 2800 50  0000 L CNN
F 1 "22 pF" H 8750 2600 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 8763 2550 50  0001 C CNN
F 3 "" H 8725 2700 50  0001 C CNN
	1    8725 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 59A85761
P 8175 2850
F 0 "#PWR01" H 8175 2600 50  0001 C CNN
F 1 "GND" H 8175 2700 50  0000 C CNN
F 2 "" H 8175 2850 50  0001 C CNN
F 3 "" H 8175 2850 50  0001 C CNN
	1    8175 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 59A863A5
P 8725 2850
F 0 "#PWR02" H 8725 2600 50  0001 C CNN
F 1 "GND" H 8725 2700 50  0000 C CNN
F 2 "" H 8725 2850 50  0001 C CNN
F 3 "" H 8725 2850 50  0001 C CNN
	1    8725 2850
	1    0    0    -1  
$EndComp
$Comp
L SlideSwitch_MV9QS1 Switch1
U 1 1 59B04F3D
P 6325 5175
F 0 "Switch1" H 6300 5525 60  0000 C CNN
F 1 "SlideSwitch_MV9QS1" H 6325 4675 60  0000 C CNN
F 2 "Switches:SlideSwitch_MV9QS1" H 7075 6775 60  0001 C CNN
F 3 "" H 7075 6775 60  0001 C CNN
	1    6325 5175
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 59B30B63
P 5400 5075
F 0 "R5" V 5480 5075 50  0000 C CNN
F 1 "10K" V 5400 5075 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 5330 5075 50  0001 C CNN
F 3 "" H 5400 5075 50  0001 C CNN
	1    5400 5075
	0    1    1    0   
$EndComp
$Comp
L GND #PWR03
U 1 1 59B30CEB
P 5175 5150
F 0 "#PWR03" H 5175 4900 50  0001 C CNN
F 1 "GND" H 5175 5000 50  0000 C CNN
F 2 "" H 5175 5150 50  0001 C CNN
F 3 "" H 5175 5150 50  0001 C CNN
	1    5175 5150
	1    0    0    -1  
$EndComp
NoConn ~ 5950 5275
NoConn ~ 6700 5275
NoConn ~ 5950 4975
NoConn ~ 6700 4975
$Comp
L MCP1700-3302E_TO92-RESCUE-GPS_tracker_PCB VReg1
U 1 1 59BAD2EE
P 3625 2950
F 0 "VReg1" H 3650 2725 50  0000 C CNN
F 1 "MCP1700-3302E_TO92" H 3225 2825 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Wide_Oval" H 3625 2750 50  0001 C CIN
F 3 "" H 3625 2950 50  0001 C CNN
	1    3625 2950
	1    0    0    1   
$EndComp
$Comp
L C C11
U 1 1 59C8349F
P 8175 2700
F 0 "C11" H 8200 2800 50  0000 L CNN
F 1 "22 pF" H 8200 2600 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 8213 2550 50  0001 C CNN
F 3 "" H 8175 2700 50  0001 C CNN
	1    8175 2700
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 59C95BF0
P 4050 3175
F 0 "C5" H 4075 3275 50  0000 L CNN
F 1 "1uF" H 4075 3075 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4088 3025 50  0001 C CNN
F 3 "" H 4050 3175 50  0001 C CNN
	1    4050 3175
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 59C95C9D
P 3225 3175
F 0 "C4" H 3250 3275 50  0000 L CNN
F 1 "1uF" H 3250 3075 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 3263 3025 50  0001 C CNN
F 3 "" H 3225 3175 50  0001 C CNN
	1    3225 3175
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x02 J1
U 1 1 59F7A247
P 3425 1525
F 0 "J1" H 3425 1625 50  0000 C CNN
F 1 "Conn_Batt" H 3425 1325 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 3425 1525 50  0001 C CNN
F 3 "" H 3425 1525 50  0001 C CNN
	1    3425 1525
	-1   0    0    -1  
$EndComp
NoConn ~ 6700 5075
$Comp
L GND #PWR04
U 1 1 59FB936F
P 3625 1900
F 0 "#PWR04" H 3625 1650 50  0001 C CNN
F 1 "GND" H 3625 1750 50  0000 C CNN
F 2 "" H 3625 1900 50  0001 C CNN
F 3 "" H 3625 1900 50  0001 C CNN
	1    3625 1900
	1    0    0    -1  
$EndComp
$Comp
L MCP73831 U1
U 1 1 5A0CAB0A
P 2150 1400
F 0 "U1" H 2150 1150 50  0000 C CNN
F 1 "MCP73831" H 2150 1600 50  0000 C CNN
F 2 "MCP73831T-2ATI_OT:SOT95P300X145-5N" H 2350 1700 60  0000 C CNN
F 3 "" H 2150 1400 60  0000 C CNN
	1    2150 1400
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5A0CAC7B
P 1175 1450
F 0 "C1" H 1200 1550 50  0000 L CNN
F 1 "4.7uF" H 1200 1350 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 1213 1300 50  0001 C CNN
F 3 "" H 1175 1450 50  0001 C CNN
	1    1175 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 5A0CB14C
P 1175 1750
F 0 "#PWR05" H 1175 1500 50  0001 C CNN
F 1 "GND" H 1175 1600 50  0000 C CNN
F 2 "" H 1175 1750 50  0001 C CNN
F 3 "" H 1175 1750 50  0001 C CNN
	1    1175 1750
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 5A0CB23D
P 1475 1800
F 0 "D1" H 1475 1900 50  0000 C CNN
F 1 "LED" H 1475 1700 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" H 1475 1800 50  0001 C CNN
F 3 "" H 1475 1800 50  0001 C CNN
	1    1475 1800
	0    -1   -1   0   
$EndComp
$Comp
L USB_A J2
U 1 1 5A0CBD2C
P 1400 6125
F 0 "J2" H 1200 6575 50  0000 L CNN
F 1 "USB_A" H 1200 6475 50  0000 L CNN
F 2 "Connectors:USB_A" H 1550 6075 50  0001 C CNN
F 3 "" H 1550 6075 50  0001 C CNN
	1    1400 6125
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR06
U 1 1 5A0CD634
P 975 1300
F 0 "#PWR06" H 975 1150 50  0001 C CNN
F 1 "+5V" H 975 1440 50  0000 C CNN
F 2 "" H 975 1300 50  0001 C CNN
F 3 "" H 975 1300 50  0001 C CNN
	1    975  1300
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR07
U 1 1 5A0CE056
P 2625 1750
F 0 "#PWR07" H 2625 1500 50  0001 C CNN
F 1 "GND" H 2625 1600 50  0000 C CNN
F 2 "" H 2625 1750 50  0001 C CNN
F 3 "" H 2625 1750 50  0001 C CNN
	1    2625 1750
	1    0    0    -1  
$EndComp
$Comp
L R_Small R2
U 1 1 5A0CE28D
P 2825 1550
F 0 "R2" H 2700 1550 50  0000 L CNN
F 1 "10K" V 2900 1475 50  0000 L CNN
F 2 "Resistors_SMD:R_1206" H 2825 1550 50  0001 C CNN
F 3 "" H 2825 1550 50  0001 C CNN
	1    2825 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5A0CE320
P 2825 1750
F 0 "#PWR08" H 2825 1500 50  0001 C CNN
F 1 "GND" H 2825 1600 50  0000 C CNN
F 2 "" H 2825 1750 50  0001 C CNN
F 3 "" H 2825 1750 50  0001 C CNN
	1    2825 1750
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5A0CE764
P 3125 1500
F 0 "C2" H 3150 1600 50  0000 L CNN
F 1 "4.7uF" H 3150 1400 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 3163 1350 50  0001 C CNN
F 3 "" H 3125 1500 50  0001 C CNN
	1    3125 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5A0CEB0C
P 3125 1750
F 0 "#PWR09" H 3125 1500 50  0001 C CNN
F 1 "GND" H 3125 1600 50  0000 C CNN
F 2 "" H 3125 1750 50  0001 C CNN
F 3 "" H 3125 1750 50  0001 C CNN
	1    3125 1750
	1    0    0    -1  
$EndComp
$Comp
L Q_PMOS_DGS Q1
U 1 1 5A0CFE8C
P 3925 1200
F 0 "Q1" H 4125 1250 50  0000 L CNN
F 1 "DMP1045U" H 4125 1150 50  0000 L CNN
F 2 "Diodes:DMP1045U" H 4125 1300 50  0001 C CNN
F 3 "" H 3925 1200 50  0001 C CNN
	1    3925 1200
	0    1    1    0   
$EndComp
$Comp
L D_Schottky D2
U 1 1 5A0D1088
P 4275 1050
F 0 "D2" H 4275 1150 50  0000 C CNN
F 1 "B130LAW" H 4275 950 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 4275 1050 50  0001 C CNN
F 3 "" H 4275 1050 50  0001 C CNN
	1    4275 1050
	0    -1   -1   0   
$EndComp
$Comp
L C C3
U 1 1 5A0D13CE
P 4275 1500
F 0 "C3" H 4300 1600 50  0000 L CNN
F 1 "1uF" H 4300 1400 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4313 1350 50  0001 C CNN
F 3 "" H 4275 1500 50  0001 C CNN
	1    4275 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 5A0D145D
P 4275 1750
F 0 "#PWR010" H 4275 1500 50  0001 C CNN
F 1 "GND" H 4275 1600 50  0000 C CNN
F 2 "" H 4275 1750 50  0001 C CNN
F 3 "" H 4275 1750 50  0001 C CNN
	1    4275 1750
	1    0    0    -1  
$EndComp
$Comp
L R_Small R3
U 1 1 5A0D2ADC
P 4875 1050
F 0 "R3" H 4905 1070 50  0000 L CNN
F 1 "100K" H 4905 1010 50  0000 L CNN
F 2 "Resistors_SMD:R_1206" H 4875 1050 50  0001 C CNN
F 3 "" H 4875 1050 50  0001 C CNN
	1    4875 1050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 5A0D2C42
P 4875 1750
F 0 "#PWR011" H 4875 1500 50  0001 C CNN
F 1 "GND" H 4875 1600 50  0000 C CNN
F 2 "" H 4875 1750 50  0001 C CNN
F 3 "" H 4875 1750 50  0001 C CNN
	1    4875 1750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 5A0E04D3
P 1400 6675
F 0 "#PWR012" H 1400 6425 50  0001 C CNN
F 1 "GND" H 1400 6525 50  0000 C CNN
F 2 "" H 1400 6675 50  0001 C CNN
F 3 "" H 1400 6675 50  0001 C CNN
	1    1400 6675
	1    0    0    -1  
$EndComp
$Comp
L R_Small R1
U 1 1 5A0E2A15
P 1475 1450
F 0 "R1" H 1350 1450 50  0000 L CNN
F 1 "470" V 1550 1375 50  0000 L CNN
F 2 "Resistors_SMD:R_1206" H 1475 1450 50  0001 C CNN
F 3 "" H 1475 1450 50  0001 C CNN
	1    1475 1450
	1    0    0    -1  
$EndComp
Text Label 5950 5175 2    60   ~ 0
VCC
$Comp
L GND #PWR013
U 1 1 59B44F6B
P 3625 3975
F 0 "#PWR013" H 3625 3725 50  0001 C CNN
F 1 "GND" H 3625 3825 50  0000 C CNN
F 2 "" H 3625 3975 50  0001 C CNN
F 3 "" H 3625 3975 50  0001 C CNN
	1    3625 3975
	1    0    0    -1  
$EndComp
Text Notes 10050 1275 2    60   ~ 0
Accelerometer
$Comp
L GND #PWR014
U 1 1 5A15038C
P 3625 3250
F 0 "#PWR014" H 3625 3000 50  0001 C CNN
F 1 "GND" H 3625 3100 50  0000 C CNN
F 2 "" H 3625 3250 50  0001 C CNN
F 3 "" H 3625 3250 50  0001 C CNN
	1    3625 3250
	1    0    0    -1  
$EndComp
$Comp
L MMA8652FCR1 AC1
U 1 1 5A24590B
P 9175 1475
F 0 "AC1" H 9725 1625 50  0000 C CNN
F 1 "MMA8652FCR1" H 9725 925 50  0000 C CNN
F 2 "Housings_DFN_QFN:DFN-10_2x2mm_Pitch0.4mm" H 9725 825 50  0001 C CNN
F 3 "http://docs-europe.electrocomponents.com/webdocs/12de/0900766b812de9b9.pdf" H 9725 725 50  0001 C CNN
F 4 "MMA8652FCR1, Accelerometer, 3-Axis, 0  400 kHz Serial-I2C 1.95  3.6 V, 10-Pin DFN" H 9725 625 50  0001 C CNN "Description"
F 5 "RS" H 9725 525 50  0001 C CNN "Supplier_Name"
F 6 "8016889P" H 9725 425 50  0001 C CNN "RS Part Number"
F 7 "Nexperia" H 9725 325 50  0001 C CNN "Manufacturer_Name"
F 8 "MMA8652FCR1" H 9725 225 50  0001 C CNN "Manufacturer_Part_Number"
F 9 "1" H 10125 -75 50  0001 C CNN "Height"
	1    9175 1475
	1    0    0    -1  
$EndComp
Text Label 10275 1475 0    60   ~ 0
SDA
Text Label 9175 1575 2    60   ~ 0
SCL
$Comp
L ATMEGA328-AU MicroController1
U 1 1 5A246843
P 6575 2675
F 0 "MicroController1" H 5825 3925 50  0000 L BNN
F 1 "ATMEGA328P-AU" H 6350 1250 50  0000 L BNN
F 2 "Housings_QFP:TQFP-32_7x7mm_Pitch0.8mm" H 6575 2675 50  0001 C CIN
F 3 "" H 6575 2675 50  0001 C CNN
	1    6575 2675
	1    0    0    -1  
$EndComp
Text Label 7575 1575 0    60   ~ 0
GPS_TX
Text Label 7575 1675 0    60   ~ 0
GPS_RX
Text Label 7575 1875 0    60   ~ 0
MOSI
Text Label 7575 1975 0    60   ~ 0
MISO
Text Label 7575 2075 0    60   ~ 0
CLK
Text Label 7575 2825 0    60   ~ 0
SDA
Text Label 7575 2925 0    60   ~ 0
SCL
Text Label 7575 3175 0    60   ~ 0
RXD
Text Label 7575 3275 0    60   ~ 0
TXD
Text Label 7575 3375 0    60   ~ 0
REC
NoConn ~ 7575 2425
NoConn ~ 7575 2525
NoConn ~ 7575 2625
NoConn ~ 7575 2725
NoConn ~ 7575 3475
NoConn ~ 5675 3025
NoConn ~ 5675 2925
Text Label 9175 1475 2    60   ~ 0
VCC
NoConn ~ 10275 1675
NoConn ~ 9175 1675
NoConn ~ 9175 1775
NoConn ~ 9175 1875
Text Label 7575 3675 0    60   ~ 0
CS_SD
NoConn ~ 7575 1775
$Comp
L +3.3V #PWR015
U 1 1 5A25F7BD
P 4450 2950
F 0 "#PWR015" H 4450 2800 50  0001 C CNN
F 1 "+3.3V" H 4450 3090 50  0000 C CNN
F 2 "" H 4450 2950 50  0001 C CNN
F 3 "" H 4450 2950 50  0001 C CNN
	1    4450 2950
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR016
U 1 1 5A2607AA
P 6925 5575
F 0 "#PWR016" H 6925 5425 50  0001 C CNN
F 1 "+3.3V" H 6925 5715 50  0000 C CNN
F 2 "" H 6925 5575 50  0001 C CNN
F 3 "" H 6925 5575 50  0001 C CNN
	1    6925 5575
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 5A2619DA
P 10525 2000
F 0 "#PWR017" H 10525 1750 50  0001 C CNN
F 1 "GND" H 10525 1850 50  0000 C CNN
F 2 "" H 10525 2000 50  0001 C CNN
F 3 "" H 10525 2000 50  0001 C CNN
	1    10525 2000
	1    0    0    -1  
$EndComp
$Comp
L CP2102 U2
U 1 1 5A25B2BD
P 3450 6175
F 0 "U2" H 3450 6175 60  0000 C CNN
F 1 "CP2102" H 3425 5275 60  0000 C CNN
F 2 "Housings_DFN_QFN:QFN-28-1EP_5x5mm_Pitch0.5mm" H 3450 6175 60  0001 C CNN
F 3 "" H 3450 6175 60  0001 C CNN
	1    3450 6175
	1    0    0    -1  
$EndComp
Text Label 7575 3025 0    60   ~ 0
RESET
Text Label 3350 5325 1    60   ~ 0
RXD
Text Label 3450 5325 1    60   ~ 0
TXD
Text Label 3050 5325 1    60   ~ 0
GND
$Comp
L +5V #PWR018
U 1 1 5A25F3E4
P 1775 7400
F 0 "#PWR018" H 1775 7250 50  0001 C CNN
F 1 "+5V" H 1775 7540 50  0000 C CNN
F 2 "" H 1775 7400 50  0001 C CNN
F 3 "" H 1775 7400 50  0001 C CNN
	1    1775 7400
	1    0    0    -1  
$EndComp
NoConn ~ 3300 6925
$Comp
L C C9
U 1 1 5A271952
P 3150 5000
F 0 "C9" H 3175 5100 50  0000 L CNN
F 1 "100nF" H 3175 4900 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 3188 4850 50  0001 C CNN
F 3 "" H 3150 5000 50  0001 C CNN
	1    3150 5000
	1    0    0    -1  
$EndComp
Text Label 3525 4550 0    60   ~ 0
RESET
$Comp
L R R4
U 1 1 5A273EC0
P 2475 4775
F 0 "R4" V 2555 4775 50  0000 C CNN
F 1 "10K" V 2475 4775 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 2405 4775 50  0001 C CNN
F 3 "" H 2475 4775 50  0001 C CNN
	1    2475 4775
	0    1    1    0   
$EndComp
$Comp
L SW_Push SW1
U 1 1 5A275653
P 2475 4550
F 0 "SW1" H 2525 4650 50  0000 L CNN
F 1 "SW_Reset" H 2475 4490 50  0000 C CNN
F 2 "Buttons:TactileButton_2Pin" H 2475 4750 50  0001 C CNN
F 3 "" H 2475 4750 50  0001 C CNN
	1    2475 4550
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 5A277177
P 2250 6575
F 0 "C7" H 2275 6675 50  0000 L CNN
F 1 "100nF" H 2275 6475 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 2288 6425 50  0001 C CNN
F 3 "" H 2250 6575 50  0001 C CNN
	1    2250 6575
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 5A2773B4
P 2250 6875
F 0 "#PWR019" H 2250 6625 50  0001 C CNN
F 1 "GND" H 2250 6725 50  0000 C CNN
F 2 "" H 2250 6875 50  0001 C CNN
F 3 "" H 2250 6875 50  0001 C CNN
	1    2250 6875
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 5A27C30C
P 2800 7150
F 0 "#PWR020" H 2800 6900 50  0001 C CNN
F 1 "GND" H 2800 7000 50  0000 C CNN
F 2 "" H 2800 7150 50  0001 C CNN
F 3 "" H 2800 7150 50  0001 C CNN
	1    2800 7150
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 5A27C35A
P 2800 6900
F 0 "C8" H 2825 7000 50  0000 L CNN
F 1 "100nF" H 2825 6800 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 2838 6750 50  0001 C CNN
F 3 "" H 2800 6900 50  0001 C CNN
	1    2800 6900
	1    0    0    -1  
$EndComp
NoConn ~ 3500 6925
NoConn ~ 3600 6925
NoConn ~ 3550 5325
NoConn ~ 3650 5325
Text Label 2750 6025 2    60   ~ 0
GND
NoConn ~ 2750 5825
NoConn ~ 2750 5925
NoConn ~ 3250 5325
Text Label 5725 4800 0    60   ~ 0
REC
$Comp
L Conn_01x05 J3
U 1 1 5A289656
P 8875 5200
F 0 "J3" H 8875 5400 50  0000 C CNN
F 1 "Conn_GPS" H 8875 4900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05_Pitch2.54mm" H 8875 5200 50  0001 C CNN
F 3 "" H 8875 5200 50  0001 C CNN
	1    8875 5200
	1    0    0    -1  
$EndComp
Text Label 8675 5300 2    60   ~ 0
GPS_RX
Text Label 8675 5200 2    60   ~ 0
GPS_TX
Text Label 8675 5100 2    60   ~ 0
GND
Text Label 8675 5000 2    60   ~ 0
VCC
$Comp
L DM3AT-SF-PEJM5 SD1
U 1 1 5A2858B1
P 10225 3250
F 0 "SD1" H 10225 3872 50  0000 L BNN
F 1 "DM3AT-SF-PEJM5" H 10225 2529 50  0000 L BNN
F 2 "DM3AT-SF-PEJM5:HRS_DM3AT-SF-PEJM5" H 10225 3250 50  0001 L BNN
F 3 "Good" H 10225 3250 50  0001 L BNN
F 4 "1.92 USD" H 10225 3250 50  0001 L BNN "Price"
F 5 "DM3AT-SF-PEJM5" H 10225 3250 50  0001 L BNN "MP"
F 6 "None" H 10225 3250 50  0001 L BNN "Package"
F 7 "DM3 Series 8 Position Right Angle SMT Push Push Ejection MicroSD Card Connector" H 10225 3250 50  0001 L BNN "Description"
F 8 "Hirose" H 10225 3250 50  0001 L BNN "MF"
	1    10225 3250
	1    0    0    -1  
$EndComp
Text Label 10025 2750 2    60   ~ 0
VCC
Text Label 10025 2850 2    60   ~ 0
MISO
Text Label 10025 3150 2    60   ~ 0
CS_SD
Text Label 10025 3250 2    60   ~ 0
MOSI
Text Label 10025 3350 2    60   ~ 0
CLK
Text Label 10025 3450 2    60   ~ 0
GND
NoConn ~ 10025 2950
NoConn ~ 10025 3050
NoConn ~ 10025 3650
NoConn ~ 10025 3750
$Comp
L Conn_01x01 J4
U 1 1 5A299672
P 8025 4150
F 0 "J4" H 8025 4250 50  0000 C CNN
F 1 "Conn_GPS_LED" H 8025 4050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch1.00mm" H 8025 4150 50  0001 C CNN
F 3 "" H 8025 4150 50  0001 C CNN
	1    8025 4150
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 5A2A045A
P 8075 3775
F 0 "R7" V 8155 3775 50  0000 C CNN
F 1 "220" V 8075 3775 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 8005 3775 50  0001 C CNN
F 3 "" H 8075 3775 50  0001 C CNN
	1    8075 3775
	0    1    1    0   
$EndComp
$Comp
L LED D4
U 1 1 5A2A059A
P 8525 3775
F 0 "D4" H 8525 3875 50  0000 C CNN
F 1 "LED_GPS" H 8525 3625 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" H 8525 3775 50  0001 C CNN
F 3 "" H 8525 3775 50  0001 C CNN
	1    8525 3775
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 5A2A088A
P 8850 3925
F 0 "#PWR021" H 8850 3675 50  0001 C CNN
F 1 "GND" H 8850 3775 50  0000 C CNN
F 2 "" H 8850 3925 50  0001 C CNN
F 3 "" H 8850 3925 50  0001 C CNN
	1    8850 3925
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5A2A74A7
P 6975 5175
F 0 "R6" V 7055 5175 50  0000 C CNN
F 1 "220" V 6975 5175 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 6905 5175 50  0001 C CNN
F 3 "" H 6975 5175 50  0001 C CNN
	1    6975 5175
	0    1    1    0   
$EndComp
$Comp
L LED D3
U 1 1 5A2A7642
P 7350 5175
F 0 "D3" H 7350 5275 50  0000 C CNN
F 1 "LED_PWR" H 7350 5025 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" H 7350 5175 50  0001 C CNN
F 3 "" H 7350 5175 50  0001 C CNN
	1    7350 5175
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 5A2A78A9
P 7625 5300
F 0 "#PWR022" H 7625 5050 50  0001 C CNN
F 1 "GND" H 7625 5150 50  0000 C CNN
F 2 "" H 7625 5300 50  0001 C CNN
F 3 "" H 7625 5300 50  0001 C CNN
	1    7625 5300
	1    0    0    -1  
$EndComp
Text Label 9875 5800 2    60   ~ 0
MISO
Text Label 9875 5900 2    60   ~ 0
CLK
Text Label 9875 6000 2    60   ~ 0
RESET
Text Label 10375 5800 0    60   ~ 0
VCC
Text Label 10375 5900 0    60   ~ 0
MOSI
Text Label 10375 6000 0    60   ~ 0
GND
NoConn ~ 7575 3575
$Comp
L Conn_02x03_Counter_Clockwise CON1
U 1 1 5A2AA0A8
P 10075 5900
F 0 "CON1" H 10125 6100 50  0000 C CNN
F 1 "Conn_ICSP" H 10125 5700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 10075 5900 50  0001 C CNN
F 3 "" H 10075 5900 50  0001 C CNN
	1    10075 5900
	1    0    0    -1  
$EndComp
NoConn ~ 8675 5400
Text Notes 4125 6725 0    60   ~ 0
USB Connection\n
Text Notes 10250 4150 0    60   ~ 0
SD card holder
Text Notes 2075 2050 0    60   ~ 0
Battery charge circuit
Text Notes 3900 2675 0    60   ~ 0
Voltage regulator
Text Notes 5125 1175 0    60   ~ 0
R4 calculated value:\n6.56K at 4.2 volts\n10.5K at 3.0 volts
$Comp
L VCC #PWR023
U 1 1 5A6654D6
P 5450 1475
F 0 "#PWR023" H 5450 1325 50  0001 C CNN
F 1 "VCC" H 5450 1625 50  0000 C CNN
F 2 "" H 5450 1475 50  0001 C CNN
F 3 "" H 5450 1475 50  0001 C CNN
	1    5450 1475
	1    0    0    -1  
$EndComp
$Comp
L C_Small C10
U 1 1 5A665BDF
P 5475 2375
F 0 "C10" H 5485 2445 50  0000 L CNN
F 1 "0.1uF" H 5485 2295 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 5475 2375 50  0001 C CNN
F 3 "" H 5475 2375 50  0001 C CNN
	1    5475 2375
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 5A666247
P 5475 2575
F 0 "#PWR024" H 5475 2325 50  0001 C CNN
F 1 "GND" H 5475 2425 50  0000 C CNN
F 2 "" H 5475 2575 50  0001 C CNN
F 3 "" H 5475 2575 50  0001 C CNN
	1    5475 2575
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR025
U 1 1 5A668CC2
P 4850 3750
F 0 "#PWR025" H 4850 3600 50  0001 C CNN
F 1 "VCC" H 4850 3900 50  0000 C CNN
F 2 "" H 4850 3750 50  0001 C CNN
F 3 "" H 4850 3750 50  0001 C CNN
	1    4850 3750
	1    0    0    -1  
$EndComp
$Comp
L C_Small C6
U 1 1 5A668D20
P 4850 4000
F 0 "C6" H 4860 4070 50  0000 L CNN
F 1 "0.1uF" H 4860 3920 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4850 4000 50  0001 C CNN
F 3 "" H 4850 4000 50  0001 C CNN
	1    4850 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR026
U 1 1 5A668E3C
P 4850 4275
F 0 "#PWR026" H 4850 4025 50  0001 C CNN
F 1 "GND" H 4850 4125 50  0000 C CNN
F 2 "" H 4850 4275 50  0001 C CNN
F 3 "" H 4850 4275 50  0001 C CNN
	1    4850 4275
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 5A6781E0
P 5500 4025
F 0 "#PWR027" H 5500 3775 50  0001 C CNN
F 1 "GND" H 5500 3875 50  0000 C CNN
F 2 "" H 5500 4025 50  0001 C CNN
F 3 "" H 5500 4025 50  0001 C CNN
	1    5500 4025
	1    0    0    -1  
$EndComp
Connection ~ 8175 2475
Connection ~ 8725 2475
Wire Wire Line
	8725 2475 8600 2475
Wire Wire Line
	8175 2475 8300 2475
Wire Wire Line
	8175 2275 8175 2550
Wire Wire Line
	8725 2175 8725 2550
Wire Wire Line
	5250 5075 5175 5075
Wire Wire Line
	5175 5075 5175 5150
Wire Wire Line
	3225 2300 3225 3025
Connection ~ 3225 2950
Wire Wire Line
	4050 3025 4050 2950
Connection ~ 4050 2950
Wire Wire Line
	3925 2950 4450 2950
Wire Wire Line
	975  1300 1750 1300
Connection ~ 1475 1300
Wire Wire Line
	1475 1550 1475 1650
Wire Wire Line
	1175 1600 1175 1750
Wire Wire Line
	1750 1500 1750 2100
Wire Wire Line
	1750 2100 1475 2100
Wire Wire Line
	1475 2100 1475 1950
Connection ~ 1175 1300
Wire Wire Line
	2550 1500 2625 1500
Wire Wire Line
	2625 1500 2625 1750
Wire Wire Line
	2550 1400 2825 1400
Wire Wire Line
	2825 1400 2825 1450
Wire Wire Line
	2825 1650 2825 1750
Wire Wire Line
	2550 1300 3725 1300
Wire Wire Line
	3125 1300 3125 1350
Wire Wire Line
	3125 1650 3125 1750
Connection ~ 3125 1300
Wire Wire Line
	1475 850  1475 1350
Wire Wire Line
	1475 850  4875 850 
Wire Wire Line
	3925 850  3925 1000
Wire Wire Line
	4275 850  4275 900 
Connection ~ 3925 850 
Wire Wire Line
	4275 1200 4275 1350
Wire Wire Line
	4125 1300 4650 1300
Connection ~ 4275 1300
Wire Wire Line
	4275 1650 4275 1750
Wire Wire Line
	4650 1300 4650 2300
Wire Wire Line
	4650 2300 3225 2300
Wire Wire Line
	4875 850  4875 950 
Connection ~ 4275 850 
Wire Wire Line
	4875 1150 4875 1750
Wire Wire Line
	3325 2950 3225 2950
Wire Wire Line
	1400 6675 1400 6525
Wire Wire Line
	1300 6525 1300 6675
Wire Wire Line
	925  6675 1400 6675
Wire Wire Line
	3625 1525 3625 1300
Connection ~ 3625 1300
Wire Wire Line
	3625 1900 3625 1625
Connection ~ 3625 3750
Wire Wire Line
	4050 3750 4050 3325
Wire Wire Line
	3225 3325 3225 3750
Wire Wire Line
	3225 3750 4050 3750
Wire Wire Line
	3625 3750 3625 3975
Wire Wire Line
	7575 2275 8175 2275
Wire Wire Line
	7575 2175 8725 2175
Wire Wire Line
	5950 5375 5950 5575
Wire Wire Line
	5950 5575 6925 5575
Wire Wire Line
	10275 1575 10525 1575
Wire Wire Line
	10525 1575 10525 2000
Wire Wire Line
	10275 1775 10525 1775
Connection ~ 10525 1775
Wire Wire Line
	10275 1875 10525 1875
Connection ~ 10525 1875
Wire Wire Line
	1775 7400 3200 7400
Wire Wire Line
	3150 5325 3150 5150
Wire Wire Line
	3150 4550 3150 4850
Wire Wire Line
	2625 4775 3150 4775
Connection ~ 3150 4775
Wire Wire Line
	2325 4775 1975 4775
Wire Wire Line
	2675 4550 3525 4550
Connection ~ 3150 4550
Wire Wire Line
	2750 6325 2250 6325
Wire Wire Line
	2250 6325 2250 6425
Wire Wire Line
	2250 6725 2250 6875
Connection ~ 1975 7400
Wire Wire Line
	3200 7400 3200 6925
Connection ~ 2575 7400
Wire Wire Line
	2750 6425 2575 6425
Wire Wire Line
	2575 6425 2575 7400
Wire Wire Line
	2800 6750 2800 6675
Wire Wire Line
	2800 6675 2575 6675
Connection ~ 2575 6675
Wire Wire Line
	2800 7050 2800 7150
Wire Wire Line
	5550 5075 5950 5075
Wire Wire Line
	5725 4800 5725 5075
Connection ~ 5725 5075
Wire Wire Line
	2750 6125 1700 6125
Wire Wire Line
	2750 6225 1700 6225
Wire Wire Line
	1700 5925 1975 5925
Wire Wire Line
	1975 4775 1975 7400
Connection ~ 1975 5925
Wire Wire Line
	2275 4550 925  4550
Wire Wire Line
	925  4550 925  6675
Connection ~ 1300 6675
Wire Wire Line
	7575 3875 7825 3875
Wire Wire Line
	7825 3875 7825 4150
Wire Wire Line
	7575 3775 7925 3775
Wire Wire Line
	8225 3775 8375 3775
Wire Wire Line
	8675 3775 8850 3775
Wire Wire Line
	8850 3775 8850 3925
Wire Wire Line
	6700 5375 6700 5575
Connection ~ 6700 5575
Wire Wire Line
	6700 5175 6825 5175
Wire Wire Line
	7125 5175 7200 5175
Wire Wire Line
	7500 5175 7625 5175
Wire Wire Line
	7625 5175 7625 5300
Wire Wire Line
	5675 1575 5450 1575
Wire Wire Line
	5450 1475 5450 1875
Wire Wire Line
	5450 1675 5675 1675
Connection ~ 5450 1575
Wire Wire Line
	5450 1875 5675 1875
Connection ~ 5450 1675
Wire Wire Line
	5675 2175 5475 2175
Wire Wire Line
	5475 2175 5475 2275
Wire Wire Line
	5475 2475 5475 2575
Wire Wire Line
	4850 3750 4850 3900
Wire Wire Line
	4850 4100 4850 4275
Wire Wire Line
	5675 3675 5500 3675
Wire Wire Line
	5500 3675 5500 4025
Wire Wire Line
	5675 3775 5500 3775
Connection ~ 5500 3775
Wire Wire Line
	5675 3875 5500 3875
Connection ~ 5500 3875
$EndSCHEMATC
