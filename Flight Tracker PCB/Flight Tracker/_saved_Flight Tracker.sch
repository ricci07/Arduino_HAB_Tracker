EESchema Schematic File Version 4
LIBS:Flight Tracker-cache
EELAYER 26 0
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
L MCU_Module:Arduino_UNO_R3 A1
U 1 1 5E8F8079
P 7600 3300
F 0 "A1" H 7250 4250 50  0000 C CNN
F 1 "Arduino_UNO_R3" H 8100 2200 50  0000 C CNN
F 2 "Module:Arduino_UNO_R3" H 7750 2250 50  0001 L CNN
F 3 "https://www.arduino.cc/en/Main/arduinoBoardUno" H 7400 4350 50  0001 C CNN
	1    7600 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 4400 7500 4500
Wire Wire Line
	7500 4500 7600 4500
Wire Wire Line
	7700 4500 7700 4400
$Comp
L power:GND #PWR03
U 1 1 5E8F8291
P 7600 4500
F 0 "#PWR03" H 7600 4250 50  0001 C CNN
F 1 "GND" H 7605 4327 50  0000 C CNN
F 2 "" H 7600 4500 50  0001 C CNN
F 3 "" H 7600 4500 50  0001 C CNN
	1    7600 4500
	1    0    0    -1  
$EndComp
Connection ~ 7600 4500
Wire Wire Line
	7600 4500 7700 4500
Wire Wire Line
	7600 4400 7600 4500
$Comp
L NTX2:NTX2 U1
U 1 1 5E8F8691
P 4450 3350
F 0 "U1" H 4450 3350 50  0001 L BNN
F 1 "NTX2" H 4450 3350 50  0001 L BNN
F 2 "NTX2:NTX2" H 4450 3350 50  0001 L BNN
F 3 "" H 4450 3350 50  0001 C CNN
	1    4450 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5E8F8B74
P 5200 4450
F 0 "#PWR01" H 5200 4200 50  0001 C CNN
F 1 "GND" H 5205 4277 50  0000 C CNN
F 2 "" H 5200 4450 50  0001 C CNN
F 3 "" H 5200 4450 50  0001 C CNN
	1    5200 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 4450 5200 4250
Text GLabel 4800 4200 3    50   Input ~ 0
5V
Wire Wire Line
	4800 4200 4800 4100
Wire Wire Line
	5000 4000 5000 4100
Wire Wire Line
	5000 4100 4800 4100
Connection ~ 4800 4100
Wire Wire Line
	4800 4100 4800 4000
$Comp
L Device:R R6
U 1 1 5E8F928A
P 5750 4250
F 0 "R6" V 5543 4250 50  0000 C CNN
F 1 "4K7" V 5634 4250 50  0000 C CNN
F 2 "" V 5680 4250 50  0001 C CNN
F 3 "~" H 5750 4250 50  0001 C CNN
	1    5750 4250
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5E8F9301
P 5350 4250
F 0 "R5" V 5150 4200 50  0000 C CNN
F 1 "4K7" V 5250 4200 50  0000 C CNN
F 2 "" V 5280 4250 50  0001 C CNN
F 3 "~" H 5350 4250 50  0001 C CNN
	1    5350 4250
	0    1    1    0   
$EndComp
Connection ~ 5200 4250
Wire Wire Line
	5200 4250 5200 4000
Wire Wire Line
	5500 4250 5550 4250
Wire Wire Line
	5400 4000 5400 4100
Wire Wire Line
	5400 4100 5550 4100
Wire Wire Line
	5550 4100 5550 4250
Connection ~ 5550 4250
Wire Wire Line
	5550 4250 5600 4250
Text GLabel 6000 4250 2    50   Input ~ 0
5V
$Comp
L Device:R R7
U 1 1 5E8F953D
P 5750 4500
F 0 "R7" H 5820 4546 50  0000 L CNN
F 1 "47K" H 5820 4455 50  0000 L CNN
F 2 "" V 5680 4500 50  0001 C CNN
F 3 "~" H 5750 4500 50  0001 C CNN
	1    5750 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 4250 5900 4250
Text GLabel 7800 2300 1    50   Output ~ 0
5V
Text Label 7100 4000 2    50   ~ 0
RADIO_PIN
Text Label 6000 4500 0    50   ~ 0
RADIO_PIN
Wire Wire Line
	6000 4500 5900 4500
$Comp
L NTX2:UBlox_MAX8 U2
U 1 1 5E8FAF38
P 6500 5450
F 0 "U2" H 7191 5496 50  0000 L CNN
F 1 "UBlox_MAX8" H 7191 5405 50  0000 L CNN
F 2 "NTX2:Ublox Max 8" H 6900 6600 50  0001 C CNN
F 3 "" H 6900 6600 50  0001 C CNN
	1    6500 5450
	1    0    0    -1  
$EndComp
Text GLabel 7150 5300 2    50   Input ~ 0
3V3
Wire Wire Line
	5600 4500 5550 4500
Wire Wire Line
	5550 4250 5550 4500
$Comp
L power:GND #PWR02
U 1 1 5E8FAF7A
P 6700 6050
F 0 "#PWR02" H 6700 5800 50  0001 C CNN
F 1 "GND" H 6705 5877 50  0000 C CNN
F 2 "" H 6700 6050 50  0001 C CNN
F 3 "" H 6700 6050 50  0001 C CNN
	1    6700 6050
	1    0    0    -1  
$EndComp
NoConn ~ 6300 6050
Text Notes 7050 6250 0    50   ~ 0
SCL -> I2C Clock\nSDA -> I2C Data\nTP -> Time Pulse Output (NC)\nVCC -> 3V3 Supply\nGND -> GND\nNC -> NC\nTXD -> Con to MC RX\nRXD - > Con to MC TX\n
$Comp
L Transistor_FET:BSS138 Q1
U 1 1 5E8FB277
P 1750 1550
F 0 "Q1" V 2000 1550 50  0000 C CNN
F 1 "BSS138" V 2091 1550 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1950 1475 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 1750 1550 50  0001 L CNN
	1    1750 1550
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 5E8FB2ED
P 1450 1400
F 0 "R1" H 1520 1446 50  0000 L CNN
F 1 "10K" H 1520 1355 50  0000 L CNN
F 2 "" V 1380 1400 50  0001 C CNN
F 3 "~" H 1450 1400 50  0001 C CNN
	1    1450 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5E8FB347
P 2050 1400
F 0 "R3" H 2120 1446 50  0000 L CNN
F 1 "10K" H 2120 1355 50  0000 L CNN
F 2 "" V 1980 1400 50  0001 C CNN
F 3 "~" H 2050 1400 50  0001 C CNN
	1    2050 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1650 1450 1650
Wire Wire Line
	1450 1550 1450 1650
Wire Wire Line
	1950 1650 2050 1650
Wire Wire Line
	2050 1550 2050 1650
Connection ~ 2050 1650
Wire Wire Line
	2050 1650 2350 1650
Connection ~ 1450 1650
Wire Wire Line
	1250 1650 1450 1650
Wire Wire Line
	1450 1250 1450 1150
Wire Wire Line
	1750 1350 1750 1150
Wire Wire Line
	1750 1150 1450 1150
Connection ~ 1450 1150
Wire Wire Line
	1450 1150 1450 1050
Text GLabel 1450 1050 1    50   Input ~ 0
3V3
Text GLabel 2050 1000 1    50   Input ~ 0
5V
Wire Wire Line
	2050 1000 2050 1250
Text GLabel 1250 1650 0    50   BiDi ~ 0
SCL_3V3
Text GLabel 2350 1650 2    50   BiDi ~ 0
SCL_5V
$Comp
L Transistor_FET:BSS138 Q2
U 1 1 5E8FEE25
P 1750 2550
F 0 "Q2" V 2000 2550 50  0000 C CNN
F 1 "BSS138" V 2091 2550 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1950 2475 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 1750 2550 50  0001 L CNN
	1    1750 2550
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5E8FEE2C
P 1450 2400
F 0 "R2" H 1520 2446 50  0000 L CNN
F 1 "10K" H 1520 2355 50  0000 L CNN
F 2 "" V 1380 2400 50  0001 C CNN
F 3 "~" H 1450 2400 50  0001 C CNN
	1    1450 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5E8FEE33
P 2050 2400
F 0 "R4" H 2120 2446 50  0000 L CNN
F 1 "10K" H 2120 2355 50  0000 L CNN
F 2 "" V 1980 2400 50  0001 C CNN
F 3 "~" H 2050 2400 50  0001 C CNN
	1    2050 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 2650 1450 2650
Wire Wire Line
	1450 2550 1450 2650
Wire Wire Line
	1950 2650 2050 2650
Wire Wire Line
	2050 2550 2050 2650
Connection ~ 2050 2650
Wire Wire Line
	2050 2650 2350 2650
Connection ~ 1450 2650
Wire Wire Line
	1250 2650 1450 2650
Wire Wire Line
	1450 2250 1450 2150
Wire Wire Line
	1750 2350 1750 2150
Wire Wire Line
	1750 2150 1450 2150
Connection ~ 1450 2150
Wire Wire Line
	1450 2150 1450 2050
Text GLabel 1450 2050 1    50   Input ~ 0
3V3
Text GLabel 2050 2000 1    50   Input ~ 0
5V
Wire Wire Line
	2050 2000 2050 2250
Text GLabel 1250 2650 0    50   BiDi ~ 0
SDA_3V3
Text GLabel 2350 2650 2    50   BiDi ~ 0
SDA_5V
Text GLabel 8100 3700 2    50   BiDi ~ 0
SDA_5V
Text GLabel 8100 3800 2    50   BiDi ~ 0
SCL_5V
Text GLabel 5900 5300 0    50   BiDi ~ 0
SCL_3V3
Text GLabel 5900 5600 0    50   BiDi ~ 0
SDA_3V3
Text Notes 1400 3150 0    50   ~ 0
I2C Line Level Shifters
Wire Notes Line
	700  650  700  3000
Wire Notes Line
	2800 3000 700  3000
Wire Notes Line
	2800 650  700  650 
Wire Notes Line
	2800 650  2800 3000
Text GLabel 7700 2300 1    50   Output ~ 0
3V3
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5E90A2FC
P 7500 1700
F 0 "J1" V 7466 1512 50  0000 R CNN
F 1 "Conn_01x02" V 7375 1512 50  0000 R CNN
F 2 "" H 7500 1700 50  0001 C CNN
F 3 "~" H 7500 1700 50  0001 C CNN
	1    7500 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5E90AD1D
P 7600 1900
F 0 "#PWR06" H 7600 1650 50  0001 C CNN
F 1 "GND" H 7605 1727 50  0000 C CNN
F 2 "" H 7600 1900 50  0001 C CNN
F 3 "" H 7600 1900 50  0001 C CNN
	1    7600 1900
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5E911CD1
P 7500 1900
F 0 "#FLG01" H 7500 1975 50  0001 C CNN
F 1 "PWR_FLAG" V 7950 1750 50  0000 L CNN
F 2 "" H 7500 1900 50  0001 C CNN
F 3 "~" H 7500 1900 50  0001 C CNN
	1    7500 1900
	0    -1   -1   0   
$EndComp
Connection ~ 7500 1900
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5E911D6F
P 7600 1900
F 0 "#FLG02" H 7600 1975 50  0001 C CNN
F 1 "PWR_FLAG" V 7600 2028 50  0000 L CNN
F 2 "" H 7600 1900 50  0001 C CNN
F 3 "~" H 7600 1900 50  0001 C CNN
	1    7600 1900
	0    1    1    0   
$EndComp
Connection ~ 7600 1900
Text Notes 7200 1600 0    50   ~ 0
Power Supply (6-24V)\n
Text Notes 8500 3800 0    50   ~ 0
Ublox Bus
NoConn ~ 6750 4850
Wire Wire Line
	7500 1900 7500 2300
$Comp
L Device:R R8
U 1 1 5E9136AC
P 6750 2500
F 0 "R8" H 6775 2650 50  0000 L CNN
F 1 "150R" V 6650 2400 50  0000 L CNN
F 2 "" V 6680 2500 50  0001 C CNN
F 3 "~" H 6750 2500 50  0001 C CNN
	1    6750 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5E9172FA
P 7150 2100
F 0 "#PWR05" H 7150 1850 50  0001 C CNN
F 1 "GND" H 7250 2000 50  0000 C CNN
F 2 "" H 7150 2100 50  0001 C CNN
F 3 "" H 7150 2100 50  0001 C CNN
	1    7150 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5E918860
P 7000 2500
F 0 "R9" H 7025 2650 50  0000 L CNN
F 1 "150R" V 6900 2400 50  0000 L CNN
F 2 "" V 6930 2500 50  0001 C CNN
F 3 "~" H 7000 2500 50  0001 C CNN
	1    7000 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 2800 7100 2800
Wire Wire Line
	7000 2700 7100 2700
$Comp
L Device:LED D2
U 1 1 5E91A9EF
P 7000 2150
F 0 "D2" V 6900 2150 50  0000 L CNN
F 1 "WARN_LED" V 7125 2175 50  0000 L CNN
F 2 "" H 7000 2150 50  0001 C CNN
F 3 "~" H 7000 2150 50  0001 C CNN
	1    7000 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	7000 1950 7000 2000
$Comp
L power:GND #PWR04
U 1 1 5E922529
P 6600 2100
F 0 "#PWR04" H 6600 1850 50  0001 C CNN
F 1 "GND" H 6500 2000 50  0000 C CNN
F 2 "" H 6600 2100 50  0001 C CNN
F 3 "" H 6600 2100 50  0001 C CNN
	1    6600 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 2000 6750 1950
Wire Wire Line
	6750 1950 6600 1950
Wire Wire Line
	6600 1950 6600 2100
$Comp
L Device:LED D1
U 1 1 5E9216E1
P 6750 2150
F 0 "D1" V 6650 2150 50  0000 L CNN
F 1 "OK_LED" V 6875 1850 50  0000 L CNN
F 2 "" H 6750 2150 50  0001 C CNN
F 3 "~" H 6750 2150 50  0001 C CNN
	1    6750 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	7000 2650 7000 2700
Wire Wire Line
	6750 2650 6750 2800
Wire Wire Line
	7000 2350 7000 2300
Wire Wire Line
	7150 2100 7150 1950
Wire Wire Line
	7150 1950 7000 1950
Text Notes 6600 1900 0    50   ~ 0
Indicator LEDs
Wire Wire Line
	6750 2350 6750 2300
$Comp
L NTX2:BMP280 U?
U 1 1 5E93A3D7
P 4200 5400
F 0 "U?" H 4200 5903 50  0000 C CNN
F 1 "BMP280" H 4200 5812 50  0000 C CNN
F 2 "NTX2:BMP280" H 3800 5875 50  0001 C CNN
F 3 "" H 3800 5875 50  0001 C CNN
	1    4200 5400
	1    0    0    -1  
$EndComp
$Comp
L NTX2:BH1750 U?
U 1 1 5E93BA38
P 3200 5850
F 0 "U?" H 3200 6315 50  0000 C CNN
F 1 "BH1750" H 3200 6224 50  0000 C CNN
F 2 "NTX2:BH1750" H 2500 6300 50  0001 C CNN
F 3 "" H 2500 6300 50  0001 C CNN
	1    3200 5850
	1    0    0    -1  
$EndComp
$EndSCHEMATC