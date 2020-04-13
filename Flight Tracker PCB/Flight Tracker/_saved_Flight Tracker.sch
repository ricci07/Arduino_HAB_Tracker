EESchema Schematic File Version 4
LIBS:Flight Tracker-cache
EELAYER 26 0
EELAYER END
$Descr A3 16535 11693
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
P 10850 4625
F 0 "A1" H 10500 5575 50  0000 C CNN
F 1 "Arduino_UNO_R3" H 11350 3525 50  0000 C CNN
F 2 "Module:Arduino_UNO_R3" H 11000 3575 50  0001 L CNN
F 3 "https://www.arduino.cc/en/Main/arduinoBoardUno" H 10650 5675 50  0001 C CNN
	1    10850 4625
	1    0    0    -1  
$EndComp
Wire Wire Line
	10750 5725 10750 5825
Wire Wire Line
	10750 5825 10850 5825
Wire Wire Line
	10950 5825 10950 5725
$Comp
L power:GND #PWR03
U 1 1 5E8F8291
P 10850 5825
F 0 "#PWR03" H 10850 5575 50  0001 C CNN
F 1 "GND" H 10855 5652 50  0000 C CNN
F 2 "" H 10850 5825 50  0001 C CNN
F 3 "" H 10850 5825 50  0001 C CNN
	1    10850 5825
	1    0    0    -1  
$EndComp
Connection ~ 10850 5825
Wire Wire Line
	10850 5825 10950 5825
Wire Wire Line
	10850 5725 10850 5825
$Comp
L NTX2:NTX2 U1
U 1 1 5E8F8691
P 8025 4650
F 0 "U1" H 8025 4650 50  0001 L BNN
F 1 "NTX2" H 8025 4650 50  0001 L BNN
F 2 "NTX2:NTX2" H 8025 4650 50  0001 L BNN
F 3 "" H 8025 4650 50  0001 C CNN
	1    8025 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5E8F8B74
P 8775 5750
F 0 "#PWR01" H 8775 5500 50  0001 C CNN
F 1 "GND" H 8780 5577 50  0000 C CNN
F 2 "" H 8775 5750 50  0001 C CNN
F 3 "" H 8775 5750 50  0001 C CNN
	1    8775 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8775 5750 8775 5550
Text GLabel 8375 5500 3    50   Input ~ 0
5V
$Comp
L Device:R R6
U 1 1 5E8F928A
P 9325 5550
F 0 "R6" V 9118 5550 50  0000 C CNN
F 1 "4K7" V 9209 5550 50  0000 C CNN
F 2 "" V 9255 5550 50  0001 C CNN
F 3 "~" H 9325 5550 50  0001 C CNN
	1    9325 5550
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5E8F9301
P 8925 5550
F 0 "R5" V 8725 5500 50  0000 C CNN
F 1 "4K7" V 8825 5500 50  0000 C CNN
F 2 "" V 8855 5550 50  0001 C CNN
F 3 "~" H 8925 5550 50  0001 C CNN
	1    8925 5550
	0    1    1    0   
$EndComp
Connection ~ 8775 5550
Wire Wire Line
	8775 5550 8775 5300
Wire Wire Line
	9075 5550 9125 5550
Wire Wire Line
	8975 5300 8975 5400
Wire Wire Line
	8975 5400 9125 5400
Wire Wire Line
	9125 5400 9125 5550
Connection ~ 9125 5550
Wire Wire Line
	9125 5550 9175 5550
Text GLabel 9575 5550 2    50   Input ~ 0
5V
$Comp
L Device:R R7
U 1 1 5E8F953D
P 9325 5800
F 0 "R7" H 9395 5846 50  0000 L CNN
F 1 "47K" H 9395 5755 50  0000 L CNN
F 2 "" V 9255 5800 50  0001 C CNN
F 3 "~" H 9325 5800 50  0001 C CNN
	1    9325 5800
	0    1    1    0   
$EndComp
Wire Wire Line
	9575 5550 9475 5550
Text GLabel 11050 3625 1    50   Output ~ 0
5V
Text Label 10350 5325 2    50   ~ 0
RADIO_PIN
Text Label 9575 5800 0    50   ~ 0
RADIO_PIN
Wire Wire Line
	9575 5800 9475 5800
$Comp
L NTX2:UBlox_MAX8 U2
U 1 1 5E8FAF38
P 11750 7450
F 0 "U2" H 12441 7496 50  0000 L CNN
F 1 "UBlox_MAX8" H 12441 7405 50  0000 L CNN
F 2 "NTX2:Ublox Max 8" H 12150 8600 50  0001 C CNN
F 3 "" H 12150 8600 50  0001 C CNN
	1    11750 7450
	1    0    0    -1  
$EndComp
Text GLabel 12400 7300 2    50   Input ~ 0
3V3
Wire Wire Line
	9175 5800 9125 5800
Wire Wire Line
	9125 5550 9125 5800
$Comp
L power:GND #PWR02
U 1 1 5E8FAF7A
P 11950 8050
F 0 "#PWR02" H 11950 7800 50  0001 C CNN
F 1 "GND" H 11955 7877 50  0000 C CNN
F 2 "" H 11950 8050 50  0001 C CNN
F 3 "" H 11950 8050 50  0001 C CNN
	1    11950 8050
	1    0    0    -1  
$EndComp
NoConn ~ 11550 8050
Text Notes 11400 9000 0    50   ~ 0
SCL -> I2C Clock\nSDA -> I2C Data\nTP -> Time Pulse Output (NC)\nVCC -> 3V3 Supply\nGND -> GND\nNC -> NC\nTXD -> Con to MC RX\nRXD - > Con to MC TX\n
$Comp
L Transistor_FET:BSS138 Q1
U 1 1 5E8FB277
P 4350 2675
F 0 "Q1" V 4600 2675 50  0000 C CNN
F 1 "BSS138" V 4691 2675 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4550 2600 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 4350 2675 50  0001 L CNN
	1    4350 2675
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 5E8FB2ED
P 4050 2525
F 0 "R1" H 4120 2571 50  0000 L CNN
F 1 "10K" H 4120 2480 50  0000 L CNN
F 2 "" V 3980 2525 50  0001 C CNN
F 3 "~" H 4050 2525 50  0001 C CNN
	1    4050 2525
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5E8FB347
P 4650 2525
F 0 "R3" H 4720 2571 50  0000 L CNN
F 1 "10K" H 4720 2480 50  0000 L CNN
F 2 "" V 4580 2525 50  0001 C CNN
F 3 "~" H 4650 2525 50  0001 C CNN
	1    4650 2525
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 2775 4050 2775
Wire Wire Line
	4050 2675 4050 2775
Wire Wire Line
	4550 2775 4650 2775
Wire Wire Line
	4650 2675 4650 2775
Connection ~ 4650 2775
Connection ~ 4050 2775
Wire Wire Line
	3850 2775 4050 2775
Wire Wire Line
	4050 2375 4050 2275
Wire Wire Line
	4350 2475 4350 2275
Wire Wire Line
	4350 2275 4050 2275
Connection ~ 4050 2275
Wire Wire Line
	4050 2275 4050 2175
Text GLabel 4050 2175 1    50   Input ~ 0
3V3
Text GLabel 4650 2125 1    50   Input ~ 0
5V
Wire Wire Line
	4650 2125 4650 2375
Text GLabel 3850 2775 0    50   BiDi ~ 0
Ublox­_SCL_3V3
$Comp
L Transistor_FET:BSS138 Q2
U 1 1 5E8FEE25
P 4350 3675
F 0 "Q2" V 4600 3675 50  0000 C CNN
F 1 "BSS138" V 4691 3675 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4550 3600 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 4350 3675 50  0001 L CNN
	1    4350 3675
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5E8FEE2C
P 4050 3525
F 0 "R2" H 4120 3571 50  0000 L CNN
F 1 "10K" H 4120 3480 50  0000 L CNN
F 2 "" V 3980 3525 50  0001 C CNN
F 3 "~" H 4050 3525 50  0001 C CNN
	1    4050 3525
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5E8FEE33
P 4650 3525
F 0 "R4" H 4720 3571 50  0000 L CNN
F 1 "10K" H 4720 3480 50  0000 L CNN
F 2 "" V 4580 3525 50  0001 C CNN
F 3 "~" H 4650 3525 50  0001 C CNN
	1    4650 3525
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 3775 4050 3775
Wire Wire Line
	4050 3675 4050 3775
Wire Wire Line
	4550 3775 4650 3775
Wire Wire Line
	4650 3675 4650 3775
Connection ~ 4650 3775
Connection ~ 4050 3775
Wire Wire Line
	3850 3775 4050 3775
Wire Wire Line
	4050 3375 4050 3275
Wire Wire Line
	4350 3475 4350 3275
Wire Wire Line
	4350 3275 4050 3275
Connection ~ 4050 3275
Wire Wire Line
	4050 3275 4050 3175
Text GLabel 4050 3175 1    50   Input ~ 0
3V3
Text GLabel 4650 3125 1    50   Input ~ 0
5V
Wire Wire Line
	4650 3125 4650 3375
Text GLabel 3850 3775 0    50   BiDi ~ 0
Ublox_SDA_3V3
Text GLabel 11150 7600 0    50   BiDi ~ 0
Ublox_SDA_3V3
Text Notes 4800 2125 0    50   ~ 0
Ublox SCL Shifter
Text GLabel 10950 3625 1    50   Output ~ 0
3V3
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5E90A2FC
P 10750 3025
F 0 "J1" V 10716 2837 50  0000 R CNN
F 1 "Conn_01x02" V 10800 2850 50  0000 R CNN
F 2 "" H 10750 3025 50  0001 C CNN
F 3 "~" H 10750 3025 50  0001 C CNN
	1    10750 3025
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5E90AD1D
P 10850 3275
F 0 "#PWR06" H 10850 3025 50  0001 C CNN
F 1 "GND" H 10855 3102 50  0000 C CNN
F 2 "" H 10850 3275 50  0001 C CNN
F 3 "" H 10850 3275 50  0001 C CNN
	1    10850 3275
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5E911CD1
P 10750 3225
F 0 "#FLG01" H 10750 3300 50  0001 C CNN
F 1 "PWR_FLAG" V 11200 3075 50  0000 L CNN
F 2 "" H 10750 3225 50  0001 C CNN
F 3 "~" H 10750 3225 50  0001 C CNN
	1    10750 3225
	0    -1   -1   0   
$EndComp
Connection ~ 10750 3225
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5E911D6F
P 10850 3250
F 0 "#FLG02" H 10850 3325 50  0001 C CNN
F 1 "PWR_FLAG" V 10850 3378 50  0000 L CNN
F 2 "" H 10850 3250 50  0001 C CNN
F 3 "~" H 10850 3250 50  0001 C CNN
	1    10850 3250
	0    1    1    0   
$EndComp
Text Notes 10450 2925 0    50   ~ 0
Power Supply (6-24V)\n
NoConn ~ 12000 6850
$Comp
L Device:R R8
U 1 1 5E9136AC
P 10000 3825
F 0 "R8" H 10025 3975 50  0000 L CNN
F 1 "150R" V 9900 3725 50  0000 L CNN
F 2 "" V 9930 3825 50  0001 C CNN
F 3 "~" H 10000 3825 50  0001 C CNN
	1    10000 3825
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5E9172FA
P 10400 3425
F 0 "#PWR05" H 10400 3175 50  0001 C CNN
F 1 "GND" H 10500 3325 50  0000 C CNN
F 2 "" H 10400 3425 50  0001 C CNN
F 3 "" H 10400 3425 50  0001 C CNN
	1    10400 3425
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5E918860
P 10250 3825
F 0 "R9" H 10275 3975 50  0000 L CNN
F 1 "150R" V 10150 3725 50  0000 L CNN
F 2 "" V 10180 3825 50  0001 C CNN
F 3 "~" H 10250 3825 50  0001 C CNN
	1    10250 3825
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 4125 10350 4125
Wire Wire Line
	10250 4025 10350 4025
$Comp
L Device:LED D2
U 1 1 5E91A9EF
P 10250 3475
F 0 "D2" V 10150 3475 50  0000 L CNN
F 1 "WARN_LED" V 10375 3500 50  0000 L CNN
F 2 "" H 10250 3475 50  0001 C CNN
F 3 "~" H 10250 3475 50  0001 C CNN
	1    10250 3475
	0    1    1    0   
$EndComp
Wire Wire Line
	10250 3275 10250 3325
$Comp
L power:GND #PWR04
U 1 1 5E922529
P 9850 3425
F 0 "#PWR04" H 9850 3175 50  0001 C CNN
F 1 "GND" H 9750 3325 50  0000 C CNN
F 2 "" H 9850 3425 50  0001 C CNN
F 3 "" H 9850 3425 50  0001 C CNN
	1    9850 3425
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 3325 10000 3275
Wire Wire Line
	10000 3275 9850 3275
Wire Wire Line
	9850 3275 9850 3425
$Comp
L Device:LED D1
U 1 1 5E9216E1
P 10000 3475
F 0 "D1" V 9900 3475 50  0000 L CNN
F 1 "OK_LED" V 10125 3175 50  0000 L CNN
F 2 "" H 10000 3475 50  0001 C CNN
F 3 "~" H 10000 3475 50  0001 C CNN
	1    10000 3475
	0    1    1    0   
$EndComp
Wire Wire Line
	10250 3975 10250 4025
Wire Wire Line
	10000 3975 10000 4125
Wire Wire Line
	10250 3675 10250 3625
Wire Wire Line
	10400 3425 10400 3275
Wire Wire Line
	10400 3275 10250 3275
Text Notes 9850 3225 0    50   ~ 0
Indicator LEDs
Wire Wire Line
	10000 3675 10000 3625
$Comp
L NTX2:BH1750 U3
U 1 1 5E93BA38
P 6900 8275
F 0 "U3" H 6900 8740 50  0000 C CNN
F 1 "BH1750" H 6900 8649 50  0000 C CNN
F 2 "NTX2:BH1750" H 6200 8725 50  0001 C CNN
F 3 "" H 6200 8725 50  0001 C CNN
	1    6900 8275
	1    0    0    -1  
$EndComp
$Comp
L NTX2:ML8511 U5
U 1 1 5E93CEA8
P 6875 6825
F 0 "U5" H 6875 7240 50  0000 C CNN
F 1 "ML8511" H 6875 7149 50  0000 C CNN
F 2 "NTX2:ML8511" H 6125 7125 50  0001 C CNN
F 3 "" H 6125 7125 50  0001 C CNN
	1    6875 6825
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSS138 Q3
U 1 1 5E944D02
P 4375 4725
F 0 "Q3" V 4625 4725 50  0000 C CNN
F 1 "BSS138" V 4716 4725 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4575 4650 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 4375 4725 50  0001 L CNN
	1    4375 4725
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 5E944D09
P 4075 4575
F 0 "R10" H 4145 4621 50  0000 L CNN
F 1 "10K" H 4145 4530 50  0000 L CNN
F 2 "" V 4005 4575 50  0001 C CNN
F 3 "~" H 4075 4575 50  0001 C CNN
	1    4075 4575
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 5E944D10
P 4675 4575
F 0 "R12" H 4745 4621 50  0000 L CNN
F 1 "10K" H 4745 4530 50  0000 L CNN
F 2 "" V 4605 4575 50  0001 C CNN
F 3 "~" H 4675 4575 50  0001 C CNN
	1    4675 4575
	1    0    0    -1  
$EndComp
Wire Wire Line
	4175 4825 4075 4825
Wire Wire Line
	4075 4725 4075 4825
Wire Wire Line
	4575 4825 4675 4825
Wire Wire Line
	4675 4725 4675 4825
Connection ~ 4675 4825
Connection ~ 4075 4825
Wire Wire Line
	3875 4825 4075 4825
Wire Wire Line
	4075 4425 4075 4325
Wire Wire Line
	4375 4525 4375 4325
Wire Wire Line
	4375 4325 4075 4325
Connection ~ 4075 4325
Wire Wire Line
	4075 4325 4075 4225
Text GLabel 4075 4225 1    50   Input ~ 0
3V3
Text GLabel 4675 4175 1    50   Input ~ 0
5V
Wire Wire Line
	4675 4175 4675 4425
Text GLabel 3875 4825 0    50   BiDi ~ 0
BMP280_SCL_3V3
Text Notes 4800 3225 0    50   ~ 0
Ublox SDA Shifter
Text Notes 4850 4150 0    50   ~ 0
BMP280 SCL Shifter
$Comp
L Transistor_FET:BSS138 Q4
U 1 1 5E94C800
P 4475 5900
F 0 "Q4" V 4725 5900 50  0000 C CNN
F 1 "BSS138" V 4816 5900 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4675 5825 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 4475 5900 50  0001 L CNN
	1    4475 5900
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 5E94C807
P 4175 5750
F 0 "R11" H 4245 5796 50  0000 L CNN
F 1 "10K" H 4245 5705 50  0000 L CNN
F 2 "" V 4105 5750 50  0001 C CNN
F 3 "~" H 4175 5750 50  0001 C CNN
	1    4175 5750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 5E94C80E
P 4775 5750
F 0 "R13" H 4845 5796 50  0000 L CNN
F 1 "10K" H 4845 5705 50  0000 L CNN
F 2 "" V 4705 5750 50  0001 C CNN
F 3 "~" H 4775 5750 50  0001 C CNN
	1    4775 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4275 6000 4175 6000
Wire Wire Line
	4175 5900 4175 6000
Wire Wire Line
	4675 6000 4775 6000
Wire Wire Line
	4775 5900 4775 6000
Connection ~ 4775 6000
Connection ~ 4175 6000
Wire Wire Line
	3975 6000 4175 6000
Wire Wire Line
	4175 5600 4175 5500
Wire Wire Line
	4475 5700 4475 5500
Wire Wire Line
	4475 5500 4175 5500
Connection ~ 4175 5500
Wire Wire Line
	4175 5500 4175 5400
Text GLabel 4175 5400 1    50   Input ~ 0
3V3
Text GLabel 4775 5350 1    50   Input ~ 0
5V
Wire Wire Line
	4775 5350 4775 5600
Text GLabel 3975 6000 0    50   BiDi ~ 0
BMP280_SDA_3V3
Text Notes 4950 5325 0    50   ~ 0
BMP280 SDA Shifter
Text GLabel 8925 6825 0    50   BiDi ~ 0
BMP280_SDA_3V3
Text GLabel 8925 6825 0    50   BiDi ~ 0
BMP280_SDA_3V3
Text GLabel 8925 6675 0    50   BiDi ~ 0
BMP280_SCL_3V3
$Comp
L NTX2:BMP280 U6
U 1 1 5E93A3D7
P 9325 6750
F 0 "U6" H 9550 7025 50  0000 C CNN
F 1 "BMP280" H 9075 7050 50  0000 C CNN
F 2 "NTX2:BMP280" H 8925 7225 50  0001 C CNN
F 3 "" H 8925 7225 50  0001 C CNN
	1    9325 6750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5E96193B
P 9325 7075
F 0 "#PWR07" H 9325 6825 50  0001 C CNN
F 1 "GND" H 9330 6902 50  0000 C CNN
F 2 "" H 9325 7075 50  0001 C CNN
F 3 "" H 9325 7075 50  0001 C CNN
	1    9325 7075
	1    0    0    -1  
$EndComp
Text Notes 8075 7625 0    50   ~ 0
CSB -> Chip Select Pull up for I2C, pull down for SPI\nSDO -> Conn to GND gives 0x76 adress \nand conn (don't leave floating) to VCC gives 0x77 address
Text GLabel 9725 6675 2    50   Input ~ 0
3V3
$Comp
L power:GND #PWR08
U 1 1 5E962B47
P 9775 6875
F 0 "#PWR08" H 9775 6625 50  0001 C CNN
F 1 "GND" H 9780 6702 50  0000 C CNN
F 2 "" H 9775 6875 50  0001 C CNN
F 3 "" H 9775 6875 50  0001 C CNN
	1    9775 6875
	1    0    0    -1  
$EndComp
Wire Wire Line
	9775 6875 9775 6825
Wire Wire Line
	9775 6825 9725 6825
NoConn ~ 11350 5325
NoConn ~ 11350 5425
Text GLabel 6550 8125 0    50   Input ~ 0
3V3
$Comp
L power:GND #PWR09
U 1 1 5E9744BD
P 6475 8425
F 0 "#PWR09" H 6475 8175 50  0001 C CNN
F 1 "GND" H 6480 8252 50  0000 C CNN
F 2 "" H 6475 8425 50  0001 C CNN
F 3 "" H 6475 8425 50  0001 C CNN
	1    6475 8425
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 8375 6475 8375
Wire Wire Line
	6475 8375 6475 8425
$Comp
L Transistor_FET:BSS138 Q6
U 1 1 5E978525
P 4450 6975
F 0 "Q6" V 4700 6975 50  0000 C CNN
F 1 "BSS138" V 4791 6975 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4650 6900 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 4450 6975 50  0001 L CNN
	1    4450 6975
	0    1    1    0   
$EndComp
$Comp
L Device:R R15
U 1 1 5E97852C
P 4150 6825
F 0 "R15" H 4220 6871 50  0000 L CNN
F 1 "10K" H 4220 6780 50  0000 L CNN
F 2 "" V 4080 6825 50  0001 C CNN
F 3 "~" H 4150 6825 50  0001 C CNN
	1    4150 6825
	1    0    0    -1  
$EndComp
$Comp
L Device:R R17
U 1 1 5E978533
P 4750 6825
F 0 "R17" H 4820 6871 50  0000 L CNN
F 1 "10K" H 4820 6780 50  0000 L CNN
F 2 "" V 4680 6825 50  0001 C CNN
F 3 "~" H 4750 6825 50  0001 C CNN
	1    4750 6825
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 7075 4150 7075
Wire Wire Line
	4150 6975 4150 7075
Wire Wire Line
	4650 7075 4750 7075
Wire Wire Line
	4750 6975 4750 7075
Connection ~ 4150 7075
Wire Wire Line
	3950 7075 4150 7075
Wire Wire Line
	4150 6675 4150 6575
Wire Wire Line
	4450 6775 4450 6575
Wire Wire Line
	4450 6575 4150 6575
Connection ~ 4150 6575
Wire Wire Line
	4150 6575 4150 6475
Text GLabel 4150 6475 1    50   Input ~ 0
3V3
Text GLabel 4750 6425 1    50   Input ~ 0
5V
Wire Wire Line
	4750 6425 4750 6675
Text GLabel 3950 7075 0    50   BiDi ~ 0
BH1750_SCL_3V3
Text Notes 4925 6400 0    50   ~ 0
BH1750 SCL Shifter
$Comp
L Transistor_FET:BSS138 Q5
U 1 1 5E97AE47
P 4425 8100
F 0 "Q5" V 4675 8100 50  0000 C CNN
F 1 "BSS138" V 4766 8100 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4625 8025 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 4425 8100 50  0001 L CNN
	1    4425 8100
	0    1    1    0   
$EndComp
$Comp
L Device:R R14
U 1 1 5E97AE4E
P 4125 7950
F 0 "R14" H 4195 7996 50  0000 L CNN
F 1 "10K" H 4195 7905 50  0000 L CNN
F 2 "" V 4055 7950 50  0001 C CNN
F 3 "~" H 4125 7950 50  0001 C CNN
	1    4125 7950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 5E97AE55
P 4725 7950
F 0 "R16" H 4795 7996 50  0000 L CNN
F 1 "10K" H 4795 7905 50  0000 L CNN
F 2 "" V 4655 7950 50  0001 C CNN
F 3 "~" H 4725 7950 50  0001 C CNN
	1    4725 7950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4225 8200 4125 8200
Wire Wire Line
	4125 8100 4125 8200
Wire Wire Line
	4625 8200 4725 8200
Wire Wire Line
	4725 8100 4725 8200
Connection ~ 4125 8200
Wire Wire Line
	3925 8200 4125 8200
Wire Wire Line
	4125 7800 4125 7700
Wire Wire Line
	4425 7900 4425 7700
Wire Wire Line
	4425 7700 4125 7700
Connection ~ 4125 7700
Wire Wire Line
	4125 7700 4125 7600
Text GLabel 4125 7600 1    50   Input ~ 0
3V3
Text GLabel 4725 7550 1    50   Input ~ 0
5V
Wire Wire Line
	4725 7550 4725 7800
Text GLabel 3925 8200 0    50   BiDi ~ 0
BH1750_SDA_3V3
Text Notes 4900 7525 0    50   ~ 0
BH1750 SDA Shifter
Entry Wire Line
	5800 8200 5900 8300
Wire Wire Line
	5800 8200 4725 8200
Connection ~ 4725 8200
Entry Wire Line
	5750 7075 5850 7175
Wire Wire Line
	5750 7075 4750 7075
Connection ~ 4750 7075
Entry Wire Line
	5800 6000 5900 6100
Text GLabel 7250 8125 2    50   BiDi ~ 0
BH1750_SCL_3V3
Text GLabel 7250 8375 2    50   BiDi ~ 0
BH1750_SDA_3V3
Text Notes 6325 9025 0    50   ~ 0
ADDR -> Conn to GND for 0x23 addr \nor conn to Vcc (3v3) for 0x5C
Wire Wire Line
	8375 5400 8375 5500
Wire Wire Line
	8375 5300 8375 5400
Connection ~ 8375 5400
Wire Wire Line
	8575 5400 8375 5400
Wire Wire Line
	8575 5300 8575 5400
Text GLabel 11150 7300 0    50   BiDi ~ 0
Ublox­_SCL_3V3
Text Notes 6700 8275 0    50   ~ 0
Light Sens
Text Notes 6725 6850 0    50   ~ 0
UV Sens
Text GLabel 9325 6425 1    50   Input ~ 0
3V3
Text GLabel 6525 6925 0    50   Input ~ 0
3V3
NoConn ~ 6525 6725
Text GLabel 7225 6925 2    50   Input ~ 0
3V3
$Comp
L power:GND #PWR010
U 1 1 5EA8210C
P 6875 7175
F 0 "#PWR010" H 6875 6925 50  0001 C CNN
F 1 "GND" H 6880 7002 50  0000 C CNN
F 2 "" H 6875 7175 50  0001 C CNN
F 3 "" H 6875 7175 50  0001 C CNN
	1    6875 7175
	1    0    0    -1  
$EndComp
Text Notes 6575 7500 0    50   ~ 0
VIN is not used
Text Label 7225 6725 0    50   ~ 0
UV_OUT
Text Label 10350 4225 2    50   ~ 0
UV_OUT
$Comp
L power:GND #PWR011
U 1 1 5EA86519
P 6900 8625
F 0 "#PWR011" H 6900 8375 50  0001 C CNN
F 1 "GND" H 6905 8452 50  0000 C CNN
F 2 "" H 6900 8625 50  0001 C CNN
F 3 "" H 6900 8625 50  0001 C CNN
	1    6900 8625
	1    0    0    -1  
$EndComp
Text GLabel 8875 8050 0    50   Input ~ 0
5V
$Comp
L power:GND #PWR012
U 1 1 5EA968B2
P 8875 8150
F 0 "#PWR012" H 8875 7900 50  0001 C CNN
F 1 "GND" V 8880 8022 50  0000 R CNN
F 2 "" H 8875 8150 50  0001 C CNN
F 3 "" H 8875 8150 50  0001 C CNN
	1    8875 8150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5EAB3B53
P 10025 8075
F 0 "#PWR013" H 10025 7825 50  0001 C CNN
F 1 "GND" H 10030 7902 50  0000 C CNN
F 2 "" H 10025 8075 50  0001 C CNN
F 3 "" H 10025 8075 50  0001 C CNN
	1    10025 8075
	1    0    0    -1  
$EndComp
Wire Wire Line
	10025 8075 10025 8050
Wire Wire Line
	10025 8050 9775 8050
Text Notes 8100 9400 0    50   ~ 0
VCC -> 5V bcse there is 3.3V reg on breakout board\nGND -> GND\nSCL -> I2C Clock\nSDA -> I2C Data\nEDA -> External\nECL -> External Clock\nADO -> Select I2C Address (conn to 5V for 0x69, GND for 0x68)\nINT -> Interrupt\nNCS -> SPI Chip Select\nFYNC -> Unused, conn to GND
NoConn ~ 8875 8450
NoConn ~ 9775 8450
NoConn ~ 9775 8350
NoConn ~ 9775 8250
NoConn ~ 9775 8150
$Comp
L Transistor_FET:BSS138 Q8
U 1 1 5EADBBAB
P 4450 9300
F 0 "Q8" V 4700 9300 50  0000 C CNN
F 1 "BSS138" V 4791 9300 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4650 9225 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 4450 9300 50  0001 L CNN
	1    4450 9300
	0    1    1    0   
$EndComp
$Comp
L Device:R R19
U 1 1 5EADBBB2
P 4150 9150
F 0 "R19" H 4220 9196 50  0000 L CNN
F 1 "10K" H 4220 9105 50  0000 L CNN
F 2 "" V 4080 9150 50  0001 C CNN
F 3 "~" H 4150 9150 50  0001 C CNN
	1    4150 9150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R21
U 1 1 5EADBBB9
P 4750 9150
F 0 "R21" H 4820 9196 50  0000 L CNN
F 1 "10K" H 4820 9105 50  0000 L CNN
F 2 "" V 4680 9150 50  0001 C CNN
F 3 "~" H 4750 9150 50  0001 C CNN
	1    4750 9150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 9400 4150 9400
Wire Wire Line
	4150 9300 4150 9400
Wire Wire Line
	4650 9400 4750 9400
Wire Wire Line
	4750 9300 4750 9400
Connection ~ 4150 9400
Wire Wire Line
	3950 9400 4150 9400
Wire Wire Line
	4150 9000 4150 8900
Wire Wire Line
	4450 9100 4450 8900
Wire Wire Line
	4450 8900 4150 8900
Connection ~ 4150 8900
Wire Wire Line
	4150 8900 4150 8800
Text GLabel 4150 8800 1    50   Input ~ 0
3V3
Text GLabel 4750 8750 1    50   Input ~ 0
5V
Wire Wire Line
	4750 8750 4750 9000
Text GLabel 3950 9400 0    50   BiDi ~ 0
MPU9250_SCL_3V3
Text Notes 4925 8725 0    50   ~ 0
MPU9250 SCL Shifter
$Comp
L Transistor_FET:BSS138 Q7
U 1 1 5EADBBD0
P 4425 10425
F 0 "Q7" V 4675 10425 50  0000 C CNN
F 1 "BSS138" V 4766 10425 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4625 10350 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 4425 10425 50  0001 L CNN
	1    4425 10425
	0    1    1    0   
$EndComp
$Comp
L Device:R R18
U 1 1 5EADBBD7
P 4125 10275
F 0 "R18" H 4195 10321 50  0000 L CNN
F 1 "10K" H 4195 10230 50  0000 L CNN
F 2 "" V 4055 10275 50  0001 C CNN
F 3 "~" H 4125 10275 50  0001 C CNN
	1    4125 10275
	1    0    0    -1  
$EndComp
$Comp
L Device:R R20
U 1 1 5EADBBDE
P 4725 10275
F 0 "R20" H 4795 10321 50  0000 L CNN
F 1 "10K" H 4795 10230 50  0000 L CNN
F 2 "" V 4655 10275 50  0001 C CNN
F 3 "~" H 4725 10275 50  0001 C CNN
	1    4725 10275
	1    0    0    -1  
$EndComp
Wire Wire Line
	4225 10525 4125 10525
Wire Wire Line
	4125 10425 4125 10525
Wire Wire Line
	4625 10525 4725 10525
Wire Wire Line
	4725 10425 4725 10525
Connection ~ 4125 10525
Wire Wire Line
	3925 10525 4125 10525
Wire Wire Line
	4125 10125 4125 10025
Wire Wire Line
	4425 10225 4425 10025
Wire Wire Line
	4425 10025 4125 10025
Connection ~ 4125 10025
Wire Wire Line
	4125 10025 4125 9925
Text GLabel 4125 9925 1    50   Input ~ 0
3V3
Text GLabel 4725 9875 1    50   Input ~ 0
5V
Wire Wire Line
	4725 9875 4725 10125
Text GLabel 3925 10525 0    50   BiDi ~ 0
MPU9250_SDA_3V3
Text Notes 4900 9850 0    50   ~ 0
MPU9250 SDA Shifter
Wire Wire Line
	5800 10525 4725 10525
Connection ~ 4725 10525
Wire Wire Line
	5750 9400 4750 9400
Connection ~ 4750 9400
Text Notes 5725 10775 0    50   ~ 0
SCL SDA
Entry Wire Line
	5750 9400 5850 9500
Entry Wire Line
	5800 10525 5900 10625
Text GLabel 8875 8250 0    50   BiDi ~ 0
MPU9250_SCL_3V3
$Comp
L NTX2:MPU_9250 U4
U 1 1 5EA92A1E
P 9325 8250
F 0 "U4" H 9325 8715 50  0000 C CNN
F 1 "MPU_9250" H 9325 8624 50  0000 C CNN
F 2 "NTX2:MPU_9250" H 8875 8550 50  0001 C CNN
F 3 "" H 8875 8550 50  0001 C CNN
	1    9325 8250
	1    0    0    -1  
$EndComp
Text GLabel 8875 8250 0    50   BiDi ~ 0
MPU9250_SCL_3V3
Text GLabel 8875 8350 0    50   BiDi ~ 0
MPU9250_SDA_3V3
$Comp
L Device:R R22
U 1 1 5EB1634F
P 7000 9550
F 0 "R22" H 6930 9504 50  0000 R CNN
F 1 "30K" H 6930 9595 50  0000 R CNN
F 2 "" V 6930 9550 50  0001 C CNN
F 3 "~" H 7000 9550 50  0001 C CNN
	1    7000 9550
	-1   0    0    1   
$EndComp
$Comp
L Device:R R23
U 1 1 5EB16434
P 7000 9975
F 0 "R23" H 7070 10021 50  0000 L CNN
F 1 "7.5K" H 7070 9930 50  0000 L CNN
F 2 "" V 6930 9975 50  0001 C CNN
F 3 "~" H 7000 9975 50  0001 C CNN
	1    7000 9975
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 9700 7000 9775
$Comp
L power:GND #PWR014
U 1 1 5EB21381
P 7000 10225
F 0 "#PWR014" H 7000 9975 50  0001 C CNN
F 1 "GND" H 7005 10052 50  0000 C CNN
F 2 "" H 7000 10225 50  0001 C CNN
F 3 "" H 7000 10225 50  0001 C CNN
	1    7000 10225
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 10125 7000 10225
Text Label 7125 9775 0    50   ~ 0
V_S_O
Connection ~ 7000 9775
Wire Wire Line
	7000 9775 7000 9825
Wire Wire Line
	7000 9775 7125 9775
Text Label 11350 4625 0    50   ~ 0
V_S_O
Wire Wire Line
	10850 3275 10850 3250
Connection ~ 10850 3250
Wire Wire Line
	10850 3250 10850 3225
Wire Bus Line
	5850 10675 5725 10675
Wire Bus Line
	5900 10675 6025 10675
Text Label 10700 3425 2    50   ~ 0
V_S_I
Wire Wire Line
	10750 3225 10750 3425
Wire Wire Line
	10700 3425 10750 3425
Connection ~ 10750 3425
Wire Wire Line
	10750 3425 10750 3625
Text Label 6925 9375 2    50   ~ 0
V_S_I
Wire Wire Line
	6925 9375 7000 9375
Wire Wire Line
	7000 9375 7000 9400
Text Notes 6425 10825 0    50   ~ 0
Resistive voltage sensor using potenial divider\nconnected to battery source. \nInput (V_S_I) conn to VIN terminals\nOutput (V_S_O) conn to A0 of MC
NoConn ~ 10350 4325
NoConn ~ 10350 4425
NoConn ~ 10350 4525
NoConn ~ 10350 4625
NoConn ~ 10350 4725
NoConn ~ 10350 4825
NoConn ~ 10350 4925
NoConn ~ 10350 5025
NoConn ~ 10350 5125
NoConn ~ 10350 5225
NoConn ~ 11350 4025
NoConn ~ 11350 4225
NoConn ~ 11350 4425
NoConn ~ 11350 4725
NoConn ~ 11350 4825
NoConn ~ 11350 4925
NoConn ~ 11600 6850
NoConn ~ 11800 6850
NoConn ~ 6875 5300
NoConn ~ 7075 5300
NoConn ~ 7275 5300
Text Label 5575 4625 2    50   ~ 0
I2C_SCL
Text Label 5600 5850 2    50   ~ 0
I2C_SDA
Text Label 11350 5025 0    50   ~ 0
I2C_SDA
Text Label 11350 5125 0    50   ~ 0
I2C_SCL
Text Notes 6700 5675 0    50   ~ 0
Had to connect I2C_SCL and I2C_SDA \nlike this to avoid no connect errors!
Entry Wire Line
	5750 2775 5850 2875
Wire Wire Line
	4650 2775 5750 2775
Entry Wire Line
	5800 3775 5900 3875
Wire Wire Line
	4650 3775 5800 3775
Entry Wire Line
	5750 4825 5850 4925
Wire Wire Line
	4675 4825 5575 4825
Wire Bus Line
	5850 2075 5700 2075
Wire Bus Line
	5900 2075 6050 2075
Text Notes 5725 2050 0    50   ~ 0
SCL SDA
Wire Wire Line
	5600 5850 5600 6000
Wire Wire Line
	4775 6000 5600 6000
Connection ~ 5600 6000
Wire Wire Line
	5600 6000 5800 6000
Wire Wire Line
	5575 4625 5575 4825
Connection ~ 5575 4825
Wire Wire Line
	5575 4825 5750 4825
Wire Bus Line
	5900 2075 5900 10675
Wire Bus Line
	5850 2075 5850 10675
$EndSCHEMATC
