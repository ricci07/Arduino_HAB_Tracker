#include <EditConstants.au3>
#include <GUIConstantsEx.au3>
#include <StaticConstants.au3>
#include <WindowsConstants.au3>

#Region ### START Koda GUI section ### Form=
$Form1 = GUICreate("NTX2B Resistor Calculator", 483, 265, 203, 140)
$tbx_r1 = GUICtrlCreateInput("4700", 40, 24, 121, 21, $ES_NUMBER)
$lbl_r1 = GUICtrlCreateLabel("R1", 15, 26, 18, 17)
$tbx_r2 = GUICtrlCreateInput("4700", 40, 47, 121, 21, $ES_NUMBER)
$lbl_r2 = GUICtrlCreateLabel("R2", 15, 49, 18, 17)
$tbx_r3 = GUICtrlCreateInput("47000", 40, 71, 121, 21, $ES_NUMBER)
$lbl_r3 = GUICtrlCreateLabel("R3", 15, 73, 18, 17)
$lbl_txhigh = GUICtrlCreateLabel("Tx High:", 15, 120, 88, 17)
$lbl_txlow = GUICtrlCreateLabel("Tx Low:", 15, 132, 88, 17)
$lbl_vdiff = GUICtrlCreateLabel("Voltage Difference:", 15, 147, 190, 17)
$lbl_fshift = GUICtrlCreateLabel("Frequency Shift:", 15, 162, 160, 17)
$Pic1 = GUICtrlCreatePic("C:\Users\Ric\Documents\ShareX\Screenshots\2020-04\kicad_2020-04-16_16-00-30.jpg", 168, 16, 296, 229, BitOR($GUI_SS_DEFAULT_PIC,$WS_BORDER))
$lbl_author = GUICtrlCreateLabel("by Riccardo Geraci", 16, 192, 95, 17)
$btn_calc = GUICtrlCreateButton("Calculate", 25, 220, 75, 25)
$lbl_r1_img = GUICtrlCreateLabel("R1= ", 408, 136, 48, 18)
GUICtrlSetBkColor(-1, 0xFFFFFF)
$lbl_r3_img = GUICtrlCreateLabel("R3 = ", 240, 216, 27, 17)
GUICtrlSetBkColor(-1, 0xFFFFFF)
$lbl_r2_img = GUICtrlCreateLabel("R2 = ", 344, 144, 62, 17)
GUICtrlSetBkColor(-1, 0xFFFFFF)
GUISetState(@SW_SHOW)
#EndRegion ### END Koda GUI section ###
While 1
	$nMsg = GUIGetMsg()
	Switch $nMsg
		Case $GUI_EVENT_CLOSE
			Exit
		case $btn_calc

			local $r1 = GUICtrlRead($tbx_r1)
			local $r2 = GUICtrlRead($tbx_r2)
			local $r3 = GUICtrlRead($tbx_r3)

			local $R3pR1 = ($r1*$r3)/($r1+$r3)
			local $R3pR2 = ($r2*$r3)/($r2+$r3)

			local $txHighRes = ($r2/($r2+$R3pR1))*5
			local $txLowRes = ($R3pR2/($R3pR2+$R1))*5
			local $vDiff = $txHighRes - $txLowRes
			local $fShift = $vDiff *2000

			GUICtrlSetData($lbl_txhigh, "Tx High: " & round($txHighRes,2))
			GUICtrlSetData($lbl_txlow, "Tx Low: " & round($txLowRes,2))
			GUICtrlSetData($lbl_vdiff, "Voltage Difference: " & round($vDiff,2))
			GUICtrlSetData($lbl_fshift, "Frequency Shift: " & round($fShift,2) & " Hz")

		case $
	EndSwitch
WEnd