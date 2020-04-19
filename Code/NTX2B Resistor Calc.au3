#Region ;**** Directives created by AutoIt3Wrapper_GUI ****
#AutoIt3Wrapper_Icon=calc_icon_64px.ico
#AutoIt3Wrapper_Outfile_x64=Calc.Exe
#AutoIt3Wrapper_Res_Comment=A very simple program to calculator the resistance required to achieve a certain RTTY frequency shift
#AutoIt3Wrapper_Res_Description=Radiometrix NTX2B Resistor Calculator
#AutoIt3Wrapper_Res_Fileversion=1.0.0.0
#AutoIt3Wrapper_Res_LegalCopyright=Riccardo Geraci
#EndRegion ;**** Directives created by AutoIt3Wrapper_GUI ****
;
;
;Radiometrix NTX2B Resistor Calculator
;Written in AutoIt
;Date: 16 April 2020
;Author: Riccardo Geraci / ricc07
;
;

#include <EditConstants.au3>
#include <GUIConstantsEx.au3>
#include <StaticConstants.au3>
#include <WindowsConstants.au3>

#Region ### START Koda GUI section ### Form=
$Form1 = GUICreate("Radiometrix NTX2B Frequency Shift Resistor Calculator - UKHAS", 570, 345, 203, 140)
;================input area========================
$tbx_r1 = GUICtrlCreateInput("4700", 40, 24, 75, 21, $ES_NUMBER)
$lbl_r1 = GUICtrlCreateLabel("R1", 15, 28, 15, 17)

$tbx_r2 = GUICtrlCreateInput("4700", 40, 47, 75, 21, $ES_NUMBER)
$lbl_r2 = GUICtrlCreateLabel("R2", 15, 51, 15, 17)

$tbx_r3 = GUICtrlCreateInput("47000", 40, 71, 75, 21, $ES_NUMBER)
$lbl_r3 = GUICtrlCreateLabel("R3", 15, 75, 15, 17)

$tbx_vcc = GUICtrlCreateInput("5", 40, 95, 50, 21, $ES_NUMBER)
$lbl_vcc = GUICtrlCreateLabel("Vcc", 15, 97, 20, 17)

$lbl_txhigh = GUICtrlCreateLabel("Tx High:", 15, 120, 88, 17)
$lbl_txlow = GUICtrlCreateLabel("Tx Low:", 15, 134, 88, 17)
$lbl_vdiff = GUICtrlCreateLabel("Voltage Difference:", 15, 149, 190, 17)
$lbl_fshift = GUICtrlCreateLabel("Frequency Shift:", 15, 164, 160, 17)
;===================================================

$Pic1 = GUICtrlCreatePic("C:\Users\Ric\Documents\ShareX\Screenshots\2020-04\kicad_2020-04-16_17-37-09.jpg", _
		168, 16, 382, 311, BitOR($GUI_SS_DEFAULT_PIC, $WS_BORDER))

$lbl_author = GUICtrlCreateLabel("by Riccardo Geraci", 16, 192, 95, 17)

$btn_calc = GUICtrlCreateButton("Calculate", 40, 220, 75, 25)

;================img labels========================
$lbl_r1_img = GUICtrlCreateLabel("4700" & "Ω", 460, 200, 32, 18)
GUICtrlSetBkColor(-1, 0xFFFFFF)

$lbl_r2_img = GUICtrlCreateLabel("4700" & "Ω", 390, 200, 32, 17)
GUICtrlSetBkColor(-1, 0xFFFFFF)

$lbl_r3_img = GUICtrlCreateLabel("47000" & "Ω", 285, 283, 38, 17)
GUICtrlSetBkColor(-1, 0xFFFFFF)

$lbl_vcc_img = GUICtrlCreateLabel("5V", 230, 223, 13, 17)
GUICtrlSetBkColor(-1, 0xFFFFFF)
;==================================================

GUISetState(@SW_SHOW)
#EndRegion ### END Koda GUI section ###

Global $r1_lbl_img_pos_left_default = 460 ; //var to keep track of longest label pos
Global $r1_lbl_img_pos_left_current = 460 ; //var to keep track of longest label pos



Global $r1_lbl_img_width_default = 32 ; //var to keep track of longest label pos
Global $r1_lbl_img_width_current = 32 ; //var to keep track of longest label pos

Global $r2_lbl_img_width_default = 32 ; //var to keep track of longest label pos
Global $r2_lbl_img_width_current = 32 ; //var to keep track of longest label pos

Global $r3_lbl_img_width_default = 38 ; //var to keep track of longest label pos
Global $r3_lbl_img_width_current = 38 ; //var to keep track of longest label pos

Global $vcc_lbl_img_width_current = 13
Global $vcc_lbl_img_width_default = 13


calc() ;calc on startup


While 1

	$nMsg = GUIGetMsg()

	Switch $nMsg

		Case $GUI_EVENT_CLOSE
			Exit
		Case $btn_calc

			If Not GUICtrlRead($tbx_r1) = "" And Not GUICtrlRead($tbx_r2) _
					 = "" And Not GUICtrlRead($tbx_r3) = "" _
					And Not GUICtrlRead($tbx_vcc) = "" Then

				calc()

			EndIf

		Case $tbx_r1

			$r1_lbl_img_width_current = $r1_lbl_img_width_default + 6 * (StringLen(GUICtrlRead($tbx_r1)))

			GUICtrlSetData($lbl_r1_img, GUICtrlRead($tbx_r1) & "Ω")

			GUICtrlSetPos($lbl_r1_img, -1, _
					 - 1, $r1_lbl_img_width_current)


		Case $tbx_r2

			If StringLen(GUICtrlRead($tbx_r2)) > 9 Then
				$r1_lbl_img_pos_left_current = $r1_lbl_img_pos_left_default _
						 + 6 * Mod(StringLen(GUICtrlRead($tbx_r2)), 9) ;calc how many characters after 9

				GUICtrlSetPos($lbl_r1_img, $r1_lbl_img_pos_left_current, -1, _
						 - 1, -1)
			Else
				$r1_lbl_img_pos_left_current = $r1_lbl_img_pos_left_default
				GUICtrlSetPos($lbl_r1_img, $r1_lbl_img_pos_left_current, -1, _
						 - 1, -1)

			EndIf

			$r2_lbl_img_width_current = $r2_lbl_img_width_default + 6 * (StringLen(GUICtrlRead($tbx_r2)))

			GUICtrlSetData($lbl_r2_img, GUICtrlRead($tbx_r2) & "Ω")

			GUICtrlSetPos($lbl_r2_img, -1, _
					 - 1, $r2_lbl_img_width_current)


		Case $tbx_r3


			$r3_lbl_img_width_current = $r3_lbl_img_width_default + 6 * (StringLen(GUICtrlRead($tbx_r3)))

			GUICtrlSetData($lbl_r3_img, GUICtrlRead($tbx_r3) & "Ω")

			GUICtrlSetPos($lbl_r3_img, -1, _
					 - 1, $r3_lbl_img_width_current)

		Case $tbx_vcc

			$vcc_lbl_img_width_current = $vcc_lbl_img_width_default + 6 * (StringLen(GUICtrlRead($tbx_vcc)))

			GUICtrlSetData($lbl_vcc_img, GUICtrlRead($tbx_vcc) & "V")

			GUICtrlSetPos($lbl_vcc_img, -1, _
					 - 1, $vcc_lbl_img_width_current)
	EndSwitch
WEnd

Func calc()

	;======Obtain textbox values=======

	Local $r1 = GUICtrlRead($tbx_r1)
	Local $r2 = GUICtrlRead($tbx_r2)
	Local $r3 = GUICtrlRead($tbx_r3)
	Local $vcc = GUICtrlRead($tbx_vcc)

	;===================================

	;======Voltage divider calculations=======

	Local $R3pR1 = ($r1 * $r3) / ($r1 + $r3)
	Local $R3pR2 = ($r2 * $r3) / ($r2 + $r3)

	Local $txHighRes = ($r2 / ($r2 + $R3pR1)) * $vcc
	Local $txLowRes = ($R3pR2 / ($R3pR2 + $r1)) * $vcc
	Local $vDiff = $txHighRes - $txLowRes
	Local $fShift = $vDiff * 2000

	;=========================================

	;===========Populate result labels with calc results===========

	GUICtrlSetData($lbl_txhigh, "Tx High: " & Round($txHighRes, 2) & "V")
	GUICtrlSetData($lbl_txlow, "Tx Low: " & Round($txLowRes, 2) & "V")
	GUICtrlSetData($lbl_vdiff, "Voltage Difference: " & Round($vDiff, 2) & "V")
	GUICtrlSetData($lbl_fshift, "Frequency Shift: " & Round($fShift, 2) & " Hz")

	;===============================================================

EndFunc   ;==>calc
