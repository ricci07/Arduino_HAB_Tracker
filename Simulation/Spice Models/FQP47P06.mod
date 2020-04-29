**************** Power Discrete MOSFET Electrical Circuit Model *****************
** Product Name: FQPF47P06
** 60V P-Channel MOSFET and TO-220F
**-------------------------------------------------------------------------------
.SUBCKT FQPF47P06 20 10 30
Rg 10 1 1.58
M1 2 1 3 3 DMOS L=1u W=1u
.MODEL DMOS PMOS (VTO={-3.75*{-0.00084*TEMP+1.021}} KP={-0.0012*TEMP+19}
+ THETA=0.0576 VMAX=3.5E5 LEVEL=3)
Cgs 1 3 1900p
Rd 20 4 0.0005 TC=0.0063
Dds 4 3 DDS
.MODEL DDS D(BV={60*{0.001*TEMP+0.975}} M=0.57 CJO=5080p VJ=0.58)
Dbody 20 3 DBODY
.MODEL DBODY D(IS=7.5E-09 N=1.5 RS=0.042 EG=1.13 TT=130n)
Ra 4 2 0.0005 TC=0.0063
Rs 3 5 0.0086
Ls 5 30 1.0n
M2 1 8 6 6 INTER
E2 8 6 4 1 2
.MODEL INTER PMOS (VTO=0 KP=10 LEVEL=1)
Cgdmax 7 4 6240p
Rcgd 7 4 1E7
Dgd 4 6 DGD
Rdgd 4 6 1E7
.MODEL DGD D(M=0.32 CJO=6240p VJ=0.0112)
M3 7 9 1 1 INTER
E3 9 1 4 1 -2
.ENDS
**------------------------------------------------------------------------------

********************* Power Discrete MOSFET Thermal Model **********************
.SUBCKT FQPF47P06_Thermal TH TL
CTHERM1 TH 6 1.0e-3
CTHERM2 6 5 6.6e-3
CTHERM3 5 4 4.8e-2
CTHERM4 4 3 1.8e-1
CTHERM5 3 2 5.8e-1
CTHERM6 2 TL 6.4e-1

RTHERM1 TH 6 8.6e-3
RTHERM2 6 5 5.9e-2
RTHERM3 5 4 8.2e-2
RTHERM4 4 3 4.6e-1
RTHERM5 3 2 8.4e-1
RTHERM6 2 TL 9.7e-1
.ENDS
**------------------------------------------------------------------------------
** Creation: Apr.-24-2007 Rev: 1.0
** Fairchild Semiconductor