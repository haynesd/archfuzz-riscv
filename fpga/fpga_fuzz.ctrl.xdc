## ============================================================================
## fpga_fuzz_ctrl.xdc
## Target: Digilent Nexys A7
## I/O Standard: LVCMOS33
##
## UART mapping:
##   UART0 -> PMOD JA
##   UART1 -> PMOD JB
##   UART2 -> PMOD JC
##   UART3 -> PMOD JD
##   TRIG  -> PMOD JD3
##
## IMPORTANT:
## - All UART lines must be 3.3V logic.
## - Share GND between FPGA, FT232, and all boards.
## - Do NOT connect FT232 VCC to FPGA I/O.
## ============================================================================

## ----------------------------------------------------------------------------
## System Clock
## Nexys A7 100 MHz oscillator
## ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN E3  IOSTANDARD LVCMOS33 } [get_ports { clk }]
create_clock -add -name sys_clk -period 10.000 [get_ports { clk }]

## ----------------------------------------------------------------------------
## Reset
## Example: map rst_n to CPU_RESETN button if your top-level uses active-low reset
## CPU_RESETN on Nexys A7 is pin C12
## ----------------------------------------------------------------------------
set_property -dict { PACKAGE_PIN C12 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { rst_n }]

## ============================================================================
## UART0 -> Board 0  (PMOD JA)
## JA1 = C17
## JA2 = D18
## ============================================================================
set_property -dict { PACKAGE_PIN C17 IOSTANDARD LVCMOS33 } [get_ports { uart0_tx }]
set_property -dict { PACKAGE_PIN D18 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { uart0_rx }]

## ============================================================================
## UART1 -> Board 1  (PMOD JB)
## JB1 = D14
## JB2 = F16
## ============================================================================
set_property -dict { PACKAGE_PIN D14 IOSTANDARD LVCMOS33 } [get_ports { uart1_tx }]
set_property -dict { PACKAGE_PIN F16 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { uart1_rx }]

## ============================================================================
## UART2 -> Board 2  (PMOD JC)
## JC1 = K1
## JC2 = F6
## ============================================================================
set_property -dict { PACKAGE_PIN K1 IOSTANDARD LVCMOS33 } [get_ports { uart2_tx }]
set_property -dict { PACKAGE_PIN F6 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { uart2_rx }]

## ============================================================================
## UART3 -> Host FT232  (PMOD JD)
## JD1 = H4
## JD2 = H1
##
## Wiring:
##   FT232 TXD -> uart3_rx
##   FT232 RXD <- uart3_tx
## ============================================================================
set_property -dict { PACKAGE_PIN H4 IOSTANDARD LVCMOS33 } [get_ports { uart3_tx }]
set_property -dict { PACKAGE_PIN H1 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { uart3_rx }]

## ============================================================================
## Trigger Output -> Oscilloscope CH4
## JD3 = G1
## ============================================================================
set_property -dict { PACKAGE_PIN G1 IOSTANDARD LVCMOS33 DRIVE 8 SLEW FAST } [get_ports { trig_out }]