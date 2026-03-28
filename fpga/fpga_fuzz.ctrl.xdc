###############################################################################
## fpga_fuzz_ctrl.xdc
## Target: Digilent Nexys A7-100T
## Design: fpga_fuzz_ctrl
##
## UART Mapping
##   UART0 -> PMOD JA  (Board 0)
##   UART1 -> PMOD JB  (Board 1)
##   UART2 -> PMOD JC  (Board 2)
##   UART3 -> PMOD JD  (Host FT232 TTL)
##   trig_out -> PMOD JD3
##
## IMPORTANT
## - All UART signals are 3.3V TTL.
## - FT232 TX must go to FPGA uart3_rx.
## - FT232 RX must go to FPGA uart3_tx.
## - All grounds must be common.
###############################################################################

## ============================================================================
## 100 MHz system clock
## ============================================================================
set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } [get_ports { clk }]
create_clock -add -name sys_clk -period 10.000 -waveform {0 5} [get_ports { clk }]

## ============================================================================
## Active-low reset
## CPU_RESETN button on Nexys A7
## ============================================================================
set_property -dict { PACKAGE_PIN C12 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { rst_n }]

## ============================================================================
## UART0 -> Board 0 on PMOD JA
## JA1 = C17
## JA2 = D18
##
## Wiring:
##   uart0_tx -> Board0 RX
##   uart0_rx <- Board0 TX
## ============================================================================
set_property -dict { PACKAGE_PIN C17 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 } [get_ports { uart0_tx }]
set_property -dict { PACKAGE_PIN D18 IOSTANDARD LVCMOS33 PULLUP true }       [get_ports { uart0_rx }]

## ============================================================================
## UART1 -> Board 1 on PMOD JB
## JB1 = D14
## JB2 = F16
##
## Wiring:
##   uart1_tx -> Board1 RX
##   uart1_rx <- Board1 TX
## ============================================================================
set_property -dict { PACKAGE_PIN D14 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 } [get_ports { uart1_tx }]
set_property -dict { PACKAGE_PIN F16 IOSTANDARD LVCMOS33 PULLUP true }       [get_ports { uart1_rx }]

## ============================================================================
## UART2 -> Board 2 on PMOD JC
## JC1 = K1
## JC2 = F6
##
## Wiring:
##   uart2_tx -> Board2 RX
##   uart2_rx <- Board2 TX
## ============================================================================
set_property -dict { PACKAGE_PIN K1 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 } [get_ports { uart2_tx }]
set_property -dict { PACKAGE_PIN F6 IOSTANDARD LVCMOS33 PULLUP true }       [get_ports { uart2_rx }]

## ============================================================================
## UART3 -> Host FT232 on PMOD JD
## JD1 = H4
## JD2 = H1
##
## Wiring:
##   FT232 RXD <- uart3_tx
##   FT232 TXD -> uart3_rx
## ============================================================================
set_property -dict { PACKAGE_PIN H4 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 } [get_ports { uart3_tx }]
set_property -dict { PACKAGE_PIN H1 IOSTANDARD LVCMOS33 PULLUP true }       [get_ports { uart3_rx }]

## ============================================================================
## Trigger output -> Oscilloscope CH4
## JD3 = G1
## ============================================================================
set_property -dict { PACKAGE_PIN G1 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 8 } [get_ports { trig_out }]