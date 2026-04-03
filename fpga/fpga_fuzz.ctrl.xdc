# ============================================================================
# File        : fpga_fuzz_ctrl.xdc
# Target      : Digilent Nexys A7-100T (XC7A100T-1CSG324C)
#
# Clock:
#   clk   -> E3   (100 MHz onboard oscillator)
#
# Reset:
#   rst_n -> C12  (CPU_RESETN, active low)
#
# PMOD JA = Host UART + Trigger
#   JA1 / C17 -> uart_host_tx
#   JA2 / D18 -> uart_host_rx
#   JA9 / F18 -> trig_out
#
# PMOD JB = Board 0 UART
#   JB1 / D14 -> uart0_tx
#   JB2 / F16 -> uart0_rx
#
# PMOD JC = Board 1 UART
#   JC1 / K1  -> uart1_tx
#   JC2 / F6  -> uart1_rx
#
# PMOD JD = Board 2 UART
#   JD1 / H4  -> uart2_tx
#   JD2 / H1  -> uart2_rx
# ============================================================================

## Clock
set_property PACKAGE_PIN E3 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -name sys_clk_pin -period 10.000 [get_ports clk]

## Reset (active low)
set_property PACKAGE_PIN C12 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property PULLUP true [get_ports rst_n]

## JA: Host UART + Trigger
set_property PACKAGE_PIN C17 [get_ports uart_host_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_host_tx]

set_property PACKAGE_PIN D18 [get_ports uart_host_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_host_rx]

set_property PACKAGE_PIN F18 [get_ports trig_out]
set_property IOSTANDARD LVCMOS33 [get_ports trig_out]

## JB: Board 0 UART
set_property PACKAGE_PIN D14 [get_ports uart0_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart0_tx]

set_property PACKAGE_PIN F16 [get_ports uart0_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart0_rx]

## JC: Board 1 UART
set_property PACKAGE_PIN K1 [get_ports uart1_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart1_tx]

set_property PACKAGE_PIN F6 [get_ports uart1_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart1_rx]

## JD: Board 2 UART
set_property PACKAGE_PIN H4 [get_ports uart2_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart2_tx]

set_property PACKAGE_PIN H1 [get_ports uart2_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart2_rx]