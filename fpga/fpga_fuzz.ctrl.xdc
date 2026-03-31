# ============================================================================
# File        : fpga_fuzz_ctrl.xdc
# Target      : Digilent Nexys A7 (Artix-7)
#
# TITLE
# ----------------------------------------------------------------------------
# Pin Constraints for Multi-UART Fuzzing Controller
#
# DESCRIPTION
# ----------------------------------------------------------------------------
# Defines clock, UART, and trigger pin mappings for FPGA fuzzing lab.
#
# PMOD MAPPING
# ----------------------------------------------------------------------------
# JA -> Host (FT232)
# JB -> Board 0
# JC -> Board 1
# JD -> Board 2
#
# ============================================================================
## Clock
set_property PACKAGE_PIN E3 [get_ports clk]
create_clock -period 10.0 [get_ports clk]

## Host UART (JA)
set_property PACKAGE_PIN C17 [get_ports uart_host_tx]
set_property PACKAGE_PIN D18 [get_ports uart_host_rx]

## JB -> Board 0
set_property PACKAGE_PIN E18 [get_ports uart0_tx]
set_property PACKAGE_PIN F18 [get_ports uart0_rx]

## JC -> Board 1
set_property PACKAGE_PIN G17 [get_ports uart1_tx]
set_property PACKAGE_PIN G18 [get_ports uart1_rx]

## JD -> Board 2
set_property PACKAGE_PIN H17 [get_ports uart2_tx]
set_property PACKAGE_PIN H18 [get_ports uart2_rx]

## Trigger
set_property PACKAGE_PIN J17 [get_ports trig_out]