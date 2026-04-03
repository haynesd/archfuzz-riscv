# ============================================================================
# File        : fpga_fuzz_ctrl.xdc
# Target      : Digilent Nexys A7-100T (XC7A100T-1CSG324C)
#
# DESCRIPTION
# ----------------------------------------------------------------------------
# Constraints for:
#   - System clock
#   - Reset button
#   - Host UART (JA)
#   - Board UARTs (JB/JC/JD)
#   - Trigger output
#   - Debug LEDs
#
# NOTE
# ----------------------------------------------------------------------------
# Verify LED pin mapping against your Nexys A7 master XDC if needed.
# ============================================================================

# ============================================================================
# CLOCK (100 MHz onboard oscillator)
# ============================================================================
set_property PACKAGE_PIN E3 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -name sys_clk -period 10.000 [get_ports clk]

# ============================================================================
# RESET (CPU_RESETN, active low pushbutton)
# ============================================================================
set_property PACKAGE_PIN C12 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property PULLUP true [get_ports rst_n]

# ============================================================================
# PMOD JA - Host UART + Trigger
# ============================================================================
# JA1 (C17) → FPGA TX → FT232 RX
set_property PACKAGE_PIN C17 [get_ports uart_host_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_host_tx]

# JA2 (D18) → FPGA RX ← FT232 TX
set_property PACKAGE_PIN D18 [get_ports uart_host_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_host_rx]

# JA9 (F18) → Trigger output (oscilloscope)
set_property PACKAGE_PIN F18 [get_ports trig_out]
set_property IOSTANDARD LVCMOS33 [get_ports trig_out]

# ============================================================================
# PMOD JB - Board 0 UART
# ============================================================================
# JB1 (D14) → FPGA TX → Board 0 RX
set_property PACKAGE_PIN D14 [get_ports uart0_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart0_tx]

# JB2 (F16) → FPGA RX ← Board 0 TX
set_property PACKAGE_PIN F16 [get_ports uart0_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart0_rx]

# ============================================================================
# PMOD JC - Board 1 UART
# ============================================================================
# JC1 (K1) → FPGA TX → Board 1 RX
set_property PACKAGE_PIN K1 [get_ports uart1_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart1_tx]

# JC2 (F6) → FPGA RX ← Board 1 TX
set_property PACKAGE_PIN F6 [get_ports uart1_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart1_rx]

# ============================================================================
# PMOD JD - Board 2 UART
# ============================================================================
# JD1 (H4) → FPGA TX → Board 2 RX
set_property PACKAGE_PIN H4 [get_ports uart2_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart2_tx]

# JD2 (H1) → FPGA RX ← Board 2 TX
set_property PACKAGE_PIN H1 [get_ports uart2_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart2_rx]

# ============================================================================
# ONBOARD LEDs (LD0-LD3)
# ============================================================================
# LED0 → Host RX activity
set_property PACKAGE_PIN H17 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]

# LED1 → Host TX activity
set_property PACKAGE_PIN K15 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]

# LED2 → Board RX activity
set_property PACKAGE_PIN J13 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]

# LED3 → Board TX activity
set_property PACKAGE_PIN N14 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]