set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } [get_ports {clock}];

#DAC (Output)

set_property -dict { PACKAGE_PIN H14 IOSTANDARD LVCMOS33 } [get_ports { sclk_dac }]; #IO_L21N_T3_DQS_A18_15 Sch=ja[2]

set_property -dict { PACKAGE_PIN G16 IOSTANDARD LVCMOS33 } [get_ports { ldac }]; #IO_L21P_T3_DQS_15 Sch=ja[3]

set_property -dict { PACKAGE_PIN F16 IOSTANDARD LVCMOS33 } [get_ports { sd_tx }]; #IO_L18N_T2_A23_15 Sch=ja[4]

set_property -dict { PACKAGE_PIN D14 IOSTANDARD LVCMOS33 } [get_ports { cs }]; #IO_L20N_T3_A19_15 Sch=ja[1]

set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports {reset_n}];

#DAC2 (Output)

set_property -dict { PACKAGE_PIN G17 IOSTANDARD LVCMOS33 } [get_ports { sclk_dac1 }]; #IO_L21N_T3_DQS_A18_15 Sch=ja[2]

set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports { ldac1 }]; #IO_L21P_T3_DQS_15 Sch=ja[3]

set_property -dict { PACKAGE_PIN D18 IOSTANDARD LVCMOS33 } [get_ports { sd_tx1 }]; #IO_L18N_T2_A23_15 Sch=ja[4]

set_property -dict { PACKAGE_PIN C17 IOSTANDARD LVCMOS33 } [get_ports { cs1 }]; #IO_L20N_T3_A19_15 Sch=ja[1]
