#ifndef _PCIE_REGISTERS_H
#define _PCIE_REGISTERS_H

#define PCIE_ATU_VIEWPORT               0x900
#define PCIE_ATU_REGION_INBOUND         (0x1 << 31)
#define PCIE_ATU_REGION_INDEX0          (0x0 << 0)
#define PCIE_ATU_REGION_INDEX1          (0x1 << 0)
#define PCIE_ATU_CR1                    0x904
#define PCIE_ATU_TYPE_MEM               (0x0 << 0)
#define PCIE_ATU_CR2                    0x908
#define PCIE_ATU_ENABLE                 (0x1 << 31)
#define PCIE_ATU_LOWER_BASE             0x90C
#define PCIE_ATU_UPPER_BASE             0x910
#define PCIE_ATU_LIMIT                  0x914
#define PCIE_ATU_LOWER_TARGET           0x918
#define PCIE_ATU_UPPER_TARGET           0x91C

#define TARGET_FPGA_ADC_CONFIG		0x00
#define TARGET_FPGA_TRIGGER		0x04
#define TARGET_FPGA_SAMPLES		0x08
#define TARGET_FPGA_ADC_ONOFF		0x0c
#define TARGET_FPGA_PAUSE_COUNTER	0x10
//#define TARGET_FPGA_DAC_CS0		0x14
//#define TARGET_FPGA_DAC_CS1		0x18
//#define TARGET_FPGA_MUX			0x1c
#define TARGET_FPGA_AFE_MODE		0x20
#define TARGET_FPGA_AFE_STATUS		0x24
#define TARGET_FPGA_AFE_HV		0x28
#define TARGET_FPGA_AFE_HV_VALUE	0x2c
#define TARGET_FPGA_AFE_DAC1		0x30
// .. #define TARGET_FPGA_AFE_DAC7		0x48
#define TARGET_FPGA_CLOCK 		0x50

#endif
