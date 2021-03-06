#ifndef _ALTERA_HPS_0_H_
#define _ALTERA_HPS_0_H_

/*
 * This file was automatically generated by the swinfo2header utility.
 * 
 * Created from SOPC Builder system 'soc_system' in
 * file '/home/letrend/workspace/roboy3/src/roboy_plexus/de10/soc_system.sopcinfo'.
 */

/*
 * This file contains macros for module 'hps_0' and devices
 * connected to the following master:
 *   h2f_lw_axi_master
 * 
 * Do not include this header file and another header file created for a
 * different module or master group at the same time.
 * Doing so may result in duplicate macro names.
 * Instead, use the system header file which has macros with unique names.
 */

/*
 * Macros for device 'iCEbusControl_3', class 'iCEbusControl'
 * The macros are prefixed with 'ICEBUSCONTROL_3_'.
 * The prefix is the slave descriptor.
 */
#define ICEBUSCONTROL_3_COMPONENT_TYPE iCEbusControl
#define ICEBUSCONTROL_3_COMPONENT_NAME iCEbusControl_3
#define ICEBUSCONTROL_3_BASE 0x0
#define ICEBUSCONTROL_3_SPAN 4096
#define ICEBUSCONTROL_3_END 0xfff

/*
 * Macros for device 'sysid_qsys', class 'altera_avalon_sysid_qsys'
 * The macros are prefixed with 'SYSID_QSYS_'.
 * The prefix is the slave descriptor.
 */
#define SYSID_QSYS_COMPONENT_TYPE altera_avalon_sysid_qsys
#define SYSID_QSYS_COMPONENT_NAME sysid_qsys
#define SYSID_QSYS_BASE 0x1000
#define SYSID_QSYS_SPAN 8
#define SYSID_QSYS_END 0x1007
#define SYSID_QSYS_ID 2976579765
#define SYSID_QSYS_TIMESTAMP 1600176096

/*
 * Macros for device 'TLI4970', class 'TLI4970'
 * The macros are prefixed with 'TLI4970_'.
 * The prefix is the slave descriptor.
 */
#define TLI4970_COMPONENT_TYPE TLI4970
#define TLI4970_COMPONENT_NAME TLI4970
#define TLI4970_BASE 0x1008
#define TLI4970_SPAN 8
#define TLI4970_END 0x100f

/*
 * Macros for device 'LED', class 'altera_avalon_pio'
 * The macros are prefixed with 'LED_'.
 * The prefix is the slave descriptor.
 */
#define LED_COMPONENT_TYPE altera_avalon_pio
#define LED_COMPONENT_NAME LED
#define LED_BASE 0x1010
#define LED_SPAN 16
#define LED_END 0x101f
#define LED_BIT_CLEARING_EDGE_REGISTER 0
#define LED_BIT_MODIFYING_OUTPUT_REGISTER 0
#define LED_CAPTURE 0
#define LED_DATA_WIDTH 8
#define LED_DO_TEST_BENCH_WIRING 0
#define LED_DRIVEN_SIM_VALUE 0
#define LED_EDGE_TYPE NONE
#define LED_FREQ 50000000
#define LED_HAS_IN 0
#define LED_HAS_OUT 1
#define LED_HAS_TRI 0
#define LED_IRQ_TYPE NONE
#define LED_RESET_VALUE 0

/*
 * Macros for device 'BALLJOINT_4', class 'I2C'
 * The macros are prefixed with 'BALLJOINT_4_'.
 * The prefix is the slave descriptor.
 */
#define BALLJOINT_4_COMPONENT_TYPE I2C
#define BALLJOINT_4_COMPONENT_NAME BALLJOINT_4
#define BALLJOINT_4_BASE 0x1040
#define BALLJOINT_4_SPAN 64
#define BALLJOINT_4_END 0x107f

/*
 * Macros for device 'BALLJOINT_3', class 'I2C'
 * The macros are prefixed with 'BALLJOINT_3_'.
 * The prefix is the slave descriptor.
 */
#define BALLJOINT_3_COMPONENT_TYPE I2C
#define BALLJOINT_3_COMPONENT_NAME BALLJOINT_3
#define BALLJOINT_3_BASE 0x1080
#define BALLJOINT_3_SPAN 64
#define BALLJOINT_3_END 0x10bf

/*
 * Macros for device 'auxilliary_i2c_3', class 'I2C'
 * The macros are prefixed with 'AUXILLIARY_I2C_3_'.
 * The prefix is the slave descriptor.
 */
#define AUXILLIARY_I2C_3_COMPONENT_TYPE I2C
#define AUXILLIARY_I2C_3_COMPONENT_NAME auxilliary_i2c_3
#define AUXILLIARY_I2C_3_BASE 0x10c0
#define AUXILLIARY_I2C_3_SPAN 64
#define AUXILLIARY_I2C_3_END 0x10ff

/*
 * Macros for device 'auxilliary_i2c_2', class 'I2C'
 * The macros are prefixed with 'AUXILLIARY_I2C_2_'.
 * The prefix is the slave descriptor.
 */
#define AUXILLIARY_I2C_2_COMPONENT_TYPE I2C
#define AUXILLIARY_I2C_2_COMPONENT_NAME auxilliary_i2c_2
#define AUXILLIARY_I2C_2_BASE 0x1100
#define AUXILLIARY_I2C_2_SPAN 64
#define AUXILLIARY_I2C_2_END 0x113f

/*
 * Macros for device 'auxilliary_i2c_1', class 'I2C'
 * The macros are prefixed with 'AUXILLIARY_I2C_1_'.
 * The prefix is the slave descriptor.
 */
#define AUXILLIARY_I2C_1_COMPONENT_TYPE I2C
#define AUXILLIARY_I2C_1_COMPONENT_NAME auxilliary_i2c_1
#define AUXILLIARY_I2C_1_BASE 0x1140
#define AUXILLIARY_I2C_1_SPAN 64
#define AUXILLIARY_I2C_1_END 0x117f

/*
 * Macros for device 'auxilliary_i2c_0', class 'I2C'
 * The macros are prefixed with 'AUXILLIARY_I2C_0_'.
 * The prefix is the slave descriptor.
 */
#define AUXILLIARY_I2C_0_COMPONENT_TYPE I2C
#define AUXILLIARY_I2C_0_COMPONENT_NAME auxilliary_i2c_0
#define AUXILLIARY_I2C_0_BASE 0x1180
#define AUXILLIARY_I2C_0_SPAN 64
#define AUXILLIARY_I2C_0_END 0x11bf

/*
 * Macros for device 'BALLJOINT_2', class 'I2C'
 * The macros are prefixed with 'BALLJOINT_2_'.
 * The prefix is the slave descriptor.
 */
#define BALLJOINT_2_COMPONENT_TYPE I2C
#define BALLJOINT_2_COMPONENT_NAME BALLJOINT_2
#define BALLJOINT_2_BASE 0x11c0
#define BALLJOINT_2_SPAN 64
#define BALLJOINT_2_END 0x11ff

/*
 * Macros for device 'BALLJOINT_1', class 'I2C'
 * The macros are prefixed with 'BALLJOINT_1_'.
 * The prefix is the slave descriptor.
 */
#define BALLJOINT_1_COMPONENT_TYPE I2C
#define BALLJOINT_1_COMPONENT_NAME BALLJOINT_1
#define BALLJOINT_1_BASE 0x1200
#define BALLJOINT_1_SPAN 64
#define BALLJOINT_1_END 0x123f

/*
 * Macros for device 'BALLJOINT_0', class 'I2C'
 * The macros are prefixed with 'BALLJOINT_0_'.
 * The prefix is the slave descriptor.
 */
#define BALLJOINT_0_COMPONENT_TYPE I2C
#define BALLJOINT_0_COMPONENT_NAME BALLJOINT_0
#define BALLJOINT_0_BASE 0x1240
#define BALLJOINT_0_SPAN 64
#define BALLJOINT_0_END 0x127f

/*
 * Macros for device 'LED_0', class 'altera_avalon_pio'
 * The macros are prefixed with 'LED_0_'.
 * The prefix is the slave descriptor.
 */
#define LED_0_COMPONENT_TYPE altera_avalon_pio
#define LED_0_COMPONENT_NAME LED_0
#define LED_0_BASE 0x12e0
#define LED_0_SPAN 16
#define LED_0_END 0x12ef
#define LED_0_BIT_CLEARING_EDGE_REGISTER 0
#define LED_0_BIT_MODIFYING_OUTPUT_REGISTER 0
#define LED_0_CAPTURE 0
#define LED_0_DATA_WIDTH 8
#define LED_0_DO_TEST_BENCH_WIRING 0
#define LED_0_DRIVEN_SIM_VALUE 0
#define LED_0_EDGE_TYPE NONE
#define LED_0_FREQ 50000000
#define LED_0_HAS_IN 0
#define LED_0_HAS_OUT 1
#define LED_0_HAS_TRI 0
#define LED_0_IRQ_TYPE NONE
#define LED_0_RESET_VALUE 255

/*
 * Macros for device 'POWER_CONTROL', class 'altera_avalon_pio'
 * The macros are prefixed with 'POWER_CONTROL_'.
 * The prefix is the slave descriptor.
 */
#define POWER_CONTROL_COMPONENT_TYPE altera_avalon_pio
#define POWER_CONTROL_COMPONENT_NAME POWER_CONTROL
#define POWER_CONTROL_BASE 0x12f0
#define POWER_CONTROL_SPAN 16
#define POWER_CONTROL_END 0x12ff
#define POWER_CONTROL_BIT_CLEARING_EDGE_REGISTER 0
#define POWER_CONTROL_BIT_MODIFYING_OUTPUT_REGISTER 0
#define POWER_CONTROL_CAPTURE 0
#define POWER_CONTROL_DATA_WIDTH 2
#define POWER_CONTROL_DO_TEST_BENCH_WIRING 0
#define POWER_CONTROL_DRIVEN_SIM_VALUE 0
#define POWER_CONTROL_EDGE_TYPE NONE
#define POWER_CONTROL_FREQ 50000000
#define POWER_CONTROL_HAS_IN 0
#define POWER_CONTROL_HAS_OUT 1
#define POWER_CONTROL_HAS_TRI 0
#define POWER_CONTROL_IRQ_TYPE NONE
#define POWER_CONTROL_RESET_VALUE 3

/*
 * Macros for device 'POWER_SENSE', class 'altera_avalon_pio'
 * The macros are prefixed with 'POWER_SENSE_'.
 * The prefix is the slave descriptor.
 */
#define POWER_SENSE_COMPONENT_TYPE altera_avalon_pio
#define POWER_SENSE_COMPONENT_NAME POWER_SENSE
#define POWER_SENSE_BASE 0x1300
#define POWER_SENSE_SPAN 16
#define POWER_SENSE_END 0x130f
#define POWER_SENSE_BIT_CLEARING_EDGE_REGISTER 0
#define POWER_SENSE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define POWER_SENSE_CAPTURE 0
#define POWER_SENSE_DATA_WIDTH 6
#define POWER_SENSE_DO_TEST_BENCH_WIRING 0
#define POWER_SENSE_DRIVEN_SIM_VALUE 0
#define POWER_SENSE_EDGE_TYPE NONE
#define POWER_SENSE_FREQ 50000000
#define POWER_SENSE_HAS_IN 1
#define POWER_SENSE_HAS_OUT 0
#define POWER_SENSE_HAS_TRI 0
#define POWER_SENSE_IRQ_TYPE NONE
#define POWER_SENSE_RESET_VALUE 0

/*
 * Macros for device 'jtag_uart', class 'altera_avalon_jtag_uart'
 * The macros are prefixed with 'JTAG_UART_'.
 * The prefix is the slave descriptor.
 */
#define JTAG_UART_COMPONENT_TYPE altera_avalon_jtag_uart
#define JTAG_UART_COMPONENT_NAME jtag_uart
#define JTAG_UART_BASE 0x2000
#define JTAG_UART_SPAN 8
#define JTAG_UART_END 0x2007
#define JTAG_UART_IRQ 0
#define JTAG_UART_READ_DEPTH 64
#define JTAG_UART_READ_THRESHOLD 8
#define JTAG_UART_WRITE_DEPTH 64
#define JTAG_UART_WRITE_THRESHOLD 8

/*
 * Macros for device 'iCEbusControl_7', class 'iCEbusControl'
 * The macros are prefixed with 'ICEBUSCONTROL_7_'.
 * The prefix is the slave descriptor.
 */
#define ICEBUSCONTROL_7_COMPONENT_TYPE iCEbusControl
#define ICEBUSCONTROL_7_COMPONENT_NAME iCEbusControl_7
#define ICEBUSCONTROL_7_BASE 0x3000
#define ICEBUSCONTROL_7_SPAN 4096
#define ICEBUSCONTROL_7_END 0x3fff

/*
 * Macros for device 'iCEbusControl_2', class 'iCEbusControl'
 * The macros are prefixed with 'ICEBUSCONTROL_2_'.
 * The prefix is the slave descriptor.
 */
#define ICEBUSCONTROL_2_COMPONENT_TYPE iCEbusControl
#define ICEBUSCONTROL_2_COMPONENT_NAME iCEbusControl_2
#define ICEBUSCONTROL_2_BASE 0x4000
#define ICEBUSCONTROL_2_SPAN 4096
#define ICEBUSCONTROL_2_END 0x4fff

/*
 * Macros for device 'iCEbusControl_1', class 'iCEbusControl'
 * The macros are prefixed with 'ICEBUSCONTROL_1_'.
 * The prefix is the slave descriptor.
 */
#define ICEBUSCONTROL_1_COMPONENT_TYPE iCEbusControl
#define ICEBUSCONTROL_1_COMPONENT_NAME iCEbusControl_1
#define ICEBUSCONTROL_1_BASE 0x5000
#define ICEBUSCONTROL_1_SPAN 4096
#define ICEBUSCONTROL_1_END 0x5fff

/*
 * Macros for device 'iCEbusControl_0', class 'iCEbusControl'
 * The macros are prefixed with 'ICEBUSCONTROL_0_'.
 * The prefix is the slave descriptor.
 */
#define ICEBUSCONTROL_0_COMPONENT_TYPE iCEbusControl
#define ICEBUSCONTROL_0_COMPONENT_NAME iCEbusControl_0
#define ICEBUSCONTROL_0_BASE 0x6000
#define ICEBUSCONTROL_0_SPAN 4096
#define ICEBUSCONTROL_0_END 0x6fff

/*
 * Macros for device 'iCEbusControl_6', class 'iCEbusControl'
 * The macros are prefixed with 'ICEBUSCONTROL_6_'.
 * The prefix is the slave descriptor.
 */
#define ICEBUSCONTROL_6_COMPONENT_TYPE iCEbusControl
#define ICEBUSCONTROL_6_COMPONENT_NAME iCEbusControl_6
#define ICEBUSCONTROL_6_BASE 0x7000
#define ICEBUSCONTROL_6_SPAN 4096
#define ICEBUSCONTROL_6_END 0x7fff


#endif /* _ALTERA_HPS_0_H_ */
