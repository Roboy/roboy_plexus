#ifndef _ALTERA_HPS_0_H_
#define _ALTERA_HPS_0_H_

/*
 * This file was automatically generated by the swinfo2header utility.
 * 
 * Created from SOPC Builder system 'soc_system' in
 * file '/home/letrend/workspace/roboy3/src/roboy_plexus/roboy_de10_nano_soc/soc_system.sopcinfo'.
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
 * Macros for device 'iCEbusControl_6', class 'iCEbusControl'
 * The macros are prefixed with 'ICEBUSCONTROL_6_'.
 * The prefix is the slave descriptor.
 */
#define ICEBUSCONTROL_6_COMPONENT_TYPE iCEbusControl
#define ICEBUSCONTROL_6_COMPONENT_NAME iCEbusControl_6
#define ICEBUSCONTROL_6_BASE 0x0
#define ICEBUSCONTROL_6_SPAN 4096
#define ICEBUSCONTROL_6_END 0xfff

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
#define SYSID_QSYS_TIMESTAMP 1588188404

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
 * Macros for device 'FanControl_0', class 'FanControl'
 * The macros are prefixed with 'FANCONTROL_0_'.
 * The prefix is the slave descriptor.
 */
#define FANCONTROL_0_COMPONENT_TYPE FanControl
#define FANCONTROL_0_COMPONENT_NAME FanControl_0
#define FANCONTROL_0_BASE 0x1020
#define FANCONTROL_0_SPAN 32
#define FANCONTROL_0_END 0x103f

/*
 * Macros for device 'BALLJOINT_2', class 'I2C'
 * The macros are prefixed with 'BALLJOINT_2_'.
 * The prefix is the slave descriptor.
 */
#define BALLJOINT_2_COMPONENT_TYPE I2C
#define BALLJOINT_2_COMPONENT_NAME BALLJOINT_2
#define BALLJOINT_2_BASE 0x1040
#define BALLJOINT_2_SPAN 64
#define BALLJOINT_2_END 0x107f

/*
 * Macros for device 'BALLJOINT_1', class 'I2C'
 * The macros are prefixed with 'BALLJOINT_1_'.
 * The prefix is the slave descriptor.
 */
#define BALLJOINT_1_COMPONENT_TYPE I2C
#define BALLJOINT_1_COMPONENT_NAME BALLJOINT_1
#define BALLJOINT_1_BASE 0x1080
#define BALLJOINT_1_SPAN 64
#define BALLJOINT_1_END 0x10bf

/*
 * Macros for device 'BALLJOINT_0', class 'I2C'
 * The macros are prefixed with 'BALLJOINT_0_'.
 * The prefix is the slave descriptor.
 */
#define BALLJOINT_0_COMPONENT_TYPE I2C
#define BALLJOINT_0_COMPONENT_NAME BALLJOINT_0
#define BALLJOINT_0_BASE 0x10c0
#define BALLJOINT_0_SPAN 64
#define BALLJOINT_0_END 0x10ff

/*
 * Macros for device 'auxilliary_I2C_3', class 'I2C'
 * The macros are prefixed with 'AUXILLIARY_I2C_3_'.
 * The prefix is the slave descriptor.
 */
#define AUXILLIARY_I2C_3_COMPONENT_TYPE I2C
#define AUXILLIARY_I2C_3_COMPONENT_NAME auxilliary_I2C_3
#define AUXILLIARY_I2C_3_BASE 0x1100
#define AUXILLIARY_I2C_3_SPAN 64
#define AUXILLIARY_I2C_3_END 0x113f

/*
 * Macros for device 'auxilliary_I2C_2', class 'I2C'
 * The macros are prefixed with 'AUXILLIARY_I2C_2_'.
 * The prefix is the slave descriptor.
 */
#define AUXILLIARY_I2C_2_COMPONENT_TYPE I2C
#define AUXILLIARY_I2C_2_COMPONENT_NAME auxilliary_I2C_2
#define AUXILLIARY_I2C_2_BASE 0x1140
#define AUXILLIARY_I2C_2_SPAN 64
#define AUXILLIARY_I2C_2_END 0x117f

/*
 * Macros for device 'auxilliary_I2C_1', class 'I2C'
 * The macros are prefixed with 'AUXILLIARY_I2C_1_'.
 * The prefix is the slave descriptor.
 */
#define AUXILLIARY_I2C_1_COMPONENT_TYPE I2C
#define AUXILLIARY_I2C_1_COMPONENT_NAME auxilliary_I2C_1
#define AUXILLIARY_I2C_1_BASE 0x1180
#define AUXILLIARY_I2C_1_SPAN 64
#define AUXILLIARY_I2C_1_END 0x11bf

/*
 * Macros for device 'auxilliary_I2C_0', class 'I2C'
 * The macros are prefixed with 'AUXILLIARY_I2C_0_'.
 * The prefix is the slave descriptor.
 */
#define AUXILLIARY_I2C_0_COMPONENT_TYPE I2C
#define AUXILLIARY_I2C_0_COMPONENT_NAME auxilliary_I2C_0
#define AUXILLIARY_I2C_0_BASE 0x11c0
#define AUXILLIARY_I2C_0_SPAN 64
#define AUXILLIARY_I2C_0_END 0x11ff

/*
 * Macros for device 'FanControl_1', class 'FanControl'
 * The macros are prefixed with 'FANCONTROL_1_'.
 * The prefix is the slave descriptor.
 */
#define FANCONTROL_1_COMPONENT_TYPE FanControl
#define FANCONTROL_1_COMPONENT_NAME FanControl_1
#define FANCONTROL_1_BASE 0x1200
#define FANCONTROL_1_SPAN 32
#define FANCONTROL_1_END 0x121f

/*
 * Macros for device 'FanControl_2', class 'FanControl'
 * The macros are prefixed with 'FANCONTROL_2_'.
 * The prefix is the slave descriptor.
 */
#define FANCONTROL_2_COMPONENT_TYPE FanControl
#define FANCONTROL_2_COMPONENT_NAME FanControl_2
#define FANCONTROL_2_BASE 0x1220
#define FANCONTROL_2_SPAN 32
#define FANCONTROL_2_END 0x123f

/*
 * Macros for device 'FanControl_3', class 'FanControl'
 * The macros are prefixed with 'FANCONTROL_3_'.
 * The prefix is the slave descriptor.
 */
#define FANCONTROL_3_COMPONENT_TYPE FanControl
#define FANCONTROL_3_COMPONENT_NAME FanControl_3
#define FANCONTROL_3_BASE 0x1240
#define FANCONTROL_3_SPAN 32
#define FANCONTROL_3_END 0x125f

/*
 * Macros for device 'LED_0', class 'altera_avalon_pio'
 * The macros are prefixed with 'LED_0_'.
 * The prefix is the slave descriptor.
 */
#define LED_0_COMPONENT_TYPE altera_avalon_pio
#define LED_0_COMPONENT_NAME LED_0
#define LED_0_BASE 0x1260
#define LED_0_SPAN 16
#define LED_0_END 0x126f
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
#define POWER_CONTROL_BASE 0x1270
#define POWER_CONTROL_SPAN 16
#define POWER_CONTROL_END 0x127f
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
#define POWER_SENSE_BASE 0x1280
#define POWER_SENSE_SPAN 16
#define POWER_SENSE_END 0x128f
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
 * Macros for device 'SWITCHES', class 'altera_avalon_pio'
 * The macros are prefixed with 'SWITCHES_'.
 * The prefix is the slave descriptor.
 */
#define SWITCHES_COMPONENT_TYPE altera_avalon_pio
#define SWITCHES_COMPONENT_NAME SWITCHES
#define SWITCHES_BASE 0x1290
#define SWITCHES_SPAN 16
#define SWITCHES_END 0x129f
#define SWITCHES_BIT_CLEARING_EDGE_REGISTER 0
#define SWITCHES_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SWITCHES_CAPTURE 0
#define SWITCHES_DATA_WIDTH 4
#define SWITCHES_DO_TEST_BENCH_WIRING 0
#define SWITCHES_DRIVEN_SIM_VALUE 0
#define SWITCHES_EDGE_TYPE NONE
#define SWITCHES_FREQ 50000000
#define SWITCHES_HAS_IN 1
#define SWITCHES_HAS_OUT 0
#define SWITCHES_HAS_TRI 0
#define SWITCHES_IRQ_TYPE NONE
#define SWITCHES_RESET_VALUE 0

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
 * Macros for device 'MYOControl_0', class 'MYOControl'
 * The macros are prefixed with 'MYOCONTROL_0_'.
 * The prefix is the slave descriptor.
 */
#define MYOCONTROL_0_COMPONENT_TYPE MYOControl
#define MYOCONTROL_0_COMPONENT_NAME MYOControl_0
#define MYOCONTROL_0_BASE 0x40000
#define MYOCONTROL_0_SPAN 262144
#define MYOCONTROL_0_END 0x7ffff


#endif /* _ALTERA_HPS_0_H_ */
