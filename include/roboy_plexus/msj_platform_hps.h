#ifndef _ALTERA_HPS_0_H_
#define _ALTERA_HPS_0_H_

/*
 * This file was automatically generated by the swinfo2header utility.
 * 
 * Created from SOPC Builder system 'soc_system' in
 * file '/home/letrend/workspace/roboy_control/src/roboy_plexus/roboy_de10_nano_soc/soc_system.sopcinfo'.
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
 * Macros for device 'I2C_2', class 'I2C'
 * The macros are prefixed with 'I2C_2_'.
 * The prefix is the slave descriptor.
 */
#define I2C_2_COMPONENT_TYPE I2C
#define I2C_2_COMPONENT_NAME I2C_2
#define I2C_2_BASE 0x0
#define I2C_2_SPAN 32
#define I2C_2_END 0x1f

/*
 * Macros for device 'I2C_1', class 'I2C'
 * The macros are prefixed with 'I2C_1_'.
 * The prefix is the slave descriptor.
 */
#define I2C_1_COMPONENT_TYPE I2C
#define I2C_1_COMPONENT_NAME I2C_1
#define I2C_1_BASE 0x20
#define I2C_1_SPAN 32
#define I2C_1_END 0x3f

/*
 * Macros for device 'I2C_0', class 'I2C'
 * The macros are prefixed with 'I2C_0_'.
 * The prefix is the slave descriptor.
 */
#define I2C_0_COMPONENT_TYPE I2C
#define I2C_0_COMPONENT_NAME I2C_0
#define I2C_0_BASE 0x40
#define I2C_0_SPAN 32
#define I2C_0_END 0x5f

/*
 * Macros for device 'SWITCHES', class 'altera_avalon_pio'
 * The macros are prefixed with 'SWITCHES_'.
 * The prefix is the slave descriptor.
 */
#define SWITCHES_COMPONENT_TYPE altera_avalon_pio
#define SWITCHES_COMPONENT_NAME SWITCHES
#define SWITCHES_BASE 0x60
#define SWITCHES_SPAN 16
#define SWITCHES_END 0x6f
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
 * Macros for device 'LED', class 'altera_avalon_pio'
 * The macros are prefixed with 'LED_'.
 * The prefix is the slave descriptor.
 */
#define LED_COMPONENT_TYPE altera_avalon_pio
#define LED_COMPONENT_NAME LED
#define LED_BASE 0x70
#define LED_SPAN 16
#define LED_END 0x7f
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
 * Macros for device 'sysid_qsys', class 'altera_avalon_sysid_qsys'
 * The macros are prefixed with 'SYSID_QSYS_'.
 * The prefix is the slave descriptor.
 */
#define SYSID_QSYS_COMPONENT_TYPE altera_avalon_sysid_qsys
#define SYSID_QSYS_COMPONENT_NAME sysid_qsys
#define SYSID_QSYS_BASE 0x1000
#define SYSID_QSYS_SPAN 8
#define SYSID_QSYS_END 0x1007
#define SYSID_QSYS_ID 3735928559
#define SYSID_QSYS_TIMESTAMP 1543932390

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
 * Macros for device 'MSJPlatformController_0', class 'MSJPlatformController'
 * The macros are prefixed with 'MSJPLATFORMCONTROLLER_0_'.
 * The prefix is the slave descriptor.
 */
#define MSJPLATFORMCONTROLLER_0_COMPONENT_TYPE MSJPlatformController
#define MSJPLATFORMCONTROLLER_0_COMPONENT_NAME MSJPlatformController_0
#define MSJPLATFORMCONTROLLER_0_BASE 0x40000
#define MSJPLATFORMCONTROLLER_0_SPAN 262144
#define MSJPLATFORMCONTROLLER_0_END 0x7ffff


#endif /* _ALTERA_HPS_0_H_ */