/*******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of SPI
* resource as Master using HAL. The SPI master sends command packetsto the SPI
* slave to control an user LED.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <string.h>


/******************************************************************************
* Macros
*******************************************************************************/
/* SPI baud rate in Hz */
#define SPI_FREQ_HZ                (2000000UL)
/* Delay of 100ms between polls */
#define POLL_DELAY_MS              (100UL)
/* SPI transfer bits per frame */
#define BITS_PER_FRAME             (8)

/* TLE493D connections (choose available BSP aliases). These can be mapped
 * to real GPIOs in the design.modus if different aliases are required.
 * One CS uses the default CYBSP_SPI_CS; the other two use user LED pins as
 * placeholders and should be updated for the real board wiring. */
#define VDD_EN_LOW_PIN   (P9_6)    /* placeholder, active low enable */
#define CS_SENSOR_1_PIN  (P9_3)
#define CS_SENSOR_2_PIN  (P9_4)
#define CS_SENSOR_3_PIN  (P9_5)

/* Number of bytes to read from sensor (10 in Arduino sketch) */
#define SENSOR_REG_BYTES  (10)

/* sensor data struct - file scope so helper functions can use it */
typedef struct {
    int16_t bx, by, bz;
    uint16_t temperature;
    uint8_t diag;
} sensor_data_t;


/*******************************************************************************
* Global Variables
*******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
/* Helper function prototypes (implemented after main) */
static void poll_sensor(cyhal_spi_t *spi, cyhal_gpio_t cs_pin, uint8_t *rx, size_t len);
static int16_t parse_magnetic(uint8_t msb, uint8_t lsb);
static uint16_t parse_temperature(uint8_t msb, uint8_t lsb);
static void parse_sensor_data(uint8_t *rx, sensor_data_t *out);
static void print_sensor(const char *label, const sensor_data_t *s);


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* The main function.
*   1. Initializes the board, retarget-io and led
*   2. Configures the SPI Master to send command packet to the slave
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cyhal_spi_t mSPI;
    uint8_t rx_buf[SENSOR_REG_BYTES];

    sensor_data_t sensor1 = {0}, sensor2 = {0}, sensor3 = {0};

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    handle_error(result);

    /* Initialize retarget-io for uart logs */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                  CY_RETARGET_IO_BAUDRATE);
    /* Retarget-io init failed. Stop program execution */
    handle_error(result);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("*************** "
           "HAL: SPI Master "
           "*************** \r\n\n");

    printf("Configuring SPI master...\r\n");
    /* Init SPI master */
    result = cyhal_spi_init(&mSPI, CYBSP_SPI_MOSI, CYBSP_SPI_MISO, CYBSP_SPI_CLK,
                            CYBSP_SPI_CS, NULL, BITS_PER_FRAME,
                            CYHAL_SPI_MODE_10_MSB, false);
    handle_error(result);

    /* Set the SPI baud rate */
    result = cyhal_spi_set_frequency(&mSPI, SPI_FREQ_HZ);
    handle_error(result);

    /* Enable interrupts */
    __enable_irq();
    /* Initialize GPIOs for CS and VDD enable */
    cyhal_gpio_init(CS_SENSOR_1_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    cyhal_gpio_init(CS_SENSOR_2_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    cyhal_gpio_init(CS_SENSOR_3_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    /* VDD enable is active low in the Arduino sketch; drive it low to enable */
    cyhal_gpio_init(VDD_EN_LOW_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

    printf("TLE493D-P3I8 sensors example (no INT)\r\n");
    printf("Bx, By, Bz, Temp, Diag\r\n");

    for (;;)
    {
        /* Poll three sensors sequentially */
        poll_sensor(&mSPI, CS_SENSOR_1_PIN, rx_buf, SENSOR_REG_BYTES);
        parse_sensor_data(rx_buf, &sensor1);

        poll_sensor(&mSPI, CS_SENSOR_2_PIN, rx_buf, SENSOR_REG_BYTES);
        parse_sensor_data(rx_buf, &sensor2);

        poll_sensor(&mSPI, CS_SENSOR_3_PIN, rx_buf, SENSOR_REG_BYTES);
        parse_sensor_data(rx_buf, &sensor3);

        /* Print results */
        print_sensor("Sensor1", &sensor1);
        print_sensor("Sensor2", &sensor2);
        print_sensor("Sensor3", &sensor3);

        cyhal_system_delay_ms(POLL_DELAY_MS);
    }
}


/* ------------------------------------------------------------------------- */
/* Helper functions adapted from the provided Arduino sketch (C version)    */
/* ------------------------------------------------------------------------- */

static void poll_sensor(cyhal_spi_t *spi, cyhal_gpio_t cs_pin, uint8_t *rx, size_t len)
{
    cy_rslt_t res;
    /* Assert CS (active low) */
    cyhal_gpio_write(cs_pin, false);

    /* Send read command 0xC0 (start at reg 0x00, auto-increment) */
    uint8_t cmd = 0xC0;
    res = cyhal_spi_send(spi, cmd);
    if (res != CY_RSLT_SUCCESS)
    {
        printf("SPI send failed: 0x%08lx\r\n", (unsigned long)res);
    }

    /* Read bytes by sending dummy 0x00 */
    for (size_t i = 0; i < len; ++i)
    {
    uint8_t out = 0x00;
    uint8_t in = 0x00;
    /* tx pointer, tx_length, rx pointer, rx_length, write_fill */
    res = cyhal_spi_transfer(spi, &out, 1, &in, 1, 0x00);
        if (res != CY_RSLT_SUCCESS)
        {
            printf("SPI transfer failed: 0x%08lx\r\n", (unsigned long)res);
            rx[i] = 0;
        }
        else
        {
            rx[i] = in;
        }
    }

    /* Deassert CS */
    cyhal_gpio_write(cs_pin, true);
}

static int16_t parse_magnetic(uint8_t msb, uint8_t lsb)
{
    int16_t value = (int16_t)(((uint16_t)msb << 6) | (lsb >> 2));
    if (value & 0x2000) /* sign bit for 14-bit value */
    {
        value |= 0xC000; /* sign extend */
    }
    return value;
}

static uint16_t parse_temperature(uint8_t msb, uint8_t lsb)
{
    return (uint16_t)(((uint16_t)msb << 6) | (lsb >> 2));
}

static void parse_sensor_data(uint8_t *rx, sensor_data_t *out)
{
    out->bx = parse_magnetic(rx[0], rx[1]);
    out->by = parse_magnetic(rx[2], rx[3]);
    out->bz = parse_magnetic(rx[4], rx[5]);
    out->temperature = parse_temperature(rx[6], rx[7]);
    out->diag = rx[9];
}

static void print_sensor(const char *label, const sensor_data_t *s)
{
    printf("%s: Bx=%d, By=%d, Bz=%d, Temp=%u, Diag=0b%02x\r\n",
           label, s->bx, s->by, s->bz, (unsigned int)s->temperature, s->diag);
}


/* [] END OF FILE */
