/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_error_log_module Error Log Module
 * @{
 * @ingroup ble_sdk_lib
 * @brief Module for writing error and stack to flash memory.
 *
 * @details It contains functions for writing an error code, line number, filename/message and
 *          the stack to the flash during an error, e.g. in the assert handler.
 *
 */
#ifndef BLE_ERROR_LOG_H__
#define BLE_ERROR_LOG_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble_nrf6310_pins.h"

#define ERROR_MESSAGE_LENGTH  128                              /**< Length of error message to stored. */
#define STACK_DUMP_LENGTH     64                               /**< Length of stack to be stored at max: 64 entries of 4 bytes each. */
#define FLASH_PAGE_ERROR_LOG  254                              /**< Address in flash where stack trace can be stored. */
#define ROM_ADDRESS_START     0x00020000                       /**< Address in the flash where the application is located. This address MUST correspond to the Keil Target ROM address defined for the application. */
#define LOG_LED_PIN_NO        NRF6310_LED_6                    /**< Use LED 6 to identify messages in log. */

/**@brief Error Log Data structure.
 *
 * @details The structure contains the error, message/filename, line number as well as the current
 *          stack, at the time where an error occured.
 */
typedef struct
{
    bool                      failure;                         /**< Indication that a major failure has occurred during last execution of the application. */
    uint32_t                  err_code;                        /**< Error code when failure occurred. */
    uint16_t                  line_number;                     /**< Line number indicating at which line the failure occurred. */
    uint8_t                   message[ERROR_MESSAGE_LENGTH];   /**< Will just use the first 128 bytes of filename to store for debugging purposes. */
    uint32_t                  stack_info[STACK_DUMP_LENGTH];   /**< Will contain stack information, can be manually unwinded for debug purposes. */
} ble_error_log_data_t;


/**@brief Write Error Log and current program stack to flash.
 *
 * @param[in]   err_code    Error code to be logged.
 * @param[in]   p_message   Message to be written to the flash together with stack dump, usually
 *                          the file name where the error occured.
 * @param[in]   line_number Line number where the error occured.
 *
 * @return      NRF_SUCCESS on successful writing of the error log.
 *
 */
uint32_t ble_error_log_write(uint32_t err_code, const uint8_t * p_message, uint16_t line_number);


/**@brief Read Error Log from flash.
 *
 * @details If an error is present, this function will light LED6 and block current execution.
 *          Execution will continue when @see ble_error_log_clear() is called from application.
 *          @see ble_error_log_clear()  will also delete the error present in the flash.
 *
 * @param[in]   error_log   Pointer to the structure where the Error log present in the flash
 *                          will be put. If no error was present, this structure will not be 
 *                          changed.
 *
 * @return      NRF_SUCCESS if access to the flash was successful and no error was present in the
 *              flash.
 */
uint32_t ble_error_log_read(ble_error_log_data_t * error_log);


/**@brief Clear the Error Log in flash.
 *
 * @details If an error was present and execution is blocked at @see ble_error_log_read() then this
 *          function will notify the read and ensure operation continues. This function is expected
 *          to be called from an interrupt, e.g. on button press.
 */
void ble_error_log_clear(void);


/**@brief Intialize the error log module.
 *
 * @details The init funtion will ensure the flash is initialized so that error can be logged.
 */
void ble_error_log_init(void);


#endif /* BLE_ERROR_LOG_H__ */

/** @} */
