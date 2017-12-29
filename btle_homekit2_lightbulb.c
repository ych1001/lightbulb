/*
 * Copyright 2017, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
 */

/** @file
 *
 * Bluetooth Low Energy Homekit accessory sample
 *
 */
#include "sparcommon.h"
#include "wiced_bt_uuid.h"
#include "wiced_gki.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_app_common.h"
#include "wiced_bt_app_hal_common.h"
#include "wiced_result.h"

#include "wiced_bt_trace.h"
#include "wiced_transport.h"

#include "wiced_hal_platform.h"

#include "hci_control_api.h"
#include "btle_homekit2_lightbulb.h"
#include "apple_btle_homekit2.h"

#ifdef OTA_FIRMWARE_UPGRADE
#include "wiced_bt_firmware_upgrade.h"
#include "wiced_bt_fw_upgrade.h"
#endif

#include "wiced_bt_gatt.h"
#include "wiced_hal_nvram.h"


#define TRANS_UART_BUFFER_SIZE          1024

/*****************************************************************************
** Constant definitions
*****************************************************************************/
#define BTLE_HOMEKIT_LIGHTBULB_APP_TIMEOUT_IN_SECONDS                 1

#define BTLE_HOMEKIT_LIGHTBULB_APP_FINE_TIMEOUT_IN_MILLI_SECONDS      100

#define BTLE_HOMEKIT_NVRAM_VSID_START                   (WICED_NVRAM_VSID_START)
#define BTLE_HOMEKIT_NVRAM_VSID_CHAR_VALUES             (BTLE_HOMEKIT_NVRAM_VSID_START + 0)
#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_WINDOWS
#define BTLE_HOMEKIT_NVRAM_VSID_LOCAL_KEYS              (BTLE_HOMEKIT_NVRAM_VSID_START + 1)
#define BTLE_HOMEKIT_NVRAM_VSID_PEER_KEYS               (BTLE_HOMEKIT_NVRAM_VSID_START + 2)
#endif
#define BTLE_HOMEKIT_NVRAM_BDA_STATIC_RANDOM            (BTLE_HOMEKIT_NVRAM_VSID_START + 3)
#define BTLE_HOMEKIT_NVRAM_VSID_LAST                    (APPLE_HOMEKIT_NVRAM_VSID_START - 1)

#define UUID_APPLE_HOMEKIT_ACCESSORY_INFO               0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00
#define UUID_APPLE_HOMEKIT_ACCESSORY_INFO_IDENTIFY      0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00
#define UUID_APPLE_HOMEKIT_ACCESSORY_INFO_MANUFACTURER  0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00
#define UUID_APPLE_HOMEKIT_ACCESSORY_INFO_MODEL         0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00
#define UUID_APPLE_HOMEKIT_ACCESSORY_INFO_NAME          0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x23, 0x00, 0x00, 0x00
#define UUID_APPLE_HOMEKIT_ACCESSORY_INFO_SERIAL_NUMBER 0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00
#define UUID_APPLE_HOMEKIT_ACCESSORY_INFO_FIRMWARE_REVISION 0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x52, 0x00, 0x00, 0x00

#define UUID_APPLE_HOMEKIT_PROTOCOL_INFO                0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xa2, 0x00, 0x00, 0x00
#define UUID_APPLE_HOMEKIT_PROTOCOL_VERSION             0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00

#define UUID_APPLE_HOMEKIT_LIGHTBULB                    0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x43, 0x00, 0x00, 0x00
#define UUID_APPLE_HOMEKIT_LIGHTBULB_BRIGHTNESS         0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00
#define UUID_APPLE_HOMEKIT_LIGHTBULB_ON                 0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00
#define UUID_APPLE_HOMEKIT_LIGHTBULB_HUE                0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00
#define UUID_APPLE_HOMEKIT_LIGHTBULB_SATURATION         0x91, 0x52, 0x76, 0xbb, 0x26, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00

#define READ_LITTLE_ENDIAN_TO_UINT16(into, m,dl)    \
        (into) = ((m)[0] | ((m)[1]<<8));            \
        (m) +=2; (dl)-=2;
#define STRING_MAX_MAC                   18  // ( 2 characters/byte * 6 bytes ) + ( 5 colon characters ) + NULL = 18

#define APPL_STACK_SIZE   0x1000

#define BTLE_HOMEKIT_MAX_PDU_BODY       128

#define BTLE_HOMEKIT_CONN_IDLE_TIMEOUT  30

#define FLOAT_VALUE_1_BYTE_ARRAY    0x3F800000
#define FLOAT_VALUE_100_BYTE_ARRAY  0x42C80000
#define FLOAT_VALUE_360_BYTE_ARRAY  0x43B40000

#ifdef WICED_BT_TRACE_ENABLE
static void     hci_control_enable_uart( );
#endif

#define BCM920706 20706
#define BCM920707 20707

static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
static uint8_t hci_control_proc_hk_cmd(uint16_t opcode, uint8_t *p_data, uint16_t data_len);
static void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void hci_control_misc_handle_get_version( void );
/***************************************************************************
** Wiced_bt core stack configuration
****************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

/*****************************************************************************
** GATT server definitions
*****************************************************************************/

uint8_t apple_lightbulb_btle_homekit_gatt_db[] =
{
	/* Primary service GATT */
	PRIMARY_SERVICE_UUID16( HDLS_GATT, UUID_SERVICE_GATT ),

    /* Primary service GAP */
    PRIMARY_SERVICE_UUID16( HDLS_GAP, UUID_SERVICE_GAP ),

        /* Characteristic 'Device Name' */
        CHARACTERISTIC_UUID16( HDLC_GAP_DEVICE_NAME, HDLC_GAP_DEVICE_NAME_VALUE,
            GATT_UUID_GAP_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE ),

        /* Characteristic 'Appearance' */
        CHARACTERISTIC_UUID16( HDLC_GAP_APPEARANCE, HDLC_GAP_APPEARANCE_NAME_VALUE,
            GATT_UUID_GAP_ICON, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    /* Primary service 'ACCESSORY_INFO' */
    PRIMARY_SERVICE_UUID128( HDLS_ACCESSORY_INFO, UUID_APPLE_HOMEKIT_ACCESSORY_INFO),

        /* characteristic 'service instance id' */
        CHARACTERISTIC_UUID128( HDLC_ACCESSORY_INFO_INSTANCE_ID, HDLC_ACCESSORY_INFO_INSTANCE_ID_VALUE,
            UUID_APPLE_HOMEKIT_SERVICE_INSTANCE_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* characteristic 'identify' */
        CHARACTERISTIC_UUID128_WRITABLE( HDLC_ACCESSORY_INFO_IDENTIFY, HDLC_ACCESSORY_INFO_IDENTIFY_VALUE,
            UUID_APPLE_HOMEKIT_ACCESSORY_INFO_IDENTIFY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

            CHAR_DESCRIPTOR_UUID128(HDLD_ACCESSORY_INFO_IDENTIFY_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

        /* characteristic 'manufacturer */
        CHARACTERISTIC_UUID128_WRITABLE( HDLC_ACCESSORY_INFO_MANUFACTURER, HDLC_ACCESSORY_INFO_MANUFACTURER_VALUE,
            UUID_APPLE_HOMEKIT_ACCESSORY_INFO_MANUFACTURER, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

            CHAR_DESCRIPTOR_UUID128(HDLD_ACCESSORY_INFO_MANUFACTURER_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

        /* characteristic model */
        CHARACTERISTIC_UUID128_WRITABLE( HDLC_ACCESSORY_INFO_MODEL, HDLC_ACCESSORY_INFO_MODEL_VALUE,
            UUID_APPLE_HOMEKIT_ACCESSORY_INFO_MODEL, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

            CHAR_DESCRIPTOR_UUID128(HDLD_ACCESSORY_INFO_MODEL_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

        /* characteristic name */
        CHARACTERISTIC_UUID128_WRITABLE( HDLC_ACCESSORY_INFO_NAME, HDLC_ACCESSORY_INFO_NAME_VALUE,
            UUID_APPLE_HOMEKIT_ACCESSORY_INFO_NAME, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

            CHAR_DESCRIPTOR_UUID128(HDLD_ACCESSORY_INFO_NAME_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

        /* characteristic serial number */
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_ACCESSORY_INFO_SERIAL_NUMBER, HDLC_ACCESSORY_INFO_SERIAL_NUMBER_VALUE,
            UUID_APPLE_HOMEKIT_ACCESSORY_INFO_SERIAL_NUMBER, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

            CHAR_DESCRIPTOR_UUID128(HDLD_ACCESSORY_INFO_SERIAL_NUMBER_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

        /* characteristic firmware revision */
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_ACCESSORY_INFO_FIRMWARE_REVISION, HDLC_ACCESSORY_INFO_FIRMWARE_REVISION_VALUE,
            UUID_APPLE_HOMEKIT_ACCESSORY_INFO_FIRMWARE_REVISION, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

            CHAR_DESCRIPTOR_UUID128(HDLD_ACCESSORY_INFO_FIRMWARE_REVISION_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

    /* Primary service 'Protocol Information' */
    PRIMARY_SERVICE_UUID128( HDLS_PROTOCOL_INFO, UUID_APPLE_HOMEKIT_PROTOCOL_INFO ),

        /* instance id */
        CHARACTERISTIC_UUID128( HDLC_PROTOCOL_INFO_INSTANCE_ID, HDLC_PROTOCOL_INFO_INSTANCE_ID_VALUE,
            UUID_APPLE_HOMEKIT_SERVICE_INSTANCE_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* characteristic version */
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_PROTOCOL_INFO_VERSION, HDLC_PROTOCOL_INFO_VERSION_VALUE,
            UUID_APPLE_HOMEKIT_PROTOCOL_VERSION, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),

            /* Instance ID */
            CHAR_DESCRIPTOR_UUID128(HDLD_PROTOCOL_INFO_VERSION_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

    /* Primary service 'lightbulb' */
    PRIMARY_SERVICE_UUID128( HDLS_LIGHTBULB, UUID_APPLE_HOMEKIT_LIGHTBULB ),

        /* instance id */
        CHARACTERISTIC_UUID128( HDLC_LIGHTBULB_INSTANCE_ID, HDLC_LIGHTBULB_INSTANCE_ID_VALUE,
            UUID_APPLE_HOMEKIT_SERVICE_INSTANCE_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* characteristic service signature */
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_LIGHTBULB_SERVICE_SIGNATURE, HDLC_LIGHTBULB_SERVICE_SIGNATURE_VALUE,
            UUID_APPLE_HOMEKIT_SERVICE_SIGNATURE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),

            /* Instance ID */
            CHAR_DESCRIPTOR_UUID128(HDLD_LIGHTBULB_SERVICE_SIGNATURE_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

        /* characteristic brightness */
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_LIGHTBULB_BRIGHTNESS, HDLC_LIGHTBULB_BRIGHTNESS_VALUE,
            UUID_APPLE_HOMEKIT_LIGHTBULB_BRIGHTNESS, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),

            /* Instance ID */
            CHAR_DESCRIPTOR_UUID128(HDLD_LIGHTBULB_BRIGHTNESS_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

            /* client characteristic configuration descriptor */
            CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_LIGHTBULB_BRIGHTNESS_CLNT_CHAR_CFG,
                UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

        /* characteristic 'on' */
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_LIGHTBULB_ON, HDLC_LIGHTBULB_ON_VALUE,
            UUID_APPLE_HOMEKIT_LIGHTBULB_ON, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),

            /* Instance ID */
            CHAR_DESCRIPTOR_UUID128(HDLD_LIGHTBULB_ON_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE ),

            /* client characteristic configuration descriptor */
            CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_LIGHTBULB_ON_CLNT_CHAR_CFG,
                UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

        /* characteristic 'name' */
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_LIGHTBULB_NAME, HDLC_LIGHTBULB_NAME_VALUE,
            UUID_APPLE_HOMEKIT_ACCESSORY_INFO_NAME, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),

            /* Instance ID */
            CHAR_DESCRIPTOR_UUID128(HDLD_LIGHTBULB_NAME_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE ),

		/* characteristic hue */
		CHARACTERISTIC_UUID128_WRITABLE(HDLC_LIGHTBULB_HUE, HDLC_LIGHTBULB_HUE_VALUE,
			UUID_APPLE_HOMEKIT_LIGHTBULB_HUE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,
			LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),

			/* Instance ID */
			CHAR_DESCRIPTOR_UUID128(HDLD_LIGHTBULB_HUE_INSTANCE_ID,
				UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

			/* client characteristic configuration descriptor */
			CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_LIGHTBULB_HUE_CLNT_CHAR_CFG,
				UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

		/* characteristic saturation */
		CHARACTERISTIC_UUID128_WRITABLE(HDLC_LIGHTBULB_SATURATION, HDLC_LIGHTBULB_SATURATION_VALUE,
			UUID_APPLE_HOMEKIT_LIGHTBULB_SATURATION, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,
			LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),

			/* Instance ID */
			CHAR_DESCRIPTOR_UUID128(HDLD_LIGHTBULB_SATURATION_INSTANCE_ID,
				UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

			/* client characteristic configuration descriptor */
			CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_LIGHTBULB_SATURATION_CLNT_CHAR_CFG,
				UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

    /* **** Primary service 'pairing' */
    PRIMARY_SERVICE_UUID128(HDLS_PAIRING, UUID_APPLE_HOMEKIT_PAIRING),

        /* Characteristic 'Service Instance' */
        CHARACTERISTIC_UUID128 (HDLC_PAIRING_SERVICE_INSTANCE, HDLC_PAIRING_SERVICE_INSTANCE_VALUE,
            UUID_APPLE_HOMEKIT_SERVICE_INSTANCE_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

        /* characteristic 'pair_setup' */
        CHARACTERISTIC_UUID128_WRITABLE( HDLC_PAIRING_PAIR_SETUP, HDLC_PAIRING_PAIR_SETUP_VALUE,
            UUID_APPLE_HOMEKIT_PAIRING_PAIR_SETUP, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH ),

            CHAR_DESCRIPTOR_UUID128(HDLD_PAIRING_PAIR_SETUP_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

        /* characteristic 'pair_verify' */
        CHARACTERISTIC_UUID128_WRITABLE( HDLC_PAIRING_PAIR_VERIFY, HDLC_PAIRING_PAIR_VERIFY_VALUE,
            UUID_APPLE_HOMEKIT_PAIRING_PAIR_VERIFY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH ),

            CHAR_DESCRIPTOR_UUID128(HDLD_PAIRING_PAIR_VERIFY_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

        /* characteristic 'features' */
        CHARACTERISTIC_UUID128_WRITABLE( HDLC_PAIRING_FEATURES, HDLC_PAIRING_FEATURES_VALUE,
            UUID_APPLE_HOMEKIT_PAIRING_FEATURES, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

            CHAR_DESCRIPTOR_UUID128(HDLD_PAIRING_FEATURES_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

        /* characteristic 'pairings' */
        CHARACTERISTIC_UUID128_WRITABLE( HDLC_PAIRING_MANAGE, HDLC_PAIRING_MANAGE_VALUE,
            UUID_APPLE_HOMEKIT_PAIRING_MANAGE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH ),

            CHAR_DESCRIPTOR_UUID128(HDLD_PAIRING_MANAGE_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

// Added for the specific service, 03Nov17, Jack Huang
#if defined(HSENS_DEFINED)   // Begin
    // Declare proprietary Hello Service with 128 byte UUID
    PRIMARY_SERVICE_UUID128( HANDLE_HSENS_SERVICE, UUID_HELLO_SERVICE ),

        // Declare characteristic used to notify/indicate change
        CHARACTERISTIC_UUID128( HANDLE_HSENS_SERVICE_CHAR_NOTIFY, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
            UUID_HELLO_CHARACTERISTIC_NOTIFY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_READABLE ),

            // Declare client characteristic configuration descriptor
            // Value of the descriptor can be modified by the client
            // Value modified shall be retained during connection and across connection
            // for bonded devices.  Setting value to 1 tells this application to send notification
            // when value of the characteristic changes.  Value 2 is to allow indications.
            CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_CFG_DESC, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_AUTH_WRITABLE),

        // Declare characteristic Hello Configuration
        // The configuration consists of 1 bytes which indicates how many times to
        // blink the LED when user pushes the button.
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_BLINK, HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,
            UUID_HELLO_CHARACTERISTIC_CONFIG, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),
#endif   // End of added, 03Nov17, Jack Huang

#ifdef OTA_FIRMWARE_UPGRADE
#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_WINDOWS
    /* Cypress vendor specific OTA Firmware Upgrade Service */
    PRIMARY_SERVICE_UUID128(HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_FW_UPGRADE_SERVICE),

        /* characteristic WS Control Point */
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
                LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH),

        /* client characteristic configuration descriptor */
        CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR,
            UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

        /* characteristic WS Data */
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, HANDLE_OTA_FW_UPGRADE_DATA,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH),

        /* characteristic Application Info */
        CHARACTERISTIC_UUID128(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_APP_INFO, HANDLE_OTA_FW_UPGRADE_APP_INFO,
            UUID_OTA_FW_UPGRADE_SERVICE_CHARACTERISTIC_APP_INFO, LEGATTDB_CHAR_PROP_READ,
            LEGATTDB_PERM_READABLE),
#endif
#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_HAP
    /* Cypress vendor specific OTA Firmware Upgrade Service */
    PRIMARY_SERVICE_UUID128(HDLS_OTA_FW_UPGRADE_SERVICE, UUID_HAP_FW_UPDATE_SERVICE),

        /* Characteristic 'Service Instance ID' */
        CHARACTERISTIC_UUID128(HDLC_OTA_FW_UPGRADE_SERVICE_INSTANCE, HDLC_OTA_FW_UPGRADE_SERVICE_INSTANCE_VALUE,
            UUID_APPLE_HOMEKIT_SERVICE_INSTANCE_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* characteristic service signature */
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_OTA_FW_UPGRADE_SERVICE_SIGNATURE, HDLC_OTA_FW_UPGRADE_SERVICE_SIGNATURE_VALUE,
            UUID_APPLE_HOMEKIT_SERVICE_SIGNATURE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),

            /* Instance ID */
            CHAR_DESCRIPTOR_UUID128(HDLD_OTA_FW_UPGRADE_SERVICE_SIGNATURE_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

        /* characteristic WS Control Point */
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_OTA_FW_UPGRADE_CONTROL_POINT, HDLC_OTA_FW_UPGRADE_CONTROL_POINT_VALUE,
            UUID_HAP_FW_UPDATE_CHARACTERISTIC_CONTROL_POINT, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_INDICATE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH),

            /* Instance ID */
            CHAR_DESCRIPTOR_UUID128(HDLD_OTA_FW_UPGRADE_CONTROL_POINT_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

            // Declare client characteristic configuration descriptor
            // Value of the descriptor can be modified by the client
            // Value modified shall be retained during connection and across connection
            // for bonded devices.  Setting value to 1 tells this application to send notification
            // when value of the characteristic changes.  Value 2 is to allow indications.
            CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_OTA_FW_UPGRADE_CONTROL_POINT_CLNT_CHAR_CFG,
                UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

        /* characteristic WS Data */
        /* This characteristic is used to send next portion of the FW Similar to the control point */
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_OTA_FW_UPGRADE_DATA, HDLC_OTA_FW_UPGRADE_DATA_VALUE,
            UUID_HAP_FW_UPDATE_CHARACTERISTIC_DATA, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH),

            /* Instance ID */
            CHAR_DESCRIPTOR_UUID128(HDLD_OTA_FW_UPGRADE_DATA_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),

        /* characteristic Application Info */
        CHARACTERISTIC_UUID128_WRITABLE(HDLC_OTA_FW_UPGRADE_APP_INFO, HDLC_OTA_FW_UPGRADE_APP_INFO_VALUE,
            UUID_HAP_FW_UPDATE_SERVICE_CHARACTERISTIC_APP_INFO, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),

            /* Instance ID */
            CHAR_DESCRIPTOR_UUID128(HDLD_OTA_FW_UPGRADE_APP_INFO_INSTANCE_ID,
                UUID_APPLE_HOMEKIT_CHAR_INSTANCE_ID, LEGATTDB_PERM_READABLE),
#endif
#endif
};
#define LIGHTBULB_SERVER_GATT_DB_SIZE  ( sizeof( apple_lightbulb_btle_homekit_gatt_db ) )

uint8_t btle_homekit_lightbulb_appearance[]                   = {BIT16_TO_8(0x200)};
uint8_t btle_homekit_accessory_info_identify[]                = { 0 };
uint8_t btle_homekit_accessory_info_name[]                    = {'L','i','g','h','t'};
uint8_t btle_homekit_accessory_info_manufacturer[]            = {'C','y','p','r','e','s','s'};
uint8_t btle_homekit_accessory_info_model[]                   = {'A','0','0','0','1'};
uint8_t btle_homekit_accessory_info_serial_number[]           = {'A','0','0','0','1','B','0','0','0','1'};
uint8_t btle_homekit_accessory_info_firmware_revision[]       = {'1','.','1','.','1'};
uint8_t btle_homekit_protocol_info_version[]                  = {'0','2','.','0','1','.','0','0'};
uint8_t btle_homekit_lightbulb_brightness[]                   = { 0x00, 0x00, 0x00, 0x00 };
uint8_t btle_homekit_lightbulb_brightness_client_cfg[]        = { 0, 0 };
uint8_t btle_homekit_lightbulb_on[]                           = { 0x00 };
uint8_t btle_homekit_lightbulb_on_client_cfg[]                = { 0, 0 };
uint8_t btle_homekit_lightbulb_name[]                         = {'B','u','l','b'};
uint8_t btle_homekit_lightbulb_hue[]                          = { 0x00, 0x00, 0x00, 0x00 };
uint8_t btle_homekit_lightbulb_hue_client_cfg[]               = { 0, 0 };
uint8_t btle_homekit_lightbulb_saturation[]                   = { 0x00, 0x00, 0x00, 0x00 };
uint8_t btle_homekit_lightbulb_saturation_client_cfg[]        = { 0, 0 };
uint8_t btle_homekit_pair_service_features[] = {
#if defined(WICED_USE_MFI) && WICED_USE_MFI
        APPLE_BTLE_HOMEKIT_SUPPORTS_MFI_PAIR_SETUP
#else
        APPLE_BTLE_HOMEKIT_SUPPORTS_PAIR_SETUP
#endif
};

wiced_transport_buffer_pool_t* transport_pool;   // Trans pool for sending the RFCOMM data to host

const wiced_transport_cfg_t transport_cfg =
{
    WICED_TRANSPORT_UART,
// Modified for baud rate, 03Nov17, Jack Huang
#if 1   // Begin
    {  WICED_TRANSPORT_UART_HCI_MODE, 115200 },
#else   // Original
    {  WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD },
#endif   // End of modified, 03Nov17, Jack Huang
    {  TRANS_UART_BUFFER_SIZE, 2 },
    NULL,
    hci_control_proc_rx_cmd,
    NULL
};

#define BTLE_HK_DISABLE         0   // characteristic should never be accessed
#define BTLE_HK_ENC_NEVER       1   // characteristic should never be encrypted
#define BTLE_HK_ENC_ALWAYS      2   // characteristic should always be encrypted
#define BTLE_HK_ENC_VERIFIED    3   // characteristic should be encrypted if connection is verified

#define RNWN (BTLE_HK_ENC_NEVER    | (BTLE_HK_ENC_NEVER<<4))
#define RNWA (BTLE_HK_ENC_NEVER    | (BTLE_HK_ENC_ALWAYS<<4))
#define RNWV (BTLE_HK_ENC_NEVER    | (BTLE_HK_ENC_VERIFIED<<4))
#define RNWD (BTLE_HK_ENC_NEVER    | (BTLE_HK_DISABLE<<4))
#define RAWN (BTLE_HK_ENC_ALWAYS   | (BTLE_HK_ENC_NEVER<<4))
#define RAWA (BTLE_HK_ENC_ALWAYS   | (BTLE_HK_ENC_ALWAYS<<4))
#define RAWV (BTLE_HK_ENC_ALWAYS   | (BTLE_HK_ENC_VERIFIED<<4))
#define RAWD (BTLE_HK_ENC_ALWAYS   | (BTLE_HK_DISABLE<<4))
#define RVWN (BTLE_HK_ENC_VERIFIED | (BTLE_HK_ENC_NEVER<<4))
#define RVWA (BTLE_HK_ENC_VERIFIED | (BTLE_HK_ENC_ALWAYS<<4))
#define RVWV (BTLE_HK_ENC_VERIFIED | (BTLE_HK_ENC_VERIFIED<<4))
#define RVWD (BTLE_HK_ENC_VERIFIED | (BTLE_HK_DISABLE<<4))
#define RDWN (BTLE_HK_DISABLE      | (BTLE_HK_ENC_NEVER<<4))
#define RDWA (BTLE_HK_DISABLE      | (BTLE_HK_ENC_ALWAYS<<4))
#define RDWV (BTLE_HK_DISABLE      | (BTLE_HK_ENC_VERIFIED<<4))
#define RDWD (BTLE_HK_DISABLE      | (BTLE_HK_ENC_DISABLE<<4))

/* GATT attribute lookup table                                */
/* (attributes externally referenced by GATT server database) */
typedef struct
{
    uint16_t handle;
    uint8_t  encr_required;
    uint16_t max_len;
    uint16_t cur_len;
    uint8_t  *p_data;
} gatt_db_lookup_table;

#define BTLE_HOMEKIT_MAX_CHAR_VALUE_LEN     16
typedef struct
{
    uint8_t*    char_value;
    uint16_t    char_value_len;

    uint8_t*    p_timed_write;
    uint16_t    timed_write_len;
    uint16_t    timed_write_handle;
} btle_homekit_lightbulb_cb_t;

typedef struct
{
    uint8_t     brightness;
    uint8_t     on;
    uint32_t    hue;
    uint32_t    saturation;
} btle_homekit_nvram_char_values_t;

btle_homekit_nvram_char_values_t nvram_char_values;

#ifdef OTA_FIRMWARE_UPGRADE
/* Firmware Upgrade Application Identity */
#define BTLE_HOMEKIT_LIGHTBULB_APP_ID               0x4A21
#define BTLE_HOMEKIT_LIGHTBULB_APP_VERSION_MAJOR    1
#define BTLE_HOMEKIT_LIGHTBULB_APP_VERSION_MINOR    1
const uint8_t btle_homekit_lightbulb_fw_upgrade_app_info[] =
{
    (BTLE_HOMEKIT_LIGHTBULB_APP_ID & 0xff),
    ((BTLE_HOMEKIT_LIGHTBULB_APP_ID >> 8) & 0xff),
    BTLE_HOMEKIT_LIGHTBULB_APP_VERSION_MAJOR,
    BTLE_HOMEKIT_LIGHTBULB_APP_VERSION_MINOR,
};
#endif

gatt_db_lookup_table btle_homekit_lightbulb_gatt_db_attr_tbl[] =
{
    /* { attribute handle,                       encr_req maxlen, curlen, attrib data } */
    {HDLC_GAP_DEVICE_NAME_VALUE,                 RNWD,   0,      0,      NULL},
    {HDLC_GAP_APPEARANCE_NAME_VALUE,             RVWD,   2,      2,      btle_homekit_lightbulb_appearance},

    /* HAP characteristics do not need to specify encr_req, library code handles it */
    {HDLC_ACCESSORY_INFO_IDENTIFY_VALUE,         0,      1,      1,      btle_homekit_accessory_info_identify},
    {HDLC_ACCESSORY_INFO_MANUFACTURER_VALUE,     0,      sizeof(btle_homekit_accessory_info_manufacturer),  sizeof(btle_homekit_accessory_info_manufacturer),  btle_homekit_accessory_info_manufacturer},
    {HDLC_ACCESSORY_INFO_MODEL_VALUE,            0,      sizeof(btle_homekit_accessory_info_model),         sizeof(btle_homekit_accessory_info_model),         btle_homekit_accessory_info_model},
    {HDLC_ACCESSORY_INFO_NAME_VALUE,             0,      sizeof(btle_homekit_accessory_info_name),          sizeof(btle_homekit_accessory_info_name),          btle_homekit_accessory_info_name},
    {HDLC_ACCESSORY_INFO_SERIAL_NUMBER_VALUE,    0,      sizeof(btle_homekit_accessory_info_serial_number), sizeof(btle_homekit_accessory_info_serial_number), btle_homekit_accessory_info_serial_number},
    {HDLC_ACCESSORY_INFO_FIRMWARE_REVISION_VALUE,0,      sizeof(btle_homekit_accessory_info_firmware_revision), sizeof(btle_homekit_accessory_info_firmware_revision), btle_homekit_accessory_info_firmware_revision},
    {HDLC_PROTOCOL_INFO_VERSION_VALUE,           0,      sizeof(btle_homekit_protocol_info_version),        sizeof(btle_homekit_protocol_info_version),        btle_homekit_protocol_info_version},
    {HDLC_LIGHTBULB_BRIGHTNESS_VALUE,            0,      sizeof(btle_homekit_lightbulb_brightness),         sizeof(btle_homekit_lightbulb_brightness),         btle_homekit_lightbulb_brightness},
    {HDLD_LIGHTBULB_BRIGHTNESS_CLNT_CHAR_CFG,    RNWN,   sizeof(btle_homekit_lightbulb_brightness_client_cfg), sizeof(btle_homekit_lightbulb_brightness_client_cfg), btle_homekit_lightbulb_brightness_client_cfg},
    {HDLC_LIGHTBULB_ON_VALUE,                    0,      sizeof(btle_homekit_lightbulb_on),                 sizeof(btle_homekit_lightbulb_on),                 btle_homekit_lightbulb_on},
    {HDLD_LIGHTBULB_ON_CLNT_CHAR_CFG,            RNWN,   sizeof(btle_homekit_lightbulb_on_client_cfg),      sizeof(btle_homekit_lightbulb_on_client_cfg),      btle_homekit_lightbulb_on_client_cfg},
    {HDLC_LIGHTBULB_NAME_VALUE,                  0,      sizeof(btle_homekit_lightbulb_name),               sizeof(btle_homekit_lightbulb_name),               btle_homekit_lightbulb_name},
    {HDLC_LIGHTBULB_HUE_VALUE,                   0,      sizeof(btle_homekit_lightbulb_hue),                sizeof(btle_homekit_lightbulb_hue),                btle_homekit_lightbulb_hue},
    {HDLD_LIGHTBULB_HUE_CLNT_CHAR_CFG,           RNWN,   sizeof(btle_homekit_lightbulb_hue_client_cfg),     sizeof(btle_homekit_lightbulb_hue_client_cfg),     btle_homekit_lightbulb_hue_client_cfg},
    {HDLC_LIGHTBULB_SATURATION_VALUE,            0,      sizeof(btle_homekit_lightbulb_saturation),         sizeof(btle_homekit_lightbulb_saturation),         btle_homekit_lightbulb_saturation},
    {HDLD_LIGHTBULB_SATURATION_CLNT_CHAR_CFG,    RNWN,   sizeof(btle_homekit_lightbulb_saturation_client_cfg), sizeof(btle_homekit_lightbulb_saturation_client_cfg), btle_homekit_lightbulb_saturation_client_cfg},
    {HDLC_PAIRING_FEATURES_VALUE,                RNWD,   1,      1,      btle_homekit_pair_service_features},
#if defined(HSENS_DEFINED)
    {HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,        0,      sizeof(btle_homekit_lightbulb_on),                 sizeof(btle_homekit_lightbulb_on),                 btle_homekit_lightbulb_on},
#endif
#ifdef OTA_FIRMWARE_UPGRADE
    {HDLC_OTA_FW_UPGRADE_APP_INFO_VALUE,         RVWD,   sizeof(btle_homekit_lightbulb_fw_upgrade_app_info),sizeof(btle_homekit_lightbulb_fw_upgrade_app_info),(uint8_t*)btle_homekit_lightbulb_fw_upgrade_app_info},
#endif
};
#define LIGHTBULB_GATT_DB_EXT_ATTR_TBL_SIZE ( sizeof( btle_homekit_lightbulb_gatt_db_attr_tbl ) / sizeof( gatt_db_lookup_table ) )

wiced_btle_hap_char_params_t hap_char_tbl[] =
{
    /* characteristic value handle, HAP characteristic properties, presentation format, unit */

    /* accessory info service characteristics */
    { HDLC_ACCESSORY_INFO_IDENTIFY_VALUE, HDLD_ACCESSORY_INFO_IDENTIFY_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_WRITE,
      HAP_CHARACTERISTIC_FORMAT_BOOL, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_ACCESSORY_INFO_MANUFACTURER_VALUE, HDLD_ACCESSORY_INFO_MANUFACTURER_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_ACCESSORY_INFO_MODEL_VALUE, HDLD_ACCESSORY_INFO_MODEL_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_ACCESSORY_INFO_NAME_VALUE, HDLD_ACCESSORY_INFO_NAME_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_ACCESSORY_INFO_SERIAL_NUMBER_VALUE, HDLD_ACCESSORY_INFO_SERIAL_NUMBER_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_ACCESSORY_INFO_FIRMWARE_REVISION_VALUE, HDLD_ACCESSORY_INFO_FIRMWARE_REVISION_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },

    /* protocol information service characteristics */
    { HDLC_PROTOCOL_INFO_VERSION_VALUE, HDLD_PROTOCOL_INFO_VERSION_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },

    /* light bulb service characteristics */
    { HDLC_LIGHTBULB_SERVICE_SIGNATURE_VALUE, HDLD_LIGHTBULB_SERVICE_SIGNATURE_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_READ,
      HAP_CHARACTERISTIC_FORMAT_TLV8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_LIGHTBULB_BRIGHTNESS_VALUE, HDLD_LIGHTBULB_BRIGHTNESS_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ | HAP_CHARACTERISTIC_PROPERTY_SECURE_WRITE | HAP_CHARACTERISTIC_PROPERTY_NOTIFY_ALWAYS,
      HAP_CHARACTERISTIC_FORMAT_INT32, HAP_CHARACTERISTIC_FORMAT_UNIT_PERCENTAGE, 0, 100, 1 },
    { HDLC_LIGHTBULB_ON_VALUE, HDLD_LIGHTBULB_ON_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ | HAP_CHARACTERISTIC_PROPERTY_SECURE_WRITE | HAP_CHARACTERISTIC_PROPERTY_NOTIFY_ALWAYS,
      HAP_CHARACTERISTIC_FORMAT_BOOL, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_LIGHTBULB_NAME_VALUE, HDLD_LIGHTBULB_NAME_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,
      HAP_CHARACTERISTIC_FORMAT_STRING, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
	{ HDLC_LIGHTBULB_HUE_VALUE, HDLD_LIGHTBULB_HUE_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ | HAP_CHARACTERISTIC_PROPERTY_SECURE_WRITE | HAP_CHARACTERISTIC_PROPERTY_NOTIFY_ALWAYS,
	  HAP_CHARACTERISTIC_FORMAT_FLOAT, HAP_CHARACTERISTIC_FORMAT_UNIT_ARCDEGREES, 0, FLOAT_VALUE_360_BYTE_ARRAY, FLOAT_VALUE_1_BYTE_ARRAY },
	{ HDLC_LIGHTBULB_SATURATION_VALUE, HDLD_LIGHTBULB_SATURATION_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ | HAP_CHARACTERISTIC_PROPERTY_SECURE_WRITE | HAP_CHARACTERISTIC_PROPERTY_NOTIFY_ALWAYS,
	  HAP_CHARACTERISTIC_FORMAT_FLOAT, HAP_CHARACTERISTIC_FORMAT_UNIT_PERCENTAGE, 0, FLOAT_VALUE_100_BYTE_ARRAY, FLOAT_VALUE_1_BYTE_ARRAY },

    /* pairing service characteristics */
    { HDLC_PAIRING_PAIR_SETUP_VALUE, HDLD_PAIRING_PAIR_SETUP_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_READ | HAP_CHARACTERISTIC_PROPERTY_WRITE,
      HAP_CHARACTERISTIC_FORMAT_TLV8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_PAIRING_PAIR_VERIFY_VALUE, HDLD_PAIRING_PAIR_VERIFY_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_READ | HAP_CHARACTERISTIC_PROPERTY_WRITE,
      HAP_CHARACTERISTIC_FORMAT_TLV8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_PAIRING_FEATURES_VALUE, HDLD_PAIRING_FEATURES_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_READ,
      HAP_CHARACTERISTIC_FORMAT_UINT8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_PAIRING_MANAGE_VALUE, HDLD_PAIRING_MANAGE_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ | HAP_CHARACTERISTIC_PROPERTY_SECURE_WRITE,
      HAP_CHARACTERISTIC_FORMAT_TLV8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },

#ifdef OTA_FIRMWARE_UPGRADE
#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_HAP
    { HDLC_OTA_FW_UPGRADE_SERVICE_SIGNATURE_VALUE, HDLD_OTA_FW_UPGRADE_SERVICE_SIGNATURE_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_READ,
      HAP_CHARACTERISTIC_FORMAT_TLV8, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_OTA_FW_UPGRADE_CONTROL_POINT_VALUE, HDLD_OTA_FW_UPGRADE_CONTROL_POINT_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ | HAP_CHARACTERISTIC_PROPERTY_SECURE_WRITE | HAP_CHARACTERISTIC_PROPERTY_NOTIFY_ALWAYS,
      HAP_CHARACTERISTIC_FORMAT_DATA, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_OTA_FW_UPGRADE_DATA_VALUE, HDLD_OTA_FW_UPGRADE_DATA_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ | HAP_CHARACTERISTIC_PROPERTY_SECURE_WRITE,
      HAP_CHARACTERISTIC_FORMAT_DATA, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
    { HDLC_OTA_FW_UPGRADE_APP_INFO_VALUE, HDLD_OTA_FW_UPGRADE_APP_INFO_INSTANCE_ID, HAP_CHARACTERISTIC_PROPERTY_SECURE_READ,
      HAP_CHARACTERISTIC_FORMAT_DATA, HAP_CHARACTERISTIC_FORMAT_UNIT_UNITLESS, 0, 0, 0 },
#endif
#endif
};
#define HAP_CHAR_TBL_SIZE (sizeof(hap_char_tbl) / sizeof(wiced_btle_hap_char_params_t))

typedef struct
{
    const char* name;
} char_name_t;

/* this table must match hap_char_tbl item by item */
static char_name_t hap_char_names[] =
{
    { "Identify" },
    { "Manufacturer" },
    { "Model" },
    { "Name" },
    { "Serial Number" },
    { "Firmware Revision" },

    { "Protocol Info Version" },

    { "Lightbulb Service Signature" },
    { "Lightbulb Brightness" },
    { "Lightbulb On" },
    { "Lightbulb Name" },
    { "Lightbulb Hue" },
    { "Lightbulb Saturation" },
};

wiced_btle_instance_id_t hap_instance_id_tbl[] =
{
    /* handle, instance ID */

    /* accessory info service and characteristics */
	{ HDLC_ACCESSORY_INFO_INSTANCE_ID_VALUE,                1 },
	{ HDLD_ACCESSORY_INFO_IDENTIFY_INSTANCE_ID,             2 },
	{ HDLD_ACCESSORY_INFO_MANUFACTURER_INSTANCE_ID,         3 },
	{ HDLD_ACCESSORY_INFO_MODEL_INSTANCE_ID,                4 },
	{ HDLD_ACCESSORY_INFO_NAME_INSTANCE_ID,                 5 },
	{ HDLD_ACCESSORY_INFO_SERIAL_NUMBER_INSTANCE_ID,        6 },
	{ HDLD_ACCESSORY_INFO_FIRMWARE_REVISION_INSTANCE_ID,    7 },

	/* protocol information service and characteristics */
	{ HDLC_PROTOCOL_INFO_INSTANCE_ID_VALUE,                 10 },
	{ HDLD_PROTOCOL_INFO_VERSION_INSTANCE_ID,               11 },

	/* light bulb service and characteristics */
	{ HDLC_LIGHTBULB_INSTANCE_ID_VALUE,                     20 },
	{ HDLD_LIGHTBULB_SERVICE_SIGNATURE_INSTANCE_ID,         21 },
	{ HDLD_LIGHTBULB_BRIGHTNESS_INSTANCE_ID,                22 },
	{ HDLD_LIGHTBULB_ON_INSTANCE_ID,                        23 },
	{ HDLD_LIGHTBULB_NAME_INSTANCE_ID,                      24 },
	{ HDLD_LIGHTBULB_HUE_INSTANCE_ID,                       25 },
	{ HDLD_LIGHTBULB_SATURATION_INSTANCE_ID,                26 },

    /* pairing service and characteristics */
    { HDLC_PAIRING_SERVICE_INSTANCE_VALUE,                  30 },
    { HDLD_PAIRING_PAIR_SETUP_INSTANCE_ID,                  31 },
    { HDLD_PAIRING_PAIR_VERIFY_INSTANCE_ID,                 32 },
    { HDLD_PAIRING_FEATURES_INSTANCE_ID,                    33 },
    { HDLD_PAIRING_MANAGE_INSTANCE_ID,                      34 },

#ifdef OTA_FIRMWARE_UPGRADE
#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_HAP
    { HDLC_OTA_FW_UPGRADE_SERVICE_INSTANCE_VALUE,           1000 },
    { HDLD_OTA_FW_UPGRADE_SERVICE_SIGNATURE_INSTANCE_ID,    1001 },
    { HDLD_OTA_FW_UPGRADE_CONTROL_POINT_INSTANCE_ID,        1002 },
    { HDLD_OTA_FW_UPGRADE_DATA_INSTANCE_ID,                 1003 },
    { HDLD_OTA_FW_UPGRADE_APP_INFO_INSTANCE_ID,             1004 },
#endif
#endif
};
#define HAP_INSTANCE_ID_TBL_SIZE (sizeof(hap_instance_id_tbl) / sizeof(wiced_btle_instance_id_t))


static btle_homekit_lightbulb_cb_t btle_homekit_lightbulb_cb;

// finds characteristic by its handle in the table btle_homekit_lightbulb_gatt_db_attr_tbl
// returns pointer to found characteristic.
// returns NULL on error
static gatt_db_lookup_table* find_char_in_table(uint16_t handle)
{
    gatt_db_lookup_table* res = NULL;
    int         i;
    for (i = 0; i < LIGHTBULB_GATT_DB_EXT_ATTR_TBL_SIZE; i++)
    {
        if (btle_homekit_lightbulb_gatt_db_attr_tbl[i].handle == handle)
        {
            res = &btle_homekit_lightbulb_gatt_db_attr_tbl[i];
            break;
        }
    }
    return res;
}

/******************************************************
 *               Function Prototypes
 ******************************************************/
static void                   btle_homekit_lightbulb_app_init               ( );
static wiced_bt_gatt_status_t btle_homekit_lightbulb_gatt_event_handler     ( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data );

/* GATT registration call-backs */
static wiced_bt_gatt_status_t btle_homekit_lightbulb_write_handler          ( wiced_bt_gatt_write_t *p_buf, uint16_t conn_id );
static wiced_bt_gatt_status_t btle_homekit_lightbulb_read_handler           ( wiced_bt_gatt_read_t *p_buf, uint16_t conn_id );
static wiced_bt_gatt_status_t btle_homekit_lightbulb_gatt_connect_callback  ( uint8_t* bda, uint16_t conn_id, wiced_bool_t connected, wiced_bt_gatt_disconn_reason_t reason );
static wiced_bt_gatt_status_t btle_homekit_lightbulb_gatt_server_callback   ( uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data );
static wiced_result_t         btle_homekit_lightbulb_display_password       ( uint8_t* srp_pairing_password );
static void                   btle_homekit_lightbulb_timeout                ( uint32_t count );
static void                   btle_homekit_lightbulb_fine_timeout           ( uint32_t finecount );
#ifdef WICED_BT_SFLASH_TO_GPIO
static void                   btle_homekit_lightbulb_sflash_to_gpio_cback   ( uint8_t enable);
#endif
static wiced_result_t         btle_homekit_lightbulb_management_cback       ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_WINDOWS
static wiced_bool_t           btle_homekit_lightbulb_save_link_keys         (wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t           btle_homekit_lightbulb_read_link_keys         (wiced_bt_device_link_keys_t *p_keys);
#endif

static void                   btle_homekit_lightbulb_hap_request_callback   (uint16_t conn_id, uint16_t handle, uint8_t* p_data, uint16_t len);

static wiced_bt_gatt_status_t btle_homekit_set_value                        (uint16_t handle, uint8_t *p_val, uint16_t len);
static void                   btle_homekit_check_and_send_notification      (uint16_t handle);
static void                   btle_homekit_hci_send_characteristic_value    (uint16_t event, uint16_t handle, uint8_t* p_data, uint16_t data_len);
static void                   btle_homekit_lightbulb_factory_reset          ( );

/******************************************************
 *               Variable Definitions
 ******************************************************/
uint16_t btle_homekit_conn_id = 0;               // not zero if we are connected

/******************************************************
 *               Function Definitions
 ******************************************************/

////////////////////// TEMPORARY STUFF  START   /////////////////////////
//! The overlay manager's internal state.
typedef struct
{
    //! The ID of the overlay that is currnetly loaded. 0xFF is a reserved value to indicate that
    //! no overlay is loaded.
    UINT8   currentOverlayId;

    //! The total number of overlays that are currently known.
    UINT8   numberOfOverlays;
} OVERLAY_MANAGER_STATE_t;

extern void overlay_manager_ProcessOverlayEntry(UINT8* p);
extern UINT32 Config_DS_Location;

void btle_homekit_identify_overlays(void)
{
    UINT8       *p;
    UINT8       *pConfigVar;
    UINT32      pTemp[15];
    UINT8       *pData;
    UINT8       itemType;
    UINT16      itemLen;
    UINT16      dataLen;

    p = (UINT8 *)Config_DS_Location;

    // Walk the entire config record. NOT EFFICIENT.
    while (p)
    {
        /* This is to make sure that the data is word alligned */
        pData = (UINT8 *)pTemp;

        /* Extract Configuration item Type and length fields */
        config_Read_ConfigData(p, pData, 3);
        itemType = pData[0];
        itemLen = pData[1] + (pData[2] << 8);
        p += 3;

        if (itemType == 0xFE ||
            itemType == 0x02 ||
            itemType == 0xFF)
        {
            /*
            ** DS_VS_OFFSETS item will be processed
            ** while detecting the Static section.
            ** So we don't have to do anything here except
            ** reporting the end of Static Section
            */
            return;
        }

#define OVERLAY_DATA_ITEM_ID       (0xF1)

        if (itemType == OVERLAY_DATA_ITEM_ID)
        {
            // If this is an overlay, process it. Note that p is advanced by size of header.
            overlay_manager_ProcessOverlayEntry(p);
        }

        p += itemLen;

        wdog_restart();
    }
}


/* Application Start, ie, entry point to the application.
 * It is mandatory for all the applications to define this
 */
APPLICATION_START()
{
    wiced_result_t status;

    // Identify overlays.
    btle_homekit_identify_overlays();

    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    //Set the debug uart to enable the debug traces
    //wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_DBG_UART);

    //Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    //wiced_set_debug_uart( WICED_ROUTE_DEBUG_NONE );

    // WICED_ROUTE_DEBUG_TO_WICED_UART to send debug strings over the WICED
    // debug interface
    //wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE("APPLICATION_START\n");

    // initialize homekit library and assign CATEGORY for advertising
    wiced_btle_homekit_init(HOMEKIT_ACCESSORY_CATEGORY_LIGHTBULB);
    // initialize BT stack and assign management callback function
    wiced_bt_stack_init(btle_homekit_lightbulb_management_cback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);

#ifdef WICED_BT_SFLASH_TO_GPIO
    wiced_btle_homekit_register_sflash_to_gpio_cback( &btle_homekit_lightbulb_sflash_to_gpio_cback );
#endif

    transport_pool = wiced_transport_create_buffer_pool ( TRANS_UART_BUFFER_SIZE, 2 );

    if (wiced_hal_read_nvram(BTLE_HOMEKIT_NVRAM_VSID_CHAR_VALUES, sizeof(nvram_char_values), (uint8_t *)&nvram_char_values, &status) == sizeof(nvram_char_values))
    {
        btle_homekit_lightbulb_brightness[0] = nvram_char_values.brightness;
        btle_homekit_lightbulb_on[0] = nvram_char_values.on;
        memcpy(btle_homekit_lightbulb_hue, &nvram_char_values.hue, 4);
        memcpy(btle_homekit_lightbulb_saturation, &nvram_char_values.saturation, 4);
    }
    else
    {
        nvram_char_values.brightness = btle_homekit_lightbulb_brightness[0];
        nvram_char_values.on = btle_homekit_lightbulb_on[0];
        memcpy(&nvram_char_values.hue, btle_homekit_lightbulb_hue, 4);
        memcpy(&nvram_char_values.saturation, btle_homekit_lightbulb_saturation, 4);
    }

    WICED_BT_TRACE("APPLICATION_STARTexits\n");
}

// management callback function - it will be called by BT stack on management events
static wiced_result_t btle_homekit_lightbulb_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    uint8_t        *p_keys;

    WICED_BT_TRACE("management_cback: event:%d\n", event);

    switch( event )
    {
        /* Bluetooth stack enabled */
        case BTM_ENABLED_EVT:
            btle_homekit_lightbulb_app_init();
            break;

#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_WINDOWS
        /* If upgrade is done from Windows this sample app relies on standard BT security.
         * This requires to support standard BT SMP protocol messages. */
        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            btle_homekit_lightbulb_save_link_keys(&p_event_data->paired_device_link_keys_update);
            wiced_bt_dev_add_device_to_address_resolution_db(&p_event_data->paired_device_link_keys_update, p_event_data->paired_device_link_keys_update.key_data.ble_addr_type);
            break;

         case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            if (btle_homekit_lightbulb_read_link_keys(&p_event_data->paired_device_link_keys_request))
            {
                WICED_BT_TRACE("Key retrieval success\n");
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n");
            }
            break;

         case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
             /* save keys to NVRAM */
             p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
             wiced_hal_write_nvram ( BTLE_HOMEKIT_NVRAM_VSID_LOCAL_KEYS, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
             WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
             break;

         case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
             /* read keys from NVRAM */
             p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
             wiced_hal_read_nvram( BTLE_HOMEKIT_NVRAM_VSID_LOCAL_KEYS, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
             WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
             break;
#else
        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* Use the default security*/
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;

        case BTM_SECURITY_REQUEST_EVT:
            /* Use the default security*/
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* Use the default security*/
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Request to store newly generated local identity keys to NVRAM */
            /* (sample app does not store keys to NVRAM) */
            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* Request to restore local identity keys from NVRAM (requested during Bluetooth start up) */
            /* (sample app does not store keys to NVRAM. New local identity keys will be generated).   */
            result = WICED_BT_NO_RESOURCES;
            break;
#endif

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            WICED_BT_TRACE( "Advertisement State Change:%d\n", p_event_data->ble_advert_state_changed );

            // if we are not connected, adverts should be kept LOW
            if ((p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF) &&
                (btle_homekit_conn_id == 0))
            {
                wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
            }
            break;

        default:
            break;
    }

    return result;

}

#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_WINDOWS
/*
 * This function is called to save keys generated as a result of pairing or keys update
 */
static wiced_bool_t btle_homekit_lightbulb_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t          bytes_written;
    wiced_result_t   result;

    bytes_written = wiced_hal_write_nvram(BTLE_HOMEKIT_NVRAM_VSID_PEER_KEYS, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("Saved %d bytes at id:%d\n", bytes_written, BTLE_HOMEKIT_NVRAM_VSID_PEER_KEYS);
    return (bytes_written == sizeof (wiced_bt_device_link_keys_t));
}

/*
 * This function is called to read keys for specific bdaddr
 */
static wiced_bool_t btle_homekit_lightbulb_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t         bytes_read;
    wiced_result_t  result;

    bytes_read = wiced_hal_read_nvram(BTLE_HOMEKIT_NVRAM_VSID_PEER_KEYS, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("read %d bytes at id:%d\n", bytes_read, BTLE_HOMEKIT_NVRAM_VSID_PEER_KEYS);
    return (bytes_read == sizeof (wiced_bt_device_link_keys_t));
}
#endif

#ifdef WICED_BT_SFLASH_TO_GPIO
void btle_homekit_lightbulb_sflash_to_gpio_cback(uint8_t enable)
{

    if( enable )
    {
        WICED_BT_TRACE("Enabling hal_sflash_init_spi_to_gpio\n");
        hal_sflash_enable_spi_to_gpio(enable);
    }
    else
    {
        WICED_BT_TRACE("Disabling hal_sflash_init_spi_to_gpio\n");
        hal_sflash_enable_spi_to_gpio(enable);
    }
}
#endif

// increments every second in btle_homekit_lightbulb_timeout()
static uint32_t app_timer_count = 0;
// if not 0 then it means time when button is pressed to measure interval till it is released
static uint32_t app_timer_count_at_button_pressed = 0;
// timed write timer
static uint8_t timed_write_counter = 0;
// connection idle counter
static uint8_t connection_idle_counter = 0;

void btle_homekit_lightbulb_interrupt_handler(void *user_data, uint8_t value )
{
    wiced_result_t  result;

    WICED_BT_TRACE("interrupt. now==%d pressed at=%d\n", app_timer_count, app_timer_count_at_button_pressed);

    if ( wiced_hal_gpio_get_pin_input_status(WICED_GPIO_BUTTON) == WICED_BUTTON_PRESSED_VALUE)
    {
        wiced_printf(NULL, 0, "button pressed - keep pressed for 10 seconds to reset\n");
        app_timer_count_at_button_pressed = app_timer_count;
    }
    else
    {
        app_timer_count_at_button_pressed = 0;
    }
}

// disable pairing (return false) if we are paired already
static wiced_bool_t is_pair_allowed()
{
    wiced_bool_t res = wiced_homekit_is_accessory_paired();
    return !res;
}

// return true if encryption is required in pair-verified session
wiced_bool_t wiced_btle_homekit_is_encrypted_if_verified(uint16_t handle)
{
    gatt_db_lookup_table* p_char = find_char_in_table(handle);

    if (p_char)
    {
        if ((p_char->encr_required & 0x0f) == BTLE_HK_ENC_NEVER ||
            (p_char->encr_required & 0xf0) == (BTLE_HK_ENC_NEVER << 4))
        {
            return WICED_FALSE;
        }
    }

    return WICED_TRUE;
}

/* This function performs homekit lightbulb app when stack is ready */
void btle_homekit_lightbulb_app_init(void)
{
    wiced_bt_gatt_status_t  gatt_status;
    wiced_result_t result;
    uint8_t buf[16];
    wiced_bt_device_address_t bda;

#ifdef WICED_BT_TRACE_ENABLE
    hci_control_enable_uart( );
#endif

#ifdef USE_STATIC_RANDOM_ADDRESS
    if (wiced_hal_read_nvram(BTLE_HOMEKIT_NVRAM_BDA_STATIC_RANDOM, sizeof(wiced_bt_device_address_t), bda, &result) != sizeof(wiced_bt_device_address_t))
    {
        uint32_t r1[2];
        r1[0] = rbg_rand();
        r1[1] = rbg_rand();
        memcpy (bda, (uint8_t *)r1, 6);

        /* Valid static random address should have 2 most significant bits set to 1 */
        bda[0] |= 0xc0;
        wiced_hal_write_nvram(BTLE_HOMEKIT_NVRAM_BDA_STATIC_RANDOM, sizeof(wiced_bt_device_address_t), bda, &result);
    }
    WICED_BT_TRACE("setting static random bda %B \n", bda);
    wiced_bt_set_local_bdaddr(bda , BLE_ADDR_RANDOM);
#endif

    memset(&btle_homekit_lightbulb_cb, 0, sizeof(btle_homekit_lightbulb_cb_t));

    wiced_bt_app_init(); /* app init */

    /* rather than register GATT callback with the stack, pass the callback to the HomeKit so
     * that it performs encryption/descryption of data before passing it here */
    wiced_btle_homekit_start( btle_homekit_lightbulb_gatt_event_handler, wiced_btle_homekit_is_encrypted_if_verified );

    // set homekit advertisement
    wiced_btle_homekit_set_advertisement_data();

    /* Enable discoverability */
    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements: result:0x%x\n", result);

    /*  GATT DB Initialization  */
    gatt_status =  wiced_bt_gatt_db_init( apple_lightbulb_btle_homekit_gatt_db, sizeof( apple_lightbulb_btle_homekit_gatt_db ) );
    //WICED_BT_TRACE( "wiced_bt_gatt_db_init 0x%x\n", gatt_status);

    wiced_btle_homekit_gatt_db_init();

    wiced_btle_homekit_register_hap_characteristics(hap_char_tbl, HAP_CHAR_TBL_SIZE, btle_homekit_lightbulb_hap_request_callback);

    wiced_btle_homekit_register_hap_instance_ids(hap_instance_id_tbl, HAP_INSTANCE_ID_TBL_SIZE);

    wiced_btle_homekit_set_service_properties(HDLC_LIGHTBULB_SERVICE_SIGNATURE_VALUE, HAP_SERVICE_PROPERTY_PRIMARY_SERVICE);

#ifdef OTA_FIRMWARE_UPGRADE
#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_HAP
    wiced_btle_homekit_set_service_properties(HDLC_OTA_FW_UPGRADE_SERVICE_SIGNATURE_VALUE, HAP_SERVICE_PROPERTY_HIDDEN_SERVICE);
#endif
#endif

    /* Starting the app timers , seconds timer and the ms timer  */
    wiced_bt_app_start_timer( BTLE_HOMEKIT_LIGHTBULB_APP_TIMEOUT_IN_SECONDS, BTLE_HOMEKIT_LIGHTBULB_APP_FINE_TIMEOUT_IN_MILLI_SECONDS,
    		btle_homekit_lightbulb_timeout, btle_homekit_lightbulb_fine_timeout );

    /* Register for gpio/button events */
    wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_BOTH_EDGE ), WICED_GPIO_BUTTON_DEFAULT_STATE );
    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_BUTTON, btle_homekit_lightbulb_interrupt_handler, NULL );

//SETUP_CODE in build target defines hardcoded password in form NNNNNNNN (8 decimal digits)
//if it is not defined then app assumes accessory has a display and prints password to the trace during pairing
#ifdef WICED_HOMEKIT_SETUP_CODE
	{
		//set password in format NNN-NN-NNN
		char buf[16];
		sprintf(buf, "%03d-%02d-%03d",
			WICED_HOMEKIT_SETUP_CODE/100000,
			WICED_HOMEKIT_SETUP_CODE / 1000 % 100,
			WICED_HOMEKIT_SETUP_CODE % 1000);
		WICED_BT_TRACE("SETUP_CODE=%s\n", buf);
		wiced_configure_accessory_password_for_device_with_no_display(buf);
	}
#else
	wiced_configure_accessory_password_for_device_with_display(btle_homekit_lightbulb_display_password);
#endif

	//disable watch-dog. TBD: Replace by configuration when it will be available
     //wiced_hal_wdog_disable();

    //Prepare the stack to allow the app to check for stack overflow
    wiced_bt_stack_check_init();

    // to disable pairing if we are paired already set the callback function which returns 0 if it is paired already
    wiced_btle_homekit_set_is_pair_allowed_callback(is_pair_allowed);

#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_WINDOWS
    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);
#endif

#if defined(HSENS_LED_PIN)
    WICED_BT_TRACE("JH> led:%d\n", nvram_char_values.on);
    wiced_hal_gpio_set_pin_output(HSENS_LED_PIN,
            nvram_char_values.on ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH);
#endif

    WICED_BT_TRACE("_app_init exits\n");
}

/* The function invoked on timeout of app seconds timer.*/
void btle_homekit_lightbulb_timeout( uint32_t count )
{
    app_timer_count++;
    //WICED_BT_TRACE("timeout %d\n", app_timer_count);
    //wiced_printf(NULL, 0, "timeout %d mm_top:0x%x\n", app_timer_count, mm_top);

    // Reset accessory if button has been pressed more then 10 seconds
    if (app_timer_count_at_button_pressed != 0
        && app_timer_count - app_timer_count_at_button_pressed > 10)
    {
        app_timer_count_at_button_pressed = 0;
        wiced_printf(NULL, 0, "hard reset\n");
        btle_homekit_lightbulb_factory_reset();
    }

    wiced_btle_homekit_timer();

    if (!wiced_btle_homekit_is_fw_updating() && connection_idle_counter != 0 && --connection_idle_counter == 0)
    {
        WICED_BT_TRACE("JH> idle to disconnect\n");
        wiced_bt_gatt_disconnect(btle_homekit_conn_id);
    }
}

/* The function invoked on timeout of app milliseconds fine timer */
void btle_homekit_lightbulb_fine_timeout( uint32_t finecount )
{
    if (timed_write_counter && --timed_write_counter == 0)
    {
        if (btle_homekit_lightbulb_cb.p_timed_write)
        {
            free(btle_homekit_lightbulb_cb.p_timed_write);
            btle_homekit_lightbulb_cb.p_timed_write = NULL;
        }
    }
}

void btle_homekit_refresh_conn_idle_counter()
{
// Deleted to be no idle, 03Nov17, Jack Huang
#if !defined(DEV_NOIDLE)   // Begin
    connection_idle_counter = BTLE_HOMEKIT_CONN_IDLE_TIMEOUT;
#endif   // End of deleted, 03Nov17, Jack Huang
}

static wiced_result_t btle_homekit_lightbulb_display_password( uint8_t* srp_pairing_password )
{

	wiced_printf(NULL, 0, "Pairing Password: %s\n", (char*)srp_pairing_password);
    return WICED_SUCCESS;
}

#ifdef WICED_BT_TRACE_ENABLE
const char* h2name(uint16_t handle)
{
    const char *res = NULL;
    switch (handle)
    {
    case HDLC_PAIRING_FEATURES_VALUE:
        res = "PAIRING_FEATURES";
        break;
    case HDLC_GAP_DEVICE_NAME_VALUE:
        res = "DEVICE_NAME";
        break;
    case HDLC_GAP_APPEARANCE_NAME_VALUE:
        res = "APPEARANCE_NAME";
        break;
    case HDLC_ACCESSORY_INFO_INSTANCE_ID_VALUE:
        res = "INSTANCE_ID";
        break;
    case HDLC_ACCESSORY_INFO_IDENTIFY_VALUE:
        res = "IDENTIFY";
        break;
    case HDLC_ACCESSORY_INFO_MANUFACTURER_VALUE:
        res = "MANUFACTURER";
        break;
    case HDLC_ACCESSORY_INFO_MODEL_VALUE:
        res = "MODEL";
        break;
    case HDLC_ACCESSORY_INFO_NAME_VALUE:
        res = "NAME";
        break;
    case HDLC_ACCESSORY_INFO_SERIAL_NUMBER_VALUE:
        res = "SERIAL_NUMBER";
        break;
    case HDLC_LIGHTBULB_INSTANCE_ID_VALUE:
        res = "INSTANCE_ID";
        break;
    case HDLD_LIGHTBULB_BRIGHTNESS_INSTANCE_ID:
        res = "BRIGHTNESS_INSTANCE_ID";
        break;
    case HDLD_LIGHTBULB_ON_INSTANCE_ID:
        res = "ON_INSTANCE_ID";
        break;
    case HDLC_LIGHTBULB_BRIGHTNESS_VALUE:
        res = "BRIGHTNESS";
        break;
    case HDLC_LIGHTBULB_ON_VALUE:
        res = "ON";
        break;
    case HDLD_LIGHTBULB_HUE_INSTANCE_ID:
        res = "HUE_INSTANCE_ID";
        break;
    case HDLC_LIGHTBULB_HUE_VALUE:
        res = "HUE";
        break;
    case HDLD_LIGHTBULB_SATURATION_INSTANCE_ID:
        res = "SATURATION_INSTANCE_ID";
        break;
    case HDLC_LIGHTBULB_SATURATION_VALUE:
        res = "SATURATION";
        break;
#ifdef OTA_FIRMWARE_UPGRADE
#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_WINDOWS
    case HANDLE_OTA_FW_UPGRADE_CONTROL_POINT:
        res = "OTA_CONTROL_POINT";
        break;
    case HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR:
        res = "OTA_CLIENT_CONFIGURATION_DESCRIPTOR";
        break;
    case HANDLE_OTA_FW_UPGRADE_DATA:
        res = "OTA_DATA";
        break;
    case HANDLE_OTA_FW_UPGRADE_APP_INFO:
        res = "OTA_APP_INFO";
        break;
#endif
#ifdef BTLE_HOMEKIT_OTA_UPGRADE_FROM_HAP
    case HDLC_OTA_FW_UPGRADE_CONTROL_POINT_VALUE:
        res = "HAP_OTA_CONTROL_POINT";
        break;
    case HDLD_OTA_FW_UPGRADE_CONTROL_POINT_CLNT_CHAR_CFG:
        res = "HAP_OTA_CLIENT_CONFIGURATION_DESCRIPTOR";
        break;
    case HDLC_OTA_FW_UPGRADE_DATA_VALUE:
        res = "HAP_OTA_DATA";
        break;
    case HDLC_OTA_FW_UPGRADE_APP_INFO_VALUE:
        res = "HAP_OTA_APP_INFO";
        break;
#endif
#endif
    }
    return res;
}

void printKnownHandleData(int write, uint16_t handle, uint8_t* p_val, uint32_t val_len)
{
    const char* name;
    switch (handle)
{
    case HDLC_LIGHTBULB_BRIGHTNESS_VALUE:
        if (write)
            wiced_printf(NULL, 0, "LIGHTBULB_BRIGHTNESS changed to %d\n", *p_val);
        else
            wiced_printf(NULL, 0, "LIGHTBULB_BRIGHTNESS is %d\n", *p_val);
        break;
    case HDLC_LIGHTBULB_ON_VALUE:
        if (write)
            wiced_printf(NULL, 0, "LIGHTBULB turned %s\n", *p_val ? "ON" : "OFF");
        else
            wiced_printf(NULL, 0, "LIGHTBULB is %s\n", *p_val ? "ON" : "OFF");
        break;
    case HDLC_LIGHTBULB_HUE_VALUE:
        if (write)
            wiced_printf(NULL, 0, "LIGHTBULB_HUE changed to %02x %02x %02x %02x\n", p_val[0], p_val[1], p_val[2], p_val[3]);
        else
            wiced_printf(NULL, 0, "LIGHTBULB_HUE is %02x %02x %02x %02x\n", p_val[0], p_val[1], p_val[2], p_val[3]);
        break;
    case HDLC_LIGHTBULB_SATURATION_VALUE:
        if (write)
            wiced_printf(NULL, 0, "LIGHTBULB_SATURATION changed to %02x %02x %02x %02x\n", p_val[0], p_val[1], p_val[2], p_val[3]);
        else
            wiced_printf(NULL, 0, "LIGHTBULB_SATURATION is %02x %02x %02x %02x\n", p_val[0], p_val[1], p_val[2], p_val[3]);
        break;
    default:
        name = h2name(handle);
        if (name)
        {
            if (write)
                wiced_printf(NULL, 0, "***** write %s: ", name);
            else
                wiced_printf(NULL, 0, "***** read %s: ", name);
            //tracen("", p_val, val_len);
        }
    }
}
#endif

uint8_t parseTargetState(uint8_t *p, uint16_t len, uint8_t *p_val)
{
    uint8_t val;
    uint8_t found = 0;
    uint8_t foundRemote = 0;
    while (len)
    {
        if (len < 3 || len < p[1] + 2)
        {
            break;
        }
        // if it is value with 1 byte length
        if (p[0] == 0 && p[1] == 1)
        {
            // error if second value tlv is found
            if (found)
                break;
            val = p[2];
            found = 1;
        }
        // if it is remote with 1 byte length
        else if (p[0] == 2 && p[1] == 1)
        {
            // error if scond remoet tlv is found
            if (foundRemote)
                break;
            foundRemote = 1;
        }
        // ignore authirization TLVs - can be few
        else if (p[0] == 1)
        {
        }
        // fail on unknown TLV
        else
            break;
        len -= p[1] + 2;
        p += p[1] + 2;
    }
    if (!len && found && foundRemote)
    {
        *p_val = val;
    }
    return (!len && found && foundRemote) ? 1 : 0;
}

wiced_result_t btle_homekit_check_value(uint16_t handle, uint8_t *p_val, uint16_t len)
{
    uint32_t value32;
    wiced_result_t result = WICED_SUCCESS;

    switch (handle)
    {
    case HDLC_LIGHTBULB_BRIGHTNESS_VALUE:
        value32 = (uint32_t)(p_val[0] + (p_val[1] << 8) + (p_val[2] << 16) + (p_val[3] << 24));
        if (value32 > 100)
            result = WICED_BADVALUE;
        break;
    case HDLC_LIGHTBULB_HUE_VALUE:
        value32 = (uint32_t)(p_val[0] + (p_val[1] << 8) + (p_val[2] << 16) + (p_val[3] << 24));
        if (value32 > FLOAT_VALUE_360_BYTE_ARRAY)
            result = WICED_BADVALUE;
        break;
    case HDLC_LIGHTBULB_SATURATION_VALUE:
        value32 = (uint32_t)(p_val[0] + (p_val[1] << 8) + (p_val[2] << 16) + (p_val[3] << 24));
        if (value32 > FLOAT_VALUE_100_BYTE_ARRAY)
            result = WICED_BADVALUE;
        break;
    }

    return result;
}

wiced_btle_hap_char_params_t* btle_homekit_find_hap_characteristic(uint16_t handle)
{
    int i;
    wiced_btle_hap_char_params_t* p_char = NULL;

    for (i = 0; i < HAP_CHAR_TBL_SIZE; i++)
    {
        if (hap_char_tbl[i].handle == handle)
        {
            p_char = &hap_char_tbl[i];
            break;
        }
    }

    return p_char;
}

wiced_bt_gatt_status_t btle_homekit_set_value(uint16_t handle, uint8_t *p_val, uint16_t len)
{
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;
    gatt_db_lookup_table* p_char;
    uint8_t *p;
    wiced_result_t status;
    wiced_bool_t value_changed = WICED_FALSE;
    wiced_bool_t write_to_nvram = WICED_TRUE;
    wiced_btle_hap_char_params_t* p_hap_char;

    // Check for a matching handle entry
    if ((p_char = find_char_in_table(handle)) != NULL)
    {
        // Verify that size constraints have been met
        if (p_char->max_len >= len)
        {
            if (btle_homekit_check_value(handle, p_val, len) == WICED_SUCCESS)
            {
                // Value fits within the supplied buffer; copy over the value
                if (p_char->cur_len != len || memcmp(p_char->p_data, p_val, len) != 0)
                {
                    p_char->cur_len = len;
                    memcpy(p_char->p_data, p_val, len);
                    value_changed = WICED_TRUE;

                    if (handle == HDLC_LIGHTBULB_BRIGHTNESS_VALUE)
                    {
                        nvram_char_values.brightness = btle_homekit_lightbulb_brightness[0];
                    }
                    else if (handle == HDLC_LIGHTBULB_ON_VALUE)
                    {
                        nvram_char_values.on = btle_homekit_lightbulb_on[0];
#if defined(HSENS_LED_PIN)
                        wiced_hal_gpio_set_pin_output(HSENS_LED_PIN,
                                nvram_char_values.on ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH);
#endif
                    }
                    else if (handle == HDLC_LIGHTBULB_HUE_VALUE)
                    {
                        memcpy(&nvram_char_values.hue, btle_homekit_lightbulb_hue, 4);
                    }
                    else if (handle == HDLC_LIGHTBULB_SATURATION_VALUE)
                    {
                        memcpy(&nvram_char_values.saturation, btle_homekit_lightbulb_saturation, 4);
                    }
#if defined(HSENS_DEFINED)
                    else  if (handle == HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL)  {
                        nvram_char_values.on = btle_homekit_lightbulb_on[0];
#if defined(HSENS_LED_PIN)
                        wiced_hal_gpio_set_pin_output(HSENS_LED_PIN,
                                nvram_char_values.on ? GPIO_PIN_OUTPUT_LOW : GPIO_PIN_OUTPUT_HIGH);
#endif
                    }
#endif
                    else
                    {
                        write_to_nvram = WICED_FALSE;
                    }
                    if (write_to_nvram)
                    {
                        wiced_hal_write_nvram(BTLE_HOMEKIT_NVRAM_VSID_CHAR_VALUES, sizeof(nvram_char_values), (uint8_t *)&nvram_char_values, &status);
                    }
                }

                res = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                res = WICED_BT_GATT_ILLEGAL_PARAMETER;
            }
        }
        else
        {
            // Value to write does not meet size constraints
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
    }

    if (value_changed)
    {
        p_hap_char = btle_homekit_find_hap_characteristic(handle);

        if (p_hap_char && (p_hap_char->properties & HAP_CHARACTERISTIC_PROPERTY_NOTIFY_ALWAYS))
        {
            // Tell library that the global state number may need to be changed.  The library
            // keeps the logic for changing global state number once while connected and once
            // while disconnected.  If GSN changed, we will change advertisements to high duty,
            // which will stay high for 5 seconds.
            if (wiced_btle_homekit_change_global_state_number(WICED_FALSE))
            {
                if (btle_homekit_conn_id == 0)
                    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
            }
        }
    }

    return res;
}

void btle_homekit_check_and_send_notification(uint16_t handle)
{
    uint8_t client_cfg;

    switch (handle)
    {
    case HDLC_LIGHTBULB_BRIGHTNESS_VALUE:
        client_cfg = btle_homekit_lightbulb_brightness_client_cfg[0];
        break;

    case HDLC_LIGHTBULB_ON_VALUE:
        client_cfg = btle_homekit_lightbulb_on_client_cfg[0];
        break;

    case HDLC_LIGHTBULB_HUE_VALUE:
        client_cfg = btle_homekit_lightbulb_hue_client_cfg[0];
        break;

    case HDLC_LIGHTBULB_SATURATION_VALUE:
        client_cfg = btle_homekit_lightbulb_saturation_client_cfg[0];
        break;

    default:
        return;
    }

    if (client_cfg & GATT_CLIENT_CONFIG_INDICATION)
    {
        wiced_btle_homekit_char_value_changed(handle);
    }
}


/* Handles Write Requests received from Client device */
static wiced_bt_gatt_status_t btle_homekit_lightbulb_write_handler( wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

#ifdef WICED_BT_TRACE_ENABLE
    printKnownHandleData(1, p_write_req->handle, p_write_req->p_val, p_write_req->val_len);
#endif

    status = btle_homekit_set_value(p_write_req->handle, p_write_req->p_val, p_write_req->val_len);

    return status;
}

/* Handles Read Requests received from Client device */
static wiced_bt_gatt_status_t btle_homekit_lightbulb_read_handler( wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id )
{
    int        i;
    uint16_t   status   = WICED_BT_GATT_INVALID_HANDLE;
    gatt_db_lookup_table* p_char;

    p_char = find_char_in_table(p_read_req->handle);

    // return error if encryption is required but session is not pair-verified
    if (!wiced_btle_homekit_is_accessory_pair_verified()
        && (p_char == NULL || (p_char->encr_required & 0x0f) == BTLE_HK_ENC_ALWAYS))
    {
        WICED_BT_TRACE("reject read: encryption is required but session is not pair-verified\n");
        status = WICED_BT_GATT_READ_NOT_PERMIT;
    }
    else
    {
        switch (p_read_req->handle)
        {
        case HDLC_GAP_DEVICE_NAME_VALUE:
            *p_read_req->p_val_len = strlen(wiced_bt_cfg_settings.device_name);
            memcpy(p_read_req->p_val, wiced_bt_cfg_settings.device_name, strlen(wiced_bt_cfg_settings.device_name));
            status = WICED_BT_GATT_SUCCESS;
            break;

        default:
            if (p_char)
            {
                *p_read_req->p_val_len = p_char->cur_len;
                memcpy(p_read_req->p_val, p_char->p_data, p_char->cur_len);
                status = WICED_BT_GATT_SUCCESS;
            }
            break;
        }
    }
#ifdef WICED_BT_TRACE_ENABLE
    if (status == WICED_BT_GATT_SUCCESS)
    {
        printKnownHandleData(0, p_read_req->handle, p_read_req->p_val, *p_read_req->p_val_len);
    }
    else
    {
        wiced_printf(NULL, 0, "***** read failed err:0x%x:\n", status);
    }
#endif
    return status;
}

/* GATT connection status callback */
wiced_bt_gatt_status_t btle_homekit_lightbulb_gatt_connect_callback( uint8_t* bda, uint16_t conn_id, wiced_bool_t connected, wiced_bt_gatt_disconn_reason_t reason )
{
    WICED_BT_TRACE( "apple_btle_home_kit connected:%d conn_id:%d\n", connected, conn_id);

    // for now do only 1 connection
    if ( connected )
    {
        btle_homekit_conn_id = conn_id;

        btle_homekit_refresh_conn_idle_counter();
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
    }
    else
    {
        btle_homekit_conn_id = 0;
        connection_idle_counter = 0;

        // advertisement data could have changed due to new pairings, or change in the characteristic values
        wiced_btle_homekit_set_advertisement_data();

        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    }

    return (WICED_BT_GATT_SUCCESS);
}


wiced_bt_gatt_status_t btle_homekit_lightbulb_gatt_server_callback ( uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    btle_homekit_refresh_conn_idle_counter();

    switch ( type )
    {
        case GATTS_REQ_TYPE_READ:
        	WICED_BT_TRACE( "app read_max value:%d offset:%d\n", *p_data->read_req.p_val_len, p_data->read_req.offset );
            status = btle_homekit_lightbulb_read_handler( &p_data->read_req, conn_id );
            break;

        case GATTS_REQ_TYPE_WRITE:
        	WICED_BT_TRACE( "app write length:%d offset:%d\n", p_data->write_req.val_len, p_data->write_req.offset );
            status = btle_homekit_lightbulb_write_handler( &p_data->write_req, conn_id );
            break;

        case GATTS_REQ_TYPE_WRITE_EXEC:
        case GATTS_REQ_TYPE_MTU:
        case GATTS_REQ_TYPE_CONF:
        	WICED_BT_TRACE( "???btle_homekit_lightbulb_gatt_server_callback handle:%x type:%x\n", p_data->write_req.handle, type );
            break;

        default:
        	WICED_BT_TRACE( "bad : recv type (0x%02x) conn_id:%d\n", type, conn_id );
            break;
    }
    return status;
}

wiced_bt_gatt_status_t btle_homekit_lightbulb_gatt_event_handler ( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
	switch (event)
	{
	case GATT_CONNECTION_STATUS_EVT:
		return btle_homekit_lightbulb_gatt_connect_callback( p_event_data->connection_status.bd_addr, p_event_data->connection_status.conn_id, p_event_data->connection_status.connected, p_event_data->connection_status.reason );

	case GATT_ATTRIBUTE_REQUEST_EVT:
		return btle_homekit_lightbulb_gatt_server_callback( p_event_data->attribute_request.conn_id, p_event_data->attribute_request.request_type, &p_event_data->attribute_request.data );

	}

	return (WICED_BT_GATT_SUCCESS);
}

void btle_homekit_hci_send_characteristic_value(uint16_t event, uint16_t handle, uint8_t* p_data, uint16_t data_len)
{
    uint8_t *buffer;
    uint16_t buf_len;

    buf_len = sizeof(handle) + data_len;
    buffer = (uint8_t *)malloc(buf_len);
    if (buffer)
    {
        memcpy(buffer, &handle, sizeof(handle));
        memcpy(&buffer[sizeof(handle)], p_data, data_len);
        wiced_transport_send_data(event, buffer, buf_len);
        free(buffer);
    }
}

uint8_t btle_homekit_lightbulb_hap_write_handler(uint16_t handle, uint8_t* p_data, uint16_t len)
{
    hap_pdu_tlv8_t*         p_tlv;
    uint8_t                 status = HAP_STATUS_INVALID_REQUEST;

    // Unpaired identify is only allowed when the accessory is unpaired
    if (handle == HDLC_ACCESSORY_INFO_IDENTIFY_VALUE &&
        !wiced_btle_homekit_is_accessory_pair_verified() &&
        wiced_homekit_is_accessory_paired())
    {
        return HAP_STATUS_INSUFFICIENT_AUTHENTICATION;
    }

    p_tlv = wiced_btle_homekit_find_hap_param(HAP_PARAM_VALUE, p_data, len);
    if (p_tlv)
    {
        if (btle_homekit_set_value(handle, p_tlv->value, p_tlv->length) == WICED_BT_GATT_SUCCESS)
        {
            btle_homekit_hci_send_characteristic_value(HCI_CONTROL_HK_EVENT_UPDATE, handle, p_tlv->value, p_tlv->length);
            status = HAP_STATUS_SUCCESS;
        }
    }

    return status;
}

uint8_t btle_homekit_lightbulb_hap_read_handler(uint16_t handle, uint8_t* p_data, uint16_t len)
{
    gatt_db_lookup_table* p_char;
    uint8_t status = HAP_STATUS_INVALID_REQUEST;

    if ((p_char = find_char_in_table(handle)) != NULL)
    {
        btle_homekit_lightbulb_cb.char_value = p_char->p_data;
        btle_homekit_lightbulb_cb.char_value_len = p_char->cur_len;
        status = HAP_STATUS_SUCCESS;
    }

    return status;
}

uint8_t btle_homekit_lightbulb_hap_timed_write_handler(uint16_t handle, uint8_t* p_data, uint16_t len)
{
    hap_pdu_tlv8_t*         p_tlv;

    p_tlv = wiced_btle_homekit_find_hap_param(HAP_PARAM_TTL, p_data, len);
    if (!p_tlv)
    {
        WICED_BT_TRACE("error: timed write request does not contain TTL\n");
        return HAP_STATUS_INVALID_REQUEST;
    }

    if (btle_homekit_lightbulb_cb.p_timed_write)
    {
        free(btle_homekit_lightbulb_cb.p_timed_write);
        WICED_BT_TRACE("previous timed write request discarded\n");
    }

    btle_homekit_lightbulb_cb.p_timed_write = (uint8_t*)malloc(len);
    if (!btle_homekit_lightbulb_cb.p_timed_write)
    {
        WICED_BT_TRACE("error: failed to allocate memory to save timed write request\n");
        return HAP_STATUS_MAX_PROCEDURES;
    }

    memcpy(btle_homekit_lightbulb_cb.p_timed_write, p_data, len);
    btle_homekit_lightbulb_cb.timed_write_len = len;
    btle_homekit_lightbulb_cb.timed_write_handle = handle;

    timed_write_counter = p_tlv->value[0];

    return HAP_STATUS_SUCCESS;
}

uint8_t btle_homekit_lightbulb_hap_execute_write_handler(uint16_t handle, uint8_t* p_data, uint16_t len)
{
    hap_pdu_tlv8_t*         p_tlv;
    uint8_t                 status = HAP_STATUS_INVALID_REQUEST;

    if (btle_homekit_lightbulb_cb.p_timed_write && btle_homekit_lightbulb_cb.timed_write_handle == handle)
    {
        status = btle_homekit_lightbulb_hap_write_handler(btle_homekit_lightbulb_cb.timed_write_handle,
                        btle_homekit_lightbulb_cb.p_timed_write, btle_homekit_lightbulb_cb.timed_write_len);
        free(btle_homekit_lightbulb_cb.p_timed_write);
        btle_homekit_lightbulb_cb.p_timed_write = NULL;
    }

    return status;
}

void btle_homekit_lightbulb_hap_request_callback(uint16_t conn_id, uint16_t handle, uint8_t* p_data, uint16_t len)
{
    wiced_btle_hap_request_t*   p_request;
    wiced_btle_hap_response_t*  p_response;
    uint8_t                     hap_response[BTLE_HOMEKIT_MAX_CHAR_VALUE_LEN + HAP_PDU_RESPONSE_BODY_VALUE_OFFSET + 2];
    uint16_t                    hap_response_len;
    uint8_t                     status;

    p_request = (wiced_btle_hap_request_t*)p_data;
    btle_homekit_lightbulb_cb.char_value_len = 0;

    WICED_BT_TRACE("btle_homekit_lightbulb HAP request %d, handle:0x%04x\n", p_request->opcode, handle);

    btle_homekit_refresh_conn_idle_counter();

    switch(p_request->opcode)
    {
    case HAP_CHARACTERISTIC_WRITE:
        status = btle_homekit_lightbulb_hap_write_handler(handle, p_data, len);
        break;
    case HAP_CHARACTERISTIC_READ:
        status = btle_homekit_lightbulb_hap_read_handler(handle, p_data, len);
        break;
    case HAP_CHARACTERISTIC_TIMED_WRITE:
        status = btle_homekit_lightbulb_hap_timed_write_handler(handle, p_data, len);
        break;
    case HAP_CHARACTERISTIC_EXECUTE_WRITE:
        status = btle_homekit_lightbulb_hap_execute_write_handler(handle, p_data, len);
        break;
    default:
        status = HAP_STATUS_UNSUPPORTED_PDU;
        WICED_BT_TRACE("btle_homekit_lightbulb opcode not supported:0x%02x\n", p_request->opcode);
        return;
    }

    p_response = (wiced_btle_hap_response_t*)hap_response;
    p_response->control = HAP_CONTROL_FIELD_RESPONSE;
    p_response->tid = p_request->tid;
    p_response->status = status;
    if (status == HAP_STATUS_SUCCESS && btle_homekit_lightbulb_cb.char_value_len > 0)
    {
        uint16_t body_len = btle_homekit_lightbulb_cb.char_value_len + 2;
        hap_pdu_tlv8_t* p_tlv = (hap_pdu_tlv8_t*)p_response->body_value;

        p_response->body_len[0] = (uint8_t)(body_len & 0xFF);
        p_response->body_len[1] = (uint8_t)((body_len >> 8) & 0xFF);
        p_tlv->type = HAP_PARAM_VALUE;
        p_tlv->length = btle_homekit_lightbulb_cb.char_value_len;
        memcpy(p_tlv->value, btle_homekit_lightbulb_cb.char_value, btle_homekit_lightbulb_cb.char_value_len);
        hap_response_len = HAP_PDU_RESPONSE_BODY_VALUE_OFFSET + body_len;
    }
    else if (p_request->opcode == HAP_CHARACTERISTIC_READ &&
             (handle == HDLC_LIGHTBULB_SERVICE_SIGNATURE_VALUE
#ifdef OTA_FIRMWARE_UPGRADE
              || handle == HDLC_OTA_FW_UPGRADE_SERVICE_SIGNATURE_VALUE
#endif
             ))
    {
        // return <null> value if controller tries to read service signature characteristic
        p_response->body_len[0] = 0x02;
        p_response->body_len[1] = 0x00;
        p_response->body_value[0] = HAP_PARAM_VALUE;
        p_response->body_value[1] = 0;
        p_response->status = HAP_STATUS_SUCCESS;
        hap_response_len = HAP_PDU_RESPONSE_BODY_VALUE_OFFSET + 2;
    }
    else
    {
        hap_response_len = HAP_PDU_RESPONSE_HEADER_SIZE;
    }
    wiced_btle_homekit_send_hap_response(conn_id, handle, hap_response, hap_response_len);
}

#ifdef WICED_BT_TRACE_ENABLE

/*
 *  Pass protocol traces up through the UART
 */
void hci_control_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}

/*
 * Prepare HCI UART to communicate to the MCU
 */
void hci_control_enable_uart( )
{
    wiced_bt_trace_enable();

    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace( hci_control_hci_trace_cback );
}

#endif


/*
 * Handle received command over UART.
 *
*/
uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t status = 0;
    uint8_t cmd_status;

    WICED_BT_TRACE("hci_control_proc_rx_cmd:%d\n", length);

    //Expected minimum 4 byte as the wiced header
    if((length < 4) || (!p_data))
    {
        WICED_BT_TRACE("invalid params\n");
        return 1;
    }

    READ_LITTLE_ENDIAN_TO_UINT16(opcode, p_data, length);     // Get opcode
    READ_LITTLE_ENDIAN_TO_UINT16(payload_len, p_data, length); // Get pay load length

    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_HK:
        status = hci_control_proc_hk_cmd(opcode, p_data, payload_len);
        break;
    case HCI_CONTROL_GROUP_MISC:
        hci_control_misc_handle_command(opcode, p_data, payload_len);
        break;
    default:
        cmd_status = HCI_CONTROL_STATUS_UNKNOWN_GROUP;
        wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &cmd_status, 1);
        break;
    }

    return status;
}

/*
 * Handle received HCI Control API HomeKit command over UART.
 */
uint8_t hci_control_proc_hk_cmd(uint16_t opcode, uint8_t *p_data, uint16_t data_len)
{
    uint8_t status = 0;
    uint8_t cmd_status = HCI_CONTROL_STATUS_SUCCESS;
    uint16_t handle;
    uint8_t value;
    int i;

    WICED_BT_TRACE("hci_control_proc_hk_cmd opcode:0x%04x, data_len:%d\n", opcode, data_len);

    switch (opcode)
    {
    case HCI_CONTROL_HK_COMMAND_READ:
        handle = (uint16_t)(p_data[0] + (p_data[1] << 8));
        status = btle_homekit_lightbulb_hap_read_handler(handle, NULL, 0);
        if (status == HAP_STATUS_SUCCESS)
        {
            btle_homekit_hci_send_characteristic_value(HCI_CONTROL_HK_EVENT_READ_RESPONSE, handle,
                    btle_homekit_lightbulb_cb.char_value, btle_homekit_lightbulb_cb.char_value_len);
        }
        else
        {
            cmd_status = HCI_CONTROL_STATUS_BAD_HANDLE;
            wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &cmd_status, 1);
        }
        break;

    case HCI_CONTROL_HK_COMMAND_WRITE:
        handle = (uint16_t)(p_data[0] + (p_data[1] << 8));
        status = btle_homekit_set_value(handle, &p_data[2], data_len - 2);
        if (status == HAP_STATUS_SUCCESS)
        {
            btle_homekit_check_and_send_notification(handle);
        }
        else
        {
            cmd_status = HCI_CONTROL_STATUS_FAILED;
        }
        wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &cmd_status, 1);
        break;

    case HCI_CONTROL_HK_COMMAND_LIST:
        for (i = 0; i < HAP_CHAR_TBL_SIZE; i++)
        {
            uint8_t buffer[40];

            if (hap_char_tbl[i].handle == HDLC_PAIRING_PAIR_SETUP_VALUE)
                break;

            memset(buffer, 0, 40);
            buffer[0] = hap_char_tbl[i].handle & 0xff;
            buffer[1] = (hap_char_tbl[i].handle >> 8) & 0xff;
            buffer[2] = hap_char_tbl[i].format;
            strncpy(&buffer[3], hap_char_names[i].name, 36);
            wiced_transport_send_data(HCI_CONTROL_HK_EVENT_LIST_ITEM, buffer, strlen(&buffer[3]) + 4);
        }
        break;

    case HCI_CONTROL_HK_COMMAND_FACTORY_RESET:
        btle_homekit_lightbulb_factory_reset();
        break;
    }

    return status;
}

static void btle_homekit_lightbulb_factory_reset()
{
    wiced_result_t status;

    memset(&nvram_char_values, 0, sizeof(nvram_char_values));
    wiced_hal_write_nvram(BTLE_HOMEKIT_NVRAM_VSID_CHAR_VALUES, sizeof(nvram_char_values), (uint8_t *)&nvram_char_values, &status);

    btle_homekit_lightbulb_brightness[0] = nvram_char_values.brightness;
    btle_homekit_lightbulb_brightness_client_cfg[0] = 0;
    btle_homekit_lightbulb_on[0] = nvram_char_values.on;
    btle_homekit_lightbulb_on_client_cfg[0] = 0;
    memcpy(btle_homekit_lightbulb_hue, &nvram_char_values.hue, 4);
    btle_homekit_lightbulb_hue_client_cfg[0] = 0;
    memcpy(btle_homekit_lightbulb_saturation, &nvram_char_values.saturation, 4);
    btle_homekit_lightbulb_saturation_client_cfg[0] = 0;

    wiced_homekit_reset();
}

/* Handle misc command group */
void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    switch( cmd_opcode )
    {
    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        hci_control_misc_handle_get_version();
        break;
    }
}

/* Handle get version command */
void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;

    uint32_t  chip = (PLATFORM) ? BCM920707 : BCM920706;

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = POWER_CLASS;

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_HK;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
}

