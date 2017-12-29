#
# Copyright 2017, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
#  Corporation. All rights reserved. This software, including source code, documentation and  related 
# materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
#  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
# (United States and foreign), United States copyright laws and international treaty provisions. 
# Therefore, you may use this Software only as provided in the license agreement accompanying the 
# software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
# hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
# compile the Software source code solely for use in connection with Cypress's  integrated circuit 
# products. Any reproduction, modification, translation, compilation,  or representation of this 
# Software except as specified above is prohibited without the express written permission of 
# Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
# OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
# AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
# the Software without notice. Cypress does not assume any liability arising out of the application 
# or use of the Software or any product or circuit  described in the Software. Cypress does 
# not authorize its products for use in any products where a malfunction or failure of the 
# Cypress product may reasonably be expected to result  in significant property damage, injury 
# or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
#  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
# to indemnify Cypress against all liability.
#

NAME := btle_homekit2_lightbulb

########################################################################
# Add Application sources here.
########################################################################
APP_SRC  = btle_homekit2_lightbulb.c
APP_SRC += wiced_bt_cfg.c

########################################################################
# Component(s) needed
########################################################################
$(NAME)_COMPONENTS := apple_btle_homekit2_debug.a
$(NAME)_COMPONENTS += fw_upgrade_lib.a

########################################################################
# C flags
########################################################################

C_FLAGS += -DWICED_SMART_READY=TRUE 
C_FLAGS += -DWICED_BT_TRACE_ENABLE
C_FLAGS += -Dmalloc=cfa_mm_Alloc_d
C_FLAGS += -Dfree=cfa_mm_Free_d
  		  
ifdef USE_MFI
C_FLAGS += -DWICED_USE_MFI=$(USE_MFI)
endif
ifdef OTA_FW_UPGRADE
C_FLAGS += -DOTA_FIRMWARE_UPGRADE=$(OTA_FW_UPGRADE)
#C_FLAGS += -DBTLE_HOMEKIT_OTA_UPGRADE_FROM_WINDOWS
C_FLAGS += -DBTLE_HOMEKIT_OTA_UPGRADE_FROM_HAP
endif
ifdef SETUP_CODE
C_FLAGS += -DWICED_HOMEKIT_SETUP_CODE=$(SETUP_CODE)
endif
C_FLAGS += -DUSE_STATIC_RANDOM_ADDRESS

########################################################################
################ DO NOT MODIFY FILE BELOW THIS LINE ####################
########################################################################