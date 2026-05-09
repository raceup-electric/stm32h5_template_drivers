set(_drivers_instance_dir "${CMAKE_CURRENT_LIST_DIR}")
include("${CMAKE_CURRENT_LIST_DIR}/../instance_layout.cmake")

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(STM32CubeH5 REQUIRED MODULE)

file(GLOB USBX_CORE_SOURCES
  "${STM32_USBX_ROOT}/common/core/src/*.c"
)
file(GLOB USBX_CDC_SOURCES
  "${STM32_USBX_ROOT}/common/usbx_device_classes/src/ux_device_class_cdc_acm*.c"
)
file(GLOB USBX_DCD_SOURCES
  "${STM32_USBX_ROOT}/common/usbx_stm32_device_controllers/*.c"
)

list(REMOVE_ITEM DRIVERS_INSTANCE_SOURCES
  "${STM32H5_DRIVERS_ROOT}/m_can.cpp"
  "${STM32H5_DRIVERS_ROOT}/flex_can.cpp"
  "${STM32H5_DRIVERS_ROOT}/multi_can.cpp"
)

list(APPEND DRIVERS_INSTANCE_SOURCES
  "${STM32H5_DRIVERS_ROOT}/eeprom_itf_crc_stm32h5.cpp"
  "${STM32H5_DRIVERS_ROOT}/eeprom_itf_flash_stm32h5.cpp"
  "${STM32_EEPROM_EMULATION_ROOT}/core/eeprom_emul_core.c"
  "${STM32_EEPROM_EMULATION_ROOT}/core/algo/eeprom_algo_flitf.c"
  "${STM32H5_DRIVERS_ROOT}/usb/usb_cdc_standalone.c"
  "${STM32H5_DRIVERS_ROOT}/usb/ux_standalone_support.c"
  ${USBX_CORE_SOURCES}
  ${USBX_CDC_SOURCES}
  ${USBX_DCD_SOURCES}
)

list(APPEND DRIVERS_INSTANCE_PUBLIC_INCLUDE_DIRS
  "${STM32H5_DRIVERS_ROOT}/config"
)

list(APPEND DRIVERS_INSTANCE_PRIVATE_INCLUDE_DIRS
  "${STM32H5_DRIVERS_ROOT}/config"
  "${STM32H5_DRIVERS_ROOT}/usb"
)

list(APPEND DRIVERS_INSTANCE_SYSTEM_PUBLIC_INCLUDE_DIRS
  "${STM32CubeH5_HAL_INCLUDE_DIR}"
  "${STM32CubeH5_CMSIS_INCLUDE_DIR}"
  "${STM32CubeH5_CMSIS_DEVICE_INCLUDE_DIR}"
)

list(APPEND DRIVERS_INSTANCE_SYSTEM_PRIVATE_INCLUDE_DIRS
  "${STM32_EEPROM_EMULATION_ROOT}/core"
  "${STM32_EEPROM_EMULATION_ROOT}/core/algo"
  "${STM32_EEPROM_EMULATION_ROOT}/interface_flash"
  "${STM32_EEPROM_EMULATION_ROOT}/interface_crc"
  "${STM32_USBX_ROOT}/common/core/inc"
  "${STM32_USBX_ROOT}/common/usbx_device_classes/inc"
  "${STM32_USBX_ROOT}/common/usbx_stm32_device_controllers"
  "${STM32_USBX_ROOT}/ports/cortex_m4/gnu/inc"
)

list(APPEND DRIVERS_INSTANCE_COMPILE_DEFINITIONS
  USE_HAL_DRIVER
  ${STM32_DEVICE}
  UX_INCLUDE_USER_DEFINE_FILE
)

list(APPEND DRIVERS_INSTANCE_PRIVATE_LINK_LIBRARIES
  device_hal
)
