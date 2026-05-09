include(FindPackageHandleStandardArgs)

set(_stm32cubeh5_hints)

foreach(_stm32cubeh5_hint_var STM32CubeH5_ROOT STM32CubeH5_DIR)
  if(DEFINED ${_stm32cubeh5_hint_var})
    list(APPEND _stm32cubeh5_hints "${${_stm32cubeh5_hint_var}}")
  endif()
  if(DEFINED ENV{${_stm32cubeh5_hint_var}})
    list(APPEND _stm32cubeh5_hints "$ENV{${_stm32cubeh5_hint_var}}")
  endif()
endforeach()

get_filename_component(_stm32cubeh5_repo_root
  "${CMAKE_CURRENT_LIST_DIR}/../../../../../.."
  ABSOLUTE
)
list(APPEND _stm32cubeh5_hints
  "${_stm32cubeh5_repo_root}/third_party/STM32/STM32H5"
  "${_stm32cubeh5_repo_root}/third_party/STM32/STM32H5/STM32CubeH5"
)

find_path(STM32CubeH5_ROOT_DIR
  NAMES Drivers/STM32H5xx_HAL_Driver/Inc/stm32h5xx_hal.h
  HINTS ${_stm32cubeh5_hints}
  PATH_SUFFIXES
    ""
    STM32CubeH5
)

if(STM32CubeH5_ROOT_DIR)
  set(STM32CubeH5_HAL_INCLUDE_DIR
    "${STM32CubeH5_ROOT_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc"
  )
  set(STM32CubeH5_HAL_SOURCE_DIR
    "${STM32CubeH5_ROOT_DIR}/Drivers/STM32H5xx_HAL_Driver/Src"
  )
  set(STM32CubeH5_CMSIS_INCLUDE_DIR
    "${STM32CubeH5_ROOT_DIR}/Drivers/CMSIS/Include"
  )
  set(STM32CubeH5_CMSIS_DEVICE_INCLUDE_DIR
    "${STM32CubeH5_ROOT_DIR}/Drivers/CMSIS/Device/ST/STM32H5xx/Include"
  )
  set(STM32CubeH5_SYSTEM_SOURCE
    "${STM32CubeH5_ROOT_DIR}/Drivers/CMSIS/Device/ST/STM32H5xx/Source/Templates/system_stm32h5xx.c"
  )
  set(STM32CubeH5_STARTUP_DIR
    "${STM32CubeH5_ROOT_DIR}/Drivers/CMSIS/Device/ST/STM32H5xx/Source/Templates/gcc"
  )
endif()

find_package_handle_standard_args(STM32CubeH5
  REQUIRED_VARS
    STM32CubeH5_ROOT_DIR
    STM32CubeH5_HAL_INCLUDE_DIR
    STM32CubeH5_HAL_SOURCE_DIR
    STM32CubeH5_CMSIS_INCLUDE_DIR
    STM32CubeH5_CMSIS_DEVICE_INCLUDE_DIR
    STM32CubeH5_SYSTEM_SOURCE
    STM32CubeH5_STARTUP_DIR
)

mark_as_advanced(
  STM32CubeH5_ROOT_DIR
  STM32CubeH5_HAL_INCLUDE_DIR
  STM32CubeH5_HAL_SOURCE_DIR
  STM32CubeH5_CMSIS_INCLUDE_DIR
  STM32CubeH5_CMSIS_DEVICE_INCLUDE_DIR
  STM32CubeH5_SYSTEM_SOURCE
  STM32CubeH5_STARTUP_DIR
)
