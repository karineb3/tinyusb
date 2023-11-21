# Only include this library when the TinyUSB config path is defined.
set(TINY_USB_CONFIG_PATH examples/device/cdc_uac2/src CACHE INTERNAL "Path to tinyusb cfg files used by CMakeLists.")
    add_library(tinyusb "")

    target_sources(tinyusb
        PRIVATE
            src/tusb.c
            src/class/cdc/cdc_device.c
            src/common/tusb_fifo.c
            src/device/usbd_control.c
            src/device/usbd.c
            src/portable/synopsys/dwc2/dcd_dwc2.c
            hw/bsp/board.c
            hw/bsp/stm32u5
            ${TINY_USB_CONFIG_PATH}/usb_descriptors.c
        PUBLIC
            src/portable/synopsys/dwc2/dwc2_stm32.h
            hw/bsp/board_api.h
            hw/bsp/board_mcu.h
    )

    target_include_directories(tinyusb
        PUBLIC
            src
            hw/bsp
            src/class/cdc
            src/common
            src/device
            src/portable/synopsys/dwc2
            ${TINY_USB_CONFIG_PATH}
    )

    target_link_libraries(tinyusb PUBLIC hardware)

