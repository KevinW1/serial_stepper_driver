#include <usb_names.h>
#include "name.h"

// Use definitions from name.h
struct usb_string_descriptor_struct usb_string_manufacturer_name = {
    2 + MANUFACTURER_NAME_LEN * 2,
    3,
    MANUFACTURER_NAME};

struct usb_string_descriptor_struct usb_string_product_name = {
    2 + PRODUCT_NAME_LEN * 2,
    3,
    PRODUCT_NAME};

struct usb_string_descriptor_struct usb_string_serial_number = {
    2 + SERIAL_NUMBER_LEN * 2,
    3,
    SERIAL_NUMBER};