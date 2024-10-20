#include <usb_names.h>

#define MANUFACTURER_NAME                                               \
    {                                                                   \
        'V', 'I', 'R', 'T', 'U', 'A', 'L', 'M', 'A', 'T', 'T', 'E', 'R' \
    }
#define MANUFACTURER_NAME_LEN 13

#define PRODUCT_NAME                                                    \
    {                                                                   \
        'T', 'E', 'E', 'N', 'S', 'Y', 'S', 'T', 'E', 'P' \
    }
#define PRODUCT_NAME_LEN 10

#define SERIAL_NUMBER                               \
    {                                               \
        'V', 'M', '_', 'T', 'S', 'T', 'E', 'P', '_', '0', '1' \
    }
#define SERIAL_NUMBER_LEN 11

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