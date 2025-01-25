#ifndef Name_h
#define Name_h

// Manufacturer name definition
#undef MANUFACTURER_NAME
#define MANUFACTURER_NAME                                               \
    {                                                                   \
        'V', 'I', 'R', 'T', 'U', 'A', 'L', 'M', 'A', 'T', 'T', 'E', 'R' \
    }
#undef MANUFACTURER_NAME_LEN
#define MANUFACTURER_NAME_LEN 13

// Product name definition
#undef PRODUCT_NAME
#define PRODUCT_NAME \
    { 'V', 'M', 'S', 'T', 'E', 'P' }
#undef PRODUCT_NAME_LEN
#define PRODUCT_NAME_LEN 6

// Serial number definition  
#define SERIAL_NUMBER \
    { '2', '0', '2', '4', '_', '0', '1', '.', '0', '2' }
#define SERIAL_NUMBER_LEN 10

#define FIRMWARE_VERSION \
    { '0', '0', '.', '0', '2' }
#define FIRMWARE_VERSION_LEN 5

#endif