// This header is meant to undo the effect of having included
// usb_desc.h.  Everything usb_desc.h, usb_undef.h undefines.
// This stuff is not supposed to be made visible to user level
// programs, but it's nice if we can use it within the headers
// included by user programs, to automatically define only the
// appropriate APIs for the types of USB interfaces used.

#ifdef _usb_desc_h_
  #undef _usb_desc_h_
#endif
#ifdef ENDPOINT_UNUSED
  #undef ENDPOINT_UNUSED
#endif
#ifdef ENDPOINT_TRANSIMIT_ONLY
  #undef ENDPOINT_TRANSIMIT_ONLY
#endif
#ifdef ENDPOINT_RECEIVE_ONLY
  #undef ENDPOINT_RECEIVE_ONLY
#endif
#ifdef ENDPOINT_TRANSMIT_AND_RECEIVE
  #undef ENDPOINT_TRANSMIT_AND_RECEIVE
#endif
#ifdef VENDOR_ID
  #undef VENDOR_ID
#endif
#ifdef PRODUCT_ID
  #undef PRODUCT_ID
#endif
#ifdef DEVICE_CLASS
  #undef DEVICE_CLASS
#endif
#ifdef MANUFACTURER_NAME
  #undef MANUFACTURER_NAME
#endif
#ifdef MANUFACTURER_NAME_LEN
  #undef MANUFACTURER_NAME_LEN
#endif
#ifdef PRODUCT_NAME
  #undef PRODUCT_NAME
#endif
#ifdef PRODUCT_NAME_LEN
  #undef PRODUCT_NAME_LEN
#endif
#ifdef EP0_SIZE
  #undef EP0_SIZE
#endif
#ifdef NUM_ENDPOINTS
  #undef NUM_ENDPOINTS
#endif
#ifdef NUM_USB_BUFFERS
  #undef NUM_USB_BUFFERS
#endif
#ifdef NUM_INTERFACE
  #undef NUM_INTERFACE
#endif
#ifdef CDC_STATUS_INTERFACE
  #undef CDC_STATUS_INTERFACE
#endif
#ifdef CDC_DATA_INTERFACE
  #undef CDC_DATA_INTERFACE
#endif
#ifdef CDC_ACM_ENDPOINT
  #undef CDC_ACM_ENDPOINT
#endif
#ifdef CDC_RX_ENDPOINT
  #undef CDC_RX_ENDPOINT
#endif
#ifdef CDC_TX_ENDPOINT
  #undef CDC_TX_ENDPOINT
#endif
#ifdef CDC_ACM_SIZE
  #undef CDC_ACM_SIZE
#endif
#ifdef CDC_RX_SIZE
  #undef CDC_RX_SIZE
#endif
#ifdef CDC_TX_SIZE
  #undef CDC_TX_SIZE
#endif
#ifdef CDC_IAD_DESCRIPTOR
  #undef CDC_IAD_DESCRIPTOR
#endif
#ifdef ENDPOINT2_CONFIG
  #undef ENDPOINT2_CONFIG
#endif
#ifdef ENDPOINT3_CONFIG
  #undef ENDPOINT3_CONFIG
#endif
#ifdef ENDPOINT4_CONFIG
  #undef ENDPOINT4_CONFIG
#endif
