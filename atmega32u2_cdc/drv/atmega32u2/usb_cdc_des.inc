/*
 * usb_cdc_des.inc
 *
 *  Created: 22.02.2016 20:04:21
 *   Author: Dominik Koch
 */\
 .ifndef usb_des_include
.set usb_des_include = 1


.CSEG
;##################################################################################
DEV_DES:
.DB	0x12,						/* bLength */\
    0x01,						/* bDescriptorType */\
    0x00, 0x02,					/* bcdUSB */\
    0x02,						/* bDeviceClass */\
    0x00,						/* bDeviceSubclass */\
    0x00, 						/* bDeviceProtocol */\
    EP0_SIZE,					/* bMaxPacketSize0 */\
    0xEB, 0x03,					/* Vendor-ID. */\
    0xAB, 0xEB,					/* Geräte-ID. */\
    0x01, 0x01,					/* Releasenummer des Geräts in BCD-Form. */\
    0x01,						/* ID des Stringdescriptors, der den Herstellernamen angibt. */\
    0x02,						/* ID des Stringdescriptors, der den Gerätenamen angibt. */\
    0x03,						/* ID des Stringdescriptors, der die Seriennummer angibt. */\
    0x01						/* Anzahl der möglichen Konfigurationen. */
DEV_DES_END:		
;##################################################################################
.SET CONF_DES_LEN = 62
CONF_DES:
.DB 9,							/* bLength */\
    CONFIG_DESCRIPTOR,						/* bDescriptorType */\
    LOW(CONF_DES_LEN), HIGH(CONF_DES_LEN),	/* wTotalLength */\
    0x02,									/* bNumInterfaces */\
    USB_CDC_CONFIGURATION_NUM,				/* bConfigurationValue */\
    0x00,									/* iConfiguration */\
    0x80,									/* bmAttributes */\
    0x32,									/* bMaxPower */\
	\
    /** CDC Class-Specific descriptor **/\
    5,							/* sizeof(usbDescrCDC_HeaderFn): length of descriptor in bytes */\
    0x24,						/* descriptor type */\
    0x00,						/* header functional descriptor */\
    0x10,0x01,					\
	\
    4,							/* sizeof(usbDescrCDC_AcmFn): length of descriptor in bytes    */\
    0x24,						/* descriptor type */\
    0x02,						/* abstract control management functional descriptor */\
    0x06,						/* SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE    */\
	\
    5,							/* sizeof(usbDescrCDC_UnionFn): length of descriptor in bytes  */\
    0x24,						/* descriptor type */\
    0x06,						/* union functional descriptor */\
    0,							/* CDC_COMM_INTF_ID */\
    1,							/* CDC_DATA_INTF_ID */\
	\
    /** Interfacedescriptor Communication  **/\
    9,							/* bLength */\
    4,							/* bDescriptorType */\
    0x00,						/* bInterfaceNumber */\
    0x00,						/* bAlternateSetting */\
    0x01,						/* bNumEndpoints */\
    0x02,						/* bInterfaceClass */\
    0x02,						/* bInterfaceSubClass */\
    0x01,						/* bInterfaceProtocol */\
    0x00,						/* iInterface */\
    /** EP_CDC_C_IN **/\
    7,							/* bLength */\
    ENDPOINT_DESCRIPTOR,		/* bDescriptorType */\
    EP_CDC_C_IN,				/* bEndpointAddress */\
    bmAttr_Interrupt,			/* bmAttributes */\
    EP_CDC_C_SIZE, 0x00,		/* wMaxPacketSize */\
    0x0A,						/* bInterval 0x0A */\
	/***********************************/\
    /** Interfacedescriptor Data **/\
    9,							/* bLength */\
    INTERFACE_DESCRIPTOR,		/* bDescriptorType */\
    0x01,						/* bInterfaceNumber */\
    0x00,						/* bAlternateSetting */\
    0x02,						/* bNumEndpoints */\
    0x0A,						/* bInterfaceClass */\
    0x00,						/* bInterfaceSubClass */\
    0x00,						/* bInterfaceProtocol */\
    0x00,						/* iInterface */\
	/** EP_CDC_D_IN **/\
    7,							/* bLength */\
    ENDPOINT_DESCRIPTOR,		/* bDescriptorType */\
    EP_CDC_D_IN,				/* bEndpointAddress */\
    bmAttr_Bulk,				/* bmAttributes */\
    EP_CDC_D_SIZE, 0x00,		/* wMaxPacketSize */\
    0x0A,						/* bInterval 0x0A */\
    /** EP_CDC_D_OUT **/\
    7,							/* bLength */\
    ENDPOINT_DESCRIPTOR,		/* bDescriptorType */\
    EP_CDC_D_OUT,				/* bEndpointAddress */\
    bmAttr_Bulk,				/* bmAttributes */\
    EP_CDC_D_SIZE, 0x00,		/* wMaxPacketSize */\
    0x0A						/* bInterval */
CONF_DES_END:


;##################################################################################
.SET LANG_DES_LEN = ((1*2)+2)
LANG_DES:
.DB	LANG_DES_LEN, 0x03
.DB	0x07,0x04 ;LANGID = 0x0407 = German
LANG_DES_END:
;##################################################################################
.SET STRING_DES_1_LEN = (7*2)+2
STRING_DES_1:
.DB	STRING_DES_1_LEN, 0x03
.DB	'U',0,'S',0,'B',0,' ',0,'C',0,'D',0,'C',0
STRING_DES_1_END:
;##################################################################################
.SET STRING_DES_2_LEN = (10*2)+2
STRING_DES_2:
.DB STRING_DES_2_LEN, 0x03
.DB 'C',0,'D',0,'C',0,' ',0,'D',0,'E',0,'V',0,'I',0,'C',0,'E',0
STRING_DES_2_END:
;##################################################################################
.SET STRING_DES_3_LEN = (12*2)+2
STRING_DES_3:
.DB STRING_DES_3_LEN, 0x03
.DB '0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0
STRING_DES_3_END:

.endif