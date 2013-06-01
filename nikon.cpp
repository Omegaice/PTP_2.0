/* Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

Circuits At Home, LTD
Web      :  http://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
*/
#include "nikon.h"

NikonDSLR::NikonDSLR( USB *pusb, PTPStateHandlers *s )
	: PTP( pusb, s ) {
}

uint8_t NikonDSLR::Init( uint8_t parent, uint8_t port, bool lowspeed ) {
	uint8_t		buf[10];
	uint8_t		rcode;
	UsbDevice	*p = NULL;
	EpInfo		*oldep_ptr = NULL;

	PTPTRACE( "NK Init\r\n" );

	AddressPool	&addrPool = pUsb->GetAddressPool();

	if( devAddress ) {
		return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;
	}

	// Get pointer to pseudo device with address 0 assigned
	p = addrPool.GetUsbDevicePtr( 0 );

	if( !p ) {
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
	}

	if( !p->epinfo ) {
		PTPTRACE( "epinfo\r\n" );
		return USB_ERROR_EPINFO_IS_NULL;
	}

	// Save old pointer to EP_RECORD of address 0
	oldep_ptr = p->epinfo;

	// Temporary assign new pointer to epInfo to p->epinfo in order to avoid toggle inconsistence
	p->epinfo = epInfo;

	// Get device descriptor
	rcode = pUsb->getDevDescr( 0, 0, 10, ( uint8_t * )buf );

	// Restore p->epinfo
	p->epinfo = oldep_ptr;

	if( rcode ) {
		PTPTRACE2( "getDevDesc:", rcode );
		return rcode;
	}

	if( ( ( USB_DEVICE_DESCRIPTOR * )buf )->idVendor == 0x04B0 ) {
		return PTP::Init( parent, port, lowspeed );
	} else {
		PTPTRACE( "Camera isn't Nikon\r\n" );
		return USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED;
	}
}
