/*
 *  Lib(X)SVF  -  A library for implementing SVF and XSVF JTAG players
 *
 *  Copyright (C) 2009  RIEGL Research ForschungsGmbH
 *  Copyright (C) 2009  Clifford Wolf <clifford@clifford.at>
 *  
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "libxsvf.h"

int libxsvf_scan(struct libxsvf_host *h)
{
	int i, j;

	if (libxsvf_tap_walk(h, LIBXSVF_TAP_RESET) < 0)
		return -1;

	if (libxsvf_tap_walk(h, LIBXSVF_TAP_DRSHIFT) < 0)
		return -1;

	for (i=0; i<256; i++)
	{
		int bit = LIBXSVF_HOST_PULSE_TCK(0, 1, -1, 0, 1);

		if (bit < 0)
			return -1;

		if (bit == 0) {
			LIBXSVF_HOST_REPORT_DEVICE(0);
		} else {
			unsigned long idcode = 1;
			for (j=1; j<32; j++) {
				int bit = LIBXSVF_HOST_PULSE_TCK(0, 1, -1, 0, 1);
				if (bit < 0)
					return -1;
				idcode |= ((unsigned long)bit) << j;
			}
			if (idcode == 0xffffffff)
				break;
			LIBXSVF_HOST_REPORT_DEVICE(idcode);
		}
	}

	return 0;
}

