/*
 * bitoperations.h
 *
 * Created: 13.12.2015 12:00:00
 * Author : AKAEM
 */ 

#ifndef _BIT_OPERATIONS_H_
#define _BIT_OPERATIONS_H_

// declare some bit operations
#define bit_get(p,m)		((p) & (m))
#define bit_set(p,m)		((p) |= (m))
#define bit_clear(p,m)		((p) &= ~(m))
#define bit_flip(p,m)		((p) ^= (m))
#define bit_write(c,p,m)	(c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x)				(0x01 << (x))
#define LONGBIT(x)			((unsigned long)0x00000001 << (x))

#define ON					0x01
#define OFF					0x00

#endif