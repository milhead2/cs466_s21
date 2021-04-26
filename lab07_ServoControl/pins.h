#pragma once
#ifndef _PINS_H_INCLUDED_
#define _PINS_H_INCLUDED_

/*
** Normal Tiva Switch and LED assignemnts
*/
#define LED_R (1<<1)  /* F1 */
#define LED_G (1<<3)  /* F3 */
#define LED_B (1<<2)  /* F2 */
#define SW1   (1<<4)  /* F4 */ 
#define SW2   (1<<0)  /* F0 */

/* 
** 
*/
#define OUT1  (1<<6)  /* D6 */
#define OUT2  (1<<1)  /* D1 */

/* 
** Phase signals from quadrature encoder.
*/
#define PHA   (1<<4)  /* C4 */
#define PHB   (1<<5)  /* C5 */

/*
** Directional output signals send to h-bridge
*/
#define IN1   (1<<2)  /* E2 */
#define IN2   (1<<3)  /* E3 */

/*
** PWM Out to drive Enable
*/
#define PO    (1<<6)  /* B6 */

#endif // _PINS_H_INCLUDED_ 