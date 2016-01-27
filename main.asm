;-------------------------------------------------------------------------------------
; FILE: main.asm
; DESC: PIC motor controller
; DATE: 8-12-2015
; AUTH: Sam Pavlenko
; DEVICE: PICmicro (PIC18F1220)
;-------------------------------------------------------------------------------------

	list p=18F1220
	radix hex
	config WDT=OFF, LVP=OFF

; these are just used for some fake stack-ish space for functions.
#define ARG0	0x80
#define ARG1	0x81
#define ARG2	0x82
#define ARG3	0x83
#define ARG4	0x84
#define target	0x85


#include p18f1220.inc
	org	0x00
	GOTO	start

	org	0x08
	GOTO	PW_adjustment

	org 0x20
start:

	CLRF	PORTA
	CLRF	PORTB
	; INITIALIZATION
	; 1) Use default voltage, Select analog input to AN0 (Pin 1), Disable A/D initially
	MOVLW	0x00 ; ?00 0 000 0 0?
	MOVWF	ADCON0
	; 2) Configure AN0 as an analog input
	BCF	ADCON1, PCFG0 ; Analog
	BSF	TRISA,0 ; Input
	; 3) Set A/D conversion clock (Fosc/8), acquisition time (2TAD),
	; Digital value is left justified 8-bit results in register ADREFSH
	MOVLW	0x09 ;?0 0 001 001
	MOVWF	ADCON2
	BSF	ADCON0, ADON ;turn on converte; modifiying registers to enable interupts when A/D is complete
	BSF	INTCON, GIE
	BSF	INTCON, PEIE
	BSF	IPR1, ADIP
	BSF	PIE1, ADIE
	BCF	PIR1, ADIF
	; 4) enable A/D module
	MOVLW	0x00
	MOVWF	TRISB

	; PWM setup
	; Set CCPR1L:CCP1CON<5:4> to  0x000 for 0% power
	; Set CCPR1L:CCP1CON<5:4> to  0xFF3 for 100% power.
	; But you should use the function ( write_pwm ) that I wrote.

	; Setup the PWM frequency
	MOVLW	0xFF
	MOVWF	PR2 ; Use the full range of timer2.

	; Clear and Configure Timer 2 (PWM requires Timer 2)
	CLRF	TMR2 ; Timer 2 Register
	MOVLW	0x04 ; Enable timer and set prescale to 1.
	MOVWF	T2CON
	BCF	PIR1, TMR2IF ; Clear Timer 2 interrupt flag .

	MOVLW	0x00C
	MOVWF	CCP1CON ; PWM mode; P1A (Pin 18), P1C active-high; P1B, P1D active-high.

	; Set the duty cycle to 0x000 i.e 0%
	CLRF	ARG1
	CLRF	ARG0
	CALL	write_pwm

	BSF	ADCON0, GO ; Start the A/D conversion

	CLRF	target
main:
	BCF	PIE1,ADIE ; Disable A/D interrupt TODO should this disable all interrupt to be safe

	MOVFF	target, ARG0 ; Copy the target value to ARG0
	CALL	linear_interpolate,1

	BSF	PIE1,	ADIE ; Enable A/D interrupt

	MOVLW	0x00
	MOVWF	ARG0
	MOVLW	0x00
	MOVWF	ARG1
	CALL    delay,1 ; delay 66.56ms	

	;CLRF	ARG0
	;CLRF	ARG1
	;CALL    delay,1 ; delay 66.56ms

	;CLRF	ARG0
	;CLRF	ARG1
	;CALL    delay,1 ; delay 66.56ms

	;CLRF	ARG0
	;CLRF	ARG1
	;CALL    delay,1 ; delay 66.56ms

	; that is a delay of .26624s is that enough?
	BRA main



; Linear interpolates increasing CCPR1L by one
; until CCPR1L = ARG0.
; if ARG0 is less than CCPR1L then CCPR1L is set
; to ARG0
;
; This function will add one to CCPR1L with
; each invocation, therfore you must keep calling
; it until CCPR1L = ARG0 or if ARG0 is continuously
; changing just keep calling this function.
;
; TODO make this function interpolate the full 10-bits
; of the (CCPR1L:CCP1CON<5:4>)
linear_interpolate:
; target_less_equal_current:
	MOVF	ARG0, W
	SUBWF	CCPR1L, W
	BTFSS	STATUS, 0
	BRA	target_greater_current
	MOVFF	ARG0, CCPR1L
	NOP ; TODO is this nop needed?
	RETURN 1

target_greater_current:
	INCF    CCPR1L
	RETURN 1


PW_adjustment:
	MOVFF	ADRESH, target

	BCF	PIR1, ADIF ; clear flag
	BSF	ADCON0, GO   ; start next conversion

	RETFIE 1

; Sets the full 10-bit PWM duty cycle.
; The function take 2 arguments.
; ARG1	    the most significant byte for the duty cycle.
; ARG0	    the lest significant byte for the duty cycle.
;
;
; Set ARG1|ARG0 to  0x0000 for 0% power.
; Set ARG1|ARG0 to  0x03FF for 100% power.

; void(ARG1, ARG0)
write_pwm:
	; Transfer bits 0,1
	MOVLW	0xCF
	ANDWF	CCP1CON, F ; clear first

	MOVLW	0x03
	ANDWF	ARG0, W
	RRNCF	WREG, W
	RRNCF	WREG, W
	RRNCF	WREG, W
	RRNCF	WREG, W
	IORWF	CCP1CON, F

	; Transfer bits 2-8
	MOVLW	0xC0
	ANDWF	CCPR1L, F ; clear first

	MOVLW	0xFC
	ANDWF	ARG0, W
	RRNCF	WREG, W
	RRNCF	WREG, W
	IORWF	CCPR1L, F


	; Transfer bits 9,10
	MOVLW	0x3F
	ANDWF	CCPR1L, F ; clear first

	MOVLW	0x03
	ANDWF	ARG1, W
	RRNCF	WREG, W
	RRNCF	WREG, W
	IORWF	CCPR1L, F

	RETURN 1

; Just Do (ARG0)*(ARG1) loops to kill time
; arg0 arg1
delay:
        MOVFF	ARG0, ARG2
        NOP
delay_loop:
        MOVFF	ARG1, ARG3
	NOP
delay_inner_loop:
        DECFSZ  ARG3, F
        GOTO    delay_inner_loop
        DECFSZ  ARG2, F
        GOTO    delay_loop
        RETURN	1

	end
