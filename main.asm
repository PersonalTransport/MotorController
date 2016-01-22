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

ARG0 equ 0x80
ARG1 equ 0x81
ARG2 equ 0x82
 
#define current CCPR1L
#define target	0x83

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
	
	; PWM Initialization using TOSC = 32 us, PWM on P1A (pin 18)
	; 2) PWMperiod = (PR2 + 1) * 4 * TOSC * (TMR2 Prescale)
	; = (99 + 1) * 4 * 32 us * 4 = 51 msec
	MOVLW	99 ; TODO the file's radix is hex should this be 0x63	 HEREHEREHEREHEREHEREHEREHEREHEREHEREHERE					
	MOVWF	PR2
	; 3) Set PWM Mode
	MOVLW	0x00C ; "0000 1100?
	MOVWF	CCP1CON ; PWM output on P1A (Pin 18)
	; 4) PWMdutyCycle = (CCPR1L:CCP1CON<5:4>)*TOSC*(TMR2 Prescale)
	; = (CCPR1L:11)* 32 * 4 us where CCPR1L control high value.
	CLRF	CCPR1L ; Set the duty cycle to 0 for 0% power
	; Set to (51 ms/(4*32)=398) >>2 or 0x63 or 99 for 100% power
	; 5) Clear and Configure Timer 2 (PWM requires Timer 2)
	CLRF	TMR2 ; Timer 2 Register
	MOVLW	0x05 ; Enable timer and set prescale to 4
	MOVWF	T2CON
	BCF	PIR1, TMR2IF ; Clear Timer 2 flag 
		
	BSF	ADCON0,GO
	
	CLRF	target
loop:
	MOVFF target, ARG0
	NOP ; TODO is this nop needed?
	CALL	linear_interpolate
	BRA	loop
	

; A copy of target is used in linear_interpolate
; as to make sure things go boom!
; i.e. this function is reentrant.
; void linear_interpolate(uint8_t target);
; NOTE check with Nick to make sure my logic is correct!
linear_interpolate:
; target_less_equal_current:
	MOVF	ARG0, W
	SUBWF	current, W
	BTFSS	STATUS, 0
	BRA	target_greater_current
	MOVFF	ARG0, current
	NOP ; TODO is this nop needed?
	RETURN	0
target_greater_current:
	INCF    current
	RETURN	0

	
PW_adjustment:

	MOVFF	ADRESH, target
	
	BCF	PIR1, ADIF ; clear flag
	BSF	ADCON0, GO   ; start next conversion
	RETFIE
	
	end



