.include "m328pdef.inc"
.include "delay.inc"
.include "UART.inc"
.include "LCD_1602.inc"

.def A = r16
.def AH = r17
.def LCD = r18
.def LCD_BAK = r19


.org 0x00
	LCD_init
	LCD_backlight_ON
	
	; 0: Controlled by Sensor
	; 1: Controlled by App
	; LCD[1] = Pump State
	; 0: OFF
	; 1: ON
	; LCD[2] = Moisture Level
	; 0: HIGH
	; 1: LOW
	ldi LCD, 0b00000000 ; Init LCD Indicator
	ldi LCD_BAK, 0xFF
	; P4 for LED
	sbi DDRB,PB4 
	sbi PORTB,PB4
	; ADC Configuration
	LDI A,0b11000111 ; [ADEN ADSC ADATE ADIF ADIE ADIE ADPS2 ADPS1 ADPS0]
	STS ADCSRA,A
	LDI A,0b01100000 ; [REFS1 REFS0 ADLAR – MUX3 MUX2 MUX1 MUX0]
	STS ADMUX,A ; Select ADC0 (PC0) pin
	SBI PORTC,PC0 ; Enable Pull-up Resistor
	ldi r16, 0x00
	ldi r17, 0x00
	ldi r20, 0x00

	Serial_begin ; initilize UART serial communication

loop:
	delay 1000
	call getSerialData
	call analogRead ; Read Soil Moisture Detector Sensor value in AH
	Serial_writeReg_ASCII AH ; sending the received value to UART
	Serial_writeNewLine
	; LCD[0] is clear LCD in Sensor Mode
	SBRS LCD, 0
		call UPDATE_LCD_REG
	cp LCD, LCD_BAK
	brne DISPLAY_LCD
rjmp loop

UPDATE_LCD_REG:
	cpi AH, 170
	brmi HIGH_MOISTURE
	rjmp LOW_MOISTURE
END_UPDATE_LCD_REG:
	ret

HIGH_MOISTURE:
	andi LCD, 0b11111011
	call PUMP_OFF
	rjmp END_UPDATE_LCD_REG
	
LOW_MOISTURE:
	ori LCD, 0b00000100
	call PUMP_ON
	rjmp END_UPDATE_LCD_REG

PUMP_ON:
	ori LCD , 0b00000010
	ret
	
PUMP_OFF:
	andi LCD,0b11111101
	ret

DISPLAY_LCD:
	SBRS LCD, 0
		call UPDATE_LCD_REG
	mov LCD_BAK, LCD
	; Clear LCD
	LCD_clear
	; LCD[0] is set LCD in Manual Mode
	SBRC LCD, 0
		call display_manual_mode
	; LCD[0] is clear LCD in Sensor Mode
	SBRS LCD, 0
		call display_moisture_level 
	; Move to Next Line
	LCD_send_a_command 0xC0
	; Display Pump
	call display_pump
	; LCD[1] is set LED ON
	SBRC LCD, 1
		call LED_ON
	; LCD[1] is clear LED OFF
	SBRS LCD, 1
		call LED_OFF
	rjmp loop

display_moisture_level:
	call display_moisture
	; LCD[2] is set Moisture Level Low
	SBRC LCD, 2
		call display_low
	; LCD[2] is clear LCD in Moisture Level High
	SBRS LCD, 2
		call display_high 
	ret

LED_ON:
	LDI ZL, LOW (2 * on_string)
	LDI ZH, HIGH (2 * on_string)
	Serial_writeStr

	SBI PORTB, PB4 ; LED ON
	call display_on
	ret

LED_OFF:
	LDI ZL, LOW (2 * off_string)
	LDI ZH, HIGH (2 * off_string)
	Serial_writeStr

	CBI  PORTB, PB4 ; LED ON
	call display_off
	ret

analogRead:
	LDS A,ADCSRA ; Start Analog to Digital Conversion
	ORI A,(1<<ADSC)
	STS ADCSRA,A
	wait:
		LDS A,ADCSRA ; wait for conversion to complete
		sbrc A,ADSC
		rjmp wait

	LDS A,ADCL ; Must Read ADCL before ADCH
	LDS AH,ADCH
	delay 100
	ret

getSerialData:
	Serial_read
	cpi r20, '1'
	breq MANUAL_ON
	cpi r20, '2'
	breq MANUAL_OFF
	cpi r20, '3'
	breq MANUAL_PUMP_ON
	cpi r20, '4'
	breq MANUAL_PUMP_OFF

	endGetSerialData:
	ret

MANUAL_ON:
	ori LCD, 0b00000001
	LDI ZL, LOW (2 * manual_string)
	LDI ZH, HIGH (2 * manual_string)
	Serial_writeStr
	rjmp endGetSerialData
	
MANUAL_OFF:
	andi LCD, 0b11111110
	LDI ZL, LOW (2 * sensor_string)
	LDI ZH, HIGH (2 * sensor_string)
	Serial_writeStr
	rjmp endGetSerialData

MANUAL_PUMP_ON:
	call PUMP_ON
	rjmp endGetSerialData

MANUAL_PUMP_OFF:
	call PUMP_OFF
	rjmp endGetSerialData

display_moisture:
	LDI ZL, LOW (2 * moisture_string)
	LDI ZH, HIGH (2 * moisture_string)
	LCD_send_a_string
	ret

display_low:
	LDI ZL, LOW (2 * low_string)
	LDI ZH, HIGH (2 * low_string)
	LCD_send_a_string
	ret
	
display_high:
	LDI ZL, LOW (2 * high_string)
	LDI ZH, HIGH (2 * high_string)
	LCD_send_a_string
	ret

display_pump:
	LDI ZL, LOW (2 * pump_string)
	LDI ZH, HIGH (2 * pump_string)
	LCD_send_a_string
	ret
	
display_on:
	LDI ZL, LOW (2 * on_string)
	LDI ZH, HIGH (2 * on_string)
	LCD_send_a_string
	ret

display_off:
	LDI ZL, LOW (2 * off_string)
	LDI ZH, HIGH (2 * off_string)
	LCD_send_a_string
	ret

display_manual_mode:
	LDI ZL, LOW (2 * manual_mode_string)
	LDI ZH, HIGH (2 * manual_mode_string)
	LCD_send_a_string
	ret

manual_mode_string:	.db	" MANUAL MODE ON",0
off_string:	.db	"OFF",0x0D,0x0A,0
on_string:	.db	"ON",0x0D,0x0A,0
pump_string:	.db	"   Pump: ",0x0D,0x0A,0
high_string:	.db	"HIGH",0x0D,0x0A,0
low_string:	.db	"LOW",0x0D,0x0A,0
moisture_string:	.db	" Moisture: ",0x0D,0x0A,0
manual_string: .db "Manual",0x0D,0x0A,0
sensor_string: .db "Sensor",0x0D,0x0A,0