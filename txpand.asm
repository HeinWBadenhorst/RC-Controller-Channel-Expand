;=======================================================================================
;                   Extra Channels for 2 ~ 6 Channel Transmitter
;=======================================================================================
;                    Bruce Abbott bhabbott@paradise.net.nz
;
; for Microchip PIC12F615
;
; Features:
;
;     - 2 extra channels 
;     - Fully Proportional  
;
; Changes:
;
; 2010-05-22 V0.0 created
; 2010-06-30 V0.1 support 5 and 6 channel transmitters
;
		ifdef	__12F615
                PROCESSOR PIC12F615
                INCLUDE   <P12F615.inc>
CMCON		equ	CMCON0
VREN		equ	CMVREN
		endif

                radix     dec

		errorlevel -302,-305	; not reporting bank<>0, dest=file

;#DEFINE SIMULATE	; for debugging only!

#DEFINE	BASECHANNELS 6	; number of channels in transmitter (2~6)

#define VERSION " T X P 6 1 5   0 . 1"

; =====================================================================================
; Configuration is:
;   Master Clear pin is disabled (used for servo pulse input)
;   Code Protection is OFF
;   Watchdog Timer is ON
;   Oscillator is Internal RC
;
		ifdef	__12F615
		__config  _MCLRE_OFF & _INTRC_OSC_NOCLKOUT & _IOSCFS_8MHZ
		endif

; GPIO register bits 
CHAbit          EQU       GP0 ; pin 7. first expansion channel analog input 
CHBbit          EQU       GP1 ; pin 6. second expansion channel analog input
PPMoutBit       EQU       GP2 ; pin 5. PPM pulse stream output
PPMinBit        EQU       GP3 ; pin 4. PPM pulse stream input
;               EQU       GP4 ; pin 3.  
;               EQU       GP5 ; pin 2. 

; I/O direction register value
TrisBits        EQU       (1<<CHAbit)|(1<<CHBbit)|(1<<PPMinBit)	; inputs

; Bits to be set with the OPTION instruction
;     7 = Weak pullups disabled
;     6 = Interrupt on Rising Edge
;     5 = Timer 0 source internal
;     4 = Which edge is don't care
;   3~1 = Prescaler to watchdog, set to give 16*17 = 272ms timeout

OptionBits      EQU       B'11011100'

; =====================================================================================
;                        General Purpose Register Definitions  
; =====================================================================================
		ifdef	__12F615
		CBLOCK	0x40
		ENDC
		endif

		CBLOCK
                flags           ; Various boolean flags
		Pot1:2		; pot 1 current position (16 bits)
		Pot2:2		; pot 2 current position (16 bits)
		samples		; number of samples per pot measurement
		ChannelA	; channel A width 0-255 = 1~2mS
		ChannelB	; channel B width 0-255 = 1~2mS
                PulseDelay      ; output pulse delay timer   
                temp		; general purpose variable
		ENDC

; Bits in Flags

SyncBit 	EQU       1	; sync gap detected

;==================================================================================
;                     Macro for generating short time delays
;===================================================================================
NO_OP           MACRO   count
NO_OP_COUNT     SET     count
                WHILE   NO_OP_COUNT>1
		goto	$+1		; 2 clocks
NO_OP_COUNT     SET     NO_OP_COUNT-2
                ENDW
		IF	NO_OP_COUNT
		nop			; 1 clock
		ENDIF
                ENDM

;==================================================================================
;                       16 Bit Math Macros  (little-endian)
;==================================================================================

; 16 bit unsigned addition with carry out.
;
; Operation: DST = DST + SRC                       
;
; DST is replaced, SRC is preserved
; Carry Set if SRC + DST > 65535
;
ADD16   MACRO   SRC,DST 
        MOVF    (SRC),W         ; Get low byte
        ADDWF   (DST),F         ; Add to destination
        MOVF    (SRC)+1,W       ; Get high byte
        BTFSC   STATUS,C        ; Check for carry
        INCF    (SRC)+1,W       ; Add one for carry
        ADDWF   (DST)+1,F       ; Add high byte into DST
        ENDM

; 16 bit unsigned subtraction with carry out.
; Word format is little endian (LSB at lower address)
;
; Operation: DST = DST - SRC
;
; DST is replaced, SRC is preserved
; Carry Set if DEST >= SRC (result >=0) 
;
SUB16   MACRO   SRC,DST
        MOVF    (SRC),W         ; Get low byte of subtrahend
        SUBWF   (DST),F         ; Subtract DST(low) - SRC(low)
        MOVF    (SRC)+1,W       ; Get high byte of subtrahend
        BTFSS   STATUS,C        ; If there was a borrow then
        INCF    (SRC)+1,W       ; decrement high byte of dst
        SUBWF   (DST)+1,F       ; Subtract the high byte 
        ENDM

; 16 bit copy
;
MOV16   MACRO   SRC,DST
        MOVF    SRC,W
        MOVWF   DST
        MOVF    SRC+1,W
        MOVWF   DST+1
        ENDM

; 16 bit clear 
;
CLR16   MACRO   VAR16
        CLRF    VAR16
        CLRF    VAR16+1
        ENDM
        
; 16 bit Logical Shift Right (= divide by 2)
;
LSR16   MACRO   VAR16
        CLRC		        ; Clear carry
        RRF     (VAR16)+1       ; Rotate high byte right
        RRF     (VAR16)         ; Rotate low byte right
        ENDM
        
; 16 bit Logical Shift Left  (= multiply by 2)
;
LSL16   MACRO   VAR16
        CLRC		        ; Clear carry
        RLF     (VAR16)         ; Rotate low byte left
        RLF     (VAR16)+1       ; Rotate upper byte left
        ENDM  

; 16 bit Rotate Left
;
RL16	MACRO	VAR16
	RLF	(VAR16)
	RLF	(VAR16)+1
	ENDM

; 16 bit Rotate Right
;
RR16	MACRO	VAR16
	RRF	(VAR16)+1
	RRF	(VAR16)
	ENDM

;==================================================================================
;                                     RESET
;==================================================================================
;
Coldstart:      ORG     0                

		goto	start

		org	4
		goto	isr

		org	0x8
                dw      VERSION

;----------------------------------------------------------------------------------
;                            Interrupt Service Routine
;----------------------------------------------------------------------------------

isr:		return	


;==================================================================================
;                         Wait for start of next PPM pulse
;==================================================================================
;
waitpulse:	clrwdt 
		btfsc   GPIO, PPMinBit	; wait for end of last pulse
                goto    waitpulse
waitpulse1: 	btfss   GPIO, PPMinBit  ; wait for start of next pulse
                goto    waitpulse1
		return

;==================================================================================
;                                Wait for sync gap
;==================================================================================
;
waitsync:	clrwdt 
		btfsc   GPIO, PPMinBit
                goto    waitsync	; Wait for end of last pulse
		movlw	2500/10
		movwf	temp
waitsync2:	clrwdt 			; prevent watchdog timeout
		btfsc	GPIO, PPMinBit	; 
                goto    waitsync	; Restart if pulse detected 	
		btfsc	GPIO, PPMinBit	; during next 20 cycles (10uS)
                goto    waitsync	;	
		btfsc	GPIO, PPMinBit	;
                goto    waitsync	;
		btfsc	GPIO, PPMinBit	;	
                goto    waitsync	; 
		btfsc	GPIO, PPMinBit	;
                goto    waitsync	;
		btfsc	GPIO, PPMinBit	;
                goto    waitsync	;	
		btfsc	GPIO, PPMinBit	;
                goto    waitsync	;
		btfsc	GPIO, PPMinBit	;	
                goto    waitsync	;
		decfsz	temp		; Repeat 250 times = 2500uS	
		goto	waitsync2	;
		return

;=================================================================================
;                Measure voltage on a potentiometer  
;=================================================================================
; input: w = pointer to 16 bit variable 'potx'  
;
; output: pot = 10 bit voltage ratio (0~1023 = 0~Vref)  
;  
GetPot:		movwf	FSR		; FSR = pot
		movlw	(200-3)/4
pot_delay:	addlw	-1
		skpnc			; wait 100uS for A/D input to stabalize
		goto	pot_delay	
		movlw	8		; 8 x oversampling
		movwf	samples
		clrf	INDF		; pot (low byte) = 0		  
		incf	FSR		
		clrf	INDF		; pot (high byte) = 0
pot_read:	bsf	ADCON0,GO	; start A/D conversion
pot_wait:	btfsc	ADCON0,GO	; wait for conversion to complete
		goto	pot_wait
		movf	ADRESH,w	; get A/D result (high byte)
		addwf	INDF		; add to pot (high byte)
		banksel	ADRESL
		movf	ADRESL,w	; get A/D result (low byte)
		banksel	0
		decf	FSR
		addwf	INDF		; add to pot (low byte)
		incf	FSR
		skpnc			; low byte overflow?
		incf	INDF		; yes, add 1 to high byte
		decfsz	samples
		goto	pot_read	; next sample 
		movlw	3
pot_div:	clrc
		rrf	INDF
		decf	FSR
		rrf	INDF		;  divide by 8
		incf	FSR
		addlw	-1
		skpz
		goto	pot_div
		return

;=================================================================================
;                               Get Pot positions
;=================================================================================
;
GetPots:	bcf	ADCON0,CHS0	; select A/D channel 0
		movlw	Pot1
		call	GetPot		; get pot1 voltage (10 bit)
		RR16	Pot1
		RR16	Pot1		; rotate right * 2 = / 4 = convert to 8 bit
		movf	Pot1,w
		movwf	ChannelA	; channelA = pot1 position (0~255)
		bsf	ADCON0,CHS0	; select A/D channel 1
		movlw	Pot2
		call	GetPot		; get pot2 voltage (10 bit)
		RR16	Pot2
		RR16	Pot2		; rotate right * 2 = convert to 8 bit
		movf	Pot2,w
		movwf	ChannelB	; channelB = pot2 position (0~255)
		return
 
;================================================================================
;                         Generate PPM output pulse  
;================================================================================
; finish previous channel pulse, create channel width, start channel ending pulse 
;
; input: w = channel width
;
DoChannel:	movwf	PulseDelay	
		call	delay300	; delay 300uS
		bcf	GPIO,PPMoutBit	; finish previous pulse
		movlw	((700-12)/4)-1
dp_2:		NO_OP	4
		addlw	-1
		skpnc			; delay +690uS
		goto	dp_2
		incf	PulseDelay
dp_3:		NO_OP	5
		decfsz	PulseDelay	; total delay = 0.99~2.01mS
		goto	dp_3
		bsf	GPIO,PPMoutBit	; start next pulse
                retlw   0              

;================================================================================
;                               300uS Delay
;================================================================================		
delay300:	movlw	((300-2)/4)-1
delay_300a:	NO_OP	4
		addlw	-1
		skpnc			
		goto	delay_300a
		return


;==================================================================================
;                       Copy PPM pulse from input to output
;==================================================================================
;
copypulse:	btfss   GPIO, PPMinBit
		goto	copypulse
                bsf	GPIO,PPMoutBit	
copypulse1:	btfsc   GPIO, PPMinBit
		goto	copypulse1
                bcf	GPIO,PPMoutBit	
		return

;------------------------------------------------------------------------------------
;                                     STARTUP 
;------------------------------------------------------------------------------------
;
; Initialise GPIO register

start:          clrf    GPIO		; all outputs will be low
                movlw   TrisBits
                tris    GPIO		; set I/O directions

 		banksel	ANSEL		; Register bank 1

		movlw	(1<<ADCS2)|(1<<ADCS0)|(1<<AN0)|(1<<AN1)
		movwf	ANSEL		; clk/8, Analog I/O on GP0 and GP1 
		
		banksel	0		; register bank 0

;
; Move the prescaler from tmr0 to the watchdog, without accidental resets.
;
                clrwdt                    
                clrf      TMR0            
                movlw     OptionBits | 7  
                option                    
                clrwdt
                movlw     OptionBits
                option
                clrwdt

; set up comparator
		movlw	b'00000111'	; comparator disabled
		movwf	CMCON0

; set up A/D convertor
		movlw	(1<<ADFM)|(1<<ADON) ; right-justify, A/D on
		movwf	ADCON0

; ready to go!            
		clrf	flags
		call	GetPots		; get initial pot positions

		call	waitpulse	; must NOT start in middle of sync gap!		

mainloop:	call	waitsync	; wait for sync gap
		variable channel=0
		while	channel<BASECHANNELS		
		call	copypulse	; pass through base channel pulses 
channel=channel+1
		endw
		call	waitpulse	; wait for start of final base channel pulse
		bsf	GPIO,PPMoutBit	; start output pulse
		movf	ChannelA,w
		call	DoChannel	; add first expansion channel 

		movf	ChannelB,w
		call	DoChannel	; add second expansion channel 

		call	delay300	
		bcf	GPIO,PPMoutBit	; finish final output pulse 

		call	GetPots		; get new pot positions

		goto	mainloop

                END

