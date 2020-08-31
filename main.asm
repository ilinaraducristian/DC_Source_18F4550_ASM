#include "p18f4550.inc"
    
; CONFIG1L
  CONFIG  PLLDIV = 1            ; PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
  CONFIG  CPUDIV = OSC1_PLL2    ; System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
  CONFIG  USBDIV = 1            ; USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

; CONFIG1H
  CONFIG  FOSC = INTOSCIO_EC    ; Oscillator Selection bits (Internal oscillator, port function on RA6, EC used by USB (INTIO))
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
  CONFIG  IESO = OFF            ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

; CONFIG2L
  CONFIG  PWRT = OFF            ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  BOR = OFF             ; Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
  CONFIG  BORV = 3              ; Brown-out Reset Voltage bits (Minimum setting 2.05V)
  CONFIG  VREGEN = OFF          ; USB Voltage Regulator Enable bit (USB voltage regulator disabled)

; CONFIG2H
  CONFIG  WDT = OFF             ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
  CONFIG  WDTPS = 32768         ; Watchdog Timer Postscale Select bits (1:32768)

; CONFIG3H
  CONFIG  CCP2MX = ON           ; CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
  CONFIG  PBADEN = OFF          ; PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
  CONFIG  LPT1OSC = ON          ; Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for low-power operation)
  CONFIG  MCLRE = OFF           ; MCLR Pin Enable bit (RE3 input pin enabled; MCLR pin disabled)

; CONFIG4L
  CONFIG  STVREN = OFF          ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
  CONFIG  LVP = ON              ; Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
  CONFIG  ICPRT = OFF           ; Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
  CONFIG  XINST = OFF           ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

; CONFIG5L
  CONFIG  CP0 = OFF             ; Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
  CONFIG  CP1 = OFF             ; Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
  CONFIG  CP2 = OFF             ; Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
  CONFIG  CP3 = OFF             ; Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

; CONFIG5H
  CONFIG  CPB = OFF             ; Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
  CONFIG  CPD = OFF             ; Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

; CONFIG6L
  CONFIG  WRT0 = OFF            ; Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
  CONFIG  WRT1 = OFF            ; Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
  CONFIG  WRT2 = OFF            ; Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
  CONFIG  WRT3 = OFF            ; Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

; CONFIG6H
  CONFIG  WRTC = OFF            ; Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
  CONFIG  WRTD = OFF            ; Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

; CONFIG7L
  CONFIG  EBTR0 = OFF           ; Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
  CONFIG  EBTR1 = OFF           ; Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
  CONFIG  EBTR2 = OFF           ; Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
  CONFIG  EBTR3 = OFF           ; Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

; CONFIG7H
  CONFIG  EBTRB = OFF           ; Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

   GPR_VAR  UDATA
   VMR	    RES	1 ; view_mode<0> = latch ; view_mode<1> = real value
   TEMP	    RES	1

VML	EQU  .0
VM	EQU  .1
   
RES_VECT  CODE    0x0000
    goto    START

ISRHV     CODE    0x0008
    goto    button_press

MAIN_PROG CODE

START

    ; Oscillator setup
    lfsr    OSCON, b'01110000'	    ; OSCON = 01110000
    
    ; I/O Setup
    lfsr    TRISA, .1		    ; TRISA = 1
    clrf    LATA		    ; LATA = 0
    
    lfsr    TRISB, h'80'	    ; TRISB = 128
    clrf    LATB		    ; LATB = 0
    
    clrf    TRISC		    ; TRISC = 0
    clrf    LATC		    ; LATC = 0
    
    clrf    TRISD		    ; TRISD = 0
    clrf    LATD		    ; LATD = 0
    
    clrf    TRISE		    ; TRISE = 0
    clrf    LATE		    ; LATE = 0
    
    ; Interrupts setup
    lfsr    INTCON, b'11001000'	    ; INTCON = 11001000
    bsf	INTCON2, RBPU		    ; INTCON2<RBPU> = 1
    
    ; Timer 0 setup
    lfsr    T0CON, b'10001000'	    ; T0CON = 0
    clrf    TMR0H		    ; TMR0H = 0
    clrf    TMR0L		    ; TMR0L = 0
    
    ; Timer 1 setup
    bsf	    T1CON, TMR1ON	    ; T1CON<TMR1ON> = 1
    clrf    TMR1L		    ; TMR1L = 0
    
    ; PWM module setup
    setf    PR2			    ; PR2 = 255
    lfsr    CCP2CON, b'00001100'    ; CCP2CON = 00001100
    bsf	    T2CON, T2CKPS1	    ; T2CON<T2CKPS1> = 1
    bsf	    T2CON, TMR2ON	    ; T2CON<TMR2ON> = 1
    clrf    CCPR2L		    ; CCPR2L = 0
    
    ; A/D setup
    bsf	    ADCON0, ADON	    ; ADCON0<ADON> = 1
    lfsr    ADCON1, b'00001110'	    ; ADCON1 = 00001110
    bsf	    ADCON2, ADCS0	    ; ADCON2<ADCS0> = 1
    clrf    ADRESH		    ; ADRESH = 0
    clrf    ADRESL		    ; ADRESL = 0
    
mainloop
    movlw   .200		    ; W = 200
    cpfsgt  TMR1L		    ; if( TMR1L > W ) skip
    goto    mainloop		    ; if( TMR1L <= W ) loop main
    bsf	    ADCON, GO		    ; start A/D conversion
adc_conversion
    btfsc   ADCON, DONE		    ; if( A/D conversion is done ) skip
    goto    adc_conversion	    ; if( A/D conversion is working ) loop adc
    
    rrncf, ADRL, F		    ; ADRL >>= 1
    rrncf, ADRL, F		    ; ADRL >>= 1
    movlw   b'00110000'		    ; W = 00110000
    andwf   ADRL, W		    ; W &= ADRL
    addlw   b'00001100'		    ; W += 00001100
    movwf   CCP2CON		    ; CCP2CON = W
    movff   ADRESH, CCPR2L	    ; CCPR2L = ADRESH
    
    btfsc   VMR, VM		    ; if( view mode == 0 ) skip
    goto    el1			    ; if( view mode == 1 ) go to else
    movff   ADRESH, LATD	    ; LATD = ADRESH
    goto    fi1			    ; end if1
el1
    movlw   .255		    ; W = 255
    movwf   TEMP		    ; TEMP = W
    movlw   .114		    ; W = 114
    bcf	    STATUS, N		    ; STATUS<N> = 0
    subwf   ADRESH, 1		    ; ADRESH -= W
    
division
    btfsc   STATUS, N		    ; if( STATUS<N> == 0 ) skip
    goto    ew1			    ; if( STATUS<N> == 1 ) end ew1
    rrncf   TEMP, 1		    ; TEMP >>= 1
    subwf   ADRESH, 1		    ; ADRESH -= W
    goto    division		    ; loop division
ew1
    movff   TEMP, LATD		    ; LATD = TEMP
fi1
    goto    mainloop		    ; loop main

; Button interrupt
button_press
    btfss   PORTB, RB7		    ; if( PORTB<RB7> == 1 ) skip
    goto    el2			    ; if( PORTB<RB7> == 0 ) go to else
    bcf	    INTCON, RBIF	    ; INTCON<RBIF> = 0
    retfie			    ; return from interrupt
el2
    bcf	    INTCON, RBIF	    ; INTCON<RBIF> = 0
    clrf    TMR0L		    ; TMR0L = 0
    movlw   .251		    ; W = 251
delay500ms
    cpfslt  TMR0L		    ; if( TMR0L < W ) skip
    goto    ew2			    ; if( TMR0L >= W ) exit while
    btfss   INTCON, RBIF	    ; if( INTCON<RBIF> == 1 ) skip
    goto    delay500ms		    ; if( INTCON<RBIF> == 0 ) loop delay
ew2
    btfsc   INTCON, RBIF	    ; if( INTCON<RBIF> == 0 ) skip
    goto    el3			    ; if( INTCON<RBIF> == 1 ) go to else
    ; long press
    movf    VMR, W		    ; W = VMR
    rlncf   VMR, F		    ; VMR <<= 1
    iorwf   VMR, F		    ; VMR |= W
el3
    ; tap
    btg	    VMR, VML		    ; VMR<VML> = !VMR<VML>
    btfss   VM, VML		    ; if( VMR<VML> == 1 ) skip
    goto    el5			    ; if( VMR<VML> == 0 ) go to else
    bsf	    LATC, LC0		    ; LATC<LC0> = 1
    goto    fi5			    ; end fi5
el5
    bcf	    LATC, LC0		    ; LATC<LC0> = 0
fi5
    retfie
    
    END