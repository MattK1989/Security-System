;;;;;;;;;;;;;;;;;;SecuritySystem.asm;;;;;;;;;;;;;;;;;;;;;;;
;; Security.asm
;;    Lights a green LED if SI matches W-Reg, red LED if not
;; Inputs:       None
;; Outputs:      None
;; Side Effects: None
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; QUESTIONS
;; 1. Does the command 'Return' return to the method that called it
;;    or terminate the program?
;; 2. 
;;
;;
;;
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;





#include <P18F4520.INC> ;processor specific variable definitions
;******************************************************************************
;                   Configuration bits
;******************************************************************************
 CONFIG OSC = INTIO67 ; internal clock @ 8MHz (1MHz with prescaler)
; for timing purposes 1 instruction cycle requires
; 4 clock cycles ... or 4 microseconds
 CONFIG  LVP = OFF      ; Disable low voltage programming for debug
 CONFIG  WDT = OFF ; watch dog timer OFF
 CONFIG  MCLRE = OFF ; MCLEAR (master clear)pin enabled
 CONFIG  PBADEN = OFF ; PORTB pins digital (disable A/D for PORTB)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define Constants
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Found on page 4-4
;; Conversions found on page 2-13
RegF_Addr: EQU 0x00 ;; Address of Reg F
RegG_Addr: EQU 0x01 ;; Address of Reg G
SI_Addr:   EQU 0x02 ;; Address of Switch Input
ALU_Addr:  EQU 0x03 ;; Address of ALU
;; Found on page 2-14
RSP_TSF:   EQU 0x8A ;; T&C code, transfer SI -> [F]
RSP_TGF:   EQU 0x89 ;; T&C code, transfer [G] -> [F]
RSP_TFG:   EQU 0x98 ;; T&C code, transfer [F] -> [G]
RSP_TSG:   EQU 0x9A ;; T&C code, transfer SI -> [G]
RSP_ADDF:  EQU 0x8B ;; T&C code, Add [F] + [G], result -> [F]
RSP_ADDG:  EQU 0x9B ;; T&C code, Add [F] + [G], result -> [G]
DataStack  EQU 0x5FF;; Initialization value for data stack

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Variables Block
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Del_1   RES     1      ;; Variable used in delay routine
Del_2   RES     1      ;; Variable used in delay routine
IterationCounter RES 2 ;; 2-byte variable counter
ALU_Data         RES 1 ;; ALU Data
SI_Data          RES 1 ;; Switch Input Data
RegF_Data        RES 1 ;; Register F Data
RegG_Data        RES 1 ;; Register G Data
RSPWrite_Data    RES 1 ;; Data value passed to RSPWrite subroutine
RSPWrite_Addr    RES 1 ;; Address value passed to RSPWrite subroutine

MyTimer:         RES 2 ; 16-bit timer

TC_Port          equ PORTA
TC_Dir           equ TRISA
E_Clk_Port       equ PORTB
E_Clk_Dir        equ TRISB
Data_Bus_Port    equ PORTC
Data_Bus_Dir     equ TRISC

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Macros
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Macro PUSHF, push a file register onto the data stack
;; Usage: PUSHF FileReg
;; Side Effect: [FileReg] moved onto stack, [FSR2] - 1 -> [FSR2]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PUSHF: macro FileReg
   MOVFF FileReg, POSTDEC2
   endm

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Macro PULLF, pull data from the stack, and place a file register
;; Usage: PULLF FileReg
;; Side Effect: [FSR2] + 1 -> [FSR2] and [[FSR2]] written to [FileReg]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PUSHF: macro FileReg
   MOVFF PREINC2, FileReg
   endm

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Macro INCF16, Increment a 16 bit value, with the low byte at FileReg
;; Usage: INCF16 FileReg
;; Side Effect: (FileReg+1):FileReg  is incremented
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
INCF16: macro FileReg
  INFSNZ  FileReg,F
  INCF    FileReg+1,F
  endm

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Code Block
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 Main:
  CALL     Init_SecuritySystem.asm   ;; Initializes Program
  CALL     CCP1_ISR                  ;; Calls CCP1_ISR

CCP1_ISR:
  BCF PIR1, CCP1IF    ;; Reset CCP1IF flag
  CALL MySystem

MySystem:
  PUSHF    STATUS
  PUSHF    WREG
  TSTFSZ   E_Clk_Port,1      ;; Test E_Clk_Port 1, skip branch if == 0
  BRA      PortBNotZero      ;; Branch to PortBNotZero
  INCF16   MyTimer           ;; Increment MyTimer using macro INCF16
  MOVLW    d'20'             ;; 20T is the count for 20 seconds
  CPFSEQ   MyTimer           ;; Compare Reg W with MyTimer
  BRA      NotEqual01        ;; Branch if 1st byte is not 2 seconds
  TSTFSZ   MyTimer+1         ;; BRA around Security if high byte > 0
  BRA      NotEqual01        ;; Branch if 1st byte is not 2 seconds
  CALL     Security          ;; Call Subroutine Security
  CALL     MySystem_Finalize ;; Call subroutine MySystem_Finalize
  Return

NotEqual01:
  CALL     MySystem_Finalize ;; Call subroutine MySystem_Finalize
  Return

PortBNotZero:
  CLRF     MyTimer           ;; Clear Timer
  CALL     MySystem_Finalize ;; Call subroutine MySystem_Finalize
  Return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Subroutine: RSPRead
;;    Read a byte of data from an RSP device into the
;;    PIC18. The RSP device address is passed in Reg A. The
;;    data is passed back in Reg W.
;; Inputs:       Reg W: Address of RSP device to read ($0...$3)
;; Outputs:      Reg W: Data read from device
;; Side Effects: None
;;
;; P.5-12 Lab Manual: Reg W is used to return the value read,
;; so it is not expected to have its initial value, and does
;; not need to be pushed to the stack. However, the status
;; register must be restored if it is changed.
;;
;; Input read off of Port A
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
RSPRead ;; P.5-16 (Needed to read SI and RegG in the Loop)
;; Obtain address from Reg W, place the data into Reg W



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Subroutine: Init_SecuritySystem.asm
;;    Initializes values for SecuritySystem.asm
;; Inputs:       None
;; Outputs:      None
;; Side Effects: Changes values for various variables
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Init_SecuritySystem.asm:
  CLRF     MyTimer      ;; Clear timer
  CLRF     MyTimer + 1  ;; Clear timer
  SI_Data  EQU 0x2A     ;; 42, 0010 1010
  Return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Subroutine: MySystem_Finalize
;;    Pulls variables off of the Stack
;; Inputs:       None
;; Outputs:      None
;; Side Effects: Pulls variables off of the Stack
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
MySystem_Finalize:
  PULLF WREG
  PULLF STATUS
  Return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Subroutine: Security
;;    Checks the value of the W-Register against the SI
;; Inputs:       None
;; Outputs:      None
;; Side Effects:
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Security:
  MOVLW     SI_Addr          ;; Move SI address into W-Reg
  CALL      RSPRead          ;; Call subroutine RSPRead
  CPFSEQ    SI_Data          ;; Compare W-Reg == SI_Data
  BRA       PasswordMismatch ;; Branch to PasswordMismatch
  MOVLW                      ;; Move $02 into W-Reg
  MOVWF                      ;; [W] -> [G]
  Return

PasswordMismatch:
  MOVLW                      ;; Move $01 into W-Reg
  MOVWF                      ;; [W] -> [G]
  Return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Subroutine: Delay
; inputs : none
; outputs : none
; Side effects: cause a 1 second delay
; clock freq = 1 mhz
; actual delay = 250000 cycles *4us/cycle = 1 second
; error = 0%
;original code generated at piclist.com website
;http://piclist.com/techref/piclist/codegen/delay.htm
;minor modifications by D. McClanahan
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Delay
    movlw 0x4E
    movwf Del_1
    movlw 0xC4
    movwf Del_2
    Delay_0
    decfsz Del_1,f                 ; This nested loop 'burns' 249993
    goto skip1                     ; instuction cycles with the constants
    decfsz Del_2,f                 ; loaded above (Del_1=0x4E, Del_2=0xC4)
skip1
    goto Delay_0

nop                            ;3 additional instruction cycles
nop
nop
return                         ;4 instruction cycles (includes call)

end