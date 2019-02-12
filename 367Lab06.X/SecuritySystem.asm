;;;;;;;;;;;;;;;;;;SecuritySystem.asm;;;;;;;;;;;;;;;;;;;;;;;
;; SecuritySystem.asm
;;    Lights a green LED if SI matches W-Reg, red LED if not
;; Inputs:       None
;; Outputs:      None
;; Side Effects: None
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; CCP1.asm
;;   A basic shell to initialize and run CCP1 interrupts
;;  Inputs: none
;;
;;  Outputs: none
;;
;;  Side effects:
;;    The CCP1_ISR executes once every 10 mS, and the
;;    CCP1_Counter is incremented at a rate of 100 Hz
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    LIST P=18F4520    ;directive to define processor
    #include <P18F4520.INC> ;processor specific variable definitions
    CONFIG  OSC = INTIO67       ; internal clock @ 8MHz (1MHz with prescaler),
    CONFIG  WDT = OFF        ; watch dog timer OFF
    CONFIG  MCLRE = ON        ; MCLEAR (master clear)pin enabled
    CONFIG  PBADEN = OFF        ; PORTB pins digital (disable A/D for PORTB)

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;  Constants Section
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ;; Standard constants for RSP
SI_Addr:             EQU     0x02   ;; Address of Switch Input
Write_RegG:          EQU     0x90   ;; T&C code, RSP Data Bus -> [G]
TC_Port:             EQU     PORTA  ;; Port for Timing and Control Code
TC_Dir:              EQU     TRISA  ;; Direction register for Timing and Control Code
E_Clk_Port:          EQU     PORTB  ;; Port for E-Clock
E_Clk_Dir:           EQU     TRISB  ;; Direction register for E-Clock
Data_Bus_Port:       EQU     PORTC  ;; Port for RSP Data Bus
Data_Bus_Dir:        EQU     TRISC  ;; Direction register for RSP Data Bus
PIC18_Top_Of_Data_Stack: EQU 0x5FF  ;; Initialization value for PIC18 Data Stack


  ;; Special constants for CCP1.asm
CountsPerInterrupt  EQU D'25000'  ;; Increment for CCP1


  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;  Variables Section
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    udata_acs  0x00
T_C_Code:        RES 1 ;; Space for application
SI_Data:         RES 1 ;; Space for application, Switch Input Data
CCP1_Counter:    RES 2 ;; Counter for CCP1 interrupt
MyTimer:         RES 2 ;; 16-bit timer
PasswordResult:  RES 1 ;; Variable used to pass result to
                       ;; Reg G in Security method

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;  Macros Section
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; Macro PUSHF, push a file register onto the data stack
  ;; Usage: PUSHF FileReg
  ;; Side Effect: [FileReg] moved onto stack, [FSR2]-1 -> [FSR2]
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PUSHF: macro FileReg
  MOVFF FileReg, POSTDEC2
  endm

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; Macro PULLF, pull data from the stack, and place a file register
  ;; Usage: PULLF FileReg
  ;; Side Effect: [FSR2]-1 -> [FSR2] and *[FSR2] written to [FileReg]
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PULLF: macro FileReg
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

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;  Program
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ORG 0x0000               ;; In the PIC18, there are 3 initial "Vectors"
  Goto  Main               ;; That is, places that the hardware jumps to.
                           ;; Address 0x0000 is the target for a reset

  ORG 0x0008               ;; Address 0x0008 is the target for the
  Goto  High_Priority_ISR  ;; High-priority interrupt

  ORG 0x0018               ;; Address 0x0018 is the target for the
  Goto  Low_Priority_ISR   ;; Low-priority interrupt

Main:

  LFSR FSR2, PIC18_Top_Of_Data_Stack ;; Initialize the data stack
  Call Init_CCP1_Interrupt           ;; Initialize the CCP1 interrupt


Loop:
  NOP
  GOTO Loop     ;;  Go into an idle loop


  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; Init_CCP1_Interrupt   Initialize the CCP1 Interrupt
  ;; Inputs:  None
  ;; Outputs: None
  ;; Side effects:  Zero CCP1_Counter,
  ;;    Set T1CON bits to activate Timer1
  ;;    Set T3Con bits for Timer3 (does not activate)
  ;;    Set CCP1CON bits, to activate compare mode and interrupt
  ;;    Set RCON:IPEN bit,  to active PIC18 mode interrupts (high/low priority)
  ;;    Set IPR1 bit,  to make CCP1 interrupt a high-priority interrupt
  ;;    Clear PIR1 bit,  to clear CCP1 interrupt flag
  ;;    Set PIE1 bit,    to enable CCP1 interrupt
  ;;    Set INTCON, GIEH to generally enable high-priority interrupts
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Init_CCP1_Interrupt:

  PUSHF  WREG           ;; WReg is used
  CALL   Init_SecuritySystem
  CLRF   TRISD          ;; Setup port D for pulses to visualize action on OScope

  CLRF   CCP1_Counter   ;; Increment CCP1_Counter
  CLRF   CCP1_Counter+1

  MOVLW  B'10000001'    ;; Setup    T1CON for Counting
                        ;; RD16:    b7, latched 16-bit read
                        ;; T1CKPS1: b5, 1:2 prescaler (options, 1:1, 1:2, 1:4, 1:8)
                        ;; T1CKPS0: b4, 1:2 prescaler (options, 1:1, 1:2, 1:4, 1:8)
                        ;; T1SYNC_bar:  b2=0, T1 clock with synchronized with internal phase clock
                        ;; TMR1ON:  b0, Turn timer 1 on
  MOVWF  T1CON,

  MOVLW  B'10000000'    ;; Setup    T3Con for Counting, and CCP1 and CCP2 from Timer1
  MOVWF  T3CON,         ;; RD16L:   latched 16-bit read

  MOVLW  B'00001010'    ;; Setup    CCP1CON for compare mode
  MOVWF  CCP1CON        ;; CCP1Mode = 1010,  Set CCP1IF bit (request interrupt)

  BSF    RCON,IPEN      ;; Active PIC18F High-priority / Low-priority mode
  BSF    IPR1,CCP1IP    ;; Make CCP1 a high-priority interrupt
  BCF    PIR1,CCP1IF    ;; Clear the CCP1 Interrupt Flag (so that it can be set to generate IRQ)
  BSF    PIE1,CCP1IE    ;; Enable the CCP1 interrupt
  ;;BSF    INTCON,GIEL    ;; Enable low-priority interrupts
  BSF    INTCON,GIEH    ;; Enable high-priority interrupts and all interrupts

  PULLF  WREG

  RETURN


  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; Interrupt Service Routine for the high priority interrupt
  ;;   The pattern in this high-level part of the interrupt service routine is:
  ;;     Check the interrupt flag
  ;;     If it is clear (not set), branch to the next check
  ;;       Otherwise (int. flag was set), service the interrupt request
  ;;       Go back up to the top of the list, and start again.
  ;;
  ;;   This pattern has the characteristic that High_Priority_ISR doesn exit
  ;;   until all interrupts in the list that are requesting service,
  ;;   have been serviced.
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
High_Priority_ISR:

  BSF    PORTD,0x00

  BTFSS  PIR1,CCP1IF        ;; Test whether CCP1IF is set (CCP1 interrupt requested)
  BRA    HP_ISR01           ;; If not set, go to next candidate
  RCALL  CCP1_ISR           ;; Call the CCP1 ISR
  BRA    High_Priority_ISR  ;; Go to top, test all IRQs again

HP_ISR01:

  BTFSS  PIR1,TMR1IF        ;; Test whether TMR1IF is set (TMR1 interrupt requested)
  BRA    HP_ISR02           ;; If not set, go to next candidate
  RCALL  TMR1_ISR           ;; Call the TMR1 ISR
  BRA    High_Priority_ISR  ;; Go to top, test all IRQs again

HP_ISR02:

  BCF     PORTD,0x00
  RETFIE  FAST              ;; Return from the interrupt, FAST for high-priorty IRQ

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; Interrupt Service Routine for the low-priority interrupt
  ;;  See description of programming patterh above
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Low_Priority_ISR:

  BSF    PORTD,0x01

  BTFSS  PIR2,CCP2IF        ;; Test whether CCP2IF is set (CCP2 interrupt requested)
  BRA    LP_ISR01           ;; If not set, go to next candidate
  RCALL  CCP2_ISR           ;; Call the CCP2 ISR
  BRA    High_Priority_ISR  ;; Go to top, test all IRQs again

LP_ISR01:

  BTFSS  PIR2,TMR3IF        ;; Test whether TMR3IF is set (TMR3 interrupt requested)
  BRA    LP_ISR02           ;; If not set, go to next candidate
  RCALL  TMR3_ISR           ;; Call the TMR3 ISR
  BRA    High_Priority_ISR  ;; Go to top, test all IRQs again

LP_ISR02:

  BCF    PORTD,0x01
  RETFIE                    ;; Return from the interrupt, No FAST for low-priority interrupt

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; CCP1_ISR  Service the needs of the CCP1 interrupt
  ;; Inputs:  none
  ;; Outputs: none
  ;; Side effects:  Clear CCP1IF, to enable next CCP1 interrupt
  ;;    Increment CCP1_Counter
  ;;    Call User subroutine
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
CCP1_ISR:
  BSF   PORTD,0x02
  PUSHF STATUS         ;; STATUS and WREG are changed in this routine
  PUSHF WREG

  INCF16 CCP1_Counter  ;; Increment the interrupt counter, CCP1_Counter

  MOVLW  LOW CountsPerInterrupt  ;; Update CCPR1H:CCPR1L for the next interrupt
  ADDWF  CCPR1L,F                ;; CountsPerInterrupt in the future
  MOVLW  HIGH CountsPerInterrupt ;;
  ADDWFC CCPR1H,F                ;;

  BCF  PIR1,CCP1IF     ;; Clear the CCP1 Interrupt Flag (so that the next IRQ can be generated)

        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;;  Put User Code Here.
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  Call  Security

  PULLF WREG           ;; Restore STATUS and WREG to previous values
  PULLF STATUS
  BCF   PORTD,0x02
  RETURN

Security:
 ; CALL     Init_SecuritySystem   ;; Initializes Program
  PUSHF    STATUS
  PUSHF    WREG
  BTFSC    E_Clk_Port,1      ;; Test E_Clk_Port 1, skip branch if == 0
  BRA      PortBNotZero      ;; Branch to PortBNotZero
  INCF     MyTimer           ;; Increment MyTimer using macro INCF16
  MOVLW    d'20'             ;; 20T is the count for 20 seconds
  CPFSEQ   MyTimer           ;; Compare Reg W with MyTimer
  BRA      NotEqual01        ;; Branch if 1st byte is not 2 seconds
  CALL     DoSecurity        ;; Call Subroutine DoSecurity
  CALL     MySystem_Finalize ;; Call subroutine MySystem_Finalize
  Return

NotEqual01:
  CALL     MySystem_Finalize ;; Call subroutine MySystem_Finalize
  Return

PortBNotZero:
  CALL     MySystem_Finalize ;; Call subroutine MySystem_Finalize
  Return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Subroutine: Init_SecuritySystem
;;    Initializes values for SecuritySystem.asm
;; Inputs:       None
;; Outputs:      0x2A -> SI_Data
;; Side Effects: Clears timers and moves security code into SI_Data
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Init_SecuritySystem:
  CLRF     MyTimer      ;; Clear timer
  CLRF     MyTimer + 1  ;; Clear timer
  MOVLW    0x2A         ;; 42, 0010 1010
  MOVWF    SI_Data
  Return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Subroutine: DoSecurity
;;    Checks the value of the W-Register against the SI
;; Inputs:       
;; Outputs:      
;; Side Effects: 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
DoSecurity:
  MOVLW     SI_Addr          ;; Move SI address into W-Reg
  CALL      RSPRead          ;; Call subroutine RSPRead
  CPFSEQ    SI_Data          ;; Compare W-Reg == SI_Data, skip if equal
  BRA       PasswordMismatch ;; Branch to PasswordMismatch
  MOVLW     0x02             ;; Move $02 into W-Reg
  MOVWF     PasswordResult   ;; [W] -> PasswordResult
  CALL      RSPWrite
  Return

PasswordMismatch:
  MOVLW    0x01              ;; Move $01 into W-Reg
  MOVWF    PasswordResult    ;; [W] -> PasswordResult
  CALL     RSPWrite
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
RSPRead: ;; P.5-16 (Needed to read SI and RegG in the Loop)
    PUSHF STATUS              ;status to stack
    PUSHF TRISC               ;TRISC to stack
;step 1
    bcf     LATB,0            ;lower clock
;step 2
    movlw   SI_Addr
    movwf   PORTA             ;push T&C code
;step 3
    bsf     LATB,0            ;raise clock
;step 4
    movf    PORTC,W           ;PORTC -> WREG
;step 5
    bcf     LATB,0            ;lower clock
;step 6
    nop                       ;do nothing
    PULLF TRISC               ;TRISC off stack
    PULLF STATUS              ;Status off stack
    RETURN

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Subroutine: RSPWrite
; Description: Write a byte of data from the PIC18 into a specified RSP
;          register
;
; Inputs:  Address of the RSP register to write ($0 or $1) is passed in static
;              variable RSPWrite_Address. The data to write are passed in static
; variable RSPWrite_Data.
;
; Outputs: None
;
; Side Effects: Writes data into an RSP register
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
RSPWrite:
    PUSHF STATUS
    PUSHF WREG
    PUSHF TRISC
;step 1
    bcf    E_Clk_Port,0        ; lower clock
;step 2
    ;movf  RSPWrite_Address,W   ; move the write address to the wreg
    ;addlw Address_mask         ; add mask to wreg to get correct T&C
    ;movwf WriteG               ; move read T&C to a var
    ;swapf WriteG,W             ; swap the nibbles making it a write T&C
    movlw   Write_RegG         ; Move 0x90 to W-Reg
    movwf   TC_Port            ; push T&C code
;step 3
    bsf     E_Clk_Port,0       ; raise the clock
;step 4
    clrf  TRISC                ; set TRISC as output
    movf  PasswordResult,W
    movwf LATC

;step 5
    bcf    E_Clk_Port,0        ; lower the clock
;step 6
    setf   TRISC               ; set TRISC as input
    nop                        ; do nothing
    PULLF TRISC
    PULLF WREG
    PULLF STATUS
    RETURN

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; TMR1_ISR  Service the needs of the CCPI interrupt
  ;; Inputs:  none
  ;; Outputs: none
  ;; Side effects:   Clear TMR1IF, to setup for next TMR1 interrupt
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
TMR1_ISR:
  BCF  PIR1,TMR1IF
  RETURN

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; CCP2_ISR  Service the needs of the CCP2 interrupt
  ;; Inputs:  none
  ;; Outputs: none
  ;; Side effects:   Clear CCP2IF, to setup for next CCP2 interrupt
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
CCP2_ISR:
  BCF PIR2,CCP2IF
  RETURN

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; TMR3_ISR  Service the needs of the CCPI interrupt
  ;; Inputs:  none
  ;; Outputs: none
  ;; Side effects:  Clear TMR3IF, to setup for next TMR3 interrupt
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
TMR3_ISR:
  BCF  PIR2,TMR3IF
  RETURN

  end