; 1 Mbit/s half-duplex serial (8N1) on PD2 (Nano D2) for FEETECH STS bus servos. ATmega328P at 16 MHz only:
; one bit is exactly 16 clock cycles. Every cycle below is counted. Call both with interrupts off.
#include <avr/io.h>

#define PORT_IO _SFR_IO_ADDR(PORTD)
#define PIN_IO  _SFR_IO_ADDR(PIND)
#define BUSBIT  2

.macro PADN n            ; 2*n cycles
  .rept \n
  rjmp .+0
  .endr
.endm

; one data bit: 4 cycles to the OUT (constant whether the bit is 0 or 1), then 12 cycles pad = 16
.macro TXBIT k
  mov  r20, r19          ; 1  assume 0
  sbrc r23, \k           ; 1 if bit set (no skip), 2 if clear (skip)
  mov  r20, r18          ; 1  bit set: use 1
  out  PORT_IO, r20      ; 1
  PADN 6                 ; 12
.endm

; one data bit: sample with SBIC (2 cycles with the ORI, either way), then 14 cycles pad = 16
.macro RXBIT k
  sbic PIN_IO, BUSBIT
  ori  r23, (1 << \k)
  PADN 7
.endm

  .text

; void sbus_tx(const uint8_t *buf, uint8_t n)   r25:r24 = buf, r22 = n (1..255)
; PD2 must already be an output, driven high.
  .global sbus_tx
  .type sbus_tx, @function
sbus_tx:
  movw r30, r24
  in   r18, PORT_IO
  mov  r19, r18
  ori  r18, (1 << BUSBIT)            ; r18 = PORTD with bus high
  andi r19, (~(1 << BUSBIT)) & 0xFF  ; r19 = PORTD with bus low
1:
  ld   r23, Z+           ; 2
  out  PORT_IO, r19      ; start bit
  PADN 6                 ; 12 (+3 into TXBIT 0 = 16)
  TXBIT 0
  TXBIT 1
  TXBIT 2
  TXBIT 3
  TXBIT 4
  TXBIT 5
  TXBIT 6
  TXBIT 7
  nop
  nop
  nop
  out  PORT_IO, r18      ; stop bit, 16 after bit 7
  PADN 6                 ; 12
  dec  r22               ; 1
  breq 8f                ; 1 not taken (loop is too long for BRNE to reach 1b)
  rjmp 1b                ; 2, then LD 2, OUT: stop bit = 19 cycles
8:
  nop                    ; last byte: hold the stop bit >= 16 cycles
  ret
  .size sbus_tx, .-sbus_tx

; uint8_t sbus_rx(uint8_t *buf, uint8_t max, uint16_t first_to, uint8_t gap_to)
;   r25:r24 = buf, r22 = max, r21:r20 = first_to, r18 = gap_to. Returns bytes received.
; Timeouts are in loop turns of 10 cycles (0.625 us): first_to before the first byte, gap_to between bytes.
; PD2 must be an input.
  .global sbus_rx
  .type sbus_rx, @function
sbus_rx:
  movw r26, r24          ; X = buf
  clr  r19               ; count
  movw r24, r20          ; timeout
  tst  r22
  brne 2f
  rjmp 9f
2:                       ; wait for a start bit (line low). Samples 2, 2, 6 cycles apart.
  sbis PIN_IO, BUSBIT
  rjmp 3f
  sbis PIN_IO, BUSBIT
  rjmp 3f
  sbis PIN_IO, BUSBIT
  rjmp 3f
  sbiw r24, 1
  brne 2b
  rjmp 9f
3:                       ; ~3.5 cycles after the edge. First data bit centre is 24 after the edge.
  clr  r23               ; 1
  PADN 9                 ; 18 -> SBIC for bit 0 lands ~1.5 bits after the edge
  RXBIT 0
  RXBIT 1
  RXBIT 2
  RXBIT 3
  RXBIT 4
  RXBIT 5
  RXBIT 6
  sbic PIN_IO, BUSBIT
  ori  r23, 0x80
  st   X+, r23           ; 2
  inc  r19               ; 1
  cp   r19, r22          ; 1
  brsh 9f                ; 1
  mov  r24, r18          ; 1  gap timeout
  clr  r25               ; 1
  PADN 2                 ; 4  back to polling only after bit 7 has ended
  rjmp 2b                ; 2
9:
  mov  r24, r19
  ret
  .size sbus_rx, .-sbus_rx
