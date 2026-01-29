#include <avr/io.h>
#include "toyos.h"

.global os_context_switch
.global fast_zero_stack
.extern os_current_task_ptr
.extern os_scheduler

/**
 * Context switch routine
 * Saves current task context, calls scheduler, restores new task context
 */
os_context_switch:
    ; --- SAVE CONTEXT ---
    push r0
    in r0, _SFR_IO_ADDR(SREG)
    push r0
    push r1
    clr r1      ; AVR C/C++ expects r1 to be zero
    push r2
    push r3
    push r4
    push r5
    push r6
    push r7
    push r8
    push r9
    push r10
    push r11
    push r12
    push r13
    push r14
    push r15
    push r16
    push r17
    push r18
    push r19
    push r20
    push r21
    push r22
    push r23
    push r24
    push r25
    push r26
    push r27
    push r28
    push r29
    push r30
    push r31

    ; Save current SP to os_current_task_ptr->stack_ptr
    ; FIXED: Use constant from toyos.h instead of hardcoded offset
    lds r26, os_current_task_ptr
    lds r27, os_current_task_ptr + 1
    sbiw r26, 0
    breq .pick_next

    ; Add offset to reach stack_ptr field
    adiw r26, TCB_STACK_PTR_OFFSET
    in r0, _SFR_IO_ADDR(SPL)
    st X+, r0
    in r0, _SFR_IO_ADDR(SPH)
    st X, r0

.pick_next:
    ; Call scheduler (C function)
    call os_scheduler

    ; Load new SP from TCB
    lds r26, os_current_task_ptr
    lds r27, os_current_task_ptr + 1
    sbiw r26, 0
    breq .no_task ; Should not happen if idle task exists

    ; Add offset to reach stack_ptr field
    adiw r26, TCB_STACK_PTR_OFFSET
    ld r0, X+
    out _SFR_IO_ADDR(SPL), r0
    ld r0, X
    out _SFR_IO_ADDR(SPH), r0

.no_task:
    ; --- RESTORE CONTEXT ---
    pop r31
    pop r30
    pop r29
    pop r28
    pop r27
    pop r26
    pop r25
    pop r24
    pop r23
    pop r22
    pop r21
    pop r20
    pop r19
    pop r18
    pop r17
    pop r16
    pop r15
    pop r14
    pop r13
    pop r12
    pop r11
    pop r10
    pop r9
    pop r8
    pop r7
    pop r6
    pop r5
    pop r4
    pop r3
    pop r2
    pop r1
    pop r0
    out _SFR_IO_ADDR(SREG), r0
    pop r0
    ret

/**
 * Fast stack zeroing helper function (OPTIMIZATION #16)
 * Zeros 'count' bytes starting from address pointed to by sp (decrementing)
 * 
 * Input: r24:r25 = sp (stack pointer)
 *        r22 = count (number of bytes to zero)
 * Output: r24:r25 = new sp (after zeroing)
 * 
 * Saves approximately 3 cycles per byte compared to C loop
 */
fast_zero_stack:
    movw r30, r24        ; Z = sp
    mov r24, r22         ; r24 = count
    clr r25              ; r25 = 0 (value to write)
    
    ; Check if count is zero
    tst r24
    breq .done
    
.zero_loop:
    st -Z, r25           ; *(--Z) = 0
    dec r24
    brne .zero_loop      ; Loop until count reaches 0
    
.done:
    movw r24, r30        ; Return new sp in r24:r25
    ret
