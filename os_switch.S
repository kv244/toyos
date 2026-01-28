#include <avr/io.h>

.global os_context_switch
.extern os_current_task_ptr
.extern os_scheduler

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
    ; TaskControlBlock struct has stack_ptr at offset 9
    lds r26, os_current_task_ptr
    lds r27, os_current_task_ptr + 1
    sbiw r26, 0
    breq .pick_next

    adiw r26, 9
    in r0, _SFR_IO_ADDR(SPL)
    st X+, r0
    in r0, _SFR_IO_ADDR(SPH)
    st X, r0

.pick_next:
    ; Since we are in ASM, calling a C function 'os_scheduler'
    ; But we must be careful with registers if os_scheduler is called here.
    ; Actually, os_context_switch is called after the state transition in C++.
    ; So we just need to call the scheduler if we want to change tasks.
    
    call os_scheduler

    ; Load new SP from TCB
    lds r26, os_current_task_ptr
    lds r27, os_current_task_ptr + 1
    sbiw r26, 0
    breq .no_task ; Should not happen if idle exists

    adiw r26, 9
    ld r0, X+
    out _SFR_IO_ADDR(SPL), r0
    ld r0, X
    out _SFR_IO_ADDR(SPH), r0

.no_task:
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
