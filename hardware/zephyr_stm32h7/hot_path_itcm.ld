/*
 * Hot path of the STM32H7 in ITCM (zero wait states, no flash cache misses).
 *
 * Included into Zephyr's .itcm output section (zephyr_linker_sources
 * ITCM_SECTION, app/CMakeLists.txt); the startup code copies it from flash.
 * The .itcm section comes before .text in the linker script, so these
 * patterns take the functions before the generic *(.text.*). They match the
 * per-function sections of -ffunction-sections by mangled name, which also
 * catches template and inline functions whatever object file kept them.
 *
 * Covered: the sample interrupt and the control step (zero latency), the
 * observer interrupt and the slow update, the trace tick of the control
 * cycle and the libc helpers they call. The kernel calls of the observer
 * interrupt (k_sem_give) stay in flash, and so does memset (cymon Tick):
 * the startup code clears the BSS with memset before it copies ITCM, so
 * memset (and memcpy, which does that copy) must not move here. Check with
 * the call graph in
 * docs/hotpath_plan.md, section 4, after changing the hot path.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/* Platform: sample and observer interrupts, timer and ADC access. */
*(.text._ZN6unimoc8hardware11foc_stm32h7*)
*(.text._ZN6unimoc8hardware8hot_path*)
*(.text._ZN6unimoc8hardware13SampleHandlerEv)
*(.text._ZN6unimoc8hardware15ObserverHandlerEv)
*(.text._ZN6unimoc8hardware6analog*)
*(.text._ZN6unimoc8hardware6timing5Probe6Record*)
*(.text._ZN6unimoc8hardware12_GLOBAL__N_111DutyToTicks*)
*(.text._ZN6unimoc8hardware12_GLOBAL__N_114SampleCurrents*)

/* Application core: control step, slow update, outer loop of the observer cycle. */
*(.text._ZN6unimoc15current_control17CurrentControlIsr*)
*(.text._ZNK6unimoc15current_control17CurrentControlIsr*)
*(.text._ZN6unimoc15current_control10SlowUpdate*)
*(.text._ZNK6unimoc15current_control10SlowUpdate*)
*(.text._ZN6unimoc15current_control9OuterLoop15OnObserverCycle*)

/* Portable coordinate systems, control algorithms and observers (lib/motor_control). */
*(.text._ZN6unimoc6system*)
*(.text._ZNK6unimoc6system*)
*(.text._ZN6unimoc7control*)
*(.text._ZNK6unimoc7control*)
*(.text._ZN6unimoc8observer*)
*(.text._ZNK6unimoc8observer*)

/* Trace sampling after each control cycle (cymon Tick). */
*(.text._ZN6unimoc5trace14OnControlCycle*)
*(.text._ZN5cymon6Device4TickEv)
*(.text._ZNK5cymon6Device4ReadEh)
*(.text._ZN5cymon7Trigger*)
*(.text._ZN5cymon12SampleBuffer*)
*(.text._ZNK5cymon12SampleBuffer*)

/* libc helper of the trace tick, and the float conversions of the observer. */
*libc.a:memmove*.o(.text .text.*)
*libgcc.a:_fixunssfdi.o(.text .text.*)
*libgcc.a:_fixsfdi.o(.text .text.*)
*libgcc.a:_arm_addsubsf3.o(.text .text.*)
