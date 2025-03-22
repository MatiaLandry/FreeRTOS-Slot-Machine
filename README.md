# FreeRTOS-Slot-Machine
Final Project for real time systems. Using the FreeRTOS libraries, our objective is to gain experience with the Working principles of RTOS.

## This includes..
- Integration of FreeRTOS (an open-source RTOS) with STM32F4 discovery Board
- Task Management in FreeRTOS and its illustration on Discovery Board
- Queue management in FreeRTOS and its illustration on Discovery Board;
- Resource management in FreeRTOS and its demonstration on Discovery Board


## After importing the project:
1. Go to Project > Properties > C/C++ General > Paths and Symbols.
2. Under Includes for GNU C, add:
   - ${ProjName}/Core/FreeRTOS-Kernel/portable/GCC
   - ${ProjName}/Core/FreeRTOS-Kernel/portable/MemMang
   - TestRTOS/Core/FreeRTOS-Kernel/include
   - (Optional) any absolute path needed for local FreeRTOS sources