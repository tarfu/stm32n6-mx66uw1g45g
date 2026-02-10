/* Flash algorithm will be loaded at this address in RAM (STM32N6 AXISRAM) */
ALGO_PLACEMENT_START_ADDRESS = 0x24000020;

/* Include the flash-algorithm crate's memory.x for section definitions */
INCLUDE memory.x
