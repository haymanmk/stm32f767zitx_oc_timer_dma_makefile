# STM32F7 Timer Output Compare Mode with Double-Buffer

This project aims to generate 200kHz pulse signals at the GPIO outputs to drive the stepper motors on the XYZ machine. In order to mitigate the impact from the interrupt routine which could be too time consuming to finish all the expressions within 5 us (1/200kHz), relatively speaking, we set up the DMA **Double-Buffer Mode** to elong the time period that allow program to prepare the next part of the data for the timer output compare mode.
