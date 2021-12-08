# STM32Discovery-Board
Task:

The objective of the task is as under:
Consider a microcontroller environment connected with the micron NAND flash (use the datasheet attached as the basis at https://drive.google.com/file/d/1xpDxEx5Bug8kzFTo6SeBebMwxRiMV8wG/view?usp=sharing ).
Assume there's a reader threader that acquires sensor data and puts it in a frame of 300bytes every 200ms and sends it to you.
You have to implement a ring buffer that keeps writing this payload of 300Bytes continuously.
1. The system can lose power at any time. You have to employ a mechanism to have resume capability (start writing from where you left off).
2. You have to implement the code for bad block management.

Please write your code in C considering the STM32 controller environment.

We assume that you wouldn't necessarily have the hardware available with you so we assume that you will write your code on top of a driver with certain assumptions.
