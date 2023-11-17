# Micromouse

## Structure
[16 Nov] Version 1 was mostly a success, needs adjustment for TOF sensors to fit (making room for 5 at 45 degree angles) and minor adjustment to get motors to fit.

## Big Picture Coding
[16 Nov] Basically defunct until other parts are implemented.  Rough framework in place, but specifics need to be implemented.  

Split into fffuncts (floodfill) and mmfuncts (micro mouse).  Floodfill functs should have everything wanted ripped from Victor's flood fill algorithm, and should work by itself.  Micro mouse functs currently runs what's necessary for floodfill, and that't it.  Is obviously missing method of sensing world, and thus flood fill doesn't actually work.

## Motor/Motor Control
[16 Nov] Dr. Baek?  Looks like we need to measure the pulses per rotation, and get both motors working together.  Arduino has a built in PWM?  But need to implement controllers.

## TOF sensors
[16 Nov] Currently have one working, working on getting multiple working