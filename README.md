# STM32-based Geiger Counter

Simple geiger counter, tailored for DIY assembly and home usage.

## Disclaimer: this is not a legal radiation measurement device. It is neither proofed to be accurate, nor reliable. It is not authorized for any regulatory or safety usage, and any results obtained with this device must be verified with a proper equipment. Do not do any operations with radioactive materials that are not complying to local regulations. I have no responsibility for any harm caused by use of this device.

## CAUTION: HIGH VOLTAGE! This device contains 400V power source. Pay attention to safety and do not use this device without front panel attached.

## Key features of stm32-geiger:

1. Neat FR-4 case that is composed with solder-jointed PCBs. PCBs can be also DIY, or outsourced to numerous manufacturers, eliminating complex tooling for single device assembly.
2. Pancake-style detector СБТ-10А, highly sensitive to alpha, beta and gamma radiation. Case is built with provision of filtering out alpha and beta via steel cover.
3. Differential measurement mode for automatic ambient radiation compensation for measuring any traces of contamination.
4. Separate reading with guaranteed 3-seconds latency for searching for high intensity sources.
5. Electronics is designed to avoid any unique or prone to failures components like custom transformers. All components except gm tube are available off-the-shelf.
6. Single 18650 Li battery and micro-USB charging port. Working time is 3-10 days, depends on capacity, radiation intensity and use of display backlight.
7. Nokia 5110 display is readable both at sunlight and in full darkness. The backlight can be turned off for prolonging battery life.
8. Led blinking and sound ticks on particle detection.

## Key shortcomings:
1. СБТ-10А is very fragile. This geiger counter is not suitable for active outdoor use. There is no way to combine alpha and beta sensitivity with ruggedness,
   and my goal was to build as sensitive device as it is possible with reasonable efforts, though the electronics circuit can be useful for other projects.
2. СБТ-10А and electronic circuit are not designed for high levels of radiation. Theoretical limit for this setup is 50mR/h and there is no indication for overloading.
   Never use this device if higher radiation levels are possible, it is intended for low level measurements.
3. Background radiation compensation feature is designed in mind that background radiation level is not changing. Don't even try to measure for 1 month for 1 nR/h difference.
   Also, pancake-style tubes are sensitive to pressure, ambient illumination and have comparably high noise, for pure gamma measurements scintillator-type counters would be better choice.
4. СБТ-10А is rare and requires tricky preparations to strip the tube from the original bulky case. It is possible to redesign the case to use more common GM tubes, but they are
   less sensitive to beta and cannot detect alpha particles at all. 
5. This device is uncalibrated and currently is not distinguishing gamma dose and alpha and beta particles density. The latter can be added in software if necessary.
   There is also a plenty room for software upgrades, i.e., logging facilities, continuous monitoring mode, interaction with PC etc but I'm lacking time and use cases to implement them for myself.

stm32-geiger is optimized for detecting radioactive artifacts like a contaminated food, uranium glass, self-illuminated paintings etc.

For maximum sensitivity, СБТ-10А tube is being used. It has large detecting surface and is very sensitive to beta-particles. 
To fit the tube into pocket-sized device, it's external case was removed. It can be done by cutting off external pins and careful sawing the case slightly above the glued joint with the tube, or by 
drilling the hole in a side wall and syringing some toluene-based solvent. Wait for a while, and the tube can be detached from the case. Be extremely careful with mechanical preparations of there tubes, the mica window have only a few microns thickness and the tubes are underpressured and can explode on shock or piercing.

The case is inspired by https://hackaday.com/2015/06/03/how-to-build-beautiful-enclosures-from-fr4-aka-pcbs/ . It uses 7 PCBs, soldered together to form a rectangular-shaped case with detachable upper wall. The upper wall is fixed with the screws and may be removed for access to the circuitry or for replacing the battery. For filtering out alpha and beta particles and additional protection, steel 
gamma filter can be attached to the back side of counter. It is fixed by magnet salvaged from old HDD glued under the battery section. The steel plate is positioned with an aluminium plate glued on the back side in such a way that it is tightly filling the sensor hole.

The display is a ubiquitous Nokia 5110 module. LCD is beneficial for it's low power consumption for rarely updating data and excellent readability with bright external illumination and ease of use with software SPI. It displays battery charge state, main infinitely averaged and secondary 5-seconds median filtered readings and their error estimations.

stm32f103 is chosen for it's convenience of programming and handful features like a built-in ADC with precision voltage reference that simplifies hardware implementation. It uses low speed main clock (8MHz) and wakes only for particle detections and updates of battery state and readings once a second for prolonging battery life. It also forms a reference 1.21v voltage via PWM for 400v boost converter. 
MCU is constantly running for use of pushbutton power switch, and high voltage converter, LED, display and buzzer are disconnected while power is down via P-fet switch. 

18650 cell is chosen over AA/AAA alkaline cells because of their higher capacity and better price and this device is not intended for outdoor use with constantly replacing batteries in field. For the same reason there is built-in USB charger. The battery can last for a couple of weeks and ensures that any practical long-term measurements can be conducted without interruptions.
Battery is charged with TP4056 charge controller an protected via DW01A.
The system is powered with 3.3v derived from XC6206 LDO.

The step-up converter is PFM-type, based on 74AC14 oscillator and MOSFET with a fixed duty cycle, and LM393 for shutting oscillator down if the output voltage reaches 400v. The boost converter is designed for 40V voltage spikes, multiplied tenfold via voltage multiplier. This design is power efficient for small loading and uses common parts. The inductor is composed from a two parts for lowering voltage spikes stress on it's winding insulation.
The output voltage is monitored from the output of multiplier for better regulation, but it is possible to wire it to C14 to reduce power dissipation on feedback resistor.

LM393 require some voltage reference for this converter. stm32f103 contains decent and low power 1.21v voltage reference, but it is only accessible by ADC. This device measures the relation between the power supply voltage and the reference voltage each second, and use it as a duty cycle setting for PWM. This signal is filtered with low pass filter and forms 1.21V external reference voltage for LM393.

## Usage:

0/Power button: long press to turn the device on/off, short press subtracts currently averaged readings from subsequent ones. The latter feature is useful for long time measurements of small differences in radiation level, just measure the smaller one for a long time, press 0 and measure the difference between old and new level. This is particularly useful for measurement of food contamination.
Short press on 0 resets background subtraction and displays total intensity.
Median filtered value is not affected with background subtraction. 

Light/Ticks button: long press to toggle clicks of particle detection, short press to toggle display illumination

Both main and secondary readings are provided with automatic error estimation. Main error is affected with total particles counted, i.e, the longer is measurement time, the lower is calculated error. However, if background cancellation is used, the total error is affected with uncertainty of background level, and it is impossible to get error margin better than that value. Also one should pay attention to possible small variation of background radiation level, there is no way to automatically distinguish random background variations from the desired changes. Combine the results of repeated background cancelated measurements for getting rid of the background variations and use some additional statistical data conditioning.

Secondary readings are median-filtered and are intended for interactive searching of highly active sources or detecting a short bursts of radiation. The error estimation is based on intensity with an assumption of it's Poisson-like distributed variations and is provided for information purpose only.

## If you want to reproduce this device, watch out for the erratas:

1) XC6206 have mirrored pinout on PCB.
2) DW01A footprint is incorrect, the actual one is SOT23-6 and is bigger.
3) Wire feedback resistors to C14. Use 1M + 1M + 1M resistors to LM393 input, and 91k from lm393 input to ground.
4) LM393 inverting and non-inverting inputs are mirrored.
5) Add reset button. The watchdog operation interferes with the reflashing.
6) Wire blue led directly to MCU because is not, it would draw the power while device is turned off
7) Wire R38 to 74AC14 VCC rail instead of ground for better sharp pulse detection and improved noise immunity. 
8) Recalibrate the counter based on your tube and set a proper NRH_PER_CPS constant in main.c

STM32-geiger was tested with the most common radiation source (potassium fertilizer). Please note that for the gamma level measurement the background radiation level have been cancelled out for a 10 hours, that low level measurements takes and extremely long time to complete.
