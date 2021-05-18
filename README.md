# freq-response-analyzer
This is Arduino software for a frequency response analyzer. 
![Picture](https://content.instructables.com/ORIG/F21/NOMO/KN0BG982/F21NOMOKN0BG982.jpg)

Arduino will control a DDS (Direct Digital Synthesis) board (AD9834), which will generate sine waves to be fed to an audio amplifier.
The output of the audio amplifier is connected to a peak detector circuit, which is in turn connected to the Arduino.
This way, Arduino can measure and graphically display the frequency response of the amplifier.

Details in https://www.instructables.com/Frequency-Response-Generator/
