# freq-response-analyzer
This is Arduino software for a frequency response analyzer. 
Arduino will control a DDS (Direct Digital Synthesis) board, which will generate sine waves to be fed to an audio amplifier.
The output of the audio amplifier is connected to a peak detector circuit, which is in turn connected to the Arduino.
This way, Arduino can meqasure and graphically display the frequency response of the amplifier.
