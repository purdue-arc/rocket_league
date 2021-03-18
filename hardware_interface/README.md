### Some info on protocols: <br/>
https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/blob/master/Protocols_Details.md <br/>

Checkout the D8 protocol <br/>

### PPM Library used: <br/>
https://www.pjrc.com/teensy/td_libs_PulsePosition.html <br/>

#### Default values for timing in the PulsePosition Library: <br/>
// The shortest time allowed between any 2 rising edges. This should be at
// least double TX_PULSE_WIDTH.
define TX_MINIMUM_SIGNAL 300.0

// The longest time allowed between any 2 rising edges for a normal signal.
define TX_MAXIMUM_SIGNAL 2500.0

// The default signal to send if nothing has been written.
define TX_DEFAULT_SIGNAL 1000.0

// When transmitting with a single pin, the minimum space signal that marks
// the end of a frame. Single wire receivers recognize the end of a frame
// by looking for a gap longer than the maximum data size. When viewing the
// waveform on an oscilloscope, set the trigger "holdoff" time to slightly
// less than TX_MINIMUM_SPACE, for the most reliable display. This parameter
// is not used when transmitting with 2 pins.
define TX_MINIMUM_SPACE 5000.0

// The minimum total frame size. Some servo motors or other devices may not
// work with pulses the repeat more often than 50 Hz. To allow transmission
// as fast as possible, set this to the same as TX_MINIMUM_SIGNAL.
define TX_MINIMUM_FRAME 20000.0

// The length of all transmitted pulses. This must be longer than the worst
// case interrupt latency, which depends on how long any other library may
// disable interrupts. This must also be no more than half TX_MINIMUM_SIGNAL.
// Most libraries disable interrupts for no more than a few microseconds.
// The OneWire library is a notable exception, so this may need to be lengthened
// if a library that imposes unusual interrupt latency is in use.
define TX_PULSE_WIDTH 100.0

// When receiving, any time between rising edges longer than this will be
// treated as the end-of-frame marker.
define RX_MINIMUM_SPACE 3500.0
