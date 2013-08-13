/**
 * Copyright (c) 2013, Gordon S. Good (velo27 [at] yahoo [dot] com)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * The author's name may not be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL GORDON S. GOOD BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
// This sketch, for a PJRC Teensy microcontroller, interfaces a bend sensor
// to the Teensy and turns it into a mod wheel controller. As you bend the
// the sensor more, a larger value is sent on MIDI continuous controller
// number 1 (mod wheel). If the sensor is, say, sewn onto a glove, the
// mod wheel value could be controlled by making a fist.

// Min, max values we see from bend sensor (using a 10k resistor - see
// http://bildr.org/2012/11/flex-sensor-arduino/ for a wiring diagram).
#define SENSOR_MIN 350
#define SENSOR_MAX 850

// The MIDI controller number to send data on
#define CONTROLLER 1
// Send MIDI data on this channel
#define MIDI_CHANNEL 1
// Send continuous controller message no more than
// every CC_INTERVAL milliseconds
#define CC_INTERVAL 20

// The last time we sent a CC value
unsigned long ccSendTime = 0L;
// The value read from the sensor
int sensorValue;
// The CC value we will send
int ccVal;
// The last CC value we sent
int lastCcVal = 0;

void setup() {
  // Enable the built-in LED for output - we'll
  // turn it on when a CC value is being sent.
  pinMode(13, OUTPUT);
  // And, if an LED is hooked up to pin 3, which
  // can do pulse-width modulation, we'll make
  // that LED's brightness follow the breath
  // value we're sending on the MIDI bus.
  pinMode(3, OUTPUT);
}

void loop() {
  // Only read the sensor if enough time has passed
  if (millis() - ccSendTime > CC_INTERVAL) {
    // read the input on analog pin 0
    sensorValue = analogRead(A0);
    // Map the value, which may range from SENSOR_MIN to
    // SENSOR_MAX, to a value in the range 0 to 127, which is
    // the valid range for a MIDI continuous controller
    // We constrain any values we receive that are outside this
    // range...
    sensorValue = constrain(sensorValue, SENSOR_MIN, SENSOR_MAX);
    // Then map the constrained value to a number in the range
    // 0 to 127. When the sensor is not bent, it produces the
    // largest analog values, so the mapping is reversed (from
    // 127 to 0, rather than 0 to 127).
    ccVal = lastCcVal = map(sensorValue, SENSOR_MIN, SENSOR_MAX, 127, 0);
    // And send the value as a MIDI CC message
    usbMIDI.sendControlChange(CONTROLLER, ccVal, MIDI_CHANNEL);
    // Set the brightness of the PWM LED
     analogWrite(3, 2 * ccVal);
    ccSendTime = millis();
  }
}

