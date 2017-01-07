WHAT IS IT
----------
This is an inexpensive wireless receiver/transmitter combination for 
quadcopters (or any rc toy) using esp8266 modules.

<pre><code>........................you          your quadcopter ............

+------+            +-----+          +-----+              +-----+
!      !----PPM---->!     !          !     !-----PPM----->!     !
!Remote!            ! ESP !<--wifi-->! ESP !              ! FC  !
!      !<--Serial---!     !          !     !<---Serial----!     !  
+------+            +-----+          +-----+              +-----+</code></pre>

![evalink example](https://raw.githubusercontent.com/emente/evalink/master/evalink.jpg)

PPM input is transported using raw packets (no wifi peering or binding 
needed). PPM input/output is supported by most remotes and flight 
controllers. An diy ESP8266-based transmitter is small enough to fit 
into the transmitter compartment of the turnigy 9x remote.

You should use ESP-07 (or similar) modules that have an connector 
for an external wifi antenna, ESP-01 is only suitable for LOS flying
range-wise.

PPM In/Out is interrupt based (timer0), raw wifi packet reception 
callback based, all the hard work is done, so there is lots of 
playroom to add your own stuff to the code. I'm using this to fly 
a gyrocopter and 250 quadcopter.

The wifi packet type for the PPM data is a beacon packet (with an unused
MFIE tag), so you should be able to see if the transmitter is 
transmitting by scanning for wifi networks on your phone or tablet.

RANGE
-----
Around 600m in nature with external wifi antennas, about 100m with chip
antennas. About the same as the Turnigy 9X 8C V2 that broke on me and
is the reason for this project.

NEXT STEPS
----------
- Make more secure, channel hopping?
- showcase uses on codeninja.de?
