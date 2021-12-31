Candlejoust
===

I played this game for the first time at the [CCC](https://en.wikipedia.org/wiki/Chaos_Communication_Congress), using [playstation move controllers](https://www.playstation.com/en-us/accessories/playstation-move-motion-controller/), and absolutely adored it (even though I lose more often than win.)

Those controllers are somewhat expensive though, and using them requires a computer around to interpret input and determine a winner. I decided to make my own using [Arduinos](https://www.adafruit.com/product/2590), [MPU-6050](https://www.adafruit.com/product/3886) accelerometers, and WS2811 LEDs.

The code is super rough right now, but since it's functional it makes sense to stash it in git, and I'd also like to share it with the world!

## how to play
Pretend your device is a candle. Your opponent(s) also have a candle. Your goal is to extinguish their candle without yours extinguishing as well.

## parts & cost
The cost for a single controller is at most $20 USD, but if you're scrappy about sourcing parts you can definitely bring the cost to <$10 USD. If that's still somewhat expensive, consider using cheaper LEDs and [vibration sensor switches](https://www.adafruit.com/product/2590) instead of the MPU-6050 (you will also need to modify the code.) You can scrap rechargable lithium ion batteries from [disposable vapes](https://www.youtube.com/watch?v=NeQKgwM5k74) and other small electronics, though take care to examine the part's data sheet for proper charging procedures. Little USB batteries from dollar stores work great - they usually have a 18650 LiIon battery and charging circuits built in.


## Some tips
- The MPU-6050 seems to require its startup calibration to be done parallel to a relatively level & stable surface
- MPU-6050s normally require individual chip calibration during programming; I've opted not to bother changing defaults per-chip but if yours seems buggy [adjust the calibration values in code](https://www.reddit.com/r/arduino/comments/2lxpl9/help_with_offset_calibration_on_my_mpu6050/clzfi9f/).

## todo
- clean up code
- provide circuit diagrams
- make the "candle" LED responsive to movement and flicker, just like a real candle
- send the MPU-6050 to sleep when the user has lost (are there other power improvements?)
- implement a [piezo buzzer](https://www.adafruit.com/product/1740) and/or [vibrating motor](https://www.adafruit.com/product/1740) for further haptic/sound feedback on warning & losing conditions
- create a 3d printed housing for the electronics & battery
