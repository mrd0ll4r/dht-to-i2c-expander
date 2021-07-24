# dht-to-i2c-expander
A project that uses an ATmega8 to translate I2C to 16 DHT22 sensors.

We have many DHT22 sensors and didn't want to/couldn't connect them all to a Raspberry Pi due to lack of pins and lack of a robust implementation (we bitbanged it from userspace, which was meh).
We therefore created a small breakout board with an ATmega8(L) on it to read the sensors and make readings available via I2C.
Readouts are triggered via I2C by the bus master through setting bit `0x01` in the status register at `0x00`.

Most of this stuff, including the resulting format and whatnot, is explained in `main.c`, so check that out.

## License

MIT, but would be nice if you would link back here.
