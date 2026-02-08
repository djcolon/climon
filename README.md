# cliMon

**Daniel Colon - 23/03/2025**

An ESP32 that reads temperature, humidity and pressure using a che BME-280 clone.
Connects to wifi and makes sensor data available as JSON on a http call.

Built with an ESP32-C3 Mini, which I bought for £1.83
[here.](https://www.aliexpress.com/item/1005005967641936.html)

And a BME-280-3.3 which is £2.69
[here.](https://www.aliexpress.com/item/1005006221391170.html)

[Built using PlatformIO](https://platformio.org/)

## Testing

To run tests you need to have configured the platformIO native environment.
See the [platformIO guides](https://piolabs.com/blog/insights/unit-testing-part-1.html)
for more detail.

After installation, run tests by navigating to `native > advanced > test` in the
platformIO sidebar.

## Images

The devboard pinout.

![The devboard pinout](images/esp.png?raw=true "The devboard pinout")
