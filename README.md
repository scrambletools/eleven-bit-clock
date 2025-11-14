# 11-bit Clock example implementation

This code implements a clock that displays time on
an 11-pixel LED strip. The time is represented in
binary with 1 bit for AM/PM, 4 bits for hours (1-12),
and 6 bits for minutes (0-59). Time is obtained from
an NTP server which can be configured via web interface.
Other config options allow color customization with
multiple presets and a clock password to prevent
unauthorized access.

This code also implements a wifi station and fallback
AP mode with captive portal to configure the wifi
station login credentials. If the device is unable to
connect to wifi after multiple tries, it will fall back
to AP mode and flash the leds to indicate this mode.

It also implements use of the ESP ADC for touch input.
When the user touches the clock (some metal part connected
to the configured GPIO), the clock will display the last
quad of its IP address allowing the user to identify the
device IP to access the web interface.

Compatibility:
This code has been tested on the ESP32-C3 using ESP-IDF v6.1,
but should work on other ESP32 variants as well. It is used
by the Pyramid Clock (<www.pyramidclock.com>).

Build notes:
Make sure to rename secrets_rename.h to secrets.h and
update the file with your appropriate api key(s).
