# Lilygo T5 4.7 inch - lilygot54in7

A GFX enabled device driver for the Lilygo T5 4.7 inch smart device

[env:esp32-lilygot54in7]
platform = espressif32
board = node32s
framework = arduino
lib_deps = 
	codewitch-honey-crisis/htcw_lilygot54in7
lib_ldf_mode = deep
build_unflags=-std=gnu++11
build_flags=-std=gnu++14
	-DBOARD_HAS_PSRAM
	-mfix-esp32-psram-cache-issue
```
Note: This driver is finicky due to the display hardware. Do not try to do partial updates too rapidly. Also you can use auto_clear() to turn off and on automatic clearing of the area behind the partial update. If you try to do too many partial updates per frame, the non-partial portions will begin to fade. To do a full update simply invalidate the entire screen through drawing to its extents. Also make sure PSRAM is enabled.