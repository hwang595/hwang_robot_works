# free drive demo for ur5 robot using wireless button
The bluetooth button we used is Satechi bluetooth button (https://satechi.net/products/satechi-bluetooth-button-series). To run this demo, you need to connect your laptop/computer with the bluetooth button first.

Since multi-media key on the keyboard has no map to a certain code, we need to read the system file on ubuntu for the wireless button. In this code, I used `evdev` in python. To install this library with all it's dependency, you can just simple run the `prepare.sh` in this repo.

It's not good, but I haven't figured out how to fix it yet that you need to run `env_dev_test.py` with root access in this repo first to fetch the `event_[id]` of the wireless button in the system. The output is something like:
```
('/dev/input/event4', 'Logitech USB Optical Mouse', 'usb-0000:00:14.0-1/input0')
('/dev/input/event17', 'HDA Intel PCH HDMI/DP,pcm=8', 'ALSA')
('/dev/input/event16', 'HDA Intel PCH HDMI/DP,pcm=7', 'ALSA')
('/dev/input/event15', 'HDA Intel PCH HDMI/DP,pcm=3', 'ALSA')
('/dev/input/event14', 'HDA Intel PCH Headphone', 'ALSA')
('/dev/input/event13', 'HDA Intel PCH Mic', 'ALSA')
('/dev/input/event12', 'HP WMI hotkeys', 'wmi/input0')
('/dev/input/event11', 'Video Bus', 'LNXVIDEO/video/input0')
('/dev/input/event10', 'Video Bus', 'LNXVIDEO/video/input0')
('/dev/input/event9', 'HP Wide Vision HD', 'usb-0000:00:14.0-4/button')
('/dev/input/event8', 'ELAN Touchscreen', 'usb-0000:00:14.0-8/input0')
('/dev/input/event7', 'ST LIS3LV02DL Accelerometer', 'lis3lv02d/input0')
('/dev/input/event6', 'HP Wireless hotkeys', 'hpq6001/input0')
('/dev/input/event18', 'Satechi Wireless button', 'stcw6001/input0')
('/dev/input/event5', 'SynPS/2 Synaptics TouchPad', 'isa0060/serio1/input0')
('/dev/input/event3', 'AT Translated Set 2 keyboard', 'isa0060/serio0/input0')
('/dev/input/event2', 'Power Button', 'LNXPWRBN/button/input0')
('/dev/input/event1', 'Power Button', 'PNP0C0C/button/input0')
('/dev/input/event0', 'Lid Switch', 'PNP0C0D/button/input0')
```
Find the `Satechi Wireless button` from it, and change this line of code in `freeDrive_demo_tmp.py`
```
device = evdev.InputDevice('/dev/input/event4')
```
