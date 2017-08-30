# free drive demo for ur5 robot using wireless button
The bluetooth button we used is Satechi bluetooth button (https://satechi.net/products/satechi-bluetooth-button-series). To run this demo, you need to connect your laptop/computer with the bluetooth button first.

Since multi-media key on the keyboard has no map to a certain code, we need to read the system file on ubuntu for the wireless button. In this code, I used `evdev` in python. To install this library with all it's dependency, you can just simple run the `prepare.sh` in this repo.
