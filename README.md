# sc2ds

Steam Controller (2026) to DualSense emulation with audio haptics support

It uses `usbip` protocol to simulate as USB Composite device that open HID + USB Audio Class 1 interfaces which allows to be detected by Proton as DualSense gamepad + speaker/haptics

## Requirements
- Linux with `usbip` module
- `usbip` package (same package name on Arch and Debian-based distros)

## Build
### Install
Arch based distros
```
sudo pacman -S asio spdlog libusb
```
Ubuntu/Debian based distros
```
sudo apt install libasio-dev libspdlog-dev libusb-1.0-0-dev
```
### Compiling
```
cmake -S . -B build
cd build
make -j$(nproc) # like make -j16
```

## Usage
Connect the Steam Controller via USB or turn on with Puck

Close all apps that can use Steam Controller
```
sudo modprobe vhci-hcd
./sc2ds
sudo usbip --tcp-port 53240 attach -r 127.0.0.1 -b 1-1
```
For proper work, it seems it should be configured as an audio device with 4 channels labeled Front Left, Front Right, Rear Left and Rear Right

## Issues
- very unstable since it using `usbip` protocol, so it emulates usb protocol
- sound may crackle occasionally since ^ and for Puck it uses 4000 Hz 8-bit mulaw playback + resampling and high/lowpassing algorithms may be not very good
- spams in dmesg with `Not yet implemented` `vhci_get_frame_number` callback errors, it doesn't seem to affect anything, but I don't think [there's a way to turn it off](https://github.com/torvalds/linux/blob/3b5f4b83c4abc0c9b0a7b9e2b44e816611b7f2ec/drivers/usb/usbip/vhci_hcd.c#L1277)
- code is very bad and random
- tested only in single game (Cronos) and KDE Plasma audio output
- doesn't support hot plugging yet
- doesn't support Windows (yet) because I don't use it and probably `usbip-win` is not ready for it (idk)

## Thanks
- https://github.com/Pixel1011/SteamHapticsPlayer for finding audio haptics feature
- https://github.com/iczero/steam-controller-stuff for exploring the more potential of this feature (which is why it works on the Puck, as it uses 4 kHz, 8-bit u-law)
- https://github.com/yunsmall/usbipdcpp for `usbip` server