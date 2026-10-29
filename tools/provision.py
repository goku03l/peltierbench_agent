#!/usr/bin/env python3
"""Store WiFi credentials in the ESP32's flash over USB. The password is typed at a hidden prompt, never echoed or saved
in any file.   Usage:  python tools/provision.py COM7 [--ssid "My Network"]
"""
import argparse, getpass, time
import serial

ap = argparse.ArgumentParser(); ap.add_argument("port"); ap.add_argument("--ssid"); a = ap.parse_args()
ssid = a.ssid or input("WiFi name (2.4 GHz): ").strip()
pw = getpass.getpass("WiFi password (hidden): ")
if ";" in ssid or ";" in pw:
    raise SystemExit("';' is the separator in the cfg command, so it cannot be used in the name or password")
s = serial.Serial(); s.port = a.port; s.baudrate = 115200; s.timeout = 0.5; s.dtr = False; s.rts = False; s.open()
s.reset_input_buffer(); s.write(("cfg wifi %s;%s\n" % (ssid, pw)).encode()); time.sleep(1.0)
print([l.decode("utf-8", "replace").strip() for l in s.readlines() if l.startswith(b"#")][-2:])
s.write(b"reboot\n"); s.close()
print("Saved. The board is rebooting and will join '%s'." % ssid)
