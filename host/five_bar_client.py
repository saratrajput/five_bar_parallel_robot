#!/usr/bin/env python3
"""Interactive serial client for the five-bar IK firmware.

Commands at the REPL:
    move <x> <y>   send M command, end-effector to (x, y) mm
    home           park at home pose
    status         read last commanded servo angles
    raw <line>     send an arbitrary line verbatim
    quit           exit
"""

import argparse
import sys
import time

import serial


def wait_for_ready(ser, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        print(f"< {line}")
        if line.startswith("READY"):
            return True
    return False


def send(ser, line):
    ser.write((line + "\n").encode())
    ser.flush()
    reply = ser.readline().decode(errors="replace").strip()
    print(f"< {reply}" if reply else "< (no reply)")


def repl(ser):
    print("Type '?' for firmware help, 'quit' to exit.")
    while True:
        try:
            raw = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            send(ser, "R")
            return
        if not raw:
            continue
        parts = raw.split()
        cmd = parts[0].lower()
        if cmd in ("quit", "exit", "q"):
            send(ser, "R")
            return
        if cmd == "move" and len(parts) == 3:
            send(ser, f"M {parts[1]} {parts[2]}")
        elif cmd == "home":
            send(ser, "H")
        elif cmd == "status":
            send(ser, "S")
        elif cmd == "release":
            send(ser, "R")
        elif cmd == "raw" and len(parts) >= 2:
            send(ser, raw[len("raw "):])
        elif cmd == "?":
            send(ser, "?")
        else:
            print("usage: move <x> <y> | home | status | release | raw <line> | ? | quit")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=1.0) as ser:
        # Nano resets on DTR; give the bootloader a moment then drain the banner.
        time.sleep(2.0)
        wait_for_ready(ser)
        repl(ser)


if __name__ == "__main__":
    main()
