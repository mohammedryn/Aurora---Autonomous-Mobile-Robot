#!/usr/bin/env python3
import curses, serial, argparse, time

KEY_TO_CMD = {
    curses.KEY_UP:    b'f',
    curses.KEY_DOWN:  b'b',
    curses.KEY_LEFT:  b'l',
    curses.KEY_RIGHT: b'r',
    ord(' '):         b's',
}

LABELS = {
    b'f': 'FORWARD',
    b'b': 'BACKWARD',
    b'l': 'STRAFE LEFT',
    b'r': 'STRAFE RIGHT',
    b's': 'STOP',
}

def main(stdscr, port, baud):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    ser = serial.Serial(port, baud, timeout=0)
    cmd = b's'
    last_sent = b''

    try:
        while True:
            key = stdscr.getch()

            if key in (27, 3):
                break
            elif key in KEY_TO_CMD:
                cmd = KEY_TO_CMD[key]
            else:
                cmd = b's'

            if cmd != last_sent:
                ser.write(cmd)
                last_sent = cmd

            stdscr.erase()
            stdscr.addstr(0, 0, f"AMR Teleop  |  {port}  {baud} baud")
            stdscr.addstr(1, 0, "-" * 44)
            stdscr.addstr(2, 0, "Arrows = drive   Space = stop   Esc = quit")
            stdscr.addstr(4, 0, f"Command:  {LABELS.get(cmd, '?')}")
            stdscr.refresh()

            time.sleep(0.05)

    finally:
        try:
            ser.write(b's')
        except Exception:
            pass
        ser.close()

def run():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/amr_mcu')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()
    curses.wrapper(main, args.port, args.baud)

if __name__ == '__main__':
    run()
