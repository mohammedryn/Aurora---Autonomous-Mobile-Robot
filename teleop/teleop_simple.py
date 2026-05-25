import curses, serial, argparse, time

KEY_TO_CMD = {
    curses.KEY_UP:    b'f',
    curses.KEY_DOWN:  b'b',
    curses.KEY_LEFT:  b'l',
    curses.KEY_RIGHT: b'r',
    ord('q'):         b'q',
    ord('Q'):         b'q',
    ord('e'):         b'e',
    ord('E'):         b'e',
    ord('z'):         b'z',
    ord('Z'):         b'z',
    ord('x'):         b'x',
    ord('X'):         b'x',
    ord(' '):         b's',
}

LABELS = {
    b'f': 'FORWARD',
    b'b': 'BACKWARD',
    b'l': 'STRAFE LEFT',
    b'r': 'STRAFE RIGHT',
    b'q': 'FWD-LEFT',
    b'e': 'FWD-RIGHT',
    b'z': 'BWD-LEFT',
    b'x': 'BWD-RIGHT',
    b's': 'STOP',
}

def main(stdscr, port, baud):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    ser = serial.Serial(port, baud, timeout=0)
    cmd = b's'

    try:
        while True:
            key = stdscr.getch()

            if key in (27, 3):
                break
            elif key in KEY_TO_CMD:
                cmd = KEY_TO_CMD[key]
                ser.write(cmd)

            stdscr.erase()
            stdscr.addstr(0, 0, "AMR Teleop")
            stdscr.addstr(1, 0, "-" * 40)
            stdscr.addstr(2, 0, "  Q  UP   E      FWD diagonals: Q / E")
            stdscr.addstr(3, 0, " LT      RT      BWD diagonals: Z / X")
            stdscr.addstr(4, 0, "  Z  DN   X      STOP: Space")
            stdscr.addstr(5, 0, "-" * 40)
            stdscr.addstr(6, 0, f"Command:  {LABELS.get(cmd, '?')}")
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