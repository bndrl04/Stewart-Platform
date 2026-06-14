"""
Xbox controller -> Stewart platform (USB serial).

Reads an Xbox 360/One/Series controller via pygame and streams `rt` commands
to the Pico over USB serial at ~50 Hz.

Mapping
-------
  Left stick X    -> roll  (deg)
  Left stick Y    -> pitch (deg, inverted: stick up = pitch forward)
  Right stick X   -> yaw   (deg)
  LT trigger      -> lower Z (toward Z_MIN)
  RT trigger      -> raise Z (toward Z_MAX)
  D-pad           -> X / Y translation (mm)
  A button        -> home (send `home` once)
  B button        -> quit (returns to home, sends `stream off`, exits)
  Start button    -> emergency stop (sends `stop`)

Requirements
------------
  pip install pygame pyserial
"""

import os
import sys
import time
import argparse

import pygame
import serial

# Enable ANSI escape sequences on legacy Windows cmd.exe (no-op on modern terminals).
if os.name == "nt":
    os.system("")

# ----- Tunables (match Pico-side RT_LIMIT_* in Stewart.c) -----
Z_HOME            = 125.0
Z_MIN             = 115.0
Z_MAX             = 140.0
LIMIT_TRANS_MM    = 10.0
LIMIT_ROLL_PITCH  = 10.0
LIMIT_YAW         = 15.0

STICK_DEADZONE    = 0.12       # ignore tiny stick noise
TRIGGER_DEADZONE  = 0.05
LOOP_HZ           = 50         # send rate

# Xbox controller axis layout on Windows via pygame's default driver:
#   axis 0 = LSX, axis 1 = LSY, axis 2 = LT, axis 3 = RSX, axis 4 = RSY, axis 5 = RT
# Note: triggers are normalized to [-1, +1] by pygame; 0 means halfway.
AXIS_LSX, AXIS_LSY = 0, 1
AXIS_RSX, AXIS_RSY = 3, 4
AXIS_LT,  AXIS_RT  = 2, 5

BTN_A, BTN_B, BTN_X, BTN_Y = 0, 1, 2, 3
BTN_LB, BTN_RB             = 4, 5
BTN_BACK, BTN_START        = 6, 7


def apply_deadzone(v, dz):
    if abs(v) < dz:
        return 0.0
    # rescale so output is 0 at edge of deadzone, +/-1 at full deflection
    s = (abs(v) - dz) / (1.0 - dz)
    return s if v > 0 else -s


def trigger_to_unit(raw):
    """pygame reports triggers in [-1, +1] where -1 = released, +1 = pressed."""
    v = (raw + 1.0) * 0.5  # -> [0, 1]
    return v if v > TRIGGER_DEADZONE else 0.0


def open_serial(port, baud=115200):
    s = serial.Serial(port, baud, timeout=0.1, write_timeout=0.5)
    # Give the Pico a moment to settle / drain any stale stdin.
    time.sleep(0.5)
    s.reset_input_buffer()
    return s


def send(ser, line, verbose=False):
    data = (line + "\n").encode("ascii")
    ser.write(data)
    if verbose:
        print(f">>> {line}")


# ANSI escape to clear from cursor to end of line (works on Win10+ terminals).
_CLR_EOL = "\033[K"


def _emit_status(state, x, y, z, roll, pitch, yaw):
    """Overwrite a single status line in-place (state + current pose)."""
    line = (f"\r{state:<13s} "
            f"x={x:+6.2f} y={y:+6.2f} z={z:6.2f} "
            f"roll={roll:+6.2f} pitch={pitch:+6.2f} yaw={yaw:+6.2f}"
            f"{_CLR_EOL}")
    sys.stdout.write(line)
    sys.stdout.flush()


def _emit_error(msg):
    """Print an error on its own line without disturbing the status overwrite."""
    sys.stdout.write(f"\rError: {msg}{_CLR_EOL}\n")
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description="Xbox -> Stewart platform bridge")
    ap.add_argument("--port", default="COM4", help="Serial port (default COM4)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--verbose", action="store_true", help="print every command")
    args = ap.parse_args()

    print("Initializing... opening serial")
    try:
        ser = open_serial(args.port, args.baud)
    except Exception as e:
        _emit_error(f"cannot open {args.port}: {e}")
        sys.exit(1)

    print("Initializing... starting pygame / detecting controller")
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        _emit_error("no joystick/controller detected. Plug in the Xbox controller and retry.")
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Initializing... controller = {js.get_name()} "
          f"({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)")

    # Reset Pico to a known state and enter stream mode.
    print("Initializing... calibrating Stewart (home + stream on)")
    try:
        send(ser, "release")
        send(ser, "home")
        time.sleep(0.5)
        send(ser, "stream on")
        time.sleep(0.1)
    except Exception as e:
        _emit_error(f"serial write failed during init: {e}")
        sys.exit(1)

    print("Bindings: LStick=roll/pitch  RStick X=yaw  LT/RT=z  D-pad=x/y  "
          "A=home  Start=stop  B=quit\n")

    period = 1.0 / LOOP_HZ
    # State tracking: 'Moving...' while sticks are deflected, 'Ready' when at rest.
    last_state = None
    last_pose = (0.0, 0.0, Z_HOME, 0.0, 0.0, 0.0)
    idle_since = time.perf_counter()
    IDLE_HOLD_S = 0.20  # require this much stillness before declaring Ready

    try:
        while True:
            loop_start = time.perf_counter()
            pygame.event.pump()

            # Read axes
            lsx = apply_deadzone(js.get_axis(AXIS_LSX), STICK_DEADZONE)
            lsy = apply_deadzone(js.get_axis(AXIS_LSY), STICK_DEADZONE)
            rsx = apply_deadzone(js.get_axis(AXIS_RSX), STICK_DEADZONE)
            lt  = trigger_to_unit(js.get_axis(AXIS_LT))
            rt  = trigger_to_unit(js.get_axis(AXIS_RT))

            # D-pad (hat) for X/Y translation
            hat_x, hat_y = (0, 0)
            if js.get_numhats() > 0:
                hat_x, hat_y = js.get_hat(0)

            # Buttons
            btn_a     = js.get_button(BTN_A)
            btn_b     = js.get_button(BTN_B)
            btn_start = js.get_button(BTN_START)

            # Map to pose
            roll  = LIMIT_ROLL_PITCH * lsx
            pitch = LIMIT_ROLL_PITCH * (-lsy)   # invert: stick forward = pitch nose-down
            yaw   = LIMIT_YAW        * rsx
            z = Z_HOME + (rt * (Z_MAX - Z_HOME)) - (lt * (Z_HOME - Z_MIN))
            x = LIMIT_TRANS_MM * hat_x
            y = LIMIT_TRANS_MM * hat_y

            # Button actions
            if btn_a:
                try:
                    send(ser, "home", args.verbose)
                except Exception as e:
                    _emit_error(f"serial write failed: {e}")
                    break
                # Force a Ready redraw after home settles.
                sys.stdout.write("\n")
                last_state = None
                time.sleep(0.3)
                continue
            if btn_start:
                try:
                    send(ser, "stop", args.verbose)
                except Exception as e:
                    _emit_error(f"serial write failed: {e}")
                    break
                time.sleep(0.1)
                continue
            if btn_b:
                raise KeyboardInterrupt

            # Detect whether any input is active (movement) or all at rest.
            input_active = (lsx or lsy or rsx or lt or rt or hat_x or hat_y)
            now = time.perf_counter()
            if input_active:
                idle_since = now
                state = "Moving..."
            else:
                state = "Ready" if (now - idle_since) >= IDLE_HOLD_S else "Moving..."

            # Stream the pose to the Pico
            cmd = f"rt {x:.2f} {y:.2f} {z:.2f} {roll:.2f} {pitch:.2f} {yaw:.2f}"
            try:
                send(ser, cmd, args.verbose)
            except Exception as e:
                _emit_error(f"serial write failed: {e}")
                break

            # Detect Pico-side rejection (REJECT lines come back on the same port)
            try:
                pending = ser.in_waiting
            except Exception:
                pending = 0
            if pending:
                try:
                    chunk = ser.read(pending).decode("ascii", errors="replace")
                    for ln in chunk.splitlines():
                        ln = ln.strip()
                        if ln.startswith("REJECT") or ln.startswith("ERR") or ln.startswith("FATAL"):
                            _emit_error(ln)
                        elif ln.startswith("WARN"):
                            _emit_error(ln)
                except Exception:
                    pass

            # Update status line (only when state or pose changes meaningfully)
            pose = (x, y, z, roll, pitch, yaw)
            pose_changed = any(abs(a - b) > 0.05 for a, b in zip(pose, last_pose))
            if state != last_state or pose_changed:
                _emit_status(state, x, y, z, roll, pitch, yaw)
                last_state = state
                last_pose = pose

            # Pace the loop
            elapsed = time.perf_counter() - loop_start
            sleep_left = period - elapsed
            if sleep_left > 0:
                time.sleep(sleep_left)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        try:
            send(ser, "stream off")
            time.sleep(0.1)
            send(ser, "home")
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()
