from __future__ import annotations

from raspberry_pi.hardware.gimbal import GimbalConfig, GimbalHardware


def main() -> None:
    hw = GimbalHardware(GimbalConfig())

    print("Enter: <pan> <tilt>, for example: 0 0 or 15 -10")
    print("Type q to quit.")
    print("Hardware range: pan=[-135, 135], tilt=[-90, 90]")

    try:
        while True:
            raw = input("pan tilt > ").strip()
            if raw.lower() in {"q", "quit", "exit"}:
                break
            parts = raw.split()
            if len(parts) != 2:
                print("Expected exactly two numbers: <pan> <tilt>")
                continue
            try:
                pan = float(parts[0])
                tilt = float(parts[1])
            except ValueError:
                print("Invalid input. Use numbers, for example: 0 15")
                continue

            print(f"[WRITE] pan={pan:+.1f}, tilt={tilt:+.1f}")
            hw.write(pan, tilt)
    finally:
        print("[DONE] stopping PWM and cleaning GPIO")
        hw.cleanup()


if __name__ == "__main__":
    main()
