#!/usr/bin/env python3
"""Dora keyboard teleop for the Feetech Kiwi chassis.

Run without arguments as a Dora node. Run with ``--teleop`` in a second SSH
terminal to send keys through the FIFO.
"""

import argparse
import json
import os
import select
import sys
import termios
import time
import tty


FIFO_PATH = os.environ.get("KEYBOARD_FIFO", "/tmp/feetech_kiwi_keyboard_fifo")
LINEAR_SPEED = float(os.environ.get("KEY_LINEAR_SPEED", "0.04"))
ANGULAR_SPEED = float(os.environ.get("KEY_ANGULAR_SPEED", "0.10"))
COMMAND_TIMEOUT = float(os.environ.get("KEY_COMMAND_TIMEOUT", "0.50"))


def velocity_for(command):
    commands = {
        "w": (LINEAR_SPEED, 0.0, 0.0),
        "s": (-LINEAR_SPEED, 0.0, 0.0),
        "q": (0.0, LINEAR_SPEED, 0.0),
        "e": (0.0, -LINEAR_SPEED, 0.0),
        "a": (0.0, 0.0, ANGULAR_SPEED),
        "d": (0.0, 0.0, -ANGULAR_SPEED),
    }
    return commands.get(command, (0.0, 0.0, 0.0))


def twist_bytes(command):
    x, y, yaw = velocity_for(command)
    return json.dumps(
        {
            "linear": {"x": x, "y": y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": yaw},
        },
        separators=(",", ":"),
    ).encode("utf-8")


def ensure_fifo():
    try:
        os.mkfifo(FIFO_PATH, 0o600)
    except FileExistsError:
        if not os.path.exists(FIFO_PATH):
            raise


def run_teleop():
    if not sys.stdin.isatty():
        raise SystemExit("teleop requires an interactive terminal")
    ensure_fifo()
    print("W/S 前进后退  Q/E 左右横移  A/D 左右旋转")
    print("Space 停车  X 停车并退出")
    print(f"速度: {LINEAR_SPEED:.3f} m/s, {ANGULAR_SPEED:.3f} rad/s")

    old_settings = termios.tcgetattr(sys.stdin)
    command = " "
    writer = None
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if writer is None:
                try:
                    writer = os.open(FIFO_PATH, os.O_WRONLY | os.O_NONBLOCK)
                    print("已连接 Dora 键盘节点")
                except OSError:
                    print("等待 Dora 建图启动...", end="\r", flush=True)
                    time.sleep(0.2)
                    continue

            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if readable:
                key = sys.stdin.read(1).lower()
                if key == "x" or key == "\x03":
                    command = " "
                    os.write(writer, b"stop\n")
                    break
                if key in "wsqead":
                    command = key
                elif key == " ":
                    command = " "
                else:
                    continue
                label = "STOP" if command == " " else command.upper()
                print(f"\r当前命令: {label}   ", end="", flush=True)

            try:
                payload = b"stop\n" if command == " " else command.encode() + b"\n"
                os.write(writer, payload)
            except (BrokenPipeError, OSError):
                if writer is not None:
                    os.close(writer)
                writer = None
    finally:
        if writer is not None:
            try:
                os.write(writer, b"stop\n")
            except OSError:
                pass
            os.close(writer)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\n键盘控制已退出，停车命令已发送")


def run_dora_node():
    from dora import Node

    ensure_fifo()
    fifo = os.open(FIFO_PATH, os.O_RDWR | os.O_NONBLOCK)
    node = Node()
    latest = "stop"
    received_at = 0.0
    buffered = b""
    print(f"keyboard FIFO: {FIFO_PATH}", flush=True)

    try:
        while True:
            event = node.next()
            event_type = event.get("type")
            if event_type == "STOP":
                break
            if event_type != "INPUT":
                continue

            try:
                chunk = os.read(fifo, 4096)
            except BlockingIOError:
                chunk = b""
            if chunk:
                buffered += chunk
                lines = buffered.split(b"\n")
                buffered = lines.pop()
                for line in lines:
                    candidate = line.decode("ascii", errors="ignore").strip().lower()
                    if candidate in {"w", "s", "q", "e", "a", "d", "stop"}:
                        latest = candidate
                        received_at = time.monotonic()

            command = latest
            if time.monotonic() - received_at > COMMAND_TIMEOUT:
                command = "stop"
            node.send_output("CmdVelTwist", twist_bytes(command), event.get("metadata", {}))
    finally:
        try:
            node.send_output("CmdVelTwist", twist_bytes("stop"), {})
        except Exception:
            pass
        os.close(fifo)


def main():
    parser = argparse.ArgumentParser(description="Feetech Kiwi Dora keyboard control")
    parser.add_argument("--teleop", action="store_true", help="read keys in this terminal")
    args = parser.parse_args()
    if args.teleop:
        run_teleop()
    else:
        run_dora_node()


if __name__ == "__main__":
    main()
