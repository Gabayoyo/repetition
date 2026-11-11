"""Minimal arm capture loop: camera -> landmarks -> arm chain -> packet."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Callable

from pi.protocol import ArmPacket
from vision import arm_pose
from vision.arm_chain import ArmChain
from vision.oak import DEFAULT_FPS, DEFAULT_HEIGHT, DEFAULT_WIDTH, OakCapture
from vision.smoothing import ArmSmoother
from vision.tracker import HolisticTracker
from vision.visualisation import Visualisation


def _packet_from_chain(
    chain: ArmChain,
    hand: arm_pose.HandTracker,
    *,
    seq: int,
    t: float,
) -> ArmPacket:
    """Convert the measured chain and hand state into wire-ready values."""
    return ArmPacket(
        seq=seq,
        t=t,
        shoulder=tuple(float(value) for value in chain.shoulder),
        elbow=tuple(float(value) for value in chain.elbow),
        wrist=tuple(float(value) for value in chain.wrist),
        quat=tuple(float(value) for value in hand.quat),
        gripper_open=hand.gripper_open,
        links=(chain.upper, chain.forearm),
    )


def _print_packet(packet: ArmPacket) -> None:
    print(packet.to_json(), flush=True)


def run(
    capture,
    tracker,
    *,
    visualiser: Visualisation | None = None,
    smoother: ArmSmoother | None = None,
    print_packets: bool = False,
    send_packet: Callable[[ArmPacket], None] | None = None,
) -> int:
    """Track one arm until the capture ends or the user quits."""
    hand = arm_pose.HandTracker()
    seq = 0

    try:
        while True:
            frame = capture.read()
            if frame is None:
                print("no frame from the camera, stopping", file=sys.stderr)
                break

            result = tracker.process(frame)
            pose_world = result.pose_world_landmarks or None
            hand_world = (
                result.right_hand_world_landmarks[0]
                if result.right_hand_world_landmarks
                else None
            )

            chain = arm_pose.measured_arm(pose_world)
            if chain is not None and smoother is not None:
                chain = smoother.update_chain(chain, frame.t_s)

            hand_seen = hand.update(hand_world) if hand_world is not None else False
            if hand_seen and smoother is not None:
                hand.quat = smoother.update_quat(hand.quat, frame.t_s)

            if chain is not None:
                packet = _packet_from_chain(chain, hand, seq=seq, t=frame.t_s)
                seq += 1
                if print_packets:
                    _print_packet(packet)
                if send_packet is not None:
                    send_packet(packet)

            if visualiser is not None:
                key = visualiser.show(
                    frame.bgr, result, chain, hand.quat, hand.gripper_open
                )
                if key in (27, ord("q")):
                    break
    except KeyboardInterrupt:
        pass

    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--visualise",
        "--visualize",
        action="store_true",
        help="show the camera overlay window",
    )
    parser.add_argument(
        "--packets",
        action="store_true",
        help="write one JSON packet per tracked frame to stdout",
    )
    parser.add_argument(
        "--no-smoothing",
        action="store_true",
        help="send the raw measured chain instead of the filtered chain",
    )
    args = parser.parse_args(argv)

    capture = None
    tracker = None
    visualiser = None

    try:
        capture = OakCapture(DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_FPS)
        tracker = HolisticTracker()
        smoother = None if args.no_smoothing else ArmSmoother()
        visualiser = Visualisation() if args.visualise else None
        return run(
            capture,
            tracker,
            visualiser=visualiser,
            smoother=smoother,
            print_packets=args.packets,
        )
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"startup failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if tracker is not None:
            tracker.close()
        if capture is not None:
            capture.close()
        if visualiser is not None:
            visualiser.close()


if __name__ == "__main__":
    raise SystemExit(main())
