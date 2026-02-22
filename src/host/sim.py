"""Simulation utilities for synthetic camera projection.

This module also provides a small closed-loop simulation runner so the host
pipeline can be exercised deterministically without UDP receivers.
"""

from __future__ import annotations

from datetime import datetime
import socket
import threading
import time
from pathlib import Path
import argparse
import json
import sys
from typing import Iterable

import cv2
import numpy as np

from host.geo import CameraParams, GeometryPipeline, Triangulator, create_dummy_calibration
from host.logger import FrameLogger
from host.replay import FrameReplay
from host.receiver import Frame, PairedFrames
from host.rigid import RigidBodyEstimator, WAIST_PATTERN
from host.visualize import TrackingVisualizer
from host.pipeline import TrackingPipeline


def _sample_gaussian_pixel_noise(
    rng: np.random.Generator,
    noise_px: float,
) -> tuple[float, float]:
    """Sample additive pixel noise for future simulation use."""
    if noise_px <= 0.0:
        return 0.0, 0.0
    dx, dy = rng.normal(0.0, noise_px, size=2)
    return float(dx), float(dy)


def verify_frame_log(path) -> dict:
    """Verify the FrameLogger log using FrameReplay utilities.

    Returns a dict with keys:
      - valid (bool)
      - camera_ids (list[str])
      - frame_count (int)
      - frame_counts_by_camera (dict[str, int])
      - error (str, optional) if invalid
    Validation rules (via FrameReplay):
      - replay.header must exist
      - replay.footer must exist
      - camera_ids derived from replay.get_cameras()
      - frame counts per camera from replay.get_frames_by_camera(cid)
      - frame_count derives from replay.frame_count
      - If footer.total_frames is present, it must equal frame_count
    """
    res: dict = {
        "valid": False,
        "camera_ids": [],
        "frame_count": 0,
        "frame_counts_by_camera": {},
    }
    try:
        replay = FrameReplay(str(path))

        # Basic structural checks
        if getattr(replay, "header", None) is None:
            res["error"] = "missing header in replay"
            return res
        if getattr(replay, "footer", None) is None:
            res["error"] = "missing footer in replay"
            return res

        # Cameras and per-camera frame counts
        camera_ids = sorted(replay.get_cameras()) if hasattr(replay, "get_cameras") else []
        res["camera_ids"] = [str(cid) for cid in camera_ids]
        frame_counts_by_camera: dict[str, int] = {}
        for cid in camera_ids:
            frames = replay.get_frames_by_camera(cid) if hasattr(replay, "get_frames_by_camera") else []
            frame_counts_by_camera[str(cid)] = int(len(frames))
        res["frame_counts_by_camera"] = frame_counts_by_camera

        # Total frame count and validation against footer
        frame_count = int(getattr(replay, "frame_count", 0))
        res["frame_count"] = frame_count
        footer_total = getattr(replay.footer, "total_frames", None) if hasattr(replay, "footer") else None
        if footer_total is not None:
            try:
                if int(footer_total) != frame_count:
                    res["error"] = (
                        f"footer total_frames {int(footer_total)} != counted {frame_count}"
                    )
                    return res
            except Exception:
                # If total_frames is not an int-able value, treat as mismatch
                res["error"] = f"invalid footer total_frames value: {footer_total}"
                return res

        res["valid"] = True
        return res
    except Exception as e:
        res["error"] = f"unexpected error: {e}"
        return res


class VirtualCamera:
    """Project world points to 2D image points using OpenCV conventions."""

    def __init__(self, camera_params: CameraParams):
        self.camera_params = camera_params
        self._rotation = np.asarray(camera_params.rotation, dtype=np.float64)
        self._translation = np.asarray(camera_params.translation, dtype=np.float64).reshape(3)
        self._rvec, _ = cv2.Rodrigues(self._rotation)

    def project_points(self, points_world: Iterable[Iterable[float]]) -> list[tuple[float, float] | None]:
        points = np.asarray(points_world, dtype=np.float64)

        if points.size == 0:
            return []
        if points.ndim == 1:
            if points.shape[0] != 3:
                raise ValueError("points_world must contain 3D points")
            points = points.reshape(1, 3)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points_world must be shape (N, 3)")

        x_cam = (self._rotation @ points.T).T + self._translation
        valid_mask = x_cam[:, 2] > 0.0

        results: list[tuple[float, float] | None] = [None] * points.shape[0]
        if not np.any(valid_mask):
            return results

        projected, _ = cv2.projectPoints(
            points[valid_mask],
            self._rvec,
            self._translation,
            self.camera_params.intrinsic_matrix,
            self.camera_params.distortion_coeffs,
        )

        projected_2d = projected.reshape(-1, 2)
        valid_indices = np.flatnonzero(valid_mask)
        for idx, (x, y) in zip(valid_indices, projected_2d):
            results[int(idx)] = (float(x), float(y))

        return results


def _rotation_error_deg(r_pred: np.ndarray, r_gt: np.ndarray) -> float:
    """Rotation error in degrees between two rotation matrices."""
    r_pred = np.asarray(r_pred, dtype=np.float64).reshape(3, 3)
    r_gt = np.asarray(r_gt, dtype=np.float64).reshape(3, 3)
    cos = (np.trace(r_pred @ r_gt.T) - 1.0) / 2.0
    cos = float(np.clip(cos, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos)))


def _rotz(theta_rad: float) -> np.ndarray:
    c = float(np.cos(theta_rad))
    s = float(np.sin(theta_rad))
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


class SimulatedRigidBody:
    """Deterministic waist rigid body trajectory and marker ground truth."""

    def __init__(self, seed: int = 0, trajectory: str = "static", fps: float = 60.0):
        self.seed = int(seed)
        self.trajectory = str(trajectory)
        self.fps = float(fps)
        self._rng = np.random.default_rng(self.seed)

        if self.fps <= 0.0:
            raise ValueError("fps must be > 0")
        if self.trajectory not in {"static", "linear", "circle"}:
            raise ValueError(
                f"Unknown trajectory {self.trajectory!r}; expected one of: static, linear, circle"
            )

        # Small deterministic per-seed offsets so simulations can vary by seed.
        self._base_pos = np.array(
            [
                0.25 + float(self._rng.uniform(-0.03, 0.03)),
                float(self._rng.uniform(-0.03, 0.03)),
                2.0 + float(self._rng.uniform(-0.05, 0.05)),
            ],
            dtype=np.float64,
        )
        self._base_yaw = float(self._rng.uniform(-0.25, 0.25))

    def _pose_components(self, frame_index: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        t_sec = float(frame_index) / self.fps

        if self.trajectory == "static":
            pos = self._base_pos.copy()
            yaw = self._base_yaw
        elif self.trajectory == "linear":
            vx = 0.10  # m/s
            pos = self._base_pos + np.array([vx * t_sec, 0.0, 0.0], dtype=np.float64)
            yaw = self._base_yaw + 0.05 * t_sec
        else:  # self.trajectory == "circle"
            radius = 0.08
            omega = 0.5  # rad/s
            pos = np.array(
                [
                    self._base_pos[0] + radius * np.cos(omega * t_sec),
                    self._base_pos[1] + radius * np.sin(omega * t_sec),
                    self._base_pos[2],
                ],
                dtype=np.float64,
            )
            yaw = self._base_yaw + omega * t_sec

        rot = _rotz(yaw)
        quat_wxyz = np.array([
            np.cos(0.5 * yaw),
            0.0,
            0.0,
            np.sin(0.5 * yaw),
        ], dtype=np.float64)
        return rot, pos, quat_wxyz

    def pose_world(self, frame_index: int) -> dict:
        _, pos, quat_wxyz = self._pose_components(frame_index)
        return {
            "position_m": pos.tolist(),
            "quaternion_wxyz": quat_wxyz.tolist(),
        }

    def marker_positions_world(self, frame_index: int) -> list[list[float]]:
        rot, pos, _ = self._pose_components(frame_index)
        body_pts = np.asarray(WAIST_PATTERN.marker_positions, dtype=np.float64)
        world_pts = (rot @ body_pts.T).T + pos
        return world_pts.tolist()


class SyntheticFrameGenerator:
    def __init__(
        self,
        camera_ids: list[str],
        seed: int = 0,
        fps: float = 60.0,
        noise_px: float = 0.0,
        marker_dropout: float = 0.0,
        camera_dropout: float = 0.0,
        trajectory: str = "static",
    ):
        if fps <= 0.0:
            raise ValueError("fps must be > 0")
        if noise_px < 0.0:
            raise ValueError("noise_px must be >= 0")
        if not (0.0 <= marker_dropout <= 1.0):
            raise ValueError("marker_dropout must be in [0, 1]")
        if not (0.0 <= camera_dropout <= 1.0):
            raise ValueError("camera_dropout must be in [0, 1]")

        self._rng = np.random.default_rng(int(seed))
        self._fps = float(fps)
        self._noise_px = float(noise_px)
        self._marker_dropout = float(marker_dropout)
        self._camera_dropout = float(camera_dropout)
        self._frame_index = 0
        self._dt_us = int(round(1_000_000.0 / self._fps))
        self._camera_ids_sorted = sorted(str(cam_id) for cam_id in camera_ids)

        calibration = create_dummy_calibration(self._camera_ids_sorted)
        self._cameras = {cid: VirtualCamera(calibration[cid]) for cid in self._camera_ids_sorted}
        self._body = SimulatedRigidBody(seed=int(seed), trajectory=trajectory, fps=self._fps)

    def next_paired(self) -> PairedFrames | None:
        frame_index = int(self._frame_index)
        timestamp_us = int(frame_index * self._dt_us)

        present_cam_ids: list[str] = []
        for cam_id in self._camera_ids_sorted:
            if self._camera_dropout > 0.0 and float(self._rng.random()) < self._camera_dropout:
                continue
            present_cam_ids.append(cam_id)

        if len(present_cam_ids) < 2:
            self._frame_index += 1
            return None

        marker_world = np.asarray(self._body.marker_positions_world(frame_index), dtype=np.float64)
        projections: dict[str, list[tuple[float, float] | None]] = {
            cam_id: self._cameras[cam_id].project_points(marker_world)
            for cam_id in present_cam_ids
        }

        keep_marker: list[bool] = []
        for marker_idx in range(marker_world.shape[0]):
            if self._marker_dropout > 0.0 and float(self._rng.random()) < self._marker_dropout:
                keep_marker.append(False)
                continue
            keep_marker.append(all(projections[cam_id][marker_idx] is not None for cam_id in present_cam_ids))

        frames_dict: dict[str, Frame] = {}
        for cam_id in present_cam_ids:
            blobs = []
            for marker_idx, keep in enumerate(keep_marker):
                if not keep:
                    continue
                proj = projections[cam_id][marker_idx]
                if proj is None:
                    continue
                dx, dy = _sample_gaussian_pixel_noise(self._rng, self._noise_px)
                blobs.append(
                    {
                        "x": float(proj[0] + dx),
                        "y": float(proj[1] + dy),
                        "area": 50.0,
                    }
                )
            frames_dict[cam_id] = Frame(
                camera_id=cam_id,
                timestamp=timestamp_us,
                frame_index=frame_index,
                blobs=blobs,
                received_at=0.0,
            )

        paired = PairedFrames(timestamp=timestamp_us, frames=frames_dict, timestamp_range_us=0)
        self._frame_index += 1
        return paired


def run_closed_loop(
    *,
    frames: int,
    fps: float = 60.0,
    noise_px: float = 0.0,
    marker_dropout: float = 0.0,
    camera_dropout: float = 0.0,
    trajectory: str = "static",
    seed: int = 0,
    out_dir: str | None = None,
) -> dict:
    """Run an in-process closed-loop simulation and write artifacts.

    Returns a summary dict with keys exactly:
      mean_position_error_m, mean_rotation_error_deg, drop_rate, frame_log, pose_log, eval_json
    """

    if frames <= 0:
        raise ValueError("frames must be > 0")
    if fps <= 0:
        raise ValueError("fps must be > 0")

    rng = np.random.default_rng(int(seed))
    camera_ids = ["cam0", "cam1"]
    calibration = create_dummy_calibration(camera_ids)
    cameras = {cid: VirtualCamera(calibration[cid]) for cid in camera_ids}

    geo = GeometryPipeline()
    geo.camera_params = calibration
    geo.triangulator = Triangulator(calibration)

    # Use a larger marker_diameter for simulation to ensure waist markers cluster
    # into a single cluster in the DBSCAN-based estimator. This avoids the
    # pathological drop_rate=1.0 and NaN metrics observed in the static/no-noise
    # scenario due to overly small eps in clustering.
    rigid = RigidBodyEstimator(patterns=[WAIST_PATTERN], marker_diameter=0.2)

    if out_dir is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = Path("./output/sim") / f"{ts}-{seed}"
    else:
        out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    frame_logger = FrameLogger(log_dir=str(out_path))
    frame_log = frame_logger.start_recording(session_name="frames")

    visualizer = TrackingVisualizer(output_dir=str(out_path), enable_console=False, enable_file=True)
    visualizer.start_session(session_name="sim")
    pose_log = str(out_path / "sim_poses.jsonl")

    body = SimulatedRigidBody(seed=seed, trajectory=trajectory, fps=fps)

    dt_us = int(round(1_000_000.0 / float(fps)))
    start_ts_us = 0

    pos_errors: list[float] = []
    rot_errors: list[float] = []
    drop_count = 0

    for i in range(int(frames)):
        timestamp_us = int(start_ts_us + i * dt_us)
        gt_r, gt_t, _ = body._pose_components(i)
        gt_markers_world = np.asarray(body.marker_positions_world(i), dtype=np.float64)

        present_cam_ids: list[str] = []
        projections: dict[str, list[tuple[float, float] | None]] = {}
        for cam_id in camera_ids:
            if camera_dropout > 0.0 and float(rng.random()) < float(camera_dropout):
                continue
            present_cam_ids.append(cam_id)
            projections[cam_id] = cameras[cam_id].project_points(gt_markers_world)

        # Shared per-frame marker drop mask to preserve blob correspondence assumptions.
        keep_marker = []
        for m in range(gt_markers_world.shape[0]):
            if marker_dropout > 0.0 and float(rng.random()) < float(marker_dropout):
                keep_marker.append(False)
                continue
            ok_all = True
            for cam_id in present_cam_ids:
                if projections[cam_id][m] is None:
                    ok_all = False
                    break
            keep_marker.append(ok_all)

        frames_dict: dict[str, Frame] = {}
        for cam_id in sorted(present_cam_ids):
            blobs = []
            for m, keep in enumerate(keep_marker):
                if not keep:
                    continue
                proj = projections[cam_id][m]
                if proj is None:
                    continue
                dx, dy = _sample_gaussian_pixel_noise(rng, float(noise_px))
                blobs.append({
                    "x": float(proj[0] + dx),
                    "y": float(proj[1] + dy),
                    "area": 50.0,
                })

            frame = Frame(
                camera_id=cam_id,
                timestamp=timestamp_us,
                frame_index=i,
                blobs=blobs,
                received_at=0.0,
            )
            frames_dict[cam_id] = frame
            frame_logger.log_frame(frame.to_dict())

        paired = PairedFrames(timestamp=timestamp_us, frames=frames_dict, timestamp_range_us=0)
        geo_out = geo.process_paired_frames(paired)

        pts_list = geo_out.get("points_3d", [])
        if pts_list:
            pts = np.asarray(pts_list, dtype=np.float64).reshape(-1, 3)
        else:
            pts = np.zeros((0, 3), dtype=np.float64)

        poses = rigid.process_points(pts, timestamp_us)
        if pts.size == 0:
            for p in poses.values():
                p.timestamp = timestamp_us

        poses_dict = {name: pose.to_dict() for name, pose in poses.items()}
        visualizer.visualize_poses(poses_dict, timestamp_us, metrics=None)

        waist = poses.get("waist")
        if waist is None or not waist.valid:
            drop_count += 1
            continue

        pos_err = float(np.linalg.norm(np.asarray(waist.position) - gt_t))
        rot_err = _rotation_error_deg(np.asarray(waist.rotation), gt_r)
        pos_errors.append(pos_err)
        rot_errors.append(rot_err)

    frame_logger.stop_recording()
    visualizer.end_session()

    drop_rate = float(drop_count) / float(frames)
    mean_pos = float(np.mean(pos_errors)) if pos_errors else float("nan")
    mean_rot = float(np.mean(rot_errors)) if rot_errors else float("nan")

    eval_path = out_path / "eval.json"
    eval_summary = {
        "mean_position_error_m": mean_pos,
        "mean_rotation_error_deg": mean_rot,
        "max_position_error_m": float(np.max(pos_errors)) if pos_errors else float("nan"),
        "max_rotation_error_deg": float(np.max(rot_errors)) if rot_errors else float("nan"),
        "rms_position_error_m": float(np.sqrt(np.mean(np.square(pos_errors)))) if pos_errors else float("nan"),
        "rms_rotation_error_deg": float(np.sqrt(np.mean(np.square(rot_errors)))) if rot_errors else float("nan"),
        "drop_rate": drop_rate,
        "valid_frames": int(len(pos_errors)),
        "total_frames": int(frames),
        "params": {
            "frames": int(frames),
            "fps": float(fps),
            "noise_px": float(noise_px),
            "marker_dropout": float(marker_dropout),
            "camera_dropout": float(camera_dropout),
            "trajectory": str(trajectory),
            "seed": int(seed),
        },
        "artifacts": {
            "frame_log": str(frame_log),
            "pose_log": str(pose_log),
        },
    }
    eval_path.write_text(json.dumps(eval_summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    return {
        "mean_position_error_m": mean_pos,
        "mean_rotation_error_deg": mean_rot,
        "drop_rate": drop_rate,
        "frame_log": str(frame_log),
        "pose_log": str(pose_log),
        "eval_json": str(eval_path),
    }


def assert_metrics(
    summary: dict,
    *,
    max_mean_position_error_m: float,
    max_mean_rotation_error_deg: float | None = None,
) -> None:
    mean_pos = float(summary.get("mean_position_error_m"))
    if not np.isfinite(mean_pos) or mean_pos > float(max_mean_position_error_m):
        raise AssertionError(
            f"mean_position_error_m={mean_pos} exceeds max_mean_position_error_m={float(max_mean_position_error_m)}"
        )

    if max_mean_rotation_error_deg is None:
        return

    mean_rot = float(summary.get("mean_rotation_error_deg"))
    if not np.isfinite(mean_rot) or mean_rot > float(max_mean_rotation_error_deg):
        raise AssertionError(
            f"mean_rotation_error_deg={mean_rot} exceeds max_mean_rotation_error_deg={float(max_mean_rotation_error_deg)}"
        )


def _parse_realtime_flag(val: str) -> bool:
    v = str(val).lower()
    return v in {"true", "1", "yes", "y"}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m host.sim")
    parser.add_argument("--frames", type=int, required=True, help="Number of frames (>0)")
    parser.add_argument("--fps", type=float, default=60.0, help="Frames per second (>0)")
    parser.add_argument("--noise-px", type=float, default=0.0, help="Pixel noise stddev")
    parser.add_argument("--marker-dropout", type=float, default=0.0, help="Marker dropout probability [0,1]")
    parser.add_argument("--camera-dropout", type=float, default=0.0, help="Camera dropout probability [0,1]")
    parser.add_argument("--mode", type=str, default="inprocess", choices=["inprocess", "udp"], help="Execution mode: inprocess or udp")
    parser.add_argument("--realtime", type=str, default="true", help=" Emit in real-time; 'true'|'false'")
    parser.add_argument(
        "--trajectory",
        type=str,
        default="static",
        help="Trajectory: static|linear|circle",
    )
    parser.add_argument("--seed", type=int, default=0, help="RNG seed")
    parser.add_argument("--out-dir", type=str, default=None, help="Output directory")

    try:
        args = parser.parse_args(argv)
    except SystemExit as e:
        try:
            return int(e.code)
        except Exception:
            return 2

    def _err(msg: str) -> int:
        print(f"error: {msg}", file=sys.stderr)
        return 2

    if args.frames <= 0:
        return _err("--frames must be > 0")
    if args.fps <= 0:
        return _err("--fps must be > 0")
    if not (0.0 <= args.marker_dropout <= 1.0):
        return _err("--marker-dropout must be in [0, 1]")
    if not (0.0 <= args.camera_dropout <= 1.0):
        return _err("--camera-dropout must be in [0, 1]")
    if args.noise_px < 0.0:
        return _err("--noise-px must be >= 0")
    if args.trajectory not in {"static", "linear", "circle"}:
        return _err("--trajectory must be one of: static, linear, circle")

    # Branch on mode: inprocess vs udp
    if getattr(args, "mode", "inprocess") == "udp":
        # UDP loopback mode: emit synthetic frames over UDP for the receiver to process.
        def _udp_emit_loop(frames: int, fps: float, noise_px: float, marker_dropout: float,
                           camera_dropout: float, trajectory: str, seed: int,
                           out_dir: str | None, realtime_flag: bool) -> None:
            camera_ids = ["cam0", "cam1"]
            # Setup dummy calibration and projection utilities
            calibration = create_dummy_calibration(camera_ids)
            cameras = {cid: VirtualCamera(calibration[cid]) for cid in camera_ids}

            geo = GeometryPipeline()
            geo.camera_params = calibration
            geo.triangulator = Triangulator(calibration)

            # Use a larger marker_diameter for better clustering consistency
            rigid = RigidBodyEstimator(patterns=[WAIST_PATTERN], marker_diameter=0.2)

            # UDP socket setup
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            addr = ("127.0.0.1", 5000)

            # World model for synthetic projections
            body = SimulatedRigidBody(seed=seed, trajectory=trajectory, fps=fps)

            dt_us = int(round(1_000_000.0 / float(fps)))
            start_ts_us = 0

            rng = np.random.default_rng(int(seed))

            for i in range(int(frames)):
                timestamp_us = int(start_ts_us + i * dt_us)
                gt_r, gt_t, _ = body._pose_components(i)
                gt_markers_world = np.asarray(body.marker_positions_world(i), dtype=np.float64)

                present_cam_ids = []
                projections: dict[str, list[tuple[float, float] | None]] = {}
                for cam_id in camera_ids:
                    if camera_dropout > 0.0 and float(rng.random()) < float(camera_dropout):
                        continue
                    present_cam_ids.append(cam_id)
                    projections[cam_id] = cameras[cam_id].project_points(gt_markers_world)

                keep_marker: list[bool] = []
                for m in range(gt_markers_world.shape[0]):
                    if marker_dropout > 0.0 and float(rng.random()) < float(marker_dropout):
                        keep_marker.append(False)
                        continue
                    ok_all = True
                    for cam_id in present_cam_ids:
                        if projections[cam_id][m] is None:
                            ok_all = False
                            break
                    keep_marker.append(ok_all)

                for cam_id in sorted(present_cam_ids):
                    blobs = []
                    for m, keep in enumerate(keep_marker):
                        if not keep:
                            continue
                        proj = projections[cam_id][m]
                        if proj is None:
                            continue
                        dx, dy = _sample_gaussian_pixel_noise(rng, float(noise_px))
                        blobs.append({
                            "x": float(proj[0] + dx),
                            "y": float(proj[1] + dy),
                            "area": 50.0,
                        })

                    msg = {
                        "camera_id": cam_id,
                        "timestamp": int(timestamp_us),
                        "frame_index": int(i),
                        "blobs": blobs,
                    }
                    sock.sendto(json.dumps(msg).encode("utf-8"), addr)

                if realtime_flag:
                    time.sleep(1.0 / float(fps))

            sock.close()

        # Run UDP emitter in background once the pipeline is ready to consume frames
        import numpy as _np  # noqa: F401 (use later in closure to satisfy lints)
        realtime_flag = _parse_realtime_flag(args.realtime)
        udp_thread = threading.Thread(
            target=_udp_emit_loop,
            args=(int(args.frames), float(args.fps), float(args.noise_px), float(args.marker_dropout),
                  float(args.camera_dropout), str(args.trajectory), int(args.seed), args.out_dir, realtime_flag),
            daemon=True,
        )

        # Start the tracking pipeline before emitting UDP frames
        pipeline = TrackingPipeline(udp_port=5000, enable_logging=True, log_dir=args.out_dir)
        pipeline.use_dummy_calibration(camera_ids=["cam0", "cam1"])
        pipeline.start(session_name="frame_log")

        # Now start UDP emitter thread
        udp_thread.start()

        # Wait for UDP emission to finish, then stop the pipeline
        udp_thread.join()
        time.sleep(0.2)
        pipeline.stop()

        return 0
    try:
        summary = run_closed_loop(
            frames=int(args.frames),
            fps=float(args.fps),
            noise_px=float(args.noise_px),
            marker_dropout=float(args.marker_dropout),
            camera_dropout=float(args.camera_dropout),
            trajectory=str(args.trajectory),
            seed=int(args.seed),
            out_dir=args.out_dir,
        )
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    for k in [
        "mean_position_error_m",
        "mean_rotation_error_deg",
        "drop_rate",
        "frame_log",
        "pose_log",
        "eval_json",
    ]:
        print(f"{k}: {summary[k]}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
