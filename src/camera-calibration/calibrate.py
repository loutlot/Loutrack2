#!/usr/bin/env python3
"""
Camera intrinsic calibration using Charuco boards.

Supports:
- Live capture from Pi Camera (requires picamera2)
- Offline calibration from images (--input-dir)
- Hardware-free self-test mode (--self-test)
"""
from __future__ import annotations


import argparse
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, cast

import cv2
import numpy as np


DEFAULT_SQUARES_X = 6
DEFAULT_SQUARES_Y = 8
DEFAULT_DICTIONARY = "DICT_6X6_250"
DEFAULT_CAMERA = "pi-cam-01"
DEFAULT_SELF_TEST_VIEWS = 30
DEFAULT_MIN_FRAMES = 25
DEFAULT_SEED = 0
SYNTHETIC_IMAGE_SIZE = (2304, 1296)  # width, height


@dataclass
class CalibrateConfig:
    """Configuration for calibration."""
    camera: str
    output: Optional[str]
    square_length_mm: float
    marker_length_mm: float
    squares_x: int
    squares_y: int
    dictionary: str
    input_dir: Optional[str]
    self_test: bool
    self_test_views: int
    seed: int
    min_frames: int


def parse_args() -> CalibrateConfig:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Camera intrinsic calibration using Charuco boards"
    )
    _ = parser.add_argument(
        "--camera",
        "--camera-id",
        dest="camera",
        default=DEFAULT_CAMERA,
        help=f"Camera identifier (default: {DEFAULT_CAMERA})",
    )
    _ = parser.add_argument(
        "--output",
        default=None,
        help="Output JSON path (default: calibration/calibration_intrinsics_v1_{camera}.json)",
    )
    _ = parser.add_argument(
        "--square-length-mm",
        type=float,
        required=True,
        help="Square side length in millimeters (required)",
    )
    _ = parser.add_argument(
        "--marker-length-mm",
        type=float,
        default=None,
        help="Marker side length in millimeters (default: square_length_mm * 20/30)",
    )
    _ = parser.add_argument(
        "--squares-x",
        type=int,
        default=DEFAULT_SQUARES_X,
        help=f"Number of squares in X direction (default: {DEFAULT_SQUARES_X})",
    )
    _ = parser.add_argument(
        "--squares-y",
        type=int,
        default=DEFAULT_SQUARES_Y,
        help=f"Number of squares in Y direction (default: {DEFAULT_SQUARES_Y})",
    )
    _ = parser.add_argument(
        "--dictionary",
        default=DEFAULT_DICTIONARY,
        help=f"OpenCV ArUco dictionary name (default: {DEFAULT_DICTIONARY})",
    )
    _ = parser.add_argument(
        "--input-dir",
        default=None,
        help="Directory containing calibration images (offline mode)",
    )
    _ = parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run hardware-free self-test with synthetic data",
    )
    _ = parser.add_argument(
        "--self-test-views",
        type=int,
        default=DEFAULT_SELF_TEST_VIEWS,
        help=f"Number of synthetic views for self-test (default: {DEFAULT_SELF_TEST_VIEWS})",
    )
    _ = parser.add_argument(
        "--seed",
        type=int,
        default=DEFAULT_SEED,
        help=f"Random seed for self-test (default: {DEFAULT_SEED})",
    )
    _ = parser.add_argument(
        "--min-frames",
        type=int,
        default=DEFAULT_MIN_FRAMES,
        help=f"Minimum valid frames required (default: {DEFAULT_MIN_FRAMES})",
    )
    
    namespace = parser.parse_args()
    
    # Extract values with getattr for safety
    camera = getattr(namespace, "camera", DEFAULT_CAMERA)
    output = getattr(namespace, "output", None)
    square_length_mm = getattr(namespace, "square_length_mm", None)
    marker_length_mm = getattr(namespace, "marker_length_mm", None)
    squares_x = getattr(namespace, "squares_x", DEFAULT_SQUARES_X)
    squares_y = getattr(namespace, "squares_y", DEFAULT_SQUARES_Y)
    dictionary = getattr(namespace, "dictionary", DEFAULT_DICTIONARY)
    input_dir = getattr(namespace, "input_dir", None)
    self_test = getattr(namespace, "self_test", False)
    self_test_views = getattr(namespace, "self_test_views", DEFAULT_SELF_TEST_VIEWS)
    seed = getattr(namespace, "seed", DEFAULT_SEED)
    min_frames = getattr(namespace, "min_frames", DEFAULT_MIN_FRAMES)
    
    # Post-parse validation
    if not isinstance(camera, str) or not camera:
        raise SystemExit("--camera must be a non-empty string")
    if square_length_mm is None:
        raise SystemExit("--square-length-mm is required")
    if not isinstance(square_length_mm, (int, float)) or square_length_mm <= 0:
        raise SystemExit("--square-length-mm must be a positive number")
    if marker_length_mm is not None:
        if not isinstance(marker_length_mm, (int, float)) or marker_length_mm <= 0:
            raise SystemExit("--marker-length-mm must be a positive number")
        if marker_length_mm >= square_length_mm:
            raise SystemExit("--marker-length-mm must be smaller than --square-length-mm")
    if not isinstance(squares_x, int) or squares_x < 2:
        raise SystemExit("--squares-x must be an integer >= 2")
    if not isinstance(squares_y, int) or squares_y < 2:
        raise SystemExit("--squares-y must be an integer >= 2")
    if not isinstance(dictionary, str):
        raise SystemExit("--dictionary must be a string")
    if input_dir is not None:
        if not isinstance(input_dir, str):
            raise SystemExit("--input-dir must be a string")
        if not Path(input_dir).is_dir():
            raise SystemExit(f"--input-dir '{input_dir}' is not a valid directory")
    if not isinstance(self_test, bool):
        raise SystemExit("--self-test must be a boolean flag")
    if not isinstance(self_test_views, int) or self_test_views < 1:
        raise SystemExit("--self-test-views must be a positive integer")
    if not isinstance(seed, int):
        raise SystemExit("--seed must be an integer")
    if not isinstance(min_frames, int) or min_frames < 1:
        raise SystemExit("--min-frames must be a positive integer")
    
    # Compute default marker length if not provided
    if marker_length_mm is None:
        marker_length_mm = square_length_mm * (20.0 / 30.0)
    
    return CalibrateConfig(
        camera=camera,
        output=output,
        square_length_mm=float(square_length_mm),
        marker_length_mm=float(marker_length_mm),
        squares_x=squares_x,
        squares_y=squares_y,
        dictionary=dictionary,
        input_dir=input_dir,
        self_test=self_test,
        self_test_views=self_test_views,
        seed=seed,
        min_frames=min_frames,
    )


def get_dictionary(dictionary_name: str) -> Any:
    """
    Get OpenCV ArUco dictionary by name.
    
    Args:
        dictionary_name: Name of the dictionary (e.g., "DICT_6X6_250")
        
    Returns:
        OpenCV dictionary object
        
    Raises:
        SystemExit: If dictionary name is invalid
    """
    aruco = cv2.aruco
    dictionary_id = getattr(aruco, dictionary_name, None)
    
    if dictionary_id is None:
        supported = sorted(name for name in dir(aruco) if name.startswith("DICT_"))
        joined = ", ".join(supported)
        raise SystemExit(
            f"Invalid dictionary '{dictionary_name}'. Supported values: {joined}"
        )
    
    # Try modern API first
    get_predefined = getattr(aruco, "getPredefinedDictionary", None)
    if callable(get_predefined):
        return get_predefined(dictionary_id)
    
    # Fallback to legacy API
    dictionary_get = getattr(aruco, "Dictionary_get", None)
    if callable(dictionary_get):
        return dictionary_get(dictionary_id)
    
    raise SystemExit("OpenCV ArUco dictionary API not found")


def create_charuco_board(
    squares_x: int,
    squares_y: int,
    square_length: float,
    marker_length: float,
    dictionary: Any,
) -> Any:
    """
    Create a Charuco board.
    
    Handles different OpenCV API versions.
    
    Args:
        squares_x: Number of squares in X direction
        squares_y: Number of squares in Y direction
        square_length: Square side length
        marker_length: Marker side length
        dictionary: ArUco dictionary
        
    Returns:
        CharucoBoard object
    """
    aruco = cv2.aruco
    
    # Try modern API (OpenCV 4.7+)
    if hasattr(aruco, "CharucoBoard"):
        board_cls = aruco.CharucoBoard
        try:
            return board_cls((squares_x, squares_y), square_length, marker_length, dictionary)
        except TypeError:
            # Try .create() method
            create_fn = getattr(board_cls, "create", None)
            if callable(create_fn):
                return create_fn(
                    squares_x,
                    squares_y,
                    square_length,
                    marker_length,
                    dictionary,
                )
    
    # Try legacy API
    create_legacy = getattr(aruco, "CharucoBoard_create", None)
    if callable(create_legacy):
        return create_legacy(
            squares_x,
            squares_y,
            square_length,
            marker_length,
            dictionary,
        )
    
    raise SystemExit("OpenCV CharucoBoard API not found")


def draw_charuco_image(board: Any, width_px: int, height_px: int) -> np.ndarray:
    """
    Draw a Charuco board image.
    
    Args:
        board: CharucoBoard object
        width_px: Image width in pixels
        height_px: Image height in pixels
        
    Returns:
        Board image as numpy array
    """
    size = (int(width_px), int(height_px))
    
    # Try modern API
    generate_image = getattr(board, "generateImage", None)
    if callable(generate_image):
        return cast(np.ndarray, generate_image(size))
    
    # Try legacy API
    draw_fn = getattr(board, "draw", None)
    if callable(draw_fn):
        return cast(np.ndarray, draw_fn(size))
    
    raise SystemExit("OpenCV Charuco draw API not found")
























def detect_charuco_corners(
    image: np.ndarray,
    board: Any,
    dictionary: Any,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Detect Charuco corners in an image.
    
    Args:
        image: Input image (BGR or grayscale)
        board: CharucoBoard object
        dictionary: ArUco dictionary
        
    Returns:
        Tuple of (charuco_corners, charuco_ids) or (None, None) if not enough corners
    """
    aruco = cv2.aruco
    
    # Convert to grayscale if needed
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    # Detect ArUco markers
    detect_markers = getattr(aruco, "detectMarkers", None)
    if not callable(detect_markers):
        raise SystemExit("OpenCV detectMarkers API not found")
    
    # Get detector parameters
    detector_params = cv2.aruco.DetectorParameters()
    
    # Try modern detector API first
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, detector_params)
        marker_corners, marker_ids, _ = detector.detectMarkers(gray)
    else:
        # Legacy API
        result: Any = detect_markers(gray, dictionary, parameters=detector_params)
        marker_corners, marker_ids, _ = result
    
    if marker_ids is None or len(marker_ids) < 4:
        return None, None
    
    # Interpolate Charuco corners
    interpolate = getattr(aruco, "interpolateCornersCharuco", None)
    if not callable(interpolate):
        raise SystemExit("OpenCV interpolateCornersCharuco API not found")
    
    # Try different API signatures
    try:
        # Modern API: interpolateCornersCharuco(markerCorners, markerIds, image, board)
        result: Any = interpolate(
            marker_corners, marker_ids, gray, board
        )
        charuco_corners, charuco_ids, _, _ = result
    except (TypeError, ValueError):
        try:
            # Legacy API: returns (retval, charucoCorners, charucoIds)
            result: Any = interpolate(marker_corners, marker_ids, gray, board)
            if len(result) == 3:
                retval, charuco_corners, charuco_ids = result
                if not retval or charuco_corners is None:
                    return None, None
            else:
                charuco_corners, charuco_ids, _, _ = result
        except (TypeError, ValueError) as e:
            raise SystemExit(f"interpolateCornersCharuco API signature error: {e}")
    
    # Require at least 6 Charuco corners
    if charuco_corners is None or len(charuco_corners) < 6:
        return None, None
    
    return charuco_corners, charuco_ids


def calibrate_camera_charuco(
    all_corners: List[np.ndarray],
    all_ids: List[np.ndarray],
    board: Any,
    image_size: Tuple[int, int],
) -> Tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Calibrate camera using Charuco corners.
    
    Handles different OpenCV API signatures.
    
    Args:
        all_corners: List of Charuco corner arrays
        all_ids: List of Charuco ID arrays
        board: CharucoBoard object
        image_size: Image size (width, height)
        
    Returns:
        Tuple of (rms_error, camera_matrix, distortion_coeffs, rvecs, tvecs)
    """
    aruco = cv2.aruco
    calibrate_fn = getattr(aruco, "calibrateCameraCharuco", None)
    
    if not callable(calibrate_fn):
        raise SystemExit("OpenCV calibrateCameraCharuco API not found")
    
    # Try different API signatures
    try:
        # Modern API: calibrateCameraCharuco(charucoCorners, charucoIds, board, imageSize, cameraMatrix, distCoeffs)
        result: Any = calibrate_fn(
            all_corners, all_ids, board, image_size, None, None
        )
        retval, camera_matrix, dist_coeffs, rvecs, tvecs = result
    except TypeError:
        try:
            # Alternative signature
            result: Any = calibrate_fn(
                all_corners, all_ids, board, image_size, np.eye(3), np.zeros(5)
            )
            retval, camera_matrix, dist_coeffs, rvecs, tvecs = result
        except TypeError as e:
            raise SystemExit(f"calibrateCameraCharuco API signature error: {e}")
    
    return retval, camera_matrix, dist_coeffs, rvecs, tvecs


def compute_per_view_errors(
    all_corners: List[np.ndarray],
    all_ids: List[np.ndarray],
    board: Any,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvecs: np.ndarray,
    tvecs: np.ndarray,
) -> List[float]:
    """
    Compute per-view reprojection errors.
    
    Args:
        all_corners: List of Charuco corner arrays
        all_ids: List of Charuco ID arrays
        board: CharucoBoard object
        camera_matrix: Camera intrinsic matrix
        dist_coeffs: Distortion coefficients
        rvecs: Rotation vectors from calibration
        tvecs: Translation vectors from calibration
    Returns:
        List of per-view RMS errors
    """
    errors = []
    
    # Get board object points
    board_corners = getattr(board, "chessboardCorners", None)
    if board_corners is None:
        # Try to get corners from the board
        try:
            # For newer OpenCV, board.getChessboardCorners() might exist
            get_corners = getattr(board, "getChessboardCorners", None)
            if callable(get_corners):
                board_corners = get_corners()
        except Exception:
            pass
    
    if board_corners is None:
        return []
    
    for i, (corners, ids) in enumerate(zip(all_corners, all_ids)):
        if corners is None or ids is None or len(corners) == 0:
            continue
        
        # Get corresponding object points
        try:
            board_corners_arr = np.asarray(board_corners)
            obj_points = board_corners_arr[np.asarray(ids).flatten()]
        except (IndexError, TypeError):
            continue
        
        # Project points
        try:
            projected, _ = cv2.projectPoints(
                obj_points.astype(np.float32),
                rvecs[i],
                tvecs[i],
                camera_matrix,
                dist_coeffs
            )
            
            # Compute error
            error = cv2.norm(corners.reshape(-1, 2), projected.reshape(-1, 2), cv2.NORM_L2)
            error /= len(corners)
            errors.append(float(error))
        except Exception:
            continue
    
    return errors


def generate_synthetic_views(
    board: Any,
    board_img: np.ndarray,
    num_views: int,
    image_size: Tuple[int, int],
    seed: int,
) -> List[np.ndarray]:
    """
    Generate synthetic views by warping the board image.
    
    Args:
        board: CharucoBoard object
        board_img: Clean board image
        num_views: Number of views to generate
        image_size: Output image size (width, height)
        seed: Random seed
        
    Returns:
        List of synthetic view images
    """
    rng = np.random.default_rng(seed)
    views = []
    
    board_h, board_w = board_img.shape[:2]
    img_w, img_h = image_size
    
    for _ in range(num_views):
        # Create blank image
        view = np.full((img_h, img_w), 255, dtype=np.uint8)
        
        # Generate random perspective transform
        # Keep board fully inside frame with some margin
        margin = 50
        max_scale = min(img_w - 2 * margin, img_h - 2 * margin) / max(board_w, board_h)
        scale = rng.uniform(0.3, min(0.8, max_scale))
        
        # Random rotation angle
        angle = rng.uniform(-30, 30)
        
        # Random center position
        center_x = rng.uniform(margin + board_w * scale / 2, img_w - margin - board_w * scale / 2)
        center_y = rng.uniform(margin + board_h * scale / 2, img_h - margin - board_h * scale / 2)
        
        # Compute perspective distortion
        perspective_strength = rng.uniform(0.05, 0.2)
        
        # Source points (board corners)
        src_pts = np.array([
            [0, 0],
            [board_w, 0],
            [board_w, board_h],
            [0, board_h]
        ], dtype=np.float32)
        
        # Destination points with rotation, scale, translation, and perspective
        cos_a = np.cos(np.radians(angle))
        sin_a = np.sin(np.radians(angle))
        
        # Apply transform to each corner
        dst_pts = []
        for pt in src_pts:
            # Center the point
            x = pt[0] - board_w / 2
            y = pt[1] - board_h / 2
            
            # Rotate
            x_rot = x * cos_a - y * sin_a
            y_rot = x * sin_a + y * cos_a
            
            # Scale
            x_scaled = x_rot * scale
            y_scaled = y_rot * scale
            
            # Translate
            x_final = x_scaled + center_x
            y_final = y_scaled + center_y
            
            dst_pts.append([x_final, y_final])
        
        dst_pts = np.array(dst_pts, dtype=np.float32)
        
        # Add perspective distortion
        perspective_offset = perspective_strength * max(board_w, board_h) * scale
        dst_pts[0, 0] -= rng.uniform(0, perspective_offset)
        dst_pts[0, 1] -= rng.uniform(0, perspective_offset)
        dst_pts[1, 0] += rng.uniform(0, perspective_offset)
        dst_pts[1, 1] -= rng.uniform(0, perspective_offset)
        dst_pts[2, 0] += rng.uniform(0, perspective_offset)
        dst_pts[2, 1] += rng.uniform(0, perspective_offset)
        dst_pts[3, 0] -= rng.uniform(0, perspective_offset)
        dst_pts[3, 1] += rng.uniform(0, perspective_offset)
        
        # Compute perspective transform
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        
        # Warp the board image
        warped = cv2.warpPerspective(board_img, M, (img_w, img_h), borderValue=255)
        
        views.append(warped)
    
    return views


def load_images_from_dir(input_dir: str) -> List[np.ndarray]:
    """
    Load images from a directory.
    
    Args:
        input_dir: Path to directory containing images
        
    Returns:
        List of images as numpy arrays
    """
    images = []
    input_path = Path(input_dir)
    
    extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif"}
    
    for ext in extensions:
        for img_path in sorted(input_path.glob(f"*{ext}")):
            img = cv2.imread(str(img_path))
            if img is not None:
                images.append(img)
        for img_path in sorted(input_path.glob(f"*{ext.upper()}")):
            img = cv2.imread(str(img_path))
            if img is not None:
                images.append(img)
    
    return images


def build_output_json(
    camera_id: str,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rms_error: float,
    image_size: Tuple[int, int],
    config: CalibrateConfig,
    per_view_errors: List[float],
    total_points: int,
    num_valid_frames: int,
) -> Dict[str, Any]:
    """
    Build the output JSON structure.
    
    Args:
        camera_id: Camera identifier
        camera_matrix: 3x3 camera intrinsic matrix
        dist_coeffs: Distortion coefficients
        rms_error: RMS reprojection error
        image_size: Image size (width, height)
        config: Calibration configuration
        per_view_errors: Per-view reprojection errors
        total_points: Total number of corner points used
        num_valid_frames: Number of valid frames used
        
    Returns:
        Dictionary matching calibration_intrinsics_v1.json schema
    """
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    
    # Build distortion array (k1, k2, p1, p2, k3, ...)
    dist_array = dist_coeffs.flatten().tolist()
    # Ensure at least 5 coefficients
    while len(dist_array) < 5:
        dist_array.append(0.0)
    
    # Compute quality metrics
    mean_error = float(np.mean(per_view_errors)) if per_view_errors else 0.0
    max_error = float(np.max(per_view_errors)) if per_view_errors else 0.0
    pixel_aspect_ratio = fx / fy if fy != 0 else 1.0
    
    output = {
        "schema_version": "1.0",
        "camera_id": camera_id,
        "camera_model": "pinhole",
        "resolution": {
            "width": int(image_size[0]),
            "height": int(image_size[1]),
        },
        "camera_matrix": {
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "matrix": [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
        },
        "distortion_coefficients": {
            "k1": float(dist_array[0]) if len(dist_array) > 0 else 0.0,
            "k2": float(dist_array[1]) if len(dist_array) > 1 else 0.0,
            "p1": float(dist_array[2]) if len(dist_array) > 2 else 0.0,
            "p2": float(dist_array[3]) if len(dist_array) > 3 else 0.0,
            "k3": float(dist_array[4]) if len(dist_array) > 4 else 0.0,
            "array": [float(d) for d in dist_array],
        },
        "rms_error": float(rms_error),
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "board": {
            "type": "charuco",
            "squares_x": config.squares_x,
            "squares_y": config.squares_y,
            "square_length_m": config.square_length_mm / 1000.0,
            "marker_length_m": config.marker_length_mm / 1000.0,
            "dictionary": config.dictionary,
        },
        "quality": {
            "mean_error": mean_error,
            "max_error": max_error,
            "per_view_errors": [float(e) for e in per_view_errors],
            "pixel_aspect_ratio": float(pixel_aspect_ratio),
            "num_valid_frames": int(num_valid_frames),
            "total_points": int(total_points),
        },
    }
    
    return output


def run_calibration(
    images: List[np.ndarray],
    board: Any,
    dictionary: Any,
    config: CalibrateConfig,
) -> Tuple[Optional[Dict[str, Any]], int]:
    """
    Run calibration on a list of images.
    
    Args:
        images: List of images
        board: CharucoBoard object
        dictionary: ArUco dictionary
        config: Calibration configuration
        
    Returns:
        Tuple of (output JSON dict or None, number of valid frames)
    """
    all_corners: List[np.ndarray] = []
    all_ids: List[np.ndarray] = []
    image_size: Optional[Tuple[int, int]] = None
    
    print(f"Processing {len(images)} images...")
    
    for i, img in enumerate(images):
        corners, ids = detect_charuco_corners(img, board, dictionary)
        
        if corners is not None and ids is not None:
            all_corners.append(corners)
            all_ids.append(ids)
            
            if image_size is None:
                h, w = img.shape[:2]
                image_size = (w, h)
            
            print(f"  Image {i+1}: {len(corners)} corners detected")
        else:
            print(f"  Image {i+1}: skipped (insufficient corners)")
    
    num_valid = len(all_corners)
    print(f"\nValid frames: {num_valid}/{len(images)}")
    
    if num_valid < config.min_frames:
        print(f"ERROR: Need at least {config.min_frames} valid frames, got {num_valid}")
        return None, num_valid
    
    if image_size is None:
        print("ERROR: Could not determine image size")
        return None, num_valid
    
    print(f"\nRunning calibration...")
    
    # Run calibration
    # Run calibration
    rms_error, camera_matrix, dist_coeffs, rvecs, tvecs = calibrate_camera_charuco(
        all_corners, all_ids, board, image_size
    )
    
    print(f"RMS error: {rms_error:.4f} pixels")
    if rms_error > 1.0:
        print("WARNING: RMS error > 1.0 px")
    elif rms_error < 0.5:
        print("GOOD: RMS error < 0.5 px")
    
    # Compute per-view errors
    per_view_errors = compute_per_view_errors(
        all_corners, all_ids, board, camera_matrix, dist_coeffs, rvecs, tvecs
    )
    
    # Count total points
    total_points = sum(len(c) for c in all_corners)
    
    # Build output
    output = build_output_json(
        camera_id=config.camera,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        rms_error=rms_error,
        image_size=image_size,
        config=config,
        per_view_errors=per_view_errors,
        total_points=total_points,
        num_valid_frames=num_valid,
    )
    
    return output, num_valid


def run_self_test(config: CalibrateConfig) -> int:
    """
    Run hardware-free self-test with synthetic data.
    
    Args:
        config: Calibration configuration
        
    Returns:
        Exit code (0 for success)
    """
    print("Running self-test mode...")
    print(f"  Camera: {config.camera}")
    print(f"  Board: {config.squares_x}x{config.squares_y} squares")
    print(f"  Square length: {config.square_length_mm} mm")
    print(f"  Marker length: {config.marker_length_mm} mm")
    print(f"  Dictionary: {config.dictionary}")
    print(f"  Synthetic views: {config.self_test_views}")
    print(f"  Random seed: {config.seed}")
    
    # Get dictionary
    dictionary = get_dictionary(config.dictionary)
    
    # Create board (use pixel units for synthetic test)
    square_length_px = 100.0  # Arbitrary pixel size
    marker_length_px = square_length_px * (config.marker_length_mm / config.square_length_mm)
    
    board = create_charuco_board(
        squares_x=config.squares_x,
        squares_y=config.squares_y,
        square_length=square_length_px,
        marker_length=marker_length_px,
        dictionary=dictionary,
    )
    
    # Draw board image
    board_w = int(config.squares_x * square_length_px)
    board_h = int(config.squares_y * square_length_px)
    board_img = draw_charuco_image(board, board_w, board_h)
    
    print(f"\nBoard image size: {board_w}x{board_h} pixels")
    
    # Generate synthetic views
    print(f"\nGenerating {config.self_test_views} synthetic views...")
    views = generate_synthetic_views(
        board, board_img, config.self_test_views, SYNTHETIC_IMAGE_SIZE, config.seed
    )
    
    # Run calibration
    output, num_valid = run_calibration(views, board, dictionary, config)
    
    if output is None:
        return 1
    
    # Determine output path
    if config.output:
        output_path = Path(config.output)
    else:
        output_path = Path(f"calibration/calibration_intrinsics_v1_{config.camera}.json")
    
    # Write output
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    
    print(f"\nCalibration written to: {output_path}")
    print(f"  RMS error: {output['rms_error']:.4f} pixels")
    print(f"  Valid frames: {output['quality']['num_valid_frames']}")
    print(f"  Total points: {output['quality']['total_points']}")
    
    return 0


def run_offline_calibration(config: CalibrateConfig) -> int:
    """
    Run offline calibration from images in a directory.
    
    Args:
        config: Calibration configuration
        
    Returns:
        Exit code (0 for success)
    """
    print("Running offline calibration mode...")
    print(f"  Input directory: {config.input_dir}")
    print(f"  Camera: {config.camera}")
    print(f"  Board: {config.squares_x}x{config.squares_y} squares")
    print(f"  Square length: {config.square_length_mm} mm")
    print(f"  Marker length: {config.marker_length_mm} mm")
    print(f"  Dictionary: {config.dictionary}")
    
    # Load images
    images = load_images_from_dir(config.input_dir or "")
    
    if not images:
        print(f"ERROR: No images found in {config.input_dir}")
        return 1
    
    print(f"\nLoaded {len(images)} images")
    
    # Get dictionary
    dictionary = get_dictionary(config.dictionary)
    
    # Create board (use physical units)
    square_length_m = config.square_length_mm / 1000.0
    marker_length_m = config.marker_length_mm / 1000.0
    
    board = create_charuco_board(
        squares_x=config.squares_x,
        squares_y=config.squares_y,
        square_length=square_length_m,
        marker_length=marker_length_m,
        dictionary=dictionary,
    )
    
    # Run calibration
    output, num_valid = run_calibration(images, board, dictionary, config)
    
    if output is None:
        return 1
    
    # Determine output path
    if config.output:
        output_path = Path(config.output)
    else:
        output_path = Path(f"calibration/calibration_intrinsics_v1_{config.camera}.json")
    
    # Write output
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    
    print(f"\nCalibration written to: {output_path}")
    print(f"  RMS error: {output['rms_error']:.4f} pixels")
    print(f"  Valid frames: {output['quality']['num_valid_frames']}")
    print(f"  Total points: {output['quality']['total_points']}")
    
    return 0


def run_live_capture(config: CalibrateConfig) -> int:
    """
    Run live capture calibration using Pi Camera.
    
    Args:
        config: Calibration configuration
        
    Returns:
        Exit code (0 for success)
    """
    # Try to import picamera2
    try:
        from picamera2 import Picamera2  # pyright: ignore[reportMissingImports]
    except ImportError:
        print("ERROR: picamera2 is required for live capture mode.")
        print("")
        print("To install on Raspberry Pi:")
        print("  sudo apt install -y python3-picamera2")
        print("")
        print("Alternatively, use one of these modes:")
        print("  --self-test      Hardware-free synthetic calibration")
        print("  --input-dir DIR  Offline calibration from images")
        return 1

    try:
        import libcamera  # pyright: ignore[reportMissingImports]
    except ImportError:
        libcamera = None
    
    print("Running live capture mode...")
    print(f"  Camera: {config.camera}")
    print(f"  Board: {config.squares_x}x{config.squares_y} squares")
    print(f"  Square length: {config.square_length_mm} mm")
    print(f"  Marker length: {config.marker_length_mm} mm")
    print(f"  Dictionary: {config.dictionary}")
    print(f"  Minimum frames: {config.min_frames}")
    
    # Get dictionary
    dictionary = get_dictionary(config.dictionary)
    
    # Create board
    square_length_m = config.square_length_mm / 1000.0
    marker_length_m = config.marker_length_mm / 1000.0
    
    board = create_charuco_board(
        squares_x=config.squares_x,
        squares_y=config.squares_y,
        square_length=square_length_m,
        marker_length=marker_length_m,
        dictionary=dictionary,
    )
    
    # Initialize camera
    print("\nInitializing camera...")
    picam2 = Picamera2()
    config_cam = picam2.create_preview_configuration(
        main={"size": (2304, 1296), "format": "RGB888"}
    )
    picam2.configure(config_cam)
    picam2.start()
    # Set FPS to 56 via FrameDurationLimits
    frame_duration_us = int(round(1_000_000 / 56.0))
    try:
        picam2.set_controls({"FrameDurationLimits": (frame_duration_us, frame_duration_us)})
    except Exception:
        pass

    lens_min = 0.0
    lens_max = 10.0
    lens_default = 0.0
    camera_controls = getattr(picam2, "camera_controls", None)
    if isinstance(camera_controls, dict) and "LensPosition" in camera_controls:
        lens_control = camera_controls["LensPosition"]
        if isinstance(lens_control, (tuple, list)) and len(lens_control) >= 2:
            try:
                lens_min = float(lens_control[0])
                lens_max = float(lens_control[1])
                if len(lens_control) >= 3:
                    lens_default = float(lens_control[2])
                else:
                    lens_default = lens_min
            except (TypeError, ValueError):
                lens_min = 0.0
                lens_max = 10.0
                lens_default = 0.0
        elif isinstance(lens_control, dict):
            try:
                lens_min = float(lens_control.get("min", lens_min))
                lens_max = float(lens_control.get("max", lens_max))
                lens_default = float(lens_control.get("default", lens_default))
            except (TypeError, ValueError):
                lens_min = 0.0
                lens_max = 10.0
                lens_default = 0.0

    if lens_max <= lens_min:
        lens_min = 0.0
        lens_max = 10.0
        lens_default = 0.0

    lens_slider_max = 1000

    def _lens_to_slider(lens_value: float) -> int:
        normalized = (lens_value - lens_min) / (lens_max - lens_min)
        normalized = max(0.0, min(1.0, normalized))
        return int(round(normalized * lens_slider_max))

    cv2.namedWindow("Calibration")
    cv2.createTrackbar(
        "LensPosition",
        "Calibration",
        _lens_to_slider(lens_default),
        lens_slider_max,
        lambda _value: None,
    )
    
    images: List[np.ndarray] = []
    image_size: Optional[Tuple[int, int]] = None
    
    print("\nCapture instructions:")
    print("  - Show the Charuco board to the camera")
    print("  - Press SPACE to capture a frame (only when corners detected)")
    print("  - Press 'C' to calibrate (requires at least {config.min_frames} valid frames)")
    print("  - Press 'Q' to quit without calibration")
    print(f"  - Need at least {config.min_frames} valid frames for calibration")
    print("")
    
    try:
        while True:
            slider_pos = cv2.getTrackbarPos("LensPosition", "Calibration")
            lens_pos = lens_min + (float(slider_pos) / float(lens_slider_max)) * (lens_max - lens_min)
            try:
                af_mode_manual: Any = 0
                if libcamera is not None:
                    controls_mod = getattr(libcamera, "controls", None)
                    if controls_mod is not None:
                        af_mode_enum = getattr(controls_mod, "AfModeEnum", None)
                        if af_mode_enum is not None:
                            af_mode_manual = getattr(af_mode_enum, "Manual", af_mode_manual)
                picam2.set_controls({"AfMode": af_mode_manual, "LensPosition": float(lens_pos)})
            except Exception:
                pass

            # Capture frame
            frame = picam2.capture_array()
            # Convert RGB to BGR for OpenCV
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Detect corners for preview
            corners, ids = detect_charuco_corners(frame_bgr, board, dictionary)
            
            # Draw preview
            display = frame_bgr.copy()
            if corners is not None:
                cv2.aruco.drawDetectedCornersCharuco(display, corners, ids)
                status = f"Valid: {len(corners)} corners - Press SPACE to capture"
                color = (0, 255, 0)
            else:
                status = "Show board to camera - Need >= 6 corners"
                color = (0, 0, 255)
            
            cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(display, f"Captured: {len(images)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display, f"LensPosition: {lens_pos:.3f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            cv2.imshow("Calibration", display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' ') and corners is not None:
                # SPACE: capture frame
                images.append(frame_bgr.copy())
                if image_size is None:
                    h, w = frame_bgr.shape[:2]
                    image_size = (w, h)
                print(f"Captured frame {len(images)}: {len(corners)} corners")
            elif key in (ord('c'), ord('C')):
                # C: calibrate (only if min_frames reached)
                if len(images) >= config.min_frames:
                    break
                else:
                    print(f"Need {config.min_frames} frames to calibrate, got {len(images)}")
            elif key in (ord('q'), ord('Q')):
                # Q: quit without calibrating
                return 0
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
    
    if len(images) < config.min_frames:
        print(f"\nERROR: Need at least {config.min_frames} frames, got {len(images)}")
        return 1
    
    # Run calibration
    output, num_valid = run_calibration(images, board, dictionary, config)
    
    if output is None:
        return 1
    
    # Determine output path
    if config.output:
        output_path = Path(config.output)
    else:
        output_path = Path(f"calibration/calibration_intrinsics_v1_{config.camera}.json")
    
    # Write output
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    
    print(f"\nCalibration written to: {output_path}")
    print(f"  RMS error: {output['rms_error']:.4f} pixels")
    print(f"  Valid frames: {output['quality']['num_valid_frames']}")
    print(f"  Total points: {output['quality']['total_points']}")
    
    return 0


def main() -> int:
    """Main entry point."""
    config = parse_args()
    
    if config.self_test:
        return run_self_test(config)
    elif config.input_dir:
        return run_offline_calibration(config)
    else:
        return run_live_capture(config)


if __name__ == "__main__":
    raise SystemExit(main())
