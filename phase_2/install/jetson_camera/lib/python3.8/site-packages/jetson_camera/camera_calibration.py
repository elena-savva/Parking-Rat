#!/usr/bin/env python3
"""
Offline camera calibration using Zhang's checkerboard method.

This script is intentionally kept outside the ROS node graph because
calibration is a one-time setup step:

1. We collect multiple images of a flat chessboard from different views.
2. We detect the 2D image coordinates of the chessboard corners.
3. We pair those 2D image coordinates with known 3D board coordinates.
4. OpenCV's calibrateCamera() estimates the camera matrix and distortion.
5. We store the result in a YAML file so the processing node can reuse it.

"""

from __future__ import annotations

import argparse
import glob
import os
import sys
from typing import Iterable

import cv2
import numpy as np
import yaml

try:
    from ament_index_python.packages import PackageNotFoundError
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    PackageNotFoundError = Exception
    get_package_share_directory = None



DEFAULT_COLS = 8
DEFAULT_ROWS = 6
DEFAULT_SQUARE_SIZE_MM = 25.0


def default_image_dir() -> str:
    """Return the source folder that stores captured calibration images."""
    package_source_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
    return os.path.join(package_source_dir, '..', 'camera_calib_images')


def default_output_path() -> str:

    if get_package_share_directory is not None:
        try:
            package_share_dir = get_package_share_directory('jetson_camera')
            return os.path.join(
                package_share_dir,
                'config',
                'camera_calibration.yaml',
            )
        except PackageNotFoundError:
            pass

    package_source_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
    return os.path.join(package_source_dir, 'config', 'camera_calibration.yaml')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Calibrate the camera with Zhang chessboard calibration.'
    )
    parser.add_argument(
        '--image-dir',
        default=default_image_dir(),
        help='Directory containing calibration images.',
    )
    parser.add_argument(
        '--output',
        default=default_output_path(),
        help='Path to the YAML file that will store the calibration result.',
    )
    parser.add_argument(
        '--cols',
        type=int,
        default=DEFAULT_COLS,
        help='Number of inner checkerboard corners along the columns.',
    )
    parser.add_argument(
        '--rows',
        type=int,
        default=DEFAULT_ROWS,
        help='Number of inner checkerboard corners along the rows.',
    )
    parser.add_argument(
        '--square-size-mm',
        type=float,
        default=DEFAULT_SQUARE_SIZE_MM,
        help='Physical side length of one checker square in millimetres.',
    )
    parser.add_argument(
        '--min-images',
        type=int,
        default=8,
        help='Minimum number of valid checkerboard detections required.',
    )
    return parser.parse_args()


def get_image_paths(image_dir: str) -> list[str]:
    patterns = ('*.png', '*.jpg', '*.jpeg', '*.bmp')
    image_paths: list[str] = []

    for pattern in patterns:
        image_paths.extend(glob.glob(os.path.join(image_dir, pattern)))

    def numeric_sort_key(path: str) -> tuple[int, str]:
        stem = os.path.splitext(os.path.basename(path))[0]
        try:
            return (0, f'{int(stem):08d}')
        except ValueError:
            return (1, stem)

    return sorted(image_paths, key=numeric_sort_key)


def build_object_points(
    board_size: tuple[int, int],
    square_size_mm: float,
) -> np.ndarray:
    cols, rows = board_size
    object_points = np.zeros((cols * rows, 3), np.float32)
    object_points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    object_points *= square_size_mm
    return object_points


def detect_checkerboard_corners(
    image_paths: Iterable[str],
    board_size: tuple[int, int],
    square_size_mm: float,
) -> tuple[list[np.ndarray], list[np.ndarray], tuple[int, int], list[str]]:

    termination_criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    object_points_template = build_object_points(board_size, square_size_mm)

    object_points_list: list[np.ndarray] = []
    image_points_list: list[np.ndarray] = []
    successful_images: list[str] = []
    image_size: tuple[int, int] | None = None

    classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    sb_flags = cv2.CALIB_CB_NORMALIZE_IMAGE

    def try_classic_detector(gray_image: np.ndarray) -> tuple[bool, np.ndarray | None]:

        found, corners = cv2.findChessboardCorners(
            gray_image,
            board_size,
            classic_flags,
        )
        if not found:
            return False, None

        refined_corners = cv2.cornerSubPix(
            gray_image,
            corners,
            (11, 11),
            (-1, -1),
            termination_criteria,
        )
        return True, refined_corners

    def try_sb_detector(gray_image: np.ndarray) -> tuple[bool, np.ndarray | None]:

        if not hasattr(cv2, 'findChessboardCornersSB'):
            return False, None

        try:
            found, corners = cv2.findChessboardCornersSB(
                gray_image,
                board_size,
                sb_flags,
            )
        except cv2.error as exc:
            print(f'[WARN] SB detector crashed internally: {exc}')
            return False, None

        if not found:
            return False, None

        # The SB detector already returns accurate sub-pixel corners.
        return True, corners.astype(np.float32)

    for image_path in image_paths:
        image = cv2.imread(image_path)
        if image is None:
            print(f'[SKIP] Could not read {image_path}')
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced_gray = clahe.apply(gray)
        inverted_gray = cv2.bitwise_not(gray)

        detection_method = ''
        found, refined_corners = try_classic_detector(gray)
        if found:
            detection_method = 'classic'

        if not found:
            found, refined_corners = try_classic_detector(enhanced_gray)
            if found:
                detection_method = 'classic+clahe'

        if not found:
            found, refined_corners = try_classic_detector(inverted_gray)
            if found:
                detection_method = 'classic+invert'

        if not found:
            found, refined_corners = try_sb_detector(gray)
            if found:
                detection_method = 'sb'

        if not found:
            found, refined_corners = try_sb_detector(enhanced_gray)
            if found:
                detection_method = 'sb+clahe'

        if not found:
            found, refined_corners = try_sb_detector(inverted_gray)
            if found:
                detection_method = 'sb+invert'

        if not found or refined_corners is None:
            print(f'[FAIL] Checkerboard not found in {os.path.basename(image_path)}')
            continue

        object_points_list.append(object_points_template.copy())
        image_points_list.append(refined_corners)
        successful_images.append(os.path.basename(image_path))
        print(
            f'[ OK ] Corners detected in {os.path.basename(image_path)} '
            f'with {detection_method}'
        )

    if image_size is None:
        raise RuntimeError('No readable calibration images were found.')

    return object_points_list, image_points_list, image_size, successful_images


def calculate_mean_reprojection_error(
    object_points_list: list[np.ndarray],
    image_points_list: list[np.ndarray],
    rotation_vectors: list[np.ndarray],
    translation_vectors: list[np.ndarray],
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
) -> float:

    total_error = 0.0

    for index, object_points in enumerate(object_points_list):
        projected_points, _ = cv2.projectPoints(
            object_points,
            rotation_vectors[index],
            translation_vectors[index],
            camera_matrix,
            distortion_coefficients,
        )
        image_error = cv2.norm(
            image_points_list[index],
            projected_points,
            cv2.NORM_L2,
        ) / len(projected_points)
        total_error += image_error

    return total_error / len(object_points_list)


def calibrate_from_images(
    image_dir: str,
    output_path: str,
    board_size: tuple[int, int],
    square_size_mm: float,
    min_images: int,
) -> None:

    image_paths = get_image_paths(image_dir)
    if not image_paths:
        raise FileNotFoundError(f'No calibration images found in: {image_dir}')

    print(f'Found {len(image_paths)} image(s) in {image_dir}')
    print(
        'Using checkerboard inner-corner pattern '
        f'{board_size[0]}x{board_size[1]} '
        f'with square size {square_size_mm:.2f} mm'
    )

    (
        object_points_list,
        image_points_list,
        image_size,
        successful_images,
    ) = detect_checkerboard_corners(image_paths, board_size, square_size_mm)

    print(
        f'\nDetected valid checkerboard corners in '
        f'{len(successful_images)}/{len(image_paths)} image(s)'
    )
    if len(successful_images) < min_images:
        raise RuntimeError(
            f'Need at least {min_images} valid images, '
            f'but only {len(successful_images)} were detected.'
        )

    # This is the core OpenCV step that implements Zhang's calibration method.
    reprojection_rms, camera_matrix, distortion_coefficients, rvecs, tvecs = (
        cv2.calibrateCamera(
            object_points_list,
            image_points_list,
            image_size,
            None,
            None,
        )
    )

    mean_reprojection_error = calculate_mean_reprojection_error(
        object_points_list,
        image_points_list,
        rvecs,
        tvecs,
        camera_matrix,
        distortion_coefficients,
    )

    calibration_data = {
        'board_cols': board_size[0],
        'board_rows': board_size[1],
        'square_size_mm': square_size_mm,
        'image_width': image_size[0],
        'image_height': image_size[1],
        'valid_images': successful_images,
        'camera_matrix': camera_matrix.tolist(),
        'dist_coeffs': distortion_coefficients.tolist(),
        'rms_reprojection_error': float(reprojection_rms),
        'mean_reprojection_error_px': float(mean_reprojection_error),
    }

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as output_file:
        yaml.safe_dump(calibration_data, output_file, sort_keys=False)

    print('\nCalibration complete.')
    print(f'RMS reprojection error: {reprojection_rms:.6f}')
    print(f'Mean reprojection error: {mean_reprojection_error:.6f} px')
    print(f'Calibration saved to: {output_path}')
    print('\nCamera matrix:')
    print(camera_matrix)
    print('\nDistortion coefficients:')
    print(distortion_coefficients)


def main() -> None:
    args = parse_args()
    board_size = (args.cols, args.rows)

    try:
        calibrate_from_images(
            image_dir=os.path.abspath(args.image_dir),
            output_path=os.path.abspath(args.output),
            board_size=board_size,
            square_size_mm=args.square_size_mm,
            min_images=args.min_images,
        )
    except Exception as exc:
        print(f'\nCalibration failed: {exc}', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
