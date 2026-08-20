#!/usr/bin/env python3
"""Offline cart-body point-cloud registration and visual diagnostics."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d


def arguments(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--pair", required=True)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args(argv)


def _cloud(points, colors):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    cloud.colors = o3d.utility.Vector3dVector(np.asarray(colors, dtype=float))
    return cloud


def _preprocess(cloud, voxel_m):
    down = cloud.voxel_down_sample(voxel_m)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_m * 2.5, max_nn=40)
    )
    feature = o3d.pipelines.registration.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_m * 5.0, max_nn=100),
    )
    return down, feature


def _global_registration(source, target, voxel_m):
    source_down, source_fpfh = _preprocess(source, voxel_m)
    target_down, target_fpfh = _preprocess(target, voxel_m)
    threshold = voxel_m * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        True,
        threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3,
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                threshold
            ),
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999),
    )
    return result


def _refine_colored(source, target, initial):
    transform = np.asarray(initial, dtype=float)
    for voxel_m, iterations in ((0.03, 50), (0.015, 35), (0.008, 20)):
        source_down = source.voxel_down_sample(voxel_m)
        target_down = target.voxel_down_sample(voxel_m)
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_m * 2.5, max_nn=40)
        )
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_m * 2.5, max_nn=40)
        )
        result = o3d.pipelines.registration.registration_colored_icp(
            source_down,
            target_down,
            voxel_m * 1.5,
            transform,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-7,
                relative_rmse=1e-7,
                max_iteration=iterations,
            ),
        )
        transform = result.transformation
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source.voxel_down_sample(0.008),
        target.voxel_down_sample(0.008),
        0.016,
        transform,
    )
    return transform, evaluation


def _transform(points, matrix):
    points = np.asarray(points, dtype=float)
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def _project(points_base, camera_to_base, intrinsics):
    base_to_camera = np.linalg.inv(camera_to_base)
    points_camera = _transform(points_base, base_to_camera)
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    pixels = np.column_stack((
        fx * points_camera[:, 0] / points_camera[:, 2] + cx,
        fy * points_camera[:, 1] / points_camera[:, 2] + cy,
    ))
    return pixels, points_camera[:, 2]


def _reference_geometry(reference):
    front = np.asarray(reference["platform_front_edge_base_m"], dtype=float)
    left = np.asarray(reference["platform_left_edge_base_m"], dtype=float)
    right = np.asarray(reference["platform_right_edge_base_m"], dtype=float)
    forward = np.asarray(reference["platform_forward_axis_base"], dtype=float)
    lateral = np.asarray(reference["platform_lateral_axis_base"], dtype=float)
    width = float(reference["platform_width_m"])
    depth = float(reference["platform_depth_m"])
    center = (left + right) / 2.0
    front_left = front + lateral * (width / 2.0)
    front_right = front - lateral * (width / 2.0)
    back_left = front_left + forward * depth
    back_right = front_right + forward * depth
    outline = np.asarray((front_left, back_left, back_right, front_right))
    slot_offsets = (0.17, 0.24, 0.31, 0.38, 0.45)
    slots = np.asarray([
        center + lateral * (width / 2.0 - offset)
        for offset in slot_offsets
    ])
    return outline, slots


def _draw_registration_overlay(data, transform, output):
    image = np.asarray(data["current_color_bgr"]).copy()
    reference_points = _transform(data["reference_points_base"], transform)
    pixels, depth = _project(
        reference_points,
        data["current_camera_to_base"],
        data["current_intrinsics"],
    )
    height, width = image.shape[:2]
    valid = (
        np.isfinite(pixels).all(axis=1)
        & (depth > 0.0)
        & (pixels[:, 0] >= 0)
        & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < height)
    )
    for u, v in pixels[valid][::3].astype(int):
        cv2.circle(image, (int(u), int(v)), 1, (255, 0, 255), -1)
    cv2.imwrite(str(output), image)


def _draw_geometry_overlay(data, transform, reference, output):
    image = np.asarray(data["current_color_bgr"]).copy()
    outline, slots = _reference_geometry(reference)
    outline = _transform(outline, transform)
    slots = _transform(slots, transform)
    outline_pixels, outline_depth = _project(
        outline, data["current_camera_to_base"], data["current_intrinsics"]
    )
    slot_pixels, slot_depth = _project(
        slots, data["current_camera_to_base"], data["current_intrinsics"]
    )
    if np.all(outline_depth > 0.0):
        cv2.polylines(
            image,
            [np.rint(outline_pixels).astype(np.int32)],
            True,
            (0, 255, 0),
            5,
        )
    for index, (pixel, depth) in enumerate(zip(slot_pixels, slot_depth), 1):
        if depth <= 0.0 or not np.isfinite(pixel).all():
            continue
        point = tuple(int(value) for value in np.rint(pixel))
        cv2.circle(image, point, 12, (0, 0, 255), -1)
        cv2.putText(
            image,
            f"slot {index}",
            (point[0] + 12, point[1] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
    cv2.imwrite(str(output), image)


def main(argv=None):
    args = arguments(argv)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    data = np.load(args.pair)
    reference = json.loads(Path(args.reference).read_text(encoding="utf-8"))

    source = _cloud(data["reference_points_base"], data["reference_colors_rgb"])
    target = _cloud(data["current_points_base"], data["current_colors_rgb"])
    global_result = _global_registration(source, target, voxel_m=0.025)
    transform, evaluation = _refine_colored(
        source, target, global_result.transformation
    )

    registration_overlay = output_dir / "cart_registration_overlay.jpg"
    geometry_overlay = output_dir / "cart_registered_geometry.jpg"
    _draw_registration_overlay(data, transform, registration_overlay)
    _draw_geometry_overlay(data, transform, reference, geometry_overlay)
    payload = {
        "global_fitness": float(global_result.fitness),
        "global_inlier_rmse_m": float(global_result.inlier_rmse),
        "refined_fitness": float(evaluation.fitness),
        "refined_inlier_rmse_m": float(evaluation.inlier_rmse),
        "reference_to_current_base": transform.tolist(),
        "registration_overlay": str(registration_overlay),
        "geometry_overlay": str(geometry_overlay),
    }
    result_path = output_dir / "cart_registration_result.json"
    result_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(result_path.read_text(encoding="utf-8"), end="")


if __name__ == "__main__":
    main()
