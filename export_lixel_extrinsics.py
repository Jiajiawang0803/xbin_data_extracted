"""Convert Lixel calibration matrices to direction-explicit XYZW YAML files."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path


CAMERA_INDEX_TO_NAME = {0: "camera_left", 1: "camera_right", 2: "camera_center"}


def parse_numbers(text: str) -> list[float]:
    return [
        float(value)
        for value in re.findall(
            r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", text
        )
    ]


def read_yaml_list(path: Path, key: str) -> list[float]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    for index, line in enumerate(lines):
        match = re.match(rf"^\s*{re.escape(key)}\s*:\s*(.*)$", line)
        if not match:
            continue
        remainder = match.group(1).strip()
        if remainder.startswith("["):
            return parse_numbers(remainder)
        values: list[float] = []
        for following in lines[index + 1 :]:
            item = re.match(r"^\s+-\s+(.+?)\s*$", following)
            if not item:
                break
            parsed = parse_numbers(item.group(1))
            if len(parsed) != 1:
                raise ValueError(f"Invalid {key} item in {path}: {following}")
            values.append(parsed[0])
        return values
    raise ValueError(f"Missing {key} in {path}")


def read_camera_poses(path: Path) -> dict[int, list[float]]:
    text = path.read_text(encoding="utf-8", errors="replace")
    sections = re.finditer(
        r"(?ms)^camera_(\d+):\s*\n(.*?)(?=^camera_\d+:|^calibrated:|\Z)", text
    )
    poses: dict[int, list[float]] = {}
    for section in sections:
        camera_index = int(section.group(1))
        body = section.group(2)
        match = re.search(
            r"(?ms)^\s{2}camera_pose:\s*\n((?:\s{4}-\s+[^\n]+\n?)+)", body
        )
        if not match:
            raise ValueError(f"Missing camera_pose for camera_{camera_index} in {path}")
        pose = parse_numbers(match.group(1))
        if len(pose) != 16:
            raise ValueError(
                f"camera_{camera_index} pose has {len(pose)} values, expected 16"
            )
        poses[camera_index] = pose
    if set(poses) != set(CAMERA_INDEX_TO_NAME):
        raise ValueError(f"Expected camera poses 0, 1, 2 in {path}; got {sorted(poses)}")
    return poses


def matrix4(values: list[float]) -> list[list[float]]:
    if len(values) != 16:
        raise ValueError(f"Expected 16 matrix values, got {len(values)}")
    return [values[row * 4 : row * 4 + 4] for row in range(4)]


def multiply(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4)]
        for row in range(4)
    ]


def inverse_rigid(matrix: list[list[float]]) -> list[list[float]]:
    rotation_transpose = [[matrix[column][row] for column in range(3)] for row in range(3)]
    translation = [matrix[row][3] for row in range(3)]
    inverse_translation = [
        -sum(rotation_transpose[row][column] * translation[column] for column in range(3))
        for row in range(3)
    ]
    return [
        rotation_transpose[row] + [inverse_translation[row]] for row in range(3)
    ] + [[0.0, 0.0, 0.0, 1.0]]


def quaternion_xyzw(matrix: list[list[float]]) -> tuple[float, float, float, float]:
    r00, r01, r02 = matrix[0][:3]
    r10, r11, r12 = matrix[1][:3]
    r20, r21, r22 = matrix[2][:3]
    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (r21 - r12) / scale
        qy = (r02 - r20) / scale
        qz = (r10 - r01) / scale
    elif r00 > r11 and r00 > r22:
        scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / scale
        qx = 0.25 * scale
        qy = (r01 + r10) / scale
        qz = (r02 + r20) / scale
    elif r11 > r22:
        scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / scale
        qx = (r01 + r10) / scale
        qy = 0.25 * scale
        qz = (r12 + r21) / scale
    else:
        scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / scale
        qx = (r02 + r20) / scale
        qy = (r12 + r21) / scale
        qz = 0.25 * scale
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("Rotation produced a zero quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    # q and -q describe the same rotation. A non-negative qw gives stable files.
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    return qx, qy, qz, qw


def format_number(value: float) -> str:
    if abs(value) < 5e-16:
        value = 0.0
    return f"{value:.15g}"


def write_transform(
    output_dir: Path,
    source: str,
    target: str,
    matrix: list[list[float]],
    source_calibration: str,
    notes: str,
) -> Path:
    qx, qy, qz, qw = quaternion_xyzw(matrix)
    filename = output_dir / f"{source}_to_{target}.yaml"
    translation = [matrix[row][3] for row in range(3)]
    content = (
        f"transform: {source}_to_{target}\n"
        f"source_frame: {source}\n"
        f"target_frame: {target}\n"
        'convention: "p_target = R(q_xyzw) * p_source + t"\n'
        "translation_unit: meter\n"
        "quaternion_order: [qx, qy, qz, qw]\n"
        f"x: {format_number(translation[0])}\n"
        f"y: {format_number(translation[1])}\n"
        f"z: {format_number(translation[2])}\n"
        f"qx: {format_number(qx)}\n"
        f"qy: {format_number(qy)}\n"
        f"qz: {format_number(qz)}\n"
        f"qw: {format_number(qw)}\n"
        f"source_calibration: {source_calibration}\n"
        f"notes: \"{notes}\"\n"
    )
    filename.write_text(content, encoding="utf-8")
    return filename


def write_pair(
    output_dir: Path,
    source: str,
    target: str,
    source_to_target: list[list[float]],
    source_calibration: str,
    notes: str,
) -> list[Path]:
    return [
        write_transform(
            output_dir,
            source,
            target,
            source_to_target,
            source_calibration,
            notes,
        ),
        write_transform(
            output_dir,
            target,
            source,
            inverse_rigid(source_to_target),
            source_calibration,
            f"inverse of {source}_to_{target}",
        ),
    ]


def export_extrinsics(config_dir: Path, output_dir: Path) -> list[Path]:
    required = (
        "camera.yaml",
        "extrinsic_camera_lidar.yaml",
        "extrinsic_imu_lidar.yaml",
        "extrinsic_rtk.yaml",
    )
    missing = [name for name in required if not (config_dir / name).is_file()]
    if missing:
        raise FileNotFoundError(f"Missing calibration files: {', '.join(missing)}")
    output_dir.mkdir(parents=True, exist_ok=True)

    # Vendor direction semantics:
    #   extrinsic_imu_lidar.transform = T_imu_lidar
    #   extrinsic_camera_lidar.transform = T_camera0_lidar
    #   camera_i.camera_pose = T_camera0_camerai
    # where p_target = T_target_source * p_source. Camera 0/1/2 map to
    # left/right/center respectively in the vendor decoder.
    lidar_to_imu = matrix4(read_yaml_list(config_dir / "extrinsic_imu_lidar.yaml", "transform"))
    lidar_to_camera_left = matrix4(
        read_yaml_list(config_dir / "extrinsic_camera_lidar.yaml", "transform")
    )
    camera_to_left = {
        index: matrix4(values)
        for index, values in read_camera_poses(config_dir / "camera.yaml").items()
    }

    generated: list[Path] = []
    generated += write_pair(
        output_dir,
        "lidar",
        "imu",
        lidar_to_imu,
        "extrinsic_imu_lidar.yaml:transform",
        "vendor T_imu_lidar; confirmed by CorrectLidarAndTransformToImuFrame",
    )
    generated += write_pair(
        output_dir,
        "lidar",
        "camera_left",
        lidar_to_camera_left,
        "extrinsic_camera_lidar.yaml:transform",
        "vendor T_camera0_lidar; camera_0 is camera_left",
    )

    for index in (1, 2):
        camera_name = CAMERA_INDEX_TO_NAME[index]
        # camera_pose is T_camera_left_camera_i. Therefore:
        # T_camera_i_lidar = inverse(T_camera_left_camera_i) * T_camera_left_lidar.
        lidar_to_camera = multiply(
            inverse_rigid(camera_to_left[index]), lidar_to_camera_left
        )
        generated += write_pair(
            output_dir,
            "lidar",
            camera_name,
            lidar_to_camera,
            f"extrinsic_camera_lidar.yaml + camera.yaml:camera_{index}.camera_pose",
            f"derived T_{camera_name}_lidar; camera_pose is T_camera_left_{camera_name}",
        )
        generated += write_pair(
            output_dir,
            camera_name,
            "camera_left",
            camera_to_left[index],
            f"camera.yaml:camera_{index}.camera_pose",
            f"vendor camera_pose = T_camera_left_{camera_name}",
        )

    rtk_path = config_dir / "extrinsic_rtk.yaml"
    rtk_xyz = read_yaml_list(rtk_path, "xyz_custom")
    customized_match = re.search(
        r"(?m)^customized:\s*(true|false)\s*$",
        rtk_path.read_text(encoding="utf-8", errors="replace"),
    )
    customized = bool(customized_match and customized_match.group(1) == "true")
    if not customized:
        rtk_xyz = read_yaml_list(rtk_path, "xyz_default")
    degree_key = "degree_custom" if customized else "degree_default"
    degrees = read_yaml_list(rtk_path, degree_key)
    if any(abs(value) > 1e-12 for value in degrees):
        raise ValueError(
            "Non-zero RTK Euler angles require a documented vendor rotation order; "
            f"got {degrees}"
        )
    if len(rtk_xyz) != 3:
        raise ValueError(f"RTK offset has {len(rtk_xyz)} values, expected 3")
    rtk_to_imu = [
        [1.0, 0.0, 0.0, rtk_xyz[0]],
        [0.0, 1.0, 0.0, rtk_xyz[1]],
        [0.0, 0.0, 1.0, rtk_xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
    generated += write_pair(
        output_dir,
        "rtk_antenna",
        "imu",
        rtk_to_imu,
        f"extrinsic_rtk.yaml:{'xyz_custom' if customized else 'xyz_default'}",
        "RTK antenna origin expressed in IMU frame; zero vendor Euler angles",
    )

    readme = output_dir / "README.md"
    readme.write_text(
        """# 方向明确的传感器外参

文件名采用 `source_to_target.yaml`。所有文件均使用同一变换定义：

```text
p_target = R(q_xyzw) * p_source + t
```

- `x, y, z`：平移 `t`，单位为米，表示 source 原点在 target 坐标系中的位置；
- `qx, qy, qz, qw`：单位四元数，顺序为 XYZW；
- 每一组外参同时提供正向和逆向文件，使用时无需自行猜测或求逆；
- 原厂矩阵文件保留在 `../config`，本目录是其方向明确的派生表示。

## 相机编号

- `camera_0 = camera_left`
- `camera_1 = camera_right`
- `camera_2 = camera_center`

## 原厂矩阵方向

- `extrinsic_imu_lidar.yaml`：`T_imu_lidar`，即 `lidar_to_imu`；
- `extrinsic_camera_lidar.yaml`：`T_camera_left_lidar`，即 `lidar_to_camera_left`；
- `camera_i.camera_pose`：`T_camera_left_camera_i`，即 Camera-i 到左相机；
- `extrinsic_rtk.yaml`：RTK天线原点在IMU坐标系中的杆臂，当前数据旋转为零。

以上相机姿态方向还通过同步LiDAR投影进行了验证：正确方向能使绝大多数有效点
处于中心相机前方，反向解释则几乎全部位于相机后方。
""",
        encoding="utf-8",
    )
    generated.append(readme)
    return generated


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export direction-explicit Lixel extrinsics as x/y/z + qx/qy/qz/qw"
    )
    parser.add_argument("config_dir", type=Path, help="Directory containing vendor YAML files")
    parser.add_argument("output_dir", type=Path, nargs="?", help="Default: CONFIG_DIR/../extrinsics_xyzw")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir or args.config_dir.parent / "extrinsics_xyzw"
    generated = export_extrinsics(args.config_dir, output_dir)
    print(f"Generated {len(generated)} direction-explicit calibration file(s) in {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
