#!/usr/bin/env python3
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Generate the procedural Mars world, the rover model and the orbital map.

Outputs (under --out):
  models/mars_terrain/   terrain, sand and rock meshes + textures (Gazebo model)
  models/rover/          rocker-bogie rover model built from rover.yaml
  worlds/mars.sdf        world that includes both models
  orbital/               coarse, blurred height map the rover uses as its "orbital" data
"""

import argparse
import math
import os

import numpy as np
import yaml
from PIL import Image


# --------------------------------------------------------------------------- terrain


def spectral_noise(rng, n, slope, cutoff=None):
    """Zero-mean, unit-std periodic noise with power spectrum ~ f^-slope.

    cutoff (cycles per sample) rolls off wavelengths shorter than 1 / cutoff samples.
    """
    freqs = np.fft.fftfreq(n)
    f = np.sqrt(freqs[None, :] ** 2 + freqs[:, None] ** 2)
    f[0, 0] = 1.0
    amplitude = f ** (-slope / 2.0)
    if cutoff is not None:
        amplitude *= np.exp(-((f / cutoff) ** 2))
    amplitude[0, 0] = 0.0
    phase = rng.uniform(0.0, 2.0 * math.pi, (n, n))
    field = np.real(np.fft.ifft2(amplitude * np.exp(1j * phase)))
    return (field - field.mean()) / field.std()


def crater_profile(r, depth, rim):
    """Height of a simple crater at normalized radius r (1 = rim crest)."""
    inside = depth * (r**2 - 1.0) + rim
    outside = rim * np.power(np.maximum(r, 1.0), -6.0)
    return np.where(r < 1.0, inside, outside)


def build_heightfield(cfg, rng, xs):
    n = xs.size
    gx, gy = np.meshgrid(xs, xs)  # gx[j, i] = xs[i], gy[j, i] = xs[j]
    terrain = cfg["terrain"]
    res = cfg["resolution"]
    height = terrain["rms_height"] * spectral_noise(
        rng, n, terrain["spectral_slope"], res / terrain["min_wavelength"]
    ) + terrain["roughness"] * spectral_noise(rng, n, 1.0, res / 0.6)

    craters = cfg["craters"]
    half = cfg["size"] / 2.0
    spawn = cfg["spawn"]
    placed = 0
    while placed < craters["count"]:
        diameter = math.exp(rng.uniform(math.log(craters["min_diameter"]), math.log(craters["max_diameter"])))
        radius = diameter / 2.0
        cx, cy = rng.uniform(-half + radius, half - radius, 2)
        if math.hypot(cx - spawn["x"], cy - spawn["y"]) < radius * 2.0 + spawn["clear_radius"]:
            continue
        depth = diameter * rng.uniform(craters["min_depth_ratio"], craters["max_depth_ratio"])
        rim = diameter * craters["rim_ratio"]
        r = np.hypot(gx - cx, gy - cy) / radius
        height += crater_profile(r, depth, rim)
        placed += 1

    sand = np.zeros_like(height, dtype=bool)
    for dune in cfg["dunes"]:
        yaw = math.radians(dune["yaw_deg"])
        along = (gx - dune["x"]) * math.cos(yaw) + (gy - dune["y"]) * math.sin(yaw)
        across = -(gx - dune["x"]) * math.sin(yaw) + (gy - dune["y"]) * math.cos(yaw)
        half_length = dune["length"] / 2.0
        taper = np.exp(-((along / half_length) ** 4))
        height += dune["height"] * np.exp(-(across**2) / (2.0 * dune["sigma"] ** 2)) * taper
        sand |= (np.abs(across) < 2.5 * dune["sigma"]) & (np.abs(along) < half_length * 1.1)

    # Flatten a landing pad so the rover spawns on safe, level ground.
    d = np.hypot(gx - spawn["x"], gy - spawn["y"])
    pad = d < spawn["clear_radius"]
    pad_height = float(height[pad].mean())
    weight = np.clip((d - spawn["clear_radius"]) / spawn["clear_radius"], 0.0, 1.0)
    weight = weight * weight * (3.0 - 2.0 * weight)
    height = pad_height + (height - pad_height) * weight
    sand &= ~pad
    return height, sand, pad_height


def bilinear(field, x0, res, x, y):
    fx = (x - x0) / res
    fy = (y - x0) / res
    i = int(math.floor(fx))
    j = int(math.floor(fy))
    tx = fx - i
    ty = fy - j
    top = field[j, i] * (1 - tx) + field[j, i + 1] * tx
    bottom = field[j + 1, i] * (1 - tx) + field[j + 1, i + 1] * tx
    return top * (1 - ty) + bottom * ty


# ----------------------------------------------------------------------------- rocks


def golombek_counts(k, area, d_min, d_max, bins=14):
    """Expected rock counts per diameter bin from the Golombek rock-abundance model.

    Cumulative fractional area covered by rocks of diameter >= D: F(D) = k exp(-q D),
    q = 1.79 + 0.152 / k. Each bin's area is divided by the area of one rock.
    """
    q = 1.79 + 0.152 / k
    edges = np.geomspace(d_min, d_max, bins + 1)
    covered = k * (np.exp(-q * edges[:-1]) - np.exp(-q * edges[1:]))
    mid = np.sqrt(edges[:-1] * edges[1:])
    return edges, area * covered / (math.pi * mid**2 / 4.0)


def sample_rocks(rng, k, area, d_min, d_max, sample_xy):
    rocks = []
    edges, counts = golombek_counts(k, area, d_min, d_max)
    for b, expected in enumerate(counts):
        for _ in range(rng.poisson(expected)):
            diameter = math.exp(rng.uniform(math.log(edges[b]), math.log(edges[b + 1])))
            x, y = sample_xy()
            rocks.append((x, y, diameter))
    return rocks


def place_rocks(cfg, rng, sand, xs):
    rcfg = cfg["rocks"]
    half = cfg["size"] / 2.0 - 1.0
    res = cfg["resolution"]
    x0 = xs[0]
    spawn = cfg["spawn"]

    def in_field(x, y):
        return any(math.hypot(x - f["x"], y - f["y"]) < f["radius"] for f in rcfg["fields"])

    rocks = [
        r
        for r in sample_rocks(
            rng,
            rcfg["background_k"],
            (2 * half) ** 2,
            rcfg["min_diameter"],
            rcfg["max_diameter"],
            lambda: tuple(rng.uniform(-half, half, 2)),
        )
        if not in_field(r[0], r[1])
    ]
    for field in rcfg["fields"]:

        def in_disk(field=field):
            radius = field["radius"] * math.sqrt(rng.uniform())
            angle = rng.uniform(0.0, 2.0 * math.pi)
            return field["x"] + radius * math.cos(angle), field["y"] + radius * math.sin(angle)

        rocks += sample_rocks(
            rng,
            field["k"],
            math.pi * field["radius"] ** 2,
            rcfg["min_diameter"],
            rcfg["max_diameter"],
            in_disk,
        )
    rocks += [(b["x"], b["y"], b["diameter"]) for b in rcfg["boulders"]]

    def keep(x, y, diameter):
        if math.hypot(x - spawn["x"], y - spawn["y"]) < spawn["clear_radius"] + diameter:
            return False
        i = int(round((x - x0) / res))
        j = int(round((y - x0) / res))
        return not sand[j, i]

    return [r for r in rocks if keep(*r)]


def icosphere():
    t = (1.0 + math.sqrt(5.0)) / 2.0
    verts = [
        (-1, t, 0),
        (1, t, 0),
        (-1, -t, 0),
        (1, -t, 0),
        (0, -1, t),
        (0, 1, t),
        (0, -1, -t),
        (0, 1, -t),
        (t, 0, -1),
        (t, 0, 1),
        (-t, 0, -1),
        (-t, 0, 1),
    ]
    faces = [
        (0, 11, 5),
        (0, 5, 1),
        (0, 1, 7),
        (0, 7, 10),
        (0, 10, 11),
        (1, 5, 9),
        (5, 11, 4),
        (11, 10, 2),
        (10, 7, 6),
        (7, 1, 8),
        (3, 9, 4),
        (3, 4, 2),
        (3, 2, 6),
        (3, 6, 8),
        (3, 8, 9),
        (4, 9, 5),
        (2, 4, 11),
        (6, 2, 10),
        (8, 6, 7),
        (9, 8, 1),
    ]
    verts = [np.array(v, dtype=float) / np.linalg.norm(v) for v in verts]
    midpoints = {}

    def midpoint(a, b):
        key = (min(a, b), max(a, b))
        if key not in midpoints:
            m = verts[a] + verts[b]
            verts.append(m / np.linalg.norm(m))
            midpoints[key] = len(verts) - 1
        return midpoints[key]

    subdivided = []
    for a, b, c in faces:
        ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
        subdivided += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
    return np.array(verts), np.array(subdivided)


def rock_shape(rng, diameter):
    """Semi-axes (a, b, c) and center offset of a half-buried rock. Visible height ~ D / 2."""
    a = diameter / 2.0 * rng.uniform(0.85, 1.15)
    b = diameter / 2.0 * rng.uniform(0.7, 1.05)
    c = 0.35 * diameter * rng.uniform(0.85, 1.15)
    return a, b, c, 0.15 * diameter


def build_rock_mesh(rng, rocks, height, x0, res):
    unit_verts, unit_faces = icosphere()
    all_verts, all_faces, shapes = [], [], []
    for x, y, diameter in rocks:
        a, b, c, lift = rock_shape(rng, diameter)
        yaw = rng.uniform(0.0, 2.0 * math.pi)
        bumps = 1.0 + 0.12 * rng.standard_normal(len(unit_verts))
        local = unit_verts * bumps[:, None] * np.array([a, b, c])
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        world = np.empty_like(local)
        world[:, 0] = x + cos_y * local[:, 0] - sin_y * local[:, 1]
        world[:, 1] = y + sin_y * local[:, 0] + cos_y * local[:, 1]
        zc = bilinear(height, x0, res, x, y) + lift
        world[:, 2] = zc + local[:, 2]
        all_faces.append(unit_faces + sum(len(v) for v in all_verts))
        all_verts.append(world)
        shapes.append((x, y, zc, a, b, c, yaw))
    return np.concatenate(all_verts), np.concatenate(all_faces), shapes


# ----------------------------------------------------------------------------- meshes


def vertex_normals(verts, faces):
    normals = np.zeros_like(verts)
    tri = verts[faces]
    face_normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    for k in range(3):
        np.add.at(normals, faces[:, k], face_normals)
    return normals / np.linalg.norm(normals, axis=1, keepdims=True)


def write_obj(path, material, verts, uvs, normals, faces):
    """Write a single-material OBJ, keeping only the vertices the faces use."""
    used, remap = np.unique(faces, return_inverse=True)
    faces = remap.reshape(faces.shape) + 1
    name = os.path.splitext(os.path.basename(path))[0]
    with open(path, "w") as f:
        f.write(f"mtllib {material}.mtl\no {name}\n")
        np.savetxt(f, verts[used], fmt="v %.4f %.4f %.4f")
        np.savetxt(f, uvs[used], fmt="vt %.4f %.4f")
        np.savetxt(f, normals[used], fmt="vn %.4f %.4f %.4f")
        f.write(f"usemtl {material}\n")
        np.savetxt(f, np.repeat(faces, 3, axis=1), fmt="f %d/%d/%d %d/%d/%d %d/%d/%d")


def write_mtl(path, material, texture):
    with open(path, "w") as f:
        f.write(f"newmtl {material}\nKa 0.6 0.6 0.6\nKd 1.0 1.0 1.0\nKs 0.03 0.03 0.03\nNs 10\nmap_Kd {texture}\n")


def terrain_faces(n):
    """Two triangles per grid quad, plus each triangle's quad tile index (see TILE_QUADS)."""
    i, j = np.meshgrid(np.arange(n - 1), np.arange(n - 1))
    v00 = (j * n + i).ravel()
    v10 = v00 + 1
    v01 = v00 + n
    v11 = v01 + 1
    faces = np.stack([np.stack([v00, v10, v11], 1), np.stack([v00, v11, v01], 1)], 1).reshape(-1, 3)
    tiles_per_row = (n - 2) // TILE_QUADS + 1
    tile = ((j // TILE_QUADS) * tiles_per_row + i // TILE_QUADS).ravel()
    return faces, np.repeat(tile, 2)


# The renderer indexes mesh vertices with 16 bits, so every mesh file must stay under 65536
# vertices; the terrain is split into tiles of TILE_QUADS x TILE_QUADS quads.
TILE_QUADS = 100


# --------------------------------------------------------------------------- textures


def periodic_noise(rng, n, slope):
    return np.clip(spectral_noise(rng, n, slope), -2.5, 2.5) / 2.5


def stamp_pebbles(rng, image, count, min_r, max_r, strength):
    n = image.shape[0]
    for _ in range(count):
        r = int(rng.integers(min_r, max_r + 1))
        cx, cy = rng.integers(0, n, 2)
        shade = rng.uniform(-strength, strength)
        yy, xx = np.mgrid[-r : r + 1, -r : r + 1]
        disk = (xx**2 + yy**2 <= r * r).astype(float)
        rows = (cy + yy) % n
        cols = (cx + xx) % n
        image[rows, cols] += shade * disk


def make_texture(rng, n, base_rgb, contrast, slope, pebbles, ripple=0.0):
    tone = contrast * periodic_noise(rng, n, slope)
    if ripple > 0.0:
        x = np.arange(n) / n
        tone += ripple * np.sin(2.0 * math.pi * 24.0 * (x[None, :] + 0.05 * np.sin(2.0 * math.pi * 3.0 * x[:, None])))
    if pebbles:
        stamp_pebbles(rng, tone, pebbles, 1, max(2, n // 400), 0.35)
    rgb = np.clip(np.array(base_rgb)[None, None, :] * (1.0 + tone[:, :, None]), 0.0, 1.0)
    return Image.fromarray((rgb * 255).astype(np.uint8))


# ------------------------------------------------------------------------------- sdf


def inertia_box(mass, sx, sy, sz):
    return (mass * (sy * sy + sz * sz) / 12.0, mass * (sx * sx + sz * sz) / 12.0, mass * (sx * sx + sy * sy) / 12.0)


def inertial_xml(mass, ixx, iyy, izz):
    return (
        f"<inertial><mass>{mass}</mass><inertia><ixx>{ixx:.5f}</ixx><ixy>0</ixy><ixz>0</ixz>"
        f"<iyy>{iyy:.5f}</iyy><iyz>0</iyz><izz>{izz:.5f}</izz></inertia></inertial>"
    )


def beam_visual(name, p0, p1, y, color):
    """Thin box from p0 to p1 (both (x, z) in the link frame) at lateral offset y."""
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dz)
    pitch = math.atan2(-dz, dx)
    mx, mz = (p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0
    return (
        f"<visual name='{name}'><pose>{mx:.4f} {y:.4f} {mz:.4f} 0 {pitch:.4f} 0</pose>"
        f"<geometry><box><size>{length:.4f} 0.04 0.05</size></box></geometry>"
        f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>"
    )


def rover_sdf(rover):
    """Rocker-bogie rover. Every link origin sits on its parent joint, so joint poses are zero."""
    wheel = rover["wheel"]
    radius, width = wheel["radius"], wheel["width"]
    half_track = rover["track_width"] / 2.0
    beam_y = half_track - width / 2.0 - 0.04
    rocker, bogie, body = rover["rocker"], rover["bogie"], rover["body"]
    pivot_x = rocker["pivot_x"]
    bogie_x = pivot_x + rocker["bogie_pivot"][0]
    bogie_z = rocker["bogie_pivot"][1]
    wheel_z = bogie_z - bogie["drop"]
    assert abs(wheel_z - rocker["front_wheel"][1]) < 1e-9, "front wheel and bogie wheels must share a height"
    wheels = {
        "front": (pivot_x + rocker["front_wheel"][0], "rocker"),
        "middle": (bogie_x + bogie["half_length"], "bogie"),
        "rear": (bogie_x - bogie["half_length"], "bogie"),
    }
    gray, dark = "0.8 0.8 0.78 1", "0.15 0.15 0.15 1"
    wheel_inertia = (
        wheel["mass"] * (3 * radius**2 + width**2) / 12.0,
        wheel["mass"] * radius**2 / 2.0,
        wheel["mass"] * (3 * radius**2 + width**2) / 12.0,
    )
    cam = rover["navcam"]
    imu = rover["imu"]
    # The camera head box sits just behind the lens so the camera never sees its own housing.
    head_x = cam["x"] - 0.07 * math.cos(cam["pitch"])
    head_z = cam["z"] + 0.07 * math.sin(cam["pitch"])

    def noise(stddev, bias=0.0):
        return (
            f"<noise type='gaussian'><mean>0</mean><stddev>{stddev}</stddev>"
            f"<bias_mean>{bias}</bias_mean><bias_stddev>{bias / 2.0}</bias_stddev></noise>"
        )

    gyro = "".join(f"<{a}>{noise(imu['gyro_noise'], imu['gyro_bias'])}</{a}>" for a in "xyz")
    accel = "".join(f"<{a}>{noise(imu['accel_noise'])}</{a}>" for a in "xyz")
    links = [
        f"""
    <link name='base_link'>
      {inertial_xml(body["mass"], *inertia_box(body["mass"], body["length"], body["width"], body["height"]))}
      <visual name='body'><pose>0 0 {body["center_z"]} 0 0 0</pose>
        <geometry><box><size>{body["length"]} {body["width"]} {body["height"]}</size></box></geometry>
        <material><ambient>{gray}</ambient><diffuse>{gray}</diffuse></material></visual>
      <collision name='body'><pose>0 0 {body["center_z"]} 0 0 0</pose>
        <geometry><box><size>{body["length"]} {body["width"]} {body["height"]}</size></box></geometry></collision>
      <visual name='mast'><pose>{cam["x"]} 0 {cam["z"] / 2.0} 0 0 0</pose>
        <geometry><cylinder><radius>0.025</radius><length>{cam["z"]}</length></cylinder></geometry>
        <material><ambient>{gray}</ambient><diffuse>{gray}</diffuse></material></visual>
      <visual name='navcam_head'><pose>{head_x:.4f} 0 {head_z:.4f} 0 {cam["pitch"]} 0</pose>
        <geometry><box><size>0.08 0.24 0.07</size></box></geometry>
        <material><ambient>{dark}</ambient><diffuse>{dark}</diffuse></material></visual>
      <sensor name='imu' type='imu'>
        <always_on>1</always_on><update_rate>{imu["rate"]}</update_rate><topic>imu</topic>
        <imu><angular_velocity>{gyro}</angular_velocity><linear_acceleration>{accel}</linear_acceleration></imu>
      </sensor>
      <sensor name='navcam' type='rgbd_camera'>
        <pose>{cam["x"]} 0 {cam["z"]} 0 {cam["pitch"]} 0</pose>
        <always_on>1</always_on><update_rate>{cam["rate"]}</update_rate><topic>navcam</topic>
        <camera>
          <horizontal_fov>{cam["horizontal_fov"]}</horizontal_fov>
          <image><width>{cam["width"]}</width><height>{cam["height"]}</height><format>R8G8B8</format></image>
          <clip><near>{cam["near"]}</near><far>{cam["far"]}</far></clip>
          <depth_camera><clip><near>{cam["near"]}</near><far>{cam["far"]}</far></clip></depth_camera>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.005</stddev></noise>
        </camera>
      </sensor>
    </link>"""
    ]
    joints = []
    for side, sign in (("left", 1.0), ("right", -1.0)):
        y = sign * beam_y
        rocker_mass = rocker["mass"]
        links.append(
            f"""
    <link name='rocker_{side}'>
      <pose>{pivot_x} {y:.4f} 0 0 0 0</pose>
      {inertial_xml(rocker_mass, *inertia_box(rocker_mass, 0.7, 0.04, 0.3))}
      {beam_visual("front_arm", (0.0, 0.0), rocker["front_wheel"], 0.0, gray)}
      {beam_visual("rear_arm", (0.0, 0.0), rocker["bogie_pivot"], 0.0, gray)}
    </link>"""
        )
        links.append(
            f"""
    <link name='bogie_{side}'>
      <pose>{bogie_x} {y:.4f} {bogie_z} 0 0 0</pose>
      {inertial_xml(bogie["mass"], *inertia_box(bogie["mass"], 0.5, 0.04, 0.15))}
      {beam_visual("front_arm", (0.0, 0.0), (bogie["half_length"], -bogie["drop"]), 0.0, gray)}
      {beam_visual("rear_arm", (0.0, 0.0), (-bogie["half_length"], -bogie["drop"]), 0.0, gray)}
    </link>"""
        )
        joints.append(
            f"""
    <joint name='rocker_{side}_joint' type='revolute'>
      <parent>base_link</parent><child>rocker_{side}</child>
      <axis><xyz>0 1 0</xyz><limit><lower>{-rocker["joint_limit"]}</lower><upper>{rocker["joint_limit"]}</upper></limit>
        <dynamics><damping>2.0</damping></dynamics></axis>
    </joint>
    <joint name='bogie_{side}_joint' type='revolute'>
      <parent>rocker_{side}</parent><child>bogie_{side}</child>
      <axis><xyz>0 1 0</xyz><limit><lower>{-bogie["joint_limit"]}</lower><upper>{bogie["joint_limit"]}</upper></limit>
        <dynamics><damping>1.0</damping></dynamics></axis>
    </joint>"""
        )
        for position, (wx, parent) in wheels.items():
            name = f"wheel_{position}_{side}"
            links.append(
                f"""
    <link name='{name}'>
      <pose>{wx:.4f} {sign * half_track:.4f} {wheel_z:.4f} 0 0 0</pose>
      {inertial_xml(wheel["mass"], *wheel_inertia)}
      <visual name='tire'><pose>0 0 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>{radius}</radius><length>{width}</length></cylinder></geometry>
        <material><ambient>{dark}</ambient><diffuse>{dark}</diffuse></material></visual>
      <visual name='hub'><pose>0 {sign * (width / 2.0 + 0.002):.4f} 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>{radius * 0.5}</radius><length>0.004</length></cylinder></geometry>
        <material><ambient>{gray}</ambient><diffuse>{gray}</diffuse></material></visual>
      <collision name='tire'><pose>0 0 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>{radius}</radius><length>{width}</length></cylinder></geometry>
        <surface><friction><ode><mu>{wheel["friction"]}</mu><mu2>{wheel["friction"]}</mu2></ode></friction></surface>
      </collision>
    </link>"""
            )
            joints.append(
                f"""
    <joint name='{name}_joint' type='revolute'>
      <parent>{parent}_{side}</parent><child>{name}</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>"""
            )
    drive = rover["drive"]
    left = "".join(f"<left_joint>wheel_{p}_left_joint</left_joint>" for p in wheels)
    right = "".join(f"<right_joint>wheel_{p}_right_joint</right_joint>" for p in wheels)
    diff = rover["differential"]
    plugins = f"""
    <plugin filename='ignition-gazebo-diff-drive-system' name='ignition::gazebo::systems::DiffDrive'>
      {left}{right}
      <wheel_separation>{rover["track_width"]}</wheel_separation>
      <wheel_radius>{radius}</wheel_radius>
      <max_linear_acceleration>{drive["max_accel"]}</max_linear_acceleration>
      <min_linear_acceleration>{-drive["max_accel"]}</min_linear_acceleration>
      <max_angular_acceleration>2.0</max_angular_acceleration>
      <min_angular_acceleration>-2.0</min_angular_acceleration>
      <odom_publish_frequency>1</odom_publish_frequency>
      <topic>/cmd_vel</topic>
    </plugin>
    <plugin filename='ignition-gazebo-joint-state-publisher-system'
            name='ignition::gazebo::systems::JointStatePublisher'/>
    <plugin filename='ignition-gazebo-odometry-publisher-system'
            name='ignition::gazebo::systems::OdometryPublisher'>
      <odom_frame>map</odom_frame>
      <robot_base_frame>rover</robot_base_frame>
      <odom_publish_frequency>20</odom_publish_frequency>
      <odom_topic>/sim/ground_truth</odom_topic>
      <dimensions>3</dimensions>
    </plugin>
    <plugin filename='RockerDifferential' name='rover_sim::RockerDifferential'>
      <left_joint>rocker_left_joint</left_joint>
      <right_joint>rocker_right_joint</right_joint>
      <stiffness>{diff["stiffness"]}</stiffness>
      <damping>{diff["damping"]}</damping>
    </plugin>"""
    return (
        "<?xml version='1.0'?>\n<sdf version='1.8'>\n  <model name='rover'>"
        + "".join(links)
        + "".join(joints)
        + plugins
        + "\n  </model>\n</sdf>\n"
    )


def terrain_sdf(surfaces, friction):
    """One collision + visual per mesh file; friction by surface kind (soil, sand, rock)."""
    mu_of = {"terrain": friction["soil"], "sand": friction["sand"], "rock": friction["rock"]}
    parts = []
    for name, kind in surfaces:
        mu = mu_of[kind]
        uri = f"model://mars_terrain/meshes/{name}.obj"
        parts.append(
            f"""
      <collision name='{name}'><geometry><mesh><uri>{uri}</uri></mesh></geometry>
        <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>
      <visual name='{name}'><geometry><mesh><uri>{uri}</uri></mesh></geometry></visual>"""
        )
    return (
        "<?xml version='1.0'?>\n<sdf version='1.8'>\n  <model name='mars_terrain'>\n    <static>true</static>\n"
        "    <link name='ground'>" + "".join(parts) + "\n    </link>\n  </model>\n</sdf>\n"
    )


def world_sdf(cfg, spawn_z):
    sun = cfg["sun"]
    az, el = math.radians(sun["azimuth_deg"]), math.radians(sun["elevation_deg"])
    to_sun = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
    spawn = cfg["spawn"]
    yaw = math.radians(spawn["yaw_deg"])
    plugins = "".join(
        f"\n    <plugin filename='ignition-gazebo-{f}-system' name='ignition::gazebo::systems::{n}'>{extra}</plugin>"
        for f, n, extra in (
            ("physics", "Physics", ""),
            ("user-commands", "UserCommands", ""),
            ("scene-broadcaster", "SceneBroadcaster", ""),
            ("sensors", "Sensors", "<render_engine>ogre2</render_engine>"),
            ("imu", "Imu", ""),
        )
    )
    return f"""<?xml version='1.0'?>
<sdf version='1.8'>
  <world name='mars'>
    <physics name='2ms' type='ignored'>
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 {-cfg["gravity"]}</gravity>{plugins}
    <scene>
      <ambient>0.45 0.38 0.33 1</ambient>
      <background>0.78 0.6 0.45 1</background>
      <shadows>true</shadows>
    </scene>
    <light type='directional' name='sun'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>1.0 0.95 0.88 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>{-to_sun[0]:.5f} {-to_sun[1]:.5f} {-to_sun[2]:.5f}</direction>
    </light>
    <include><uri>model://mars_terrain</uri></include>
    <include>
      <uri>model://rover</uri>
      <pose>{spawn["x"]} {spawn["y"]} {spawn_z:.4f} 0 0 {yaw:.5f}</pose>
    </include>
  </world>
</sdf>
"""


def write_model_config(directory, name):
    with open(os.path.join(directory, "model.config"), "w") as f:
        f.write(
            f"<?xml version='1.0'?>\n<model>\n  <name>{name}</name>\n  <version>1.0</version>\n"
            f"  <sdf version='1.8'>model.sdf</sdf>\n  <description>Generated by gen_world.py</description>\n</model>\n"
        )


# --------------------------------------------------------------------------- orbital


def gaussian_blur(field, sigma_px):
    radius = int(math.ceil(3 * sigma_px))
    kernel = np.exp(-(np.arange(-radius, radius + 1) ** 2) / (2 * sigma_px**2))
    kernel /= kernel.sum()
    padded = np.pad(field, radius, mode="edge")
    rows = np.apply_along_axis(lambda r: np.convolve(r, kernel, mode="valid"), 1, padded)
    return np.apply_along_axis(lambda c: np.convolve(c, kernel, mode="valid"), 0, rows)


def orbital_dem(cfg, rng, height, xs, shapes):
    """Coarse DEM: terrain plus rock tops, blurred and noisy like an orbital product."""
    orb = cfg["orbital"]
    res = orb["resolution"]
    n = int(round(cfg["size"] / res))
    centers = xs[0] + (np.arange(n) + 0.5) * res
    gx, gy = np.meshgrid(centers, centers)
    fx = (gx - xs[0]) / cfg["resolution"]
    fy = (gy - xs[0]) / cfg["resolution"]
    i, j = np.floor(fx).astype(int), np.floor(fy).astype(int)
    tx, ty = fx - i, fy - j
    dem = (
        height[j, i] * (1 - tx) * (1 - ty)
        + height[j, i + 1] * tx * (1 - ty)
        + height[j + 1, i] * (1 - tx) * ty
        + height[j + 1, i + 1] * tx * ty
    )
    for x, y, zc, a, b, c, yaw in shapes:
        lx = (gx - x) * math.cos(yaw) + (gy - y) * math.sin(yaw)
        ly = -(gx - x) * math.sin(yaw) + (gy - y) * math.cos(yaw)
        inside = 1.0 - (lx / a) ** 2 - (ly / b) ** 2
        top = zc + c * np.sqrt(np.clip(inside, 0.0, None))
        dem = np.where(inside > 0.0, np.maximum(dem, top), dem)
    dem = gaussian_blur(dem, orb["blur_sigma"] / res) + orb["noise_std"] * rng.standard_normal(dem.shape)
    return dem.astype(np.float32), float(xs[0])


# ------------------------------------------------------------------------------ main


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rover", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    with open(args.rover) as f:
        rover = yaml.safe_load(f)
    with open(args.world) as f:
        cfg = yaml.safe_load(f)
    rng = np.random.default_rng(cfg["seed"])

    n = int(round(cfg["size"] / cfg["resolution"])) + 1
    xs = -cfg["size"] / 2.0 + np.arange(n) * cfg["resolution"]
    height, sand, pad_height = build_heightfield(cfg, rng, xs)
    rocks = place_rocks(cfg, rng, sand, xs)
    rock_verts, rock_faces, shapes = build_rock_mesh(rng, rocks, height, xs[0], cfg["resolution"])

    terrain_dir = os.path.join(args.out, "models", "mars_terrain")
    mesh_dir = os.path.join(terrain_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)

    gx, gy = np.meshgrid(xs, xs)
    verts = np.stack([gx.ravel(), gy.ravel(), height.ravel()], 1)
    faces, face_tile = terrain_faces(n)
    normals = vertex_normals(verts, faces)
    centroid_sand = sand.ravel()[faces].sum(1) >= 2
    tex = cfg["textures"]
    surfaces = []  # (mesh name, surface kind)
    for kind, mask, texture_tile in (
        ("terrain", ~centroid_sand, tex["soil_tile"]),
        ("sand", centroid_sand, tex["sand_tile"]),
    ):
        write_mtl(os.path.join(mesh_dir, f"{kind}.mtl"), kind, f"{kind}.jpg")
        for t in np.unique(face_tile[mask]):
            name = f"{kind}_{t:02d}"
            tile_faces = faces[mask & (face_tile == t)]
            write_obj(os.path.join(mesh_dir, f"{name}.obj"), kind, verts, verts[:, :2] / texture_tile, normals, tile_faces)
            surfaces.append((name, kind))
    rock_uv = np.stack([rock_verts[:, 0] + rock_verts[:, 1], rock_verts[:, 2]], 1) / 0.6
    write_obj(
        os.path.join(mesh_dir, "rocks.obj"),
        "rocks",
        rock_verts,
        rock_uv,
        vertex_normals(rock_verts, rock_faces),
        rock_faces,
    )
    write_mtl(os.path.join(mesh_dir, "rocks.mtl"), "rocks", "rocks.jpg")
    assert len(rock_verts) < 65536, "rocks.obj needs splitting: the renderer uses 16-bit vertex indices"
    surfaces.append(("rocks", "rock"))

    make_texture(rng, tex["soil_px"], (0.62, 0.40, 0.26), 0.30, 1.4, 6000).save(
        os.path.join(mesh_dir, "terrain.jpg"), quality=90
    )
    make_texture(rng, tex["sand_px"], (0.76, 0.57, 0.40), 0.06, 1.8, 0, ripple=0.04).save(
        os.path.join(mesh_dir, "sand.jpg"), quality=90
    )
    make_texture(rng, tex["rock_px"], (0.42, 0.34, 0.29), 0.28, 1.2, 400).save(
        os.path.join(mesh_dir, "rocks.jpg"), quality=90
    )
    with open(os.path.join(terrain_dir, "model.sdf"), "w") as f:
        f.write(terrain_sdf(surfaces, cfg["friction"]))
    write_model_config(terrain_dir, "mars_terrain")

    rover_dir = os.path.join(args.out, "models", "rover")
    os.makedirs(rover_dir, exist_ok=True)
    with open(os.path.join(rover_dir, "model.sdf"), "w") as f:
        f.write(rover_sdf(rover))
    write_model_config(rover_dir, "rover")

    pivot_height = rover["wheel"]["radius"] - rover["rocker"]["front_wheel"][1]
    os.makedirs(os.path.join(args.out, "worlds"), exist_ok=True)
    with open(os.path.join(args.out, "worlds", "mars.sdf"), "w") as f:
        f.write(world_sdf(cfg, pad_height + pivot_height + 0.03))

    dem, origin = orbital_dem(cfg, rng, height, xs, shapes)
    orbital_dir = os.path.join(args.out, "orbital")
    os.makedirs(orbital_dir, exist_ok=True)
    dem.tofile(os.path.join(orbital_dir, "orbital_dem.f32"))
    with open(os.path.join(orbital_dir, "orbital_map.yaml"), "w") as f:
        yaml.safe_dump(
            {
                "resolution": cfg["orbital"]["resolution"],
                "width": int(dem.shape[1]),
                "height": int(dem.shape[0]),
                "origin_x": origin,
                "origin_y": origin,
                "file": "orbital_dem.f32",
            },
            f,
        )
    print(
        f"terrain {n}x{n} vertices, {len(rocks)} rocks, sand cells {int(sand.sum())}, "
        f"spawn ground z {pad_height:.3f}, orbital {dem.shape[1]}x{dem.shape[0]}"
    )


if __name__ == "__main__":
    main()
