import ifcopenshell
import ifcopenshell.geom
import csv
import math
import os
import numpy as np
import sys

# ---------------------------------------------------------------------------
# global adjustables
# ---------------------------------------------------------------------------
VERTICAL_ADHESIVE_GAP = 0.0
ACTUAL_BRICK_HEIGHT   = 39.5

# ---------------------------------------------------------------------------
# brick physical dimensions
# ---------------------------------------------------------------------------
BRICK_LONG  = 127.0
BRICK_SHORT = 59.18


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------
def get_brick_metrics(brick, verts):
    nodes    = np.array([verts[i:i+3] for i in range(0, len(verts), 3)])
    points_2d = nodes[:, :2]
    centroid  = np.mean(points_2d, axis=0)
    adjusted  = points_2d - centroid

    cov = np.cov(adjusted.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov)

    long_axis = eigenvectors[:, np.argmax(eigenvalues)]
    angle_rad = math.atan2(long_axis[1], long_axis[0])
    angle_deg = math.degrees(angle_rad) % 180

    # 0 deg = long axis along Y
    final_theta = (angle_deg - 90.0) % 180
    if final_theta > 90:
        final_theta -= 180

    try:
        obj_name = brick.Name if brick.Name else ""
        if "_FLIP" in obj_name.upper():
            final_theta = -final_theta
    except:
        pass

    return round(final_theta, 2), centroid * 1000.0, np.min(nodes[:, 2]) * 1000.0


def half_extent_x(theta):
    # projected brick half-width in global X
    rad = math.radians(theta)
    return abs(math.cos(rad)) * (BRICK_SHORT / 2) + abs(math.sin(rad)) * (BRICK_LONG / 2)


def half_extent_y(theta):
    # projected brick half-width in global Y
    rad = math.radians(theta)
    return abs(math.sin(rad)) * (BRICK_LONG / 2) + abs(math.cos(rad)) * (BRICK_SHORT / 2)


# ---------------------------------------------------------------------------
# main translation
# ---------------------------------------------------------------------------
def extract_bricks_to_csv(ifc_file_path, output_csv):
    if not os.path.exists(ifc_file_path):
        print(f"Error: {ifc_file_path} not found.")
        return

    model = ifcopenshell.open(ifc_file_path)

    brick_types = ["IfcMember", "IfcWall", "IfcBuildingElementProxy", "IfcWallStandardCase"]
    bricks = []
    for t in brick_types:
        bricks.extend(model.by_type(t))

    # physical constants
    X_MAX_LIMIT             = 700.0
    Z_MAX_LIMIT             = 1000.0
    JAW_LX_THRESHOLD        = 110.0
    BODY_LY_THRESHOLD       = 75.0
    CLAW_VERTICAL_LIFT      = 16.0
    X_OVERSHOOT_DISTANCE    = 10.0
    X_TIGHT_GAP_THRESHOLD   = 5.0

    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)

    raw_data = []
    for brick in bricks:
        try:
            shape = ifcopenshell.geom.create_shape(settings, brick)
            theta, center, z_bot = get_brick_metrics(brick, shape.geometry.verts)
            raw_data.append({
                'x': center[0], 'y': center[1], 'z': z_bot,
                'theta': theta, 'brick_obj': brick
            })
        except:
            continue

    if not raw_data:
        print("No valid geometry found.")
        return

    min_x = min(b['x'] for b in raw_data)
    min_y = min(b['y'] for b in raw_data)
    min_z = min(b['z'] for b in raw_data)

    normalized_data = []
    for b in raw_data:
        nx      = round(b['x'] - min_x, 2)
        ny      = math.floor((b['y'] - min_y) * 100) / 100  # floor to 0.01mm
        nz_raw  = b['z'] - min_z

        layer_index = int(round(nz_raw / ACTUAL_BRICK_HEIGHT))
        nz_final    = round(layer_index * ACTUAL_BRICK_HEIGHT, 3)

        if nx <= X_MAX_LIMIT and nz_raw <= Z_MAX_LIMIT:
            normalized_data.append({
                'x': nx, 'y': ny, 'z': nz_final,
                'theta': b['theta'],
                'drop_offset': 0.0,
                'x_overshoot': 0.0,
                'layer': layer_index
            })

    normalized_data.sort(key=lambda b: (round(b['z'], 1), round(b['y'], 1), round(b['x'], 1)))

    for i, b in enumerate(normalized_data):
        b['brick_id'] = f"B{i + 1}"

    # ---------------------------------------------------------------------------
    # neighbor detection
    # ---------------------------------------------------------------------------
    for i, b1 in enumerate(normalized_data):
        rad   = math.radians(b1['theta'])
        cos_t = math.cos(rad)
        sin_t = math.sin(rad)

        for j in range(i):
            b2 = normalized_data[j]
            if b1['layer'] != b2['layer']:
                continue

            dx = b2['x'] - b1['x']
            dy = b2['y'] - b1['y']

            # drop offset -- jaw envelope check
            if b1['drop_offset'] == 0.0:
                lx = dx * cos_t + dy * sin_t
                ly = -dx * sin_t + dy * cos_t
                if abs(lx) < JAW_LX_THRESHOLD and abs(ly) < BODY_LY_THRESHOLD:
                    b1['drop_offset'] = CLAW_VERTICAL_LIFT

            # x overshoot -- tight X gap check
            if b1['x_overshoot'] == 0.0:
                x_clearance = abs(dx) - half_extent_x(b1['theta']) - half_extent_x(b2['theta'])
                if x_clearance < X_TIGHT_GAP_THRESHOLD:
                    b1['x_overshoot'] = X_OVERSHOOT_DISTANCE

    # ---------------------------------------------------------------------------
    # export
    # ---------------------------------------------------------------------------
    output_dir = os.path.dirname(output_csv)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    fieldnames = ['brick_id', 'x', 'y', 'z', 'theta', 'drop_offset', 'x_overshoot']
    with open(output_csv, mode='w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(normalized_data)

    flagged   = sum(1 for b in normalized_data if b['drop_offset'] > 0)
    x_flagged = sum(1 for b in normalized_data if b['x_overshoot'] > 0)

    print("-" * 40)
    print(f"TRANSLATION COMPLETE")
    print(f"Input:             {os.path.basename(ifc_file_path)}")
    print(f"Output:            {os.path.basename(output_csv)}")
    print(f"Bricks in Build:   {len(normalized_data)}")
    print(f"Drop-offset flags: {flagged}")
    print(f"X-overshoot flags: {x_flagged}")
    print("-" * 40)


if __name__ == "__main__":
    if len(sys.argv) > 2:
        extract_bricks_to_csv(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1:
        extract_bricks_to_csv(sys.argv[1], "output_csvs/current_build.csv")