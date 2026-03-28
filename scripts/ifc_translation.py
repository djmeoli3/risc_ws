import ifcopenshell
import ifcopenshell.geom
import csv
import math
import os
import numpy as np

def get_brick_metrics(brick, verts):
    """
    Calculates rotation using PCA (geometry-only) and checks for manual 180-degree flip.
    """
    #convert flat list to (x, y, z) tuples
    nodes = np.array([verts[i:i+3] for i in range(0, len(verts), 3)])
    
    #2D footprint for rotation
    points_2d = nodes[:, :2]
    centroid = np.mean(points_2d, axis=0)
    adjusted_points = points_2d - centroid
    
    #pca to find the principal axis (length of brick)
    cov = np.cov(adjusted_points.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov)
    
    #eigenvector associated with the largest eigenvalue is the long axis
    long_axis = eigenvectors[:, np.argmax(eigenvalues)]
    angle_rad = math.atan2(long_axis[1], long_axis[0])
    angle_deg = math.degrees(angle_rad) % 180 
    
    #normalize to hardware: 0 deg = parallel to y-axis
    final_theta = (angle_deg - 90.0) % 180
    if final_theta > 90: final_theta -= 180 #standardize [-90, 90] range

    # ---VT LOGO BUILD ---
    #manually flip the face 180 degrees if the brick is named with '_FLIP'
    try:
        obj_name = brick.Name if brick.Name else ""
        if "_FLIP" in obj_name.upper():
            if final_theta <= 0:
                final_theta += 180
            else:
                final_theta -= 180
    except:
        pass

    return round(final_theta, 2), centroid * 1000.0, np.min(nodes[:, 2]) * 1000.0

def extract_bricks_to_csv(ifc_file_path, output_csv):
    if not os.path.exists(ifc_file_path):
        print(f"Error: {ifc_file_path} not found.")
        return

    model = ifcopenshell.open(ifc_file_path)
    
    #targets standard architectural types, type agnostic
    brick_types = ["IfcMember", "IfcWall", "IfcBuildingElementProxy", "IfcWallStandardCase"]
    bricks = []
    for t in brick_types:
        bricks.extend(model.by_type(t))
    
    # --- BUILD VOLUME LIMITS & CONSTANTS ---
    X_MAX_LIMIT = 800.0
    Z_MAX_LIMIT = 1000.0
    CLAW_OPEN_WIDTH_MM = 105.0  
    CLAW_VERTICAL_LIFT = 20.0   
    
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True) 
    
    raw_data = []
    for brick in bricks:
        try:
            shape = ifcopenshell.geom.create_shape(settings, brick)
            theta, center, z_bot = get_brick_metrics(brick, shape.geometry.verts)
            raw_data.append({
                'x': center[0], 
                'y': center[1], 
                'z': z_bot, 
                'theta': theta,
                'brick_obj': brick 
            })
        except:
            continue

    if not raw_data:
        print("No valid geometry found.")
        return

    # --- BOUNDING BOX NORMALIZATION ---
    #ensures all coordinates start at (0,0,0) and remain positive
    min_x = min(b['x'] for b in raw_data)
    min_y = min(b['y'] for b in raw_data)
    min_z = min(b['z'] for b in raw_data)
    
    normalized_data = []
    for b in raw_data:
        nx = round(b['x'] - min_x, 3)
        ny = round(b['y'] - min_y, 3)
        nz = round(b['z'] - min_z, 3)
        
        # --- VOLUME CLIPPING ---
        #discard bricks that exceed the gantry's physical limits
        if nx <= X_MAX_LIMIT and nz <= Z_MAX_LIMIT:
            normalized_data.append({
                'x': nx,
                'y': ny,
                'z': nz,
                'theta': b['theta'],
                'drop_offset': 0.0
            })

    cut_count = len(raw_data) - len(normalized_data)

    # --- SORT BUILD ORDER ---
    # Z -> X -> Y
    normalized_data.sort(key=lambda b: (round(b['z'], 1), round(b['x'], 1), round(b['y'], 1)))
 
    # --- ROTATIONAL NEIGHBOR DETECTION ---
    #determines if the gripper needs a vertical clearance lift (drop_offset)
    for i, b1 in enumerate(normalized_data):
        b1['brick_id'] = f"B{i+1}"
        
        #transform into local space of the current brick
        rad = math.radians(b1['theta'])
        cos_t, sin_t = math.cos(rad), math.sin(rad)

        for j in range(i): 
            b2 = normalized_data[j]
            #only check bricks on the same layer
            if abs(b1['z'] - b2['z']) < 2.0:
                dx, dy = b2['x'] - b1['x'], b2['y'] - b1['y']
                
                #check neighbor position relative to jaw orientation
                lx = dx * cos_t + dy * sin_t
                ly = -dx * sin_t + dy * cos_t
                
                if abs(lx) < (CLAW_OPEN_WIDTH_MM / 2.0) and abs(ly) < 30.0:
                    b1['drop_offset'] = CLAW_VERTICAL_LIFT
                    break

    # --- CSV QUEUE EXPORT ---
    os.makedirs(os.path.dirname(output_csv), exist_ok=True)
    with open(output_csv, mode='w', newline='') as f:
        fieldnames = ['brick_id', 'x', 'y', 'z', 'theta', 'drop_offset']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(normalized_data)
            
    print("-" * 40)
    print(f"TRANSLATION COMPLETE")
    print("-" * 40)
    print(f"Bricks in Build:  {len(normalized_data)}")
    print(f"Bricks Clipped:   {cut_count}")
    print(f"Limits Applied:   X < {X_MAX_LIMIT}mm, Z < {Z_MAX_LIMIT}mm")
    print(f"Coordinates:      Normalized to positive quadrant (0,0,0)")
    print("-" * 40)

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    extract_bricks_to_csv(os.path.join(current_dir, 'structure.ifc'), os.path.join(current_dir, 'wall_design.csv'))