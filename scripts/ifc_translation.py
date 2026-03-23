import ifcopenshell
import ifcopenshell.geom
import ifcopenshell.util.placement
import csv
import math
import os

def extract_bricks_to_csv(ifc_file_path, output_csv):
    if not os.path.exists(ifc_file_path):
        print(f"Error: {ifc_file_path} not found.")
        return

    model = ifcopenshell.open(ifc_file_path)
    
    #type agnostic search to expand file creation flexibility
    brick_types = ["IfcMember", "IfcWall", "IfcBuildingElementProxy", "IfcWallStandardCase"]
    bricks = []
    for t in brick_types:
        bricks.extend(model.by_type(t))
    
    if not bricks:
        print("No bricks found! Ensure FreeCAD objects are exported as individual solids.")
        return
    
    # --- PHYSICAL TOOL CONSTANTS  ---
    CLAW_OPEN_WIDTH_MM = 100.0  #total width of gripper claws while open NEEDS TO BE EDITED
    CLAW_VERTICAL_LIFT = 20.0   #drop offset for neighboring bricks
    
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True) 
    
    brick_data = []

    for i, brick in enumerate(bricks):
        try:
            shape = ifcopenshell.geom.create_shape(settings, brick)
            verts = shape.geometry.verts 
            grouped_verts = [verts[j:j+3] for j in range(0, len(verts), 3)]
            
            x_coords = [v[0] for v in grouped_verts]
            y_coords = [v[1] for v in grouped_verts]
            z_coords = [v[2] for v in grouped_verts]

            #calculate midpoint for x y, scalee by 1000 for mm
            dx = (max(x_coords) - min(x_coords)) * 1000.0
            dy = (max(y_coords) - min(y_coords)) * 1000.0

            x_mid = ((min(x_coords) + max(x_coords)) / 2.0) * 1000.0
            y_mid = ((min(y_coords) + max(y_coords)) / 2.0) * 1000.0
            z_bot = (min(z_coords)) * 1000.0 #get the bottom z face, matches "our zero"

            #need to add rotation logic
            theta_deg = 0.0 

            brick_data.append({
                'x': round(x_mid, 3), 
                'y': round(y_mid, 3), 
                'z': round(z_bot, 3), 
                'theta': theta_deg, 
                'drop_offset': 0.0,
                'long_dim': max(dx, dy) #helper for neighbor algorithm
            })
        except Exception as e:
            print(f"Warning: Skipping brick {i}: {e}")

    # --- SORTING FOR BUILD ORDER ---
    #sort Z --> X --> Y
    brick_data.sort(key=lambda b: (round(b['z'], 1), round(b['x'], 1), round(b['y'], 1)))

    # --- FULL-JAW NEIGHBOR DETECTION ---
    offset_count = 0
    for i, b_curr in enumerate(brick_data):
        b_curr['brick_id'] = f"B{i+1}"
        for j in range(i): 
            b_prev = brick_data[j]
            #ensure bricks are on the same layer
            if abs(b_curr['z'] - b_prev['z']) < 2.0:
                dist = math.sqrt((b_curr['x'] - b_prev['x'])**2 + (b_curr['y'] - b_prev['y'])**2)
                
                #if the distance between centers is less than the toolhead's open footprint
                if dist < CLAW_OPEN_WIDTH_MM:
                    b_curr['drop_offset'] = CLAW_VERTICAL_LIFT
                    offset_count += 1
                    break 

    #export to CSV
    os.makedirs(os.path.dirname(output_csv), exist_ok=True)
    with open(output_csv, mode='w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['brick_id', 'x', 'y', 'z', 'theta', 'drop_offset'])
        writer.writeheader()
        for row in brick_data:
            writer.writerow({k: v for k, v in row.items() if k != 'long_dim'})
            
    # --- TERMINAL SUMMARY ---
    print("-" * 30)
    print(f"EXTRACTION SUMMARY")
    print("-" * 30)
    print(f"Total Bricks Found:  {len(brick_data)}")
    print(f"Normal Placements:   {len(brick_data) - offset_count}")
    print(f"Drop-Offset Needed:  {offset_count}")
    print(f"CSV Saved To:        {output_csv}")
    print("-" * 30)

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    ifc_file = 'structure.ifc' 
    csv_file = 'wall_design.csv'
    
    extract_bricks_to_csv(os.path.join(current_dir, ifc_file), os.path.join(current_dir, csv_file))