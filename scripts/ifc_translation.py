import ifcopenshell
import ifcopenshell.util.placement
import csv
import numpy as np
import math

def extract_bricks_to_csv(ifc_file_path, output_csv):
    model = ifcopenshell.open(ifc_file_path)
    bricks = model.by_type("IfcMember")
    BRICK_W_MM = 2.3 * 25.4  # ~58.4mm
    CLAW_CLEARANCE_MM = 10.0 #placeholder for claw 
    
    brick_data = []

    for i, brick in enumerate(bricks):
        matrix = ifcopenshell.util.placement.get_matrix(brick.ObjectPlacement)
        x, y, z = matrix[:,3][:3]
        #extracts rotation around Z-axis from the 4x4 transformation matrix
        theta_rad = math.atan2(matrix[1,0], matrix[0,0])
        theta_deg = round(math.degrees(theta_rad), 2)
        brick_data.append({
            'id': f"B{i+1}",
            'x': round(x, 3),
            'y': round(y, 3),
            'z': round(z, 3),
            'theta': theta_deg,
            'drop_offset': 0.0
        })

    #check each brick against all others to find "Long Side" neighbors
    for i, b1 in enumerate(brick_data):
        for j, b2 in enumerate(brick_data):
            if i == j: continue
            #only care about bricks on the same layer
            if abs(b1['z'] - b2['z']) < 5.0: 
                dist = math.sqrt((b1['x'] - b2['x'])**2 + (b1['y'] - b2['y'])**2)
                #if they are within 110% of the brick width, they are likely neighbors
                #touching on the long sides where the gripper needs room
                if dist < (BRICK_W_MM * 1.1):
                    b1['drop_offset'] = CLAW_CLEARANCE_MM
                    break 

    #write to csv
    with open(output_csv, mode='w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['brick_id', 'x', 'y', 'z', 'theta', 'drop_offset'])
        writer.writeheader()
        for data in brick_data:
            writer.writerow(data)
            
    print(f"Extraction Complete: {len(bricks)} bricks saved with rotation and offsets.")

if __name__ == "__main__":
    extract_bricks_to_csv('structure.ifc', 'src/risc_control/wall_design.csv')