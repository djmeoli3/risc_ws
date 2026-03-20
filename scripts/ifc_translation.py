import ifcopenshell
import ifcopenshell.util.placement
import csv
import math
import os

def extract_bricks_to_csv(ifc_file_path, output_csv):
    model = ifcopenshell.open(ifc_file_path)
    bricks = model.by_type("IfcMember")
    
    # Physical Constants
    BRICK_W_MM = 2.33 * 25.4     # 59.182mm
    BRICK_H_MM = 1.5 * 25.4      # 38.1mm
    HALF_BRICK_H = BRICK_H_MM / 2 # 19.05mm
    CLAW_CLEARANCE_MM = 20.0     # Safety offset to clear 19mm claws
    
    brick_data = []

    for i, brick in enumerate(bricks):
        matrix = ifcopenshell.util.placement.get_matrix(brick.ObjectPlacement)
        x, y, z_mid = matrix[:,3][:3] # Midpoint from IFC
        
        # Shift Z from midpoint to BOTTOM face for robot placement
        z_bottom = z_mid - HALF_BRICK_H
        
        theta_rad = math.atan2(matrix[1,0], matrix[0,0])
        theta_deg = round(math.degrees(theta_rad), 2)
        
        brick_data.append({
            'brick_id': f"B{i+1}",
            'x': round(x, 3), 'y': round(y, 3), 
            'z': round(z_bottom, 3), # Target Z is now 0.0 for Layer 1
            'theta': theta_deg, 'drop_offset': 0.0
        })

    # Directional Neighbor Detection
    for i, b1 in enumerate(brick_data):
        for j in range(i): 
            b2 = brick_data[j]
            # Same layer check (using the new bottom-face Z)
            if abs(b1['z'] - b2['z']) < 5.0: 
                dist = math.sqrt((b1['x'] - b2['x'])**2 + (b1['y'] - b2['y'])**2)
                if dist < (BRICK_W_MM * 1.1):
                    b1['drop_offset'] = CLAW_CLEARANCE_MM
                    break 

    os.makedirs(os.path.dirname(output_csv), exist_ok=True)
    with open(output_csv, mode='w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['brick_id', 'x', 'y', 'z', 'theta', 'drop_offset'])
        writer.writeheader()
        writer.writerows(brick_data)
            
    print(f"Extraction Complete: Midpoints shifted by {HALF_BRICK_H}mm to define Bottom Face.")

if __name__ == "__main__":
    extract_bricks_to_csv('structure.ifc', '/home/wake/risc_ws/src/risc_control/wall_design.csv')