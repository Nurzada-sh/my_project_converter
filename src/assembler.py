import xml.etree.ElementTree as ET
import copy

def assemble_quadruped(leg_mjcf_path, torso_mjcf_path, output_path):
        
    leg_tree = ET.parse(leg_mjcf_path)
    leg_root = leg_tree.getroot()
    
    torso_tree = ET.parse(torso_mjcf_path)
    torso_root = torso_tree.getroot()
    
    worldbody = torso_root.find('worldbody')
    
    leg_positions = [
        ("front_right", "0.25 0.15 0"),
        ("front_left", "0.25 -0.15 0"),
        ("rear_right", "-0.25 0.15 0"),
        ("rear_left", "-0.25 -0.15 0")
    ]
    
    for leg_name, pos in leg_positions:
       
        leg_copy = copy.deepcopy(leg_root.find('.//body[@name="G"]'))
        
        if leg_copy is not None:
            leg_copy.set('name', f'leg_{leg_name}')
            
            leg_copy.set('pos', pos)
            
            rename_in_body(leg_copy, suffix=f'_{leg_name}')
            
            torso = worldbody.find('.//body[@name="torso"]')
            if torso is not None:
                torso.append(leg_copy)
            else:
                worldbody.append(leg_copy)
    
    ET.indent(torso_tree, space='  ', level=0)
    with open(output_path, 'wb') as f:
        f.write(b'<?xml version="1.0" encoding="utf-8"?>\n')
        torso_tree.write(f, encoding='utf-8')
    
    print(f"✅ Собран квадрупед: {output_path}")

def rename_in_body(body, suffix):
    body.set('name', body.get('name', '') + suffix)
    
    for elem in body:
        if 'name' in elem.attrib:
            elem.set('name', elem.get('name') + suffix)
        
        if elem.tag == 'joint':
            pass
        
        if elem.tag == 'body':
            rename_in_body(elem, suffix)

if __name__ == "__main__":
    
    assemble_quadruped(
        leg_mjcf_path="/home/nurzada/quadruped-assembler/configs/robot_0_FIXED.mjcf",
        torso_mjcf_path="/home/nurzada/quadruped-assembler/configs/torso.xml",
        output_path="/home/nurzada/quadruped-assembler/outputs/quadruped_assembled.mjcf"
    )
