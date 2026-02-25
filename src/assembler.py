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
    
    leg_names = []
    
    for leg_name, pos in leg_positions:
        leg_names.append(leg_name)
        
        # тело G
        leg_copy = copy.deepcopy(leg_root.find('.//body[@name="G"]'))
        
        if leg_copy is not None:
            
            leg_copy.set('name', f'leg_{leg_name}')
            leg_copy.set('pos', pos)
            
            rename_all_elements(leg_copy, suffix=f'_{leg_name}')
            
            torso = worldbody.find('.//body[@name="torso"]')
            if torso is not None:
                torso.append(leg_copy)
    
    
    copy_actuators(leg_root, torso_root, leg_names)
    
    
    copy_equality(leg_root, torso_root, leg_names)
    
    ET.indent(torso_tree, space='  ', level=0)
    with open(output_path, 'wb') as f:
        f.write(b'<?xml version="1.0" encoding="utf-8"?>\n')
        torso_tree.write(f, encoding='utf-8')
    
    print(f" квадрупед: {output_path}")
    check_equality(output_path)

def rename_all_elements(element, suffix):
    
    
    if 'name' in element.attrib:
        old_name = element.get('name')
        
        if not old_name.endswith(suffix):
            element.set('name', old_name + suffix)
    
    for child in element:
        rename_all_elements(child, suffix)

def copy_actuators(leg_root, torso_root, leg_names):
    leg_actuator = leg_root.find('actuator')
    if leg_actuator is None:
        return
    
    torso_actuator = torso_root.find('actuator')
    if torso_actuator is None:
        torso_actuator = ET.SubElement(torso_root, "actuator")
    
    for leg_name in leg_names:
        for motor in leg_actuator.findall('motor'):
            motor_copy = copy.deepcopy(motor)
            motor_copy.set('name', f"{motor.get('name')}_{leg_name}")
            
            joint = motor_copy.get('joint')
            if joint:
                motor_copy.set('joint', f"{joint}_{leg_name}")
            
            torso_actuator.append(motor_copy)

def copy_equality(leg_root, torso_root, leg_names):
    leg_equality = leg_root.find('equality')
    if leg_equality is None:
        return
    
    torso_equality = torso_root.find('equality')
    if torso_equality is None:
        torso_equality = ET.SubElement(torso_root, "equality")
    
    for leg_name in leg_names:
        for constr in leg_equality:
            constr_copy = copy.deepcopy(constr)
            
            constr_copy.set('name', f"{constr.get('name')}_{leg_name}")
            
            body1 = constr_copy.get('body1')
            if body1:
                constr_copy.set('body1', f"{body1}_{leg_name}")
            
            body2 = constr_copy.get('body2')
            if body2:
                constr_copy.set('body2', f"{body2}_{leg_name}")
            
            torso_equality.append(constr_copy)

def check_equality(mjcf_file):
    tree = ET.parse(mjcf_file)
    root = tree.getroot()
    
    body_names = set()
    for body in root.findall('.//body'):
        name = body.get('name')
        if name:
            body_names.add(name)
        
    equality = root.find('equality')
    if equality is not None:
        connects = equality.findall('connect')
        print(f"   Equality connect: {len(connects)}")
        
        for conn in connects[:4]:
            b1 = conn.get('body1')
            b2 = conn.get('body2')
            print(f"   - {conn.get('name')}")
            print(f"     body1: {b1} {'Good' if b1 in body_names else 'No'}")
            print(f"     body2: {b2} {'Good' if b2 in body_names else 'No}")

if __name__ == "__main__":
    
    assemble_quadruped(
        leg_mjcf_path="/home/nurzada/quadruped-assembler/configs/robot_8_ready.mjcf",
        torso_mjcf_path="/home/nurzada/quadruped-assembler/configs/torso.xml",
        output_path="/home/nurzada/quadruped-assembler/outputs/quadruped_assembled8.mjcf"
    )
