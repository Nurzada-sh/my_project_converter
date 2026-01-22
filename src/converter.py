
import xml.etree.ElementTree as ET
import math
import sys
import os

def rpy_to_quat(rpy_str):
    try:
        r, p, y = map(float, rpy_str.split())
    except:
        return "1 0 0 0"
    
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return f"{w} {x} {y} {z}"

def create_valid_inertial(inertial_element):
    inertial = ET.Element("inertial")
    
    inertial.set("pos", "0 0 0")
    inertial.set("mass", "1")
    inertial.set("diaginertia", "0.001 0.001 0.001")
    
    if inertial_element is not None:
        mass = inertial_element.find("mass")
        if mass is not None:
            inertial.set("mass", mass.get("value", "0.1"))
        
        origin = inertial_element.find("origin")
        if origin is not None:
            pos = origin.get("xyz", "0 0 0")
            if pos != "0 0 0":
                inertial.set("pos", pos)
        
        inertia = inertial_element.find("inertia")
        if inertia is not None:
            ixx = inertia.get("ixx", "0.001")
            iyy = inertia.get("iyy", "0.001")
            izz = inertia.get("izz", "0.001")
            inertial.set("diaginertia", f"{ixx} {iyy} {izz}")
    
    return inertial

def extract_color(visual_element):
    if visual_element is not None:
        material = visual_element.find('material')
        if material is not None:
            color = material.find('color')
            if color is not None:
                return color.get('rgba', '0.5 0.5 0.5 1')
    return "0.5 0.5 0.5 1"

def convert_urdf_to_mjcf(urdf_file, output_file=None):
    if output_file is None:
        output_file = urdf_file.replace('.urdf', '_FIXED.mjcf')
    
    tree = ET.parse(urdf_file)
    robot = tree.getroot()
    
    model_name = os.path.basename(urdf_file).replace('.urdf', '')
    mujoco = ET.Element("mujoco")
    mujoco.set("model", model_name)
    
    ET.SubElement(mujoco, "compiler", {
        "angle": "radian",
        "inertiafromgeom": "true",
        "coordinate": "local"
    })
    
    ET.SubElement(mujoco, "option", {
        "timestep": "0.01",
        "gravity": "0 0 -9.81",
        "integrator": "RK4"
    })
    
    size = ET.SubElement(mujoco, "size")
    size.set("njmax", "500")
    size.set("nconmax", "100")
    
    worldbody = ET.SubElement(mujoco, "worldbody")
    
    ET.SubElement(worldbody, "geom", {
        "name": "floor",
        "type": "plane",
        "size": "10 10 0.1",
        "pos": "0 0 0",
        "rgba": "0.8 0.9 0.9 1"
    })
    
    ET.SubElement(worldbody, "light", {
        "name": "light1",
        "pos": "0 0 3",
        "dir": "0 0 -1",
        "directional": "true"
    })
    
    links = {}
    parent_to_children = {}
    all_joints_info = {}
    
    for link in robot.findall('link'):
        links[link.get('name')] = link
    
    for joint in robot.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        
        if parent is not None and child is not None:
            parent_name = parent.get('link')
            child_name = child.get('link')
            
            if parent_name and child_name:
                if parent_name not in parent_to_children:
                    parent_to_children[parent_name] = []
                parent_to_children[parent_name].append((child_name, joint))
                
                joint_name = joint.get('name')
                joint_type = joint.get('type', 'fixed')
                all_joints_info[joint_name] = {
                    'type': joint_type,
                    'joint_elem': joint,
                    'parent': parent_name,
                    'child': child_name
                }
    
    all_children = []
    for children in parent_to_children.values():
        for child_name, _ in children:
            all_children.append(child_name)
    
    root_links = [name for name in links.keys() if name not in all_children]
    
    if not root_links:
        return False
    
    root_name = root_links[0]
        
    robot_root = ET.SubElement(worldbody, "body", 
                              name="robot_root", 
                              pos="0 0 0.5", 
                              quat="1 0 0 0")
    
    created_joints = []
    
    def build_tree(link_name, parent_body, parent_joint=None):
        link = links[link_name]
        
        body_attrib = {"name": link_name}
        
        if parent_joint is not None:
            origin = parent_joint.find('origin')
            if origin is not None:
                body_attrib["pos"] = origin.get('xyz', '0 0 0')
                rpy = origin.get('rpy', '0 0 0')
                body_attrib["quat"] = rpy_to_quat(rpy)
        
        body = ET.Element("body", attrib=body_attrib)
        
        if parent_joint is not None:
            jtype = parent_joint.get('type', 'fixed')
            joint_name = parent_joint.get('name', link_name + '_joint')
            
            if jtype != 'fixed':
                joint_elem = ET.Element("joint")
                joint_elem.set('name', joint_name)
                
                if jtype in ['revolute', 'continuous']:
                    joint_elem.set('type', 'hinge')
                elif jtype == 'prismatic':
                    joint_elem.set('type', 'slide')
                
                axis = parent_joint.find('axis')
                if axis is not None:
                    joint_elem.set('axis', axis.get('xyz', '0 1 0'))
                
                limit = parent_joint.find('limit')
                if limit is not None:
                    lower = limit.get('lower', '-3.14')
                    upper = limit.get('upper', '3.14')
                    joint_elem.set('range', f"{lower} {upper}")
                
                joint_elem.set('damping', '0.1')
                
                body.insert(0, joint_elem)
                
                created_joints.append({
                    'name': joint_name,
                    'type': jtype,
                    'element': joint_elem
                })
        
        for i, visual in enumerate(link.findall('visual')):
            rgba = extract_color(visual)
            
            geom_attrib = {
                "name": f"{link_name}_vis{i}",
                "type": "box",
                "rgba": rgba,
                "contype": "1",
                "conaffinity": "1"
            }
            
            origin = visual.find('origin')
            if origin is not None:
                geom_attrib["pos"] = origin.get('xyz', '0 0 0')
                rpy = origin.get('rpy', '0 0 0')
                geom_attrib["quat"] = rpy_to_quat(rpy)
            
            geometry = visual.find('geometry')
            if geometry is not None:
                box = geometry.find('box')
                if box is not None:
                    size_str = box.get('size', '0.1 0.1 0.1')
                    sizes = [float(x)/2 for x in size_str.split()]
                    geom_attrib["size"] = f"{sizes[0]} {sizes[1]} {sizes[2]}"
                    geom_attrib["type"] = "box"
                
                sphere = geometry.find('sphere')
                if sphere is not None:
                    geom_attrib["type"] = "sphere"
                    geom_attrib["size"] = sphere.get('radius', '0.1')
                
                cylinder = geometry.find('cylinder')
                if cylinder is not None:
                    geom_attrib["type"] = "cylinder"
                    radius = cylinder.get('radius', '0.1')
                    length = cylinder.get('length', '0.2')
                    geom_attrib["size"] = f"{radius} {float(length)/2}"
            
            ET.SubElement(body, "geom", geom_attrib)
        
        inertial_from_urdf = link.find('inertial')
        inertial_elem = create_valid_inertial(inertial_from_urdf)
        body.append(inertial_elem)
        
        parent_body.append(body)
        
        if link_name in parent_to_children:
            for child_name, child_joint in parent_to_children[link_name]:
                build_tree(child_name, body, child_joint)
        
        return body
    
    build_tree(root_name, robot_root)
    
    actuator = ET.SubElement(mujoco, "actuator")
    
    motors_created = 0
    for joint_info in created_joints:
        joint_name = joint_info['name']
        joint_type = joint_info['type']
        
        if joint_type in ['revolute', 'continuous', 'prismatic']:
            gear = "1"
            
            if 'ground' in joint_name.lower():
                gear = "80"
            elif 'knee' in joint_name.lower():
                gear = "150"
            elif 'hip' in joint_name.lower():
                gear = "120"
            elif 'ankle' in joint_name.lower():
                gear = "100"
            elif 'shoulder' in joint_name.lower():
                gear = "100"
            elif 'elbow' in joint_name.lower():
                gear = "80"
            
            motor_attrib = {
                "name": f"{joint_name}_motor",
                "joint": joint_name,
                "gear": gear,
                "ctrlrange": "-1.0 1.0",
                "forcerange": "-50 50"
            }
            
            ET.SubElement(actuator, "motor", motor_attrib)
            motors_created += 1
    
    tree = ET.ElementTree(mujoco)
    ET.indent(tree, space='  ', level=0)
    
    with open(output_file, 'wb') as f:
        f.write(b'<?xml version="1.0" encoding="utf-8"?>\n')
        tree.write(f, encoding='utf-8')
    
    check_result(output_file)
    
    return True

def check_result(mjcf_file):
    try:
        tree = ET.parse(mjcf_file)
        root = tree.getroot()
        
        bodies = root.findall('.//body')
        joints = root.findall('.//joint')
        motors = root.findall('.//motor')
        
        all_joint_names = []
        for joint in joints:
            name = joint.get('name')
            if name:
                all_joint_names.append(name)
        
        for motor in motors:
            motor_name = motor.get('name', 'unnamed')
            target_joint = motor.get('joint', 'UNKNOWN')
            
            if target_joint not in all_joint_names:
                print(f"ERROR: Motor '{motor_name}' refers to a non-existent joint '{target_joint}'")
        
    except Exception as e:
        print(f"Error while checking: {e}")

def validate_structure(mjcf_file):
    with open(mjcf_file, 'r') as f:
        content = f.read()
    
    checks = [
        ("<mujoco", "Корневой элемент mujoco"),
        ("<worldbody", "Секция worldbody"),
        ("<body", "Хотя бы одно тело"),
        ("<joint", "Хотя бы один сустав"),
        ("<actuator", "Секция актуаторов"),
        ("<motor", "Хотя бы один мотор")
    ]
    
    for pattern, description in checks:
        if pattern not in content:
            print(f"ОТСУТСТВУЕТ: {description}")
    
    lines = content.split('\n')
    in_body = 0
    errors = []
    
    for i, line in enumerate(lines, 1):
        if '<body' in line and '</body>' not in line:
            in_body += 1
        elif '</body>' in line:
            in_body -= 1
        elif '<joint' in line and in_body == 0:
            errors.append(f"Строка {i}: Joint вне body!")
    
    if errors:
        for error in errors:
            print(error)

def main():
    if len(sys.argv) < 2:
        print("Using: python3 converter_fixed.py <файл.urdf> [выходной_файл.mjcf]")
        sys.exit(1)
    
    urdf_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not os.path.exists(urdf_file):
        print(f"File not found: {urdf_file}")
        sys.exit(1)
    
    success = convert_urdf_to_mjcf(urdf_file, output_file)
    
    if success:
        output_path = output_file if output_file else urdf_file.replace('.urdf', '_FIXED.mjcf')
        validate_structure(output_path)
    else:
        print("Conversion failed")
        sys.exit(1)

if __name__ == "__main__":
    main()
