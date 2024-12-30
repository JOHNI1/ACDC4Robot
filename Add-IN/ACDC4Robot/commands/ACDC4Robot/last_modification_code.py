comment = Comment("frame_class_type : hexa-x")
robot_ele.insert(0, comment)

gazebo_ele = Element("gazebo")
gz_sim_joint_state_publisher_system = SubElement(gazebo_ele, "plugin")
gz_sim_joint_state_publisher_system.attrib = {"filename": "gz-sim-joint-state-publisher-system", "name": "gz::sim::systems::JointStatePublisher"}

robot_ele.insert(1, gazebo_ele)



for link in robot_ele.iter("link"):
    futil.log(link.attrib['name'])

gz_sim_apply_joint_force_system_rotors_list = []

for joint in robot_ele.iter("joint"):
    futil.log(joint.attrib['name'])
    if "spring" in joint.attrib['name']:

        dynamics = SubElement(joint, "dynamics")
        dynamics.attrib = {"damping": "10", "friction": "0.0"}

        gazebo_ele = Element("gazebo")
        gazebo_ele.attrib = {"reference": joint.attrib['name']}
        spring_stiffness = SubElement(gazebo_ele, "springStiffness")
        spring_stiffness.text = "500"
        spring_reference = SubElement(gazebo_ele, "springReference")
        spring_reference.text = "0"
        robot_ele.append(gazebo_ele)

    if "rotor" in joint.attrib['name'] and "prop" not in joint.attrib['name']:
        gz_sim_apply_joint_force_system_rotors_list.append(joint.attrib['name'])


gazebo_ele = Element("gazebo")
for joint in gz_sim_apply_joint_force_system_rotors_list:
    gz_sim_apply_joint_force_system = SubElement(gazebo_ele, "plugin")
    gz_sim_apply_joint_force_system.attrib = {"filename": "gz-sim-apply-joint-force-system", "name": "gz::sim::systems::ApplyJointForce"}
    joint_name = SubElement(gz_sim_apply_joint_force_system, "joint_name")
    joint_name.text = joint

robot_ele.append(gazebo_ele)

