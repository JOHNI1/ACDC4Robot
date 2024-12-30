# Add the comment and plugin to the front
comment = Comment("frame_class_type : hexa-x")
robot_ele.insert(0, comment)

gazebo_ele = Element("gazebo")
gz_sim_joint_state_publisher_system = SubElement(gazebo_ele, "plugin")
gz_sim_joint_state_publisher_system.attrib = {"filename": "gz-sim-joint-state-publisher-system", "name": "gz::sim::systems::JointStatePublisher"}

robot_ele.insert(1, gazebo_ele)

# Iterate through all links and print their names
for link in robot_ele.iter("link"):
    futil.log(link.attrib['name'])

gz_sim_apply_joint_force_system_rotors_list = []

# Iterate through all joints
for joint in robot_ele.iter("joint"):
    futil.log(joint.attrib['name'])
    if "spring" in joint.attrib['name']:

        # Add damping and friction for spring joints
        dynamics = SubElement(joint, "dynamics")
        dynamics.attrib = {"damping": "10", "friction": "0.0"}

        # Add gazebo reference for the spring joints
        gazebo_ele = Element("gazebo")
        gazebo_ele.attrib = {"reference": joint.attrib['name']}
        spring_stiffness = SubElement(gazebo_ele, "springStiffness")
        spring_stiffness.text = "500"
        spring_reference = SubElement(gazebo_ele, "springReference")
        spring_reference.text = "0"
        robot_ele.append(gazebo_ele)

    # Collect rotor joints for applying forces later
    if "rotor" in joint.attrib['name'] and "prop" not in joint.attrib['name']:
        gz_sim_apply_joint_force_system_rotors_list.append(joint.attrib['name'])

# Add the plugin for applying forces to the rotors
gazebo_ele = Element("gazebo")
for joint in gz_sim_apply_joint_force_system_rotors_list:
    gz_sim_apply_joint_force_system = SubElement(gazebo_ele, "plugin")
    gz_sim_apply_joint_force_system.attrib = {"filename": "gz-sim-apply-joint-force-system", "name": "gz::sim::systems::ApplyJointForce"}
    joint_name = SubElement(gz_sim_apply_joint_force_system, "joint_name")
    joint_name.text = joint

robot_ele.append(gazebo_ele)

# Define IMU and related elements under the existing structure using Element and SubElement

# Create IMU joint element
imu_joint = Element("joint")
imu_joint.attrib = {"name": "imu_joint", "type": "revolute"}

parent_link = SubElement(imu_joint, "parent")
parent_link.attrib = {"link": "base_link"}

child_link = SubElement(imu_joint, "child")
child_link.attrib = {"link": "imu_link"}

origin = SubElement(imu_joint, "origin")
origin.attrib = {"xyz": "0 0 0", "rpy": "0 0 0"}

axis = SubElement(imu_joint, "axis")
axis.attrib = {"xyz": "0 0 1"}

limit = SubElement(imu_joint, "limit")
limit.attrib = {"effort": "0", "velocity": "0", "lower": "0", "upper": "0"}

dynamics = SubElement(imu_joint, "dynamics")
dynamics.attrib = {"damping": "1", "friction": "0"}

# Create IMU Gazebo reference for the joint (without sensor)
imu_gazebo = Element("gazebo")
imu_gazebo.attrib = {"reference": "imu_joint"}

physics = SubElement(imu_gazebo, "physics")
ode = SubElement(physics, "ode")
implicit_spring_damper = SubElement(ode, "implicit_spring_damper")
implicit_spring_damper.text = "1"

# Create IMU link element
imu_link = Element("link")
imu_link.attrib = {"name": "imu_link"}

imu_origin = SubElement(imu_link, "origin")
imu_origin.attrib = {"xyz": "0 0 0", "rpy": "0 0 0"}

inertial = SubElement(imu_link, "inertial")
inertial_origin = SubElement(inertial, "origin")
inertial_origin.attrib = {"xyz": "0 0 0", "rpy": "0 0 0"}

mass = SubElement(inertial, "mass")
mass.attrib = {"value": "0.15"}

inertia = SubElement(inertial, "inertia")
inertia.attrib = {
    "ixx": "0.00001",
    "ixy": "0",
    "ixz": "0",
    "iyy": "0.00002",
    "iyz": "0",
    "izz": "0.00002"
}

visual = SubElement(imu_link, "visual")
geometry = SubElement(visual, "geometry")
xacro_set_mesh = SubElement(geometry, "xacro:set_mesh")
xacro_set_mesh.attrib = {"mesh": "Box", "scale": "0.09 0.09 0.09"}

imu_visual_origin = SubElement(visual, "origin")
imu_visual_origin.attrib = {"xyz": "0 0 0", "rpy": "0 0 0"}

# Create IMU Gazebo sensor reference (for imu_link)
imu_gazebo_sensor = Element("gazebo")
imu_gazebo_sensor.attrib = {"reference": "imu_link"}

sensor = SubElement(imu_gazebo_sensor, "sensor")
sensor.attrib = {"name": "imu_sensor", "type": "imu"}

pose = SubElement(sensor, "pose")
pose.attrib = {"degrees": "true"}
pose.text = "0 0 0 180 0 0"

always_on = SubElement(sensor, "always_on")
always_on.text = "1"

update_rate = SubElement(sensor, "update_rate")
update_rate.text = "1000.0"

# Append all elements to robot_ele
robot_ele.append(imu_joint)  # Joint
robot_ele.append(imu_gazebo)  # Gazebo reference for the joint (with physics)
robot_ele.append(imu_link)    # IMU link
robot_ele.append(imu_gazebo_sensor)  # Gazebo reference for the sensor
