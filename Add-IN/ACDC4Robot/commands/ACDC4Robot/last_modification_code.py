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
imu_gazebo_joint = Element("gazebo")
imu_gazebo_joint.attrib = {"reference": "imu_joint"}

physics = SubElement(imu_gazebo_joint, "physics")
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

# Create IMU Gazebo reference for the link (with sensor)
imu_gazebo_link = Element("gazebo")
imu_gazebo_link.attrib = {"reference": "imu_link"}

sensor = SubElement(imu_gazebo_link, "sensor")
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
robot_ele.append(imu_gazebo_joint)  # Gazebo reference for the joint (with physics)
robot_ele.append(imu_link)    # IMU link
robot_ele.append(imu_gazebo_link)  # Gazebo reference for the sensor
