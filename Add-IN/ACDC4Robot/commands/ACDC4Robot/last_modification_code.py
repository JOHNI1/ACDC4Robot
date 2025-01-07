import math

#clock wise is negative, counter clock wise is positive
HEXA_ROTOR_DIRECTIONS = [-1, 1, -1, 1, 1, -1]

#north is positive Y axis. east is positive X axis.
#default direction of imu is to the positive X axis.
#therefore to make the default direction of the drone
#to face north, there is a need to rotate the imu in yaw +90 degrees



# Add the comment and plugin to the front
comment = Comment("frame_class_type : hexa-x")
robot_ele.insert(0, comment)

for property in robot_ele.iter("xacro:property"):
    if property.attrib['name'] == "mesh_folder_path":
        property.attrib = {"name": "mesh_folder_path", "value": "package://drone/models/formafat/"}


gazebo = Element("gazebo")
gz_sim_joint_state_publisher_system = SubElement(gazebo, "plugin")
gz_sim_joint_state_publisher_system.attrib = {"filename": "gz-sim-joint-state-publisher-system", "name": "gz::sim::systems::JointStatePublisher"}
robot_ele.append(gazebo)




lift_drag_plugins = []
# Iterate through all links and print their names
for link in robot_ele.iter("link"):
    for visual in link.iter("visual"):
        material = SubElement(visual, "material")
        material.attrib = {"name": f"{link.attrib['name']}_material"}
        color = SubElement(material, "color")
        color.attrib = {"rgba": "0.1 0.1 0.1 1"}
    if "prop" in link.attrib['name']:
        #LiftDragPlugin
        LiftDragPlugin = Element("plugin")
        LiftDragPlugin.attrib = {"filename": "gz-sim-lift-drag-system", "name": "gz::sim::systems::LiftDrag"}

        a0 = SubElement(LiftDragPlugin, "a0")
        a0.text = "0.3"

        alpha_stall = SubElement(LiftDragPlugin, "alpha_stall")
        alpha_stall.text = "1.4"

        cla = SubElement(LiftDragPlugin, "cla")
        cla.text = "4.25"

        cda = SubElement(LiftDragPlugin, "cda")
        cda.text = "0.10"

        cma = SubElement(LiftDragPlugin, "cma")
        cma.text = "0.0"

        cla_stall = SubElement(LiftDragPlugin, "cla_stall")
        cla_stall.text = "-0.025"

        cda_stall = SubElement(LiftDragPlugin, "cda_stall")
        cda_stall.text = "0.0"

        cma_stall = SubElement(LiftDragPlugin, "cma_stall")
        cma_stall.text = "0.0"

        area = SubElement(LiftDragPlugin, "area")
        area.text = "0.0152"

        air_density = SubElement(LiftDragPlugin, "air_density")
        air_density.text = "1.2041"

        cp = SubElement(LiftDragPlugin, "cp")   #0.19
        forward = SubElement(LiftDragPlugin, "forward")

        arm_index = int(link.attrib['name'].split("_")[0].replace("arm", ""))-1


        if "prop1" in link.attrib['name']:
            cp.text = "0 -0.19 0"
            forward.text = f"{-1*HEXA_ROTOR_DIRECTIONS[arm_index]} 0 0" #temp
        elif "prop2" in link.attrib['name']:
            cp.text = "0 0.19 0"
            forward.text = f"{1*HEXA_ROTOR_DIRECTIONS[arm_index]} 0 0" #temp

        upward = SubElement(LiftDragPlugin, "upward")
        upward.text = "0 0 -1"

        link_name = SubElement(LiftDragPlugin, "link_name")
        link_name.text = f"{link.attrib['name']}"




        lift_drag_plugins.append(LiftDragPlugin)



rotors_list = []

# Iterate through all joints
for joint in robot_ele.iter("joint"):
    # futil.log(joint.attrib['name'])
    if "spring" in joint.attrib['name']:

        # Add damping and friction for spring joints
        dynamics = SubElement(joint, "dynamics")
        dynamics.attrib = {"damping": "10", "friction": "0.3"}

        arm_index = int(joint.attrib['name'].replace("_", "").replace("spring", "").replace("joint", ""))-1
        for origin in joint.iter("origin"):
            rpy = origin.attrib["rpy"].split()
            rpy[1] = f"{-1*HEXA_ROTOR_DIRECTIONS[arm_index]*5*math.pi/180}"
            origin.attrib["rpy"] = f"{rpy[0]} {rpy[1]} {rpy[2]}"


        # Add gazebo reference for the spring joints
        gazebo = Element("gazebo")
        gazebo.attrib = {"reference": joint.attrib['name']}
        spring_stiffness = SubElement(gazebo, "springStiffness")
        spring_stiffness.text = "3500"
        spring_reference = SubElement(gazebo, "springReference")
        spring_reference.text = "0"
        robot_ele.append(gazebo)

    # Collect rotor joints for applying forces later
    if "rotor" in joint.attrib['name'] and "prop" not in joint.attrib['name']:
        rotors_list.append(joint.attrib['name'])
        dynamics = SubElement(joint, "dynamics")
        dynamics.attrib = {"damping": "0.01", "friction": "0.03"}

    elif "prop" in joint.attrib['name']:
        # Add damping and friction for spring joints
        dynamics = SubElement(joint, "dynamics")
        dynamics.attrib = {"damping": "0", "friction": "0.1"}

# Add the plugin for applying forces to the rotors
gazebo = Element("gazebo")
for rotor_joint in rotors_list:
    gz_sim_apply_joint_force_system = SubElement(gazebo, "plugin")
    gz_sim_apply_joint_force_system.attrib = {"filename": "gz-sim-apply-joint-force-system", "name": "gz::sim::systems::ApplyJointForce"}
    joint_name = SubElement(gz_sim_apply_joint_force_system, "joint_name")
    joint_name.text = rotor_joint

robot_ele.append(gazebo)

# Define IMU and related elements under the existing structure using Element and SubElement

# Create IMU joint element
imu_joint = Element("joint")
imu_joint.attrib = {"name": "imu_joint", "type": "revolute"}

parent_link = SubElement(imu_joint, "parent")
parent_link.attrib = {"link": "base_link"}

child_link = SubElement(imu_joint, "child")
child_link.attrib = {"link": "imu_link"}

origin = SubElement(imu_joint, "origin")
origin.attrib = {"xyz": "0 0 0.7", "rpy": f"0 0 {math.pi/2}"} #rotate +90 degrees in yaw to face north!

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



# ArdupilotPlugin
gazebo = Element("gazebo")
ardupilot_plugin = SubElement(gazebo, "plugin")
ardupilot_plugin.attrib = {"name": "ArduPilotPlugin", "filename": "ArduPilotPlugin"}

fdm_addr = SubElement(ardupilot_plugin, "fdm_addr")
fdm_addr.text = "127.0.0.1"

fdm_port_in = SubElement(ardupilot_plugin, "fdm_port_in")
fdm_port_in.text = "9002"

connectionTimeoutMaxCount = SubElement(ardupilot_plugin, "connectionTimeoutMaxCount")
connectionTimeoutMaxCount.text = "0"

lock_step = SubElement(ardupilot_plugin, "lock_step")
lock_step.text = "0"

have_32_channels = SubElement(ardupilot_plugin, "have_32_channels")
have_32_channels.text = "0"

modelXYZToAirplaneXForwardZDown = SubElement(ardupilot_plugin, "modelXYZToAirplaneXForwardZDown")
modelXYZToAirplaneXForwardZDown.attrib = {"degrees": "true"}
modelXYZToAirplaneXForwardZDown.text = "0 0 0 180 0 0"

gazeboXYZToNED = SubElement(ardupilot_plugin, "gazeboXYZToNED")
gazeboXYZToNED.attrib = {"degrees": "true"}
gazeboXYZToNED.text = "0 0 0 180 0 90"

imuName = SubElement(ardupilot_plugin, "imuName")
imuName.text = "imu_link::imu_sensor"



for rotor_joint in rotors_list:
    rotor_index = int(rotor_joint.replace("rotor", "").replace("_joint", ""))-1
    control = SubElement(ardupilot_plugin, "control")
    control.attrib = {"channel": str(rotor_index)}

    jointName = SubElement(control, "jointName")
    jointName.text = rotor_joint

    useForce = SubElement(control, "useForce")
    useForce.text = "1"

    multiplier = SubElement(control, "multiplier")
    multiplier.text = f"{838*HEXA_ROTOR_DIRECTIONS[rotor_index]}"

    offset = SubElement(control, "offset")
    offset.text = "0"

    servo_min = SubElement(control, "servo_min")
    servo_min.text = "1000"

    servo_max = SubElement(control, "servo_max")
    servo_max.text = "2000"

    control_type = SubElement(control, "type")
    control_type.text = "VELOCITY"

    p_gain = SubElement(control, "p_gain")
    p_gain.text = "0.2"

    i_gain = SubElement(control, "i_gain")
    i_gain.text = "0"

    d_gain = SubElement(control, "d_gain")
    d_gain.text = "0"

    i_max = SubElement(control, "i_max")
    i_max.text = "1"

    i_min = SubElement(control, "i_min")
    i_min.text = "-1"

    cmd_max = SubElement(control, "cmd_max")
    cmd_max.text = "5.0"

    cmd_min = SubElement(control, "cmd_min")
    cmd_min.text = "-5.0"

    controlVelocitySlowdownSim = SubElement(control, "controlVelocitySlowdownSim")
    controlVelocitySlowdownSim.text = "1"


for lift_drag_plugin in lift_drag_plugins:
    gazebo.append(lift_drag_plugin)



robot_ele.append(gazebo)

