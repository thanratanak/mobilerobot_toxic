"""Run the mecanum robot in Gazebo (custom map) and do SLAM in RViz.

    ros2 launch mobilerobot_toxic gazebo_slam.launch.py
    ros2 launch mobilerobot_toxic gazebo_slam.launch.py enable_teleop:=true
    ros2 launch mobilerobot_toxic gazebo_slam.launch.py enable_joy:=true      # PS5 controller
    ros2 launch mobilerobot_toxic gazebo_slam.launch.py world:=/path/to/my_world.sdf
    ros2 launch mobilerobot_toxic gazebo_slam.launch.py slam:=false   # sim only

What it does
  * Starts Gazebo with the world (plugins are injected for your Gazebo version).
  * Builds a Gazebo-ready copy of RobotNavigation.urdf at launch time:
      - heavy STL collisions -> box chassis + sphere wheels
      - wheel axes flipped so positive wheel speed = forward
      - MecanumDrive, JointStatePublisher, lidar (/scan) and IMU (/imu) added
    The original URDF is NOT modified (RViz-only launch files keep working).
  * Bridges /clock /cmd_vel /odom /tf /joint_states /scan /imu to ROS 2.
  * Starts slam_toolbox + RViz (slam.rviz) on simulated time.

Gazebo flavour is picked from ROS_DISTRO (humble -> Fortress, otherwise
Harmonic).  Override with  export GZ_VERSION=fortress  or  harmonic.
"""
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'mobilerobot_toxic'
GZ_SCHEMA = 'http://gazebosim.org/schema'

# --- robot constants (same numbers as mecanum_fake_controller.py) -----------
WHEEL_RADIUS = 0.0644
WHEELBASE = 0.39            # front<->back wheel distance  (2 * 0.195)
WHEEL_SEPARATION = 0.5458   # left<->right wheel distance  (2 * 0.2729)

# wheel link -> (joint, friction direction expressed in the chassis frame)
# The rollers of an X-type mecanum wheel set push along these directions.
WHEELS = {
    'wheel_fl_link': ('wheel_fl_joint', '1 -1 0'),
    'wheel_fr_link': ('wheel_fr_joint', '1 1 0'),
    'wheel_bl_link': ('wheel_bl_joint', '1 1 0'),
    'wheel_br_link': ('wheel_br_joint', '1 -1 0'),
}


# ---------------------------------------------------------------------------
# Gazebo flavour
# ---------------------------------------------------------------------------
def gz_flavour():
    override = os.environ.get('GZ_VERSION', '').lower()
    distro = os.environ.get('ROS_DISTRO', 'jazzy').lower()
    fortress = (override == 'fortress') or (
        override == '' and distro in ('foxy', 'galactic', 'humble'))
    if fortress:
        return {
            'name': 'Fortress', 'cli': ['ign', 'gazebo'], 'sdf_cli': ['ign', 'sdf'],
            'plugin': 'ignition-gazebo', 'ns': 'ignition::gazebo',
            'msgs': 'ignition.msgs', 'attr_prefix': 'ignition',
            'frame_tag': 'ignition_frame_id', 'res_var': 'IGN_GAZEBO_RESOURCE_PATH',
        }
    return {
        'name': 'Harmonic', 'cli': ['gz', 'sim'], 'sdf_cli': ['gz', 'sdf'],
        'plugin': 'gz-sim', 'ns': 'gz::sim',
        'msgs': 'gz.msgs', 'attr_prefix': 'gz',
        'frame_tag': 'gz_frame_id', 'res_var': 'GZ_SIM_RESOURCE_PATH',
    }


# ---------------------------------------------------------------------------
# Robot description for Gazebo
# ---------------------------------------------------------------------------
def build_gazebo_urdf(urdf_path, g, mesh_prefix=None):
    """Return the URDF text with Gazebo collisions, plugins and sensors added.

    mesh_prefix: if given, every package://mobilerobot_toxic/meshes/ URI is
    replaced by it (used to point Gazebo at the light meshes in meshes_gz/).
    """
    robot = ET.parse(urdf_path).getroot()
    if mesh_prefix:
        for mesh in robot.iter('mesh'):
            mesh.set('filename', mesh.get('filename').replace(
                f'package://{PKG}/meshes/', mesh_prefix))
    links = {l.get('name'): l for l in robot.findall('link')}

    # 1) Replace every (huge) mesh collision with simple shapes.
    for link in links.values():
        for col in link.findall('collision'):
            link.remove(col)

    col = ET.SubElement(links['base_link'], 'collision', name='base_collision')
    ET.SubElement(col, 'origin', xyz='0 0 0.075', rpy='0 0 0')
    ET.SubElement(ET.SubElement(col, 'geometry'), 'box', size='0.70 0.54 0.15')

    for lname in WHEELS:
        col = ET.SubElement(links[lname], 'collision', name=lname + '_collision')
        # wheel mesh spans 0..0.07 m along its local z axis -> centre at 0.035
        ET.SubElement(col, 'origin', xyz='0 0 0.035', rpy='0 0 0')
        ET.SubElement(ET.SubElement(col, 'geometry'), 'sphere',
                      radius=str(WHEEL_RADIUS))

    # 2) The SolidWorks wheel axes point to -Y (base frame) => positive speed
    #    would drive backwards.  Flip them so positive = forward, which is what
    #    the Gazebo MecanumDrive plugin expects.
    wheel_joints = {j for j, _ in WHEELS.values()}
    for joint in robot.findall('joint'):
        if joint.get('name') in wheel_joints:
            axis = joint.find('axis')
            xyz = [float(v) for v in axis.get('xyz').split()]
            axis.set('xyz', ' '.join('%g' % (-v + 0.0) for v in xyz))

    # 3) Gazebo extensions.
    p, n = g['plugin'], g['ns']
    wheel_friction = ''.join(
        f'<gazebo reference="{l}"><mu1>1.0</mu1><mu2>1.0</mu2></gazebo>\n'
        for l in WHEELS)
    ext = f'''<ext>
<gazebo>
  <plugin filename="{p}-joint-state-publisher-system" name="{n}::systems::JointStatePublisher">
    <topic>/joint_states</topic>
  </plugin>
  <plugin filename="{p}-mecanum-drive-system" name="{n}::systems::MecanumDrive">
    <front_left_joint>wheel_fl_joint</front_left_joint>
    <front_right_joint>wheel_fr_joint</front_right_joint>
    <back_left_joint>wheel_bl_joint</back_left_joint>
    <back_right_joint>wheel_br_joint</back_right_joint>
    <wheel_separation>{WHEEL_SEPARATION}</wheel_separation>
    <wheelbase>{WHEELBASE}</wheelbase>
    <wheel_radius>{WHEEL_RADIUS}</wheel_radius>
    <min_velocity>-1.0</min_velocity>
    <max_velocity>1.0</max_velocity>
    <min_acceleration>-2.0</min_acceleration>
    <max_acceleration>2.0</max_acceleration>
    <topic>/cmd_vel</topic>
    <odom_topic>/odom</odom_topic>
    <tf_topic>/tf</tf_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_footprint</child_frame_id>
    <odom_publish_frequency>50</odom_publish_frequency>
  </plugin>
</gazebo>
{wheel_friction}
<gazebo reference="laser_frame">
  <sensor name="lidar" type="gpu_lidar">
    <pose>0 0 0 0 0 0</pose>
    <topic>/scan</topic>
    <{g['frame_tag']}>laser_frame</{g['frame_tag']}>
    <update_rate>10</update_rate>
    <always_on>true</always_on>
    <visualize>false</visualize>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.12414</max_angle>
        </horizontal>
      </scan>
      <range><min>0.12</min><max>8.0</max><resolution>0.01</resolution></range>
      <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
    </lidar>
  </sensor>
</gazebo>

<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <topic>/imu</topic>
    <{g['frame_tag']}>imu_link</{g['frame_tag']}>
    <update_rate>50</update_rate>
    <always_on>true</always_on>
    <visualize>false</visualize>
  </sensor>
</gazebo>
</ext>'''
    for el in list(ET.fromstring(ext)):
        robot.append(el)
    return ET.tostring(robot, encoding='unicode')


def make_spawn_sdf(urdf_text, g):
    """URDF -> SDF, then give the wheel collisions a *fixed* friction direction.

    A mecanum wheel only strafes if friction is anisotropic (mu2 = 0) along a
    direction that is fixed in the chassis frame.  That needs the
    `expressed_in` attribute, which is most reliable when patched into the SDF.
    Returns None if the conversion fails (caller falls back to spawning the URDF).
    """
    tmp = tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False)
    tmp.write(urdf_text)
    tmp.close()
    try:
        out = subprocess.run(g['sdf_cli'] + ['-p', tmp.name], capture_output=True,
                             text=True, timeout=60, check=True).stdout
        ET.register_namespace(g['attr_prefix'], GZ_SCHEMA)
        root = ET.fromstring(out)
        model = root.find('model')
        chassis = model.find('link').get('name')

        def sub(parent, tag):
            child = parent.find(tag)
            return child if child is not None else ET.SubElement(parent, tag)

        patched = 0
        for link in model.findall('link'):
            if link.get('name') not in WHEELS:
                continue
            direction = WHEELS[link.get('name')][1]
            for col in link.findall('collision'):
                ode = sub(sub(sub(col, 'surface'), 'friction'), 'ode')
                sub(ode, 'mu').text = '1.0'
                sub(ode, 'mu2').text = '0.0'
                fdir1 = sub(ode, 'fdir1')
                fdir1.text = direction
                fdir1.set('{%s}expressed_in' % GZ_SCHEMA, chassis)
                patched += 1
        if patched != 4:
            return None
        return ET.tostring(root, encoding='unicode')
    except Exception as exc:  # noqa: BLE001 - any failure -> URDF fallback
        print(f'[gazebo_slam] URDF->SDF conversion failed ({exc}); '
              'spawning the URDF directly (strafing may not work).')
        return None
    finally:
        os.unlink(tmp.name)


# ---------------------------------------------------------------------------
# World
# ---------------------------------------------------------------------------
def prepare_world(world_path, g):
    """Copy the world to /tmp, injecting the system plugins it doesn't have."""
    with open(world_path, 'r') as f:
        text = f.read()
    wanted = [
        ('physics-system', 'Physics', ''),
        ('user-commands-system', 'UserCommands', ''),
        ('scene-broadcaster-system', 'SceneBroadcaster', ''),
        ('sensors-system', 'Sensors', '<render_engine>ogre2</render_engine>'),
        ('imu-system', 'Imu', ''),
    ]
    block = ''
    for fname, cls, extra in wanted:
        if fname not in text:
            block += (f'  <plugin filename="{g["plugin"]}-{fname}" '
                      f'name="{g["ns"]}::systems::{cls}">{extra}</plugin>\n')
    idx = text.rfind('</world>')
    if block and idx != -1:
        text = text[:idx] + block + text[idx:]
    out = os.path.join(tempfile.gettempdir(), 'mobilerobot_toxic_world.sdf')
    with open(out, 'w') as f:
        f.write(text)
    return out


# ---------------------------------------------------------------------------
def launch_setup(context, *args, **kwargs):
    g = gz_flavour()
    pkg = get_package_share_directory(PKG)
    cfg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    # world
    world = cfg('world')
    if not os.path.isabs(world) or not os.path.exists(world):
        world = os.path.join(pkg, 'worlds', os.path.basename(world))
    world = prepare_world(world, g)

    # robot
    urdf_path = os.path.join(pkg, 'urdf', 'RobotNavigation.urdf')
    # RViz / robot_state_publisher: original high-detail meshes.
    urdf_text = build_gazebo_urdf(urdf_path, g)
    # Gazebo: light meshes (the originals are ~85 MB and stall the renderer),
    # referenced by absolute path so no resource-path lookup can fail.
    mesh_dir = os.path.join(pkg, 'meshes_gz')
    if not os.path.isfile(os.path.join(mesh_dir, 'base_link.STL')):
        print(f'[gazebo_slam] WARNING: {mesh_dir}/base_link.STL is missing - '
              'the robot will have no visual in Gazebo. Rebuild the package.')
    gz_urdf_text = build_gazebo_urdf(urdf_path, g, 'file://' + mesh_dir + '/')
    spawn_sdf = make_spawn_sdf(gz_urdf_text, g)
    spawn_src = ['-string', spawn_sdf if spawn_sdf else gz_urdf_text]

    sim_time = {'use_sim_time': True}

    # Gazebo (so package://mobilerobot_toxic/... meshes resolve)
    resource = ':'.join(p for p in [
        os.path.dirname(pkg), os.path.join(pkg, 'worlds'), os.path.join(pkg, 'models'),
        os.environ.get(g['res_var'], '')] if p)
    gz_cmd = g['cli'] + ['-r'] + ([] if cfg('gui') == 'true' else ['-s']) + [world]
    gazebo = ExecuteProcess(cmd=gz_cmd, output='screen',
                            additional_env={g['res_var']: resource})

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': urdf_text}, sim_time], output='screen')

    spawn = TimerAction(period=10.0, actions=[Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'mobilerobot', *spawn_src,
                   '-x', cfg('x'), '-y', cfg('y'), '-z', cfg('z'), '-Y', cfg('yaw')])])

    m = g['msgs']
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[sim_time],
        arguments=[
            f'/clock@rosgraph_msgs/msg/Clock[{m}.Clock',
            f'/cmd_vel@geometry_msgs/msg/Twist]{m}.Twist',
            f'/odom@nav_msgs/msg/Odometry[{m}.Odometry',
            f'/tf@tf2_msgs/msg/TFMessage[{m}.Pose_V',
            f'/joint_states@sensor_msgs/msg/JointState[{m}.Model',
            f'/scan@sensor_msgs/msg/LaserScan[{m}.LaserScan',
            f'/imu@sensor_msgs/msg/Imu[{m}.IMU',
        ])

    slam_cond = IfCondition(LaunchConfiguration('slam'))
    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                name='slam_toolbox', output='screen', condition=slam_cond,
                parameters=[os.path.join(pkg, 'config', 'slam_toolbox_params.yaml'), sim_time])
    slam_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_slam', output='screen', condition=slam_cond,
        parameters=[{'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['slam_toolbox']}, sim_time])

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', os.path.join(pkg, 'rviz', 'slam.rviz')],
                parameters=[sim_time], condition=IfCondition(LaunchConfiguration('rviz')))

    teleop = Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
                  name='teleop_twist_keyboard', output='screen',
                  prefix='xterm -T teleop_twist_keyboard -e',
                  condition=IfCondition(LaunchConfiguration('enable_teleop')))

    # PS5 controller: joy_node reads the pad, ps5_teleop turns /joy into /cmd_vel
    joy_cond = IfCondition(LaunchConfiguration('enable_joy'))
    joy = Node(package='joy', executable='joy_node', name='joy_node', output='screen',
               parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}],
               condition=joy_cond)
    ps5 = Node(package=PKG, executable='ps5_teleop.py', name='ps5_teleop', output='screen',
               condition=joy_cond)

    print(f'[gazebo_slam] Gazebo {g["name"]} | world: {world} | '
          f'spawn via {"SDF (patched friction)" if spawn_sdf else "URDF fallback"}')
    return [gazebo, rsp, spawn, bridge, slam, slam_lifecycle, rviz, teleop, joy, ps5]


def generate_launch_description():
    def arg(name, default, desc):
        return DeclareLaunchArgument(name, default_value=default, description=desc)

    return LaunchDescription([
        arg('world', 'custom_map.sdf',
            'World file: a name inside share/<pkg>/worlds or an absolute path'),
        arg('gui', 'true', 'Show the Gazebo window (false = headless server)'),
        arg('rviz', 'true', 'Start RViz with slam.rviz'),
        arg('slam', 'true', 'Start slam_toolbox'),
        arg('enable_teleop', 'false', 'Open teleop_twist_keyboard in an xterm window'),
        arg('enable_joy', 'false', 'PS5 controller teleop (joy_node + ps5_teleop)'),
        arg('x', '0.0', 'Spawn x'), arg('y', '0.0', 'Spawn y'),
        arg('z', '0.03', 'Spawn z (wheel bottoms sit 0.019 m below base_footprint)'),
        arg('yaw', '0.0', 'Spawn yaw'),
        OpaqueFunction(function=launch_setup),
    ])