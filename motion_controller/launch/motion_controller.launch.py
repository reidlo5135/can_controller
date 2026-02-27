import os;
import launch;
import launch_ros.actions;
from ament_index_python.packages import get_package_share_directory;
from launch.actions import IncludeLaunchDescription;
from launch.launch_description_sources import PythonLaunchDescriptionSource;

def generate_launch_description() -> launch.LaunchDescription:

    package_name: str = "motion_controller";
    package_shared_directory: str = get_package_share_directory(package_name);
    parameter: str = os.path.join(package_shared_directory, "config", f"{package_name}.yaml");

    motion_controller_node: launch_ros.actions.Node = launch_ros.actions.Node(
        package=package_name,
        executable=package_name,
        name=package_name,
        output="screen",
        parameters = [parameter]
    );

    ld: launch.LaunchDescription = launch.LaunchDescription();
    ld.add_action(motion_controller_node);

    return ld;