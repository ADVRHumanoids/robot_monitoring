import os

try:
    from . import ros2_wrapper
    RosWrapperClass = ros2_wrapper.Ros2Utils
    type RosWrapper = ros2_wrapper.Ros2Utils
except ImportError:
    from . import ros1_wrapper
    RosWrapperClass = ros1_wrapper.Ros2Utils
    type RosWrapper = ros1_wrapper.Ros1Utils

ros_handle: RosWrapper = None


def resolve_ros_uri(uri: str):
    
    if uri.startswith('package://'):
        tokens = uri[10:].split('/')
        pkg = tokens[0]
        pkg_path = ros_handle.get_package_share_directory(pkg)
        return os.path.join(pkg_path, *tokens[1:])
    else:
        return uri