import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    camera = launch_ros.actions.Node(
            package = 'autoRobo',
            executable = 'ImgProcess',
            output = 'screen',
            )
    serial = launch_ros.actions.Node(
            package = 'autoRobo',
            executable = 'serial',
            output = 'screen',
            )
    ope = launch_ros.actions.Node(
            package = 'autoRobo',
            executable = 'operater',
            output = 'screen',
            )
    localization = launch_ros.actions.Node(
            package = 'autoRobo',
            executable = 'Localization',
            output = 'screen',
            )
    usb_cam = launch_ros.actions.Node(
            package = 'usb_cam',
            executable = 'usb_cam_node_exe',
            parameters=[{'image_width': 1280,
                         'image_height': 720,
                         'pixel_format': "yuyv",
                         'brightness': 130,
                         'framerate': 10.0}],
            output = 'screen',
            )
    web = launch_ros.actions.Node(
            package = 'rosbridge_server',
            executable = 'rosbridge_websocket.py',
            parameters=[{'port': 9090}],
            output = 'screen',
            )
    return launch.LaunchDescription([usb_cam,camera,serial,web,localization,ope])
