from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # =============================================
    # Video Streaming Configuration
    # =============================================
    SINGLE_IP = '10.42.0.1'
    MULTICAST_IP = '239.10.10.10'

    VIDEO_PORT = 5000
    VIDEO_CONTROL_PORT = 7000

    CAMERA_DEVICE = '/dev/video0'

    #enable_usb_cam_arg = DeclareLaunchArgument(
    #    'enable_usb_cam',
    #    default_value='true',
    #    description='เปิดใช้งานกล้อง USB Webcam (true/false)'
    #)

    enable_pi_cam_arg = DeclareLaunchArgument(
        'enable_pi_cam',
        default_value='true',
        description='เปิดใช้งานกล้องสายแพ Pi Camera (true/false)'
    )
    
    #use_usb_cam = LaunchConfiguration('enable_usb_cam')
    use_pi_cam = LaunchConfiguration('enable_pi_cam')

    # หา path ของ config YAML สำหรับ teleop_twist_joy
    pkg_share = get_package_share_directory('DTIscount_robot')
    teleop_config = os.path.join(pkg_share, 'config', 'teleop_joy.yaml')
    #rviz_config = os.path.join(pkg_share, 'rviz', 'jaguar.rviz')
    
    #usb_camera_node = Node(
    #    package='camera_ros',
    #    executable='camera_node',
    #    name='usb_camera_node',
    #    namespace='usb_cam',
    #    output='screen',
    #    parameters=[{
    #        'camera': 0,
    #        'width': 640,
    #        'height': 480,
    #        'format': 'YUYV',
    #        'frame_rate': 15,
    #        'qos_reliability': 'best_effort'
    #    }],
    #    condition=IfCondition(use_usb_cam)
    #)

    video_stream_process = ExecuteProcess(
        cmd=[
            '/bin/bash',
            '-c',
            '''
            MODE_FILE=/tmp/dtiscout_video_mode

            SINGLE_IP=10.42.0.1
            MULTICAST_IP=239.10.10.10

            VIDEO_PORT=5000
            CONTROL_PORT=7000

            CAMERA_DEVICE=/dev/video0

            # =========================================
            # Initial mode
            # =========================================
            echo "single" > "$MODE_FILE"

            echo "[VIDEO] Stream controller started"
            echo "[VIDEO] Default mode = SINGLE"
            echo "[VIDEO] Control port = $CONTROL_PORT"

            # =========================================
            # UDP control server
            #
            # Notebook ส่ง:
            #   single
            #   multicast
            #
            # มาที่ Robot:7000
            # =========================================
            socat -u \
                UDP4-RECVFROM:${CONTROL_PORT},reuseaddr,fork \
                SYSTEM:'read MODE; MODE=$(echo "$MODE" | tr -d "\\r\\n "); case "$MODE" in single|multicast) echo "$MODE" > /tmp/dtiscout_video_mode; echo "[VIDEO] Mode command: $MODE" ;; esac' \
                >/dev/null 2>&1 &

            CONTROL_PID=$!

            GST_PID=""
            CURRENT_MODE=""

            # ==========================================
            # Cleanup
            # ==========================================
            cleanup()
            {
                echo "[VIDEO] Cleaning up..."

                if [ -n "$GST_PID" ]; then
                    kill "$GST_PID" 2>/dev/null
                fi

                if [ -n "$CONTROL_PID" ]; then
                    kill "$CONTROL_PID" 2>/dev/null
                fi

                exit 0
            }

            trap cleanup SIGINT SIGTERM EXIT

            # ==========================================
            # Main video supervisor
            # ==========================================
            while true
            do

                MODE=$(cat "$MODE_FILE" 2>/dev/null)

                # --------------------------------------
                # ถ้า mode เปลี่ยน
                # --------------------------------------
                if [ "$MODE" != "$CURRENT_MODE" ]
                then

                    echo "======================================"
                    echo "[VIDEO] Changing mode"
                    echo "[VIDEO] New mode = $MODE"
                    echo "======================================"

                    # ----------------------------------
                    # Stop GStreamer ตัวเก่า
                    # ----------------------------------
                    if [ -n "$GST_PID" ]
                    then
                        echo "[VIDEO] Stopping old GStreamer..."
                        kill "$GST_PID" 2>/dev/null
                        wait "$GST_PID" 2>/dev/null
                        GST_PID=""
                    fi

                    # ==================================
                    # SINGLE MODE
                    # ==================================
                    if [ "$MODE" = "single" ]
                    then

                        echo "[VIDEO] Starting SINGLE stream"
                        echo "[VIDEO] Destination = $SINGLE_IP:$VIDEO_PORT"

                        gst-launch-1.0 -q \
                            v4l2src device="$CAMERA_DEVICE" ! \
                            image/jpeg,width=640,height=480,framerate=15/1 ! \
                            jpegparse ! \
                            rtpjpegpay ! \
                            udpsink host="$SINGLE_IP" port="$VIDEO_PORT" \
                            sync=false async=false &

                        GST_PID=$!

                    # ==================================
                    # MULTICAST MODE
                    # ==================================
                    elif [ "$MODE" = "multicast" ]
                    then

                        echo "[VIDEO] Starting MULTICAST stream"
                        echo "[VIDEO] Destination = $MULTICAST_IP:$VIDEO_PORT"

                        gst-launch-1.0 -q \
                            v4l2src device="$CAMERA_DEVICE" ! \
                            image/jpeg,width=320,height=240,framerate=5/1 ! \
                            jpegparse ! \
                            rtpjpegpay ! \
                            udpsink host="$MULTICAST_IP" \
                            port="$VIDEO_PORT" \
                            auto-multicast=true \
                            ttl-mc=1 \
                            sync=false async=false &

                        GST_PID=$!

                    else

                        echo "[VIDEO] Unknown mode: $MODE"
                        echo "[VIDEO] Falling back to SINGLE"

                        echo "single" > "$MODE_FILE"

                    fi

                    CURRENT_MODE="$MODE"

                fi

                # --------------------------------------
                # ตรวจสอบ GStreamer ยังทำงานหรือไม่
                # --------------------------------------
                if [ -n "$GST_PID" ]
                then
                    if ! kill -0 "$GST_PID" 2>/dev/null
                    then
                        echo "[VIDEO] GStreamer stopped unexpectedly"

                        GST_PID=""
                        CURRENT_MODE=""
                    fi
                fi

                sleep 0.2

            done
            '''
        ],
        output='screen'
    )
            
    pi_camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='pi_camera_node',
        namespace='pi_cam',
        output='screen',
        parameters=[{
            'camera': 1,
            'width': 160,
            'height': 120,
            'format': 'YUYV',
            'frame_rate': 10,
            'qos_reliability': 'best_effort'
        }],
        condition=IfCondition(use_pi_cam)
    ) 
 
    audio_stream_process = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            'while true; do arecord -D hw:1,0 -r 16000 -f S16_LE -c 1 -t raw | socat - UDP4:10.42.0.1:5001; sleep 1; done'
        ],
        output='screen'
    )

    audio_receive_process = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            'while true; do socat UDP4-LISTEN:6000,reuseaddr - | aplay -r 16000 -f S16_LE -c 1 -t raw; sleep 1; done' 
        ],
        output='screen'
    )

    return LaunchDescription([
        # ===============================
        # Joystick driver
        # ===============================
        #Node(
        #    package='joy',
        #    executable='joy_node',
        #    name='joy_node',
        #    output='screen',
        #    parameters=[
        #        { 
        #            'device_id': 0,
        #            'device_name': '/dev/input/event2'
        #        }
        #    ]
        #),

        # ===============================
        # Teleop twist from joystick
        # ===============================
        #Node(
        #    package='teleop_twist_joy',
        #    executable='teleop_node',
        #    name='teleop_twist_joy_node',
        #    output='screen',
        #    parameters=[teleop_config]
        #),

        # ===============================
        # CmdVel to diff drive converter
        # ===============================
        Node(
            package='DTIscount_robot',
            executable='diffdrive_node',
            name='diff_drive_node',
            output='screen',
            parameters=[
                {'wheel_radius': 0.0375},   # ปรับตามล้อจริง
                {'wheel_base': 0.16}       # ระยะล้อซ้าย-ขวา
            ]
        ),

        #================================
        # Webcamera
        #================================
        #enable_usb_cam_arg,
        #enable_pi_cam_arg,
        #usb_camera_node,
        video_stream_process,
        #pi_camera_node,
        audio_stream_process,
        audio_receive_process,

        # ===============================
        # Odom and TF publisher
        # ===============================
        # Node(
        #     package='jaguar_bringup',
        #     executable='odom_node',
        #     name='odom_node',
        #     output='screen'
        # ),
        
        # ===============================
        # Static transform publisher
        # ===============================
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0','0','0','0','0','0','base_footprint','base_link']
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0','0','0','0','0','0','base_link','laser']
        # ),
        
        # ===============================
        # Rviz2
        # ===============================
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config],
        #     output='screen'
        # ),

        # ===============================
        # DDSM driver node
        # ===============================
        Node(
            package='DTIscount_robot',
            executable='DDSMdriver_node',
            name='DDSMdriver_node',
            output='screen'
        )
    ])
