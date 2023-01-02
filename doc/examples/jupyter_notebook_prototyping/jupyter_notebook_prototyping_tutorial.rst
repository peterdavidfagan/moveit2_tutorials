Jupyter Notebook Prototyping
==================================
.. raw:: html
        
        <iframe width="560" height="315" src="https://www.youtube.com/embed/7KvF7Dj7bz0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


In this tutorial we will demonstrate how you can leverage jupyter notebooks with the MoveIt 2 Python API in order to quickly prototype. This tutorial is broken down into the following sections:

* **Getting Started:** An outline of the tutorial setup requirements.
* **Understanding the launch file:** An outline of the launch file specification.
* **Notebook Setup:** Notebook imports and configuration.
* **Motion Planning Example:** An example of using the moveit_py API to plan a motion.
* **Teleoperation Example:** An example of using the moveit_py API to teleoperate a robot with a joystick.

The code for this tutorial can be found `here <https://github.com/peterdavidfagan/moveit2_tutorials/tree/moveit_py_notebook_tutorial/doc/examples/jupyter_notebook_prototyping>`_.

Getting Started
---------------
In order to complete this tutorial, you must have set up a workspace that includes MoveIt2 and its corresponding tutorials. An excellent outline on how to set up such a workspace is provided in the :doc:`Getting Started Guide </doc/tutorials/getting_started/getting_started>`, please consult this guide for further information.

Once you have setup your workspace, you can execute the code for this tutorial by running the following command: ::
        
        ros2 launch moveit2_tutorials jupyter_notebook.launch.py

This will launch the nodes that are necessary to complete this tutorial. Importantly, it will also launch a jupyter notebook server that you can connect to in order to run the code in this tutorial. If your browser doesn't automatically open the jupyter notebook interface you can connect to the notebook server by navigating to http://localhost:8888 in your browser. You will be prompted to enter a token, which is printed to the terminal when you launch the launch file, you can enter this token to connect to the notebook server. There will also be a url that contains the token printed in the terminal output you may alternatively leverage this url to connect to the notebook server, this will avoid having to enter the token manually.

Once you have completed these steps you are ready to progress with the tutorial. Before executing the notebook code a brief outline of the launch file specification is provided to ensure understanding of how to launch a notebook instance. 


Understanding the launch file
-----------------------------
The key distinguishing factor between the `launch file <https://github.com/peterdavidfagan/moveit2_tutorials/blob/moveit_py_notebook_tutorial/doc/examples/jupyter_notebook_prototyping/launch/jupyter_notebook_prototyping.launch.py>`_ used in this tutorial when compared with other tutorials is the starting of a jupyter notebook server. As a result, we will be brief when reviewing common launch file code and focus mainly on the starting of the notebook server. For a detailed review of launch files please consult the following `guide<https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.html>`_.

Importing required packages: ::

        import os
        from launch import LaunchDescription
        from launch_ros.actions import Node, SetParameter
        from launch.actions import ExecuteProcess
        from ament_index_python.packages import get_package_share_directory
        from moveit_configs_utils import MoveItConfigsBuilder

We start by defining our moveit configuration, this step will also be important for later when configuring our notebook. ::

        moveit_config = (
                MoveItConfigsBuilder(robot_name="panda", package_name="moveit_resources_panda_moveit_config")
                .robot_description(file_path="config/panda.urdf.xacro")
                .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
                .moveit_cpp(
                        file_path=get_package_share_directory("moveit2_tutorials")
                        + "/config/jupyter_notebook_prototyping.yaml"
                )
                .to_moveit_configs()
        )

Once our moveit configuration is defined we start the following set of nodes:

* **rviz_node:** starts rviz2 for visualization purposes.
* **static_tf:** publishes the static transforms between the world frame and panda base frame.
* **robot_state_publisher:** publishes updated robot state information (transforms).
* **ros2_control_node:** used to control groups of joints.

::

        rviz_config_file = (
                get_package_share_directory("moveit2_tutorials") + "/config/jupyter_notebook_prototyping.rviz"
        )
        rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config_file],
                parameters=[
                        moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                ],
        )

        static_tf = Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                output="log",
                arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
        )

        robot_state_publisher = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[moveit_config.robot_description],
        )

        ros2_controllers_path = os.path.join(
                get_package_share_directory("moveit_resources_panda_moveit_config"),
                "config",
                "ros2_controllers.yaml",
        )
        ros2_control_node = Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[moveit_config.robot_description, ros2_controllers_path],
                output="both",
        )

        load_controllers = []
        for controller in [
                "panda_arm_controller",
                "panda_hand_controller",
                "joint_state_broadcaster",
        ]:
                load_controllers += [
                        ExecuteProcess(
                        cmd=["ros2 run controller_manager spawner {}".format(controller)],
                        shell=True,
                        output="screen",)
                        ]

Having defined the setup for each of these nodes, we also define a process that starts our jupyter notebook server: ::

        start_notebook = ExecuteProcess(cmd = ["python3 -m notebook"], shell = True, output = "screen")

Finally, we return a launch description that includes all of the nodes and processes that we have defined: ::

        return LaunchDescription(
                [
                static_tf,
                robot_state_publisher,
                rviz_node,
                ros2_control_node,
                start_notebook,
                ]
                + load_controllers
                )

Notebook Setup
--------------
Now that we have launched our jupyter notebook server we can begin to execute the code in the notebook. The first step is to import the required packages: ::

        import os
        import sys
        import yaml
        import rclpy
        import numpy as np

        # message libraries
        from geometry_msgs.msg import PoseStamped, Pose

        # moveit_py
        from moveit_py.planning import MoveItPy
        from moveit_py.core import RobotState

        # config file libraries
        from moveit_configs_utils import MoveItConfigsBuilder
        from ament_index_python.packages import get_package_share_directory

Once we have imported the required packages we need to define our moveit_py node configuration. We do this through using the MoveItConfigsBuilder as follows: ::

        moveit_config = (
                MoveItConfigsBuilder(robot_name="panda", package_name="moveit_resources_panda_moveit_config")
                .robot_description(file_path="config/panda.urdf.xacro")
                .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
                .moveit_cpp(
                file_path=get_package_share_directory("moveit2_tutorials")
                + "/config/jupyter_notebook_prototyping.yaml"
                )
                .to_moveit_configs()
        ).to_dict()

where we convert the generated configuration instance to a dictionary so we can use it to initialise our moveit_py node. Finally we initialise a moveit_py node: ::
        
        # instantiate moveit_py instance and a planning component for the panda_arm
        moveit = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
        panda_arm = moveit.get_planning_component("panda_arm")

Motion Planning Example
-----------------------
We can start by demonstrating the planning and execution of a rudimentary motion from within the notebook: ::

        # set plan start state using predefined state
        panda_arm.set_start_state("ready")

        # set pose goal using predefined state
        panda_arm.set_goal(goal_state_name = "extended")

        # plan to goal
        plan_result = panda_arm.plan()

        # execute the plan
        if plan_result:
                panda_arm.execute()

Great, so we can perform motion planning interactively (see the motion planning tutorial for further details of the motion planning API).Suppose we are developing our code and we make a mistake such as follows: ::

        # set plan start state using predefined state
        panda_arm.set_start_state("ready") # This conflicts with the current robot configuration and will cause and error

        # set goal using a pose message this time
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "panda_link0"
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = 0.28
        pose_goal.pose.position.y = -0.2
        pose_goal.pose.position.z = 0.5
        panda_arm.set_goal(goal_pose_msg = pose_goal, link_name = "panda_link8")

        # plan to goal
        plan_result = panda_arm.plan()

        # execute the plan
        if plan_result:
                panda_arm.execute()

Since we are using a notebook this mistake is easy to rectify without having to recompile any files. Simply edit the above notebook to match the below and rerun the cell: ::

        # set plan start state using predefined state
        panda_arm.set_start_state_to_current_state() # This conflicts with the current robot pose and will cause and error

        # set goal using a pose message this time
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "panda_link0"
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = 0.28
        pose_goal.pose.position.y = -0.2
        pose_goal.pose.position.z = 0.5
        panda_arm.set_goal(goal_pose_msg = pose_goal, link_name = "panda_link8")

        # plan to goal
        plan_result = panda_arm.plan()

        # execute the plan
        if plan_result:
                panda_arm.execute()

Teleoperation Example
---------------------

One may also want to perform live teleoperation of their robot. Wouldn't it be nice if we could interactively start/stop teleoperation without shutting down and subsequently relaunching all processes. In this example, we are going to show how this is possible with notebooks through a motivating example of teleoperating the robot, performing motion planning and teleoperating the robot again.

For this section you will need a device that support teleoperation with moveit_py, in this case we leverage the PS4 dualshock controller.

We start by launching a servo client through directly launching the following `launch file <https://github.com/peterdavidfagan/moveit2_tutorials/blob/moveit_py_notebook_tutorial/doc/examples/jupyter_notebook_prototyping/launch/notebook_start_servo_client.launch.py>`_: ::
        
        # launch servo client from within notebook
        !ros2 launch moveit2_tutorials notebook_start_servo_client.launch.py

We can then start the teleoperation node: ::

        from moveit_py.servo_client.devices.ps4_dualshock import PS4DualShockTeleop

        # instantiate the teleoperating device
        ps4 = PS4DualShockTeleop()

        # start teleloperating the robot
        ps4.start_teleop()

Oh wait but I actually want to try perform some motion planning to bring the robot back to its default configuration, no problem: ::

        # stop teleoperating the robot
        ps4.stop_teleop()

        # plan and execute
        # set plan start state using predefined state
        panda_arm.set_start_state_to_current_state()

        # set pose goal using predefined state
        panda_arm.set_goal(goal_state_name = "ready")

        # plan to goal
        plan_result = panda_arm.plan()

        # execute the plan
        if plan_result:
                panda_arm.execute()

Ok great now we are back at our default configuration, lets start teleoperating the robot again: ::

        ps4.start_teleop()
