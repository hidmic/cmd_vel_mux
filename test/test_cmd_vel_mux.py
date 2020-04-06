# Copyright 2020 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
# * Neither the name of {copyright_holder} nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import contextlib
import functools
import re
import unittest
import yaml

import geometry_msgs.msg

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch_ros.substitutions
import launch_testing.actions
import launch_testing.tools

from rosidl_runtime_py import message_to_yaml


def generate_test_description():
    path_to_cmd_vel_mux_params = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare(package='cmd_vel_mux'),
        'config',
        'cmd_vel_mux_params.yaml'
    ])
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cmd_vel_mux',
            node_executable='cmd_vel_mux_node',
            parameters=[path_to_cmd_vel_mux_params],
            output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestCmdVelMux(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output
    ):
        @contextlib.contextmanager
        def launch_topic_command(self, arguments):
            topic_command_action = launch.actions.ExecuteProcess(
                cmd=['ros2', 'topic', *arguments],
                additional_env={'PYTHONUNBUFFERED': '1'},
                name='ros2topic-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, topic_command_action, proc_info, proc_output
            ) as topic_command:
                yield topic_command
        cls.launch_topic_command = launch_topic_command

    def test_idle_mux(self):
        with self.launch_topic_command(
            arguments=[
                'echo',
                '--qos-reliability', 'reliable',
                '--qos-durability', 'transient_local',
                'active'
            ]
        ) as command:
            assert command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    'data: idle',
                    '---'
                ], strict=True
            ), timeout=2)
        with self.launch_topic_command(arguments=['echo', 'cmd_vel']) as command:
            assert not command.wait_for_output(lambda output: len(output) > 0, timeout=2)

    def test_mux_with_single_input(self):
        default_twist = geometry_msgs.msg.Twist()
        default_twist.linear.x = 1.0
        default_twist_block_yaml = message_to_yaml(default_twist)
        default_twist_inline_yaml = yaml.dump(yaml.load(default_twist_block_yaml))
        with self.launch_topic_command(
            arguments=[
                'pub', '-r', '20',
                'input/default',
                'geometry_msgs/msg/Twist',
                f'{default_twist_inline_yaml}'
            ]
        ):
            with self.launch_topic_command(
                arguments=[
                    'echo',
                    '--qos-reliability', 'reliable',
                    '--qos-durability', 'transient_local',
                    'active'
                ]
            ) as command:
                assert command.wait_for_output(functools.partial(
                    launch_testing.tools.expect_output, expected_lines=[
                        "data: default_input",
                        '---'
                    ], strict=False
                ), timeout=2)
            with self.launch_topic_command(arguments=['echo', 'cmd_vel']) as command:
                assert command.wait_for_output(functools.partial(
                    launch_testing.tools.expect_output, expected_lines=[
                        *default_twist_block_yaml.splitlines(),
                        '---'
                    ], strict=True
                ), timeout=2)

    def test_mux_priority_override(self):
        default_twist = geometry_msgs.msg.Twist()
        default_twist.linear.x = 1.0
        default_twist_block_yaml = message_to_yaml(default_twist)
        default_twist_inline_yaml = yaml.dump(yaml.load(default_twist_block_yaml))

        joystick_twist = geometry_msgs.msg.Twist()
        joystick_twist.angular.z = 1.0
        joystick_twist_block_yaml = message_to_yaml(joystick_twist)
        joystick_twist_inline_yaml = yaml.dump(yaml.load(joystick_twist_block_yaml))

        with self.launch_topic_command(
            arguments=[
                'pub', '-r', '20',
                'input/default',
                'geometry_msgs/msg/Twist',
                f'{default_twist_inline_yaml}'
            ]
        ), self.launch_topic_command(
            arguments=[
                'pub', '-r', '20',
                'input/joystick',
                'geometry_msgs/msg/Twist',
                f'{joystick_twist_inline_yaml}'
            ]
        ):
            with self.launch_topic_command(
                arguments=[
                    'echo',
                    '--qos-reliability', 'reliable',
                    '--qos-durability', 'transient_local',
                    'active'
                ]
            ) as command:
                assert command.wait_for_output(functools.partial(
                    launch_testing.tools.expect_output, expected_lines=[
                        'data: navigation_stack_controller',
                        '---'
                    ], strict=False
                ), timeout=2)
            with self.launch_topic_command(arguments=['echo', 'cmd_vel']) as command:
                assert command.wait_for_output(functools.partial(
                    launch_testing.tools.expect_output, expected_lines=[
                        *joystick_twist_block_yaml.splitlines(),
                        '---'
                    ], strict=True
                ), timeout=2)

    def test_mux_timeout(self):
        with self.launch_topic_command(
            arguments=[
                'pub', '-r', '8',
                'input/default',
                'geometry_msgs/msg/Twist',
            ]
        ) as pub_command:
            assert pub_command.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    'publisher: beginning loop',
                    re.compile('publishing #1: .*')
                ], strict=True
            ), timeout=5)

            with self.launch_topic_command(
                arguments=[
                    'echo',
                    '--qos-reliability', 'reliable',
                    '--qos-durability', 'transient_local',
                    'active'
                ]
            ) as echo_command:
                assert echo_command.wait_for_output(functools.partial(
                    launch_testing.tools.expect_output, expected_lines=[
                        'data: default_input',
                        '---',
                        'data: idle',
                        '---'
                    ], strict=False
                ), timeout=2)
