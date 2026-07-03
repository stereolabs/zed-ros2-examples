# Copyright 2025 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Integration test for the zed_topic_benchmark node.
#
# A synthetic publisher feeds a topic at a known rate; the benchmark node is
# configured to stop after a fixed number of samples and to write its report to
# a file. The test asserts that the node prints the report, terminates on its
# own, exits cleanly and produces a report file with the expected content.

import os
import tempfile
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers

import pytest

TEST_TOPIC = '/zed_benchmark_it/test_topic'
SAMPLE_COUNT = 30
PUBLISH_RATE_HZ = 20
REPORT_FILE = os.path.join(
    tempfile.mkdtemp(prefix='zed_benchmark_it_'), 'report.txt')


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Synthetic publisher: a constant-rate std_msgs/String stream on the topic
    # that the benchmark node will subscribe to.
    publisher = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-r', str(PUBLISH_RATE_HZ),
            TEST_TOPIC, 'std_msgs/msg/String',
            'data: "zed benchmark integration-test payload"'
        ],
        output='screen'
    )

    # Benchmark node under test: stop after SAMPLE_COUNT messages and save the
    # report to REPORT_FILE.
    benchmark = launch_ros.actions.Node(
        package='zed_topic_benchmark',
        executable='zed_topic_benchmark',
        name='topic_benchmark',
        output='screen',
        parameters=[{
            'topic_name': TEST_TOPIC,
            'test_sample_count': SAMPLE_COUNT,
            'use_ros_log': True,
            'log_file_path': REPORT_FILE,
        }]
    )

    return (
        launch.LaunchDescription([
            publisher,
            benchmark,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'benchmark': benchmark, 'publisher': publisher}
    )


class TestBenchmarkRuntime(unittest.TestCase):
    """Checks performed while the launched processes are running."""

    def test_report_emitted_to_console(self, proc_output, benchmark):
        # The benchmark must print its final report on completion.
        proc_output.assertWaitFor(
            'ZED TOPIC BENCHMARK REPORT', process=benchmark, timeout=60)

    def test_stop_reason_logged(self, proc_output, benchmark):
        # The console must state why the test stopped.
        proc_output.assertWaitFor(
            'sample count reached', process=benchmark, timeout=60)

    def test_node_terminates(self, proc_info, benchmark):
        # Reaching the sample-count limit must make the node exit on its own
        # (no external shutdown needed).
        proc_info.assertWaitForShutdown(process=benchmark, timeout=60)


@launch_testing.post_shutdown_test()
class TestBenchmarkShutdown(unittest.TestCase):
    """Checks performed after the launched processes have shut down."""

    def test_clean_exit(self, proc_info, benchmark):
        # The self-terminating benchmark node must exit with code 0.
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
            process=benchmark)

    def test_report_file_written(self):
        self.assertTrue(
            os.path.isfile(REPORT_FILE),
            'The benchmark report file was not created')

        with open(REPORT_FILE, 'r') as report:
            content = report.read()

        # Header and discovered topic type.
        self.assertIn('ZED TOPIC BENCHMARK REPORT', content)
        self.assertIn('std_msgs/msg/String', content)
        # Termination reason.
        self.assertIn('sample count reached', content)
        # Exactly the requested number of messages was accounted for.
        self.assertIn(
            'Messages received: {}'.format(SAMPLE_COUNT), content)
        # The statistics block is present.
        self.assertIn('Frequency [Hz]', content)
        self.assertIn('Bandwidth [Mbps]', content)
