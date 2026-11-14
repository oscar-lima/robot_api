"""Several robots in one process: each Base, Arm and Gripper reaches the action servers of its own namespace, also
when another robot was built after it. No ROS master: the action client and the topic check are stubbed, every client
records the server name it was made for and the goals it got."""
import unittest
from unittest import mock

from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseGoal
from robot_api import core, extensions, ros_wrapper

wrapper = core._ros_wrapper


class FakeClient:
    made = []
    during_wait = []  # hooks run while a client waits for its server (another thread's work meanwhile)

    def __init__(self, name, action):
        self.name = name
        self.goals = []
        FakeClient.made.append(self)

    def wait_for_server(self, timeout=None):
        while FakeClient.during_wait:
            FakeClient.during_wait.pop(0)()
        return True

    def send_goal_and_wait(self, goal, timeout=None):
        self.goals.append(goal)
        return 3  # SUCCEEDED

    def send_goal(self, goal, done_cb=None):
        self.goals.append(goal)

    def get_result(self):
        return None


def build(namespace):
    return (core.Base(namespace, False), extensions.Arm(namespace, False), extensions.Gripper(namespace, False))


def goal():
    move_base_goal = MoveBaseGoal()
    move_base_goal.target_pose.header.frame_id = "map"
    move_base_goal.target_pose.pose = Pose()
    move_base_goal.target_pose.pose.orientation.w = 1.0
    return move_base_goal


def client_names():
    return [client.name for client in FakeClient.made]


class SeveralRobotsInOneProcess(unittest.TestCase):
    def setUp(self):
        patches = [
            mock.patch.object(ros_wrapper.actionlib, "SimpleActionClient", FakeClient),
            mock.patch.object(wrapper, "_has_action_result", lambda server_name, namespace=None: True, create=True),
            mock.patch.object(wrapper, "list_params", lambda namespace="": [], create=True),
            mock.patch.dict(wrapper.__dict__, {"_channels": {}, "_namespace": ""}),
        ]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)
        FakeClient.made.clear()
        FakeClient.during_wait.clear()

    def test_the_robot_built_first_keeps_its_own_servers(self):
        base1, arm1, gripper1 = build("/mobipick/")
        base2, arm2, gripper2 = build("/mobipick2/")  # e.g. a GUI switches to robot 2
        base1.move_to_goal(goal())
        arm1.move("home")
        gripper1.open()
        assert client_names() == ["/mobipick/move_base", "/mobipick/moveit_macros", "/mobipick/gripper_hw"]
        base2.move_to_goal(goal())
        arm2.move("home")
        gripper2.close()
        assert client_names()[3:] == ["/mobipick2/move_base", "/mobipick2/moveit_macros", "/mobipick2/gripper_hw"]
        assert [len(client.goals) for client in FakeClient.made] == [1] * 6


    def test_a_robot_built_while_another_connects_does_not_take_over_that_client(self):
        # robot 2's arm connects; while it waits for its server, robot 1 is built (another thread); afterwards robot 1's
        # arm must still reach robot 1's moveit_macros and robot 2's client must stay robot 2's
        _, arm2, _ = build("/mobipick2/")
        built = []
        FakeClient.during_wait.append(lambda: built.append(build("/mobipick/")))
        arm2.move("home")
        _, arm1, _ = built[0]
        arm1.move("home")
        assert client_names() == ["/mobipick2/moveit_macros", "/mobipick/moveit_macros"]
        moveit = wrapper.get_moveit_topic_name()
        assert arm2._channel.get_action_client(moveit).name == "/mobipick2/moveit_macros"
        assert arm1._channel.get_action_client(moveit).name == "/mobipick/moveit_macros"


    def test_the_parts_of_one_robot_share_one_channel_and_reuse_its_clients(self):
        base, arm, gripper = build("/mobipick/")
        assert base._channel is arm._channel is gripper._channel is wrapper.channel("/mobipick/")
        arm.move("home")
        core.Base("/mobipick/", False)  # building the same robot again keeps the connected clients
        arm.move("home")
        assert client_names() == ["/mobipick/moveit_macros"]


    def test_each_robot_launches_moveit_macros_in_its_own_namespace(self):
        build("/mobipick/")
        build("/mobipick2/")
        moveit = wrapper.get_moveit_topic_name()
        assert wrapper._server_spec(wrapper.channel("/mobipick/"), moveit)[1].endswith("namespace:='mobipick'")
        assert wrapper._server_spec(wrapper.channel("/mobipick2/"), moveit)[1].endswith("namespace:='mobipick2'")


    def test_the_old_wrapper_attributes_still_answer_for_the_robot_built_last(self):
        _, arm1, _ = build("/mobipick/")
        arm1.move("home")
        moveit = wrapper.get_moveit_topic_name()
        assert wrapper._action_clients[moveit].name == "/mobipick/moveit_macros"
        assert wrapper._last_server_name == moveit
        build("/mobipick2/")
        assert moveit not in wrapper._action_clients  # robot 2 has not connected yet
        assert wrapper.get_action_client(moveit) is None
        assert wrapper.get_action_client(moveit, "/mobipick/").name == "/mobipick/moveit_macros"
        # the namespace-less wrapper calls of old code go to the robot built last, as before
        assert wrapper._connect_to_action_server(moveit)
        assert client_names()[-1] == "/mobipick2/moveit_macros"


if __name__ == "__main__":
    unittest.main()
