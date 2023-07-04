import csv
import rclpy
from rclpy.node import Node
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from math import pi as PI

initial_traj_file_path = "/home/emanuele/start_traj.csv"

init_pose = [0, 0, 0, 0, 0, 0, 0]
cube_data = {
    "red": {"pose_stamped_start": init_pose, "pose_stamped_dest": init_pose},
    "green": {"pose_stamped_start": init_pose, "pose_stamped_dest": init_pose},
    "blu": {"pose_stamped_start": init_pose, "pose_stamped_dest": init_pose},
    "yellow": {"pose_stamped_start": init_pose, "pose_stamped_dest": init_pose},
}

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        with open(initial_traj_file_path) as f:
            self.pose_list = list(csv.reader(f))

        ##
        self.ee_actual_pose = PoseStamped()
        self.fsm_state = 0
        self.cube_order = ["yellow", "green", "blu", "red"]
        self.cube_idx = 0
        self.ee_rots = {
            "green": quaternion_from_euler(0, 0, -PI / 2),
            "yellow": quaternion_from_euler(0, 0, PI / 4),
            "blu": quaternion_from_euler(0, 0, 0),
            "red": quaternion_from_euler(0, 0, 0)
        }
        self.curr_traj = {"path": [], "time": [], "time_idx": 0}
        self.time_tracker = {
            "start_tick": 0,
            "active": False
        }
        self.timer_period_publisher = 0.01  # seconds
        self.traj_ts = 0.001  # seconds
        self.prev_pose_target = PoseStamped()
        self.position_epsilon = 0.1

        # POSE PUB
        self.publisher_ = self.create_publisher(
            PoseStamped, '/ur5/ee_target/pose', 1)
        self.timer = self.create_timer(
            self.timer_period_publisher, self.timer_callback)
        self.i = 0

        # GRIPPER PUB (1 close, 0 open)
        self.GRIPPER_CLOSE = True
        self.GRIPPER_OPEN = False
        self.gripper_status = -1
        self.gripper_pub = self.create_publisher(
            Bool, '/wsg_50/controller/command', 1)

        # POSE SUB
        self.subscription = self.create_subscription(
            PoseStamped,
            '/ur5/ee_actual/pose',
            self.callback_ee_actual_pose,
            10)
        self.subscription

        # CUBE POS SUB
        self.subscription2 = self.create_subscription(
            PoseStamped,
            '/cube_blu/pose',
            self.callback_blu_cube_pose,
            10)
        self.subscription2

        self.subscription3 = self.create_subscription(
            PoseStamped,
            '/cube_red/pose',
            self.callback_red_cube_pose,
            10)
        self.subscription3

        self.subscription3 = self.create_subscription(
            PoseStamped,
            '/cube_yellow/pose',
            self.callback_yellow_cube_pose,
            10)
        self.subscription3

        self.subscription4 = self.create_subscription(
            PoseStamped,
            '/cube_green/pose',
            self.callback_green_cube_pose,
            10)
        self.subscription4

        # CUBE DEST SUB
        self.subscription4 = self.create_subscription(
            PoseStamped,
            '/dest_cube_blu/pose',
            self.callback_blu_cube_pose_dest,
            10)
        self.subscription4

        self.subscription4 = self.create_subscription(
            PoseStamped,
            '/dest_cube_red/pose',
            self.callback_red_cube_pose_dest,
            10)
        self.subscription4

        self.subscription4 = self.create_subscription(
            PoseStamped,
            '/dest_cube_green/pose',
            self.callback_green_cube_pose_dest,
            10)
        self.subscription4

        self.subscription4 = self.create_subscription(
            PoseStamped,
            '/dest_cube_yellow/pose',
            self.callback_yellow_cube_pose_dest,
            10)
        self.subscription4

    def timer_callback(self):
        # 0: move manipulator and record cubes start and dest position
        if (
            self.fsm_state == 0 and
            cube_data["red"]["pose_stamped_start"] == init_pose or cube_data["red"]["pose_stamped_dest"] == init_pose or
            cube_data["green"]["pose_stamped_start"] == init_pose or cube_data["green"]["pose_stamped_dest"] == init_pose or
            cube_data["blu"]["pose_stamped_start"] == init_pose or cube_data["blu"]["pose_stamped_dest"] == init_pose or
            cube_data["yellow"]["pose_stamped_start"] == init_pose or cube_data["yellow"]["pose_stamped_dest"] == init_pose
        ):
            print("### step %d: index %d / %d\n" %
                  (self.fsm_state, self.i, len(self.pose_list)))

            curr_pose = self.pose_list[self.i]
            offset = 3

            pose = PoseStamped()
            pose.pose.position.x = float(curr_pose[0 + offset])
            pose.pose.position.y = float(curr_pose[1 + offset])
            pose.pose.position.z = float(curr_pose[2 + offset])
            pose.pose.orientation.x = float(curr_pose[3 + offset])
            pose.pose.orientation.y = float(curr_pose[4 + offset])
            pose.pose.orientation.z = float(curr_pose[5 + offset])
            pose.pose.orientation.w = float(curr_pose[6 + offset])

            self.publisher_.publish(pose)
            self.prev_pose_target = pose

            self.i += 1
            if self.i >= len(self.pose_list) - 1:
                self.i = 0
        elif self.fsm_state == 0:
            print("### step %d done" % self.fsm_state)
            self.fsm_state += 1

        # 1: open gripper
        if self.fsm_state == 1:
            print("### step %d" % self.fsm_state)
            self.open_gripper()
            self.start_timer()
            print("### step %d done" % self.fsm_state)
            self.fsm_state += 1

        # 2/3/5/6: go to cube (start or destination position)
        if self.fsm_state == 2 or self.fsm_state == 3 or self.fsm_state == 5 or self.fsm_state == 6:
            print("### step %d" % self.fsm_state)

            if self.timer_status() == True and not self.timer_finished(3000):
                return

            self.disable_timer()

            if not self.check_arrived_dest(self.prev_pose_target, self.position_epsilon):
                print("[timer_callback] not reached prev target")
                self.publisher_.publish(self.prev_pose_target)
                return  # still doing last command

            # go to target
            target_pose = PoseStamped()
            curr_color = self.cube_order[self.cube_idx]
            # todo: check if copy is necessary
            test = cube_data[curr_color].copy()
            if self.fsm_state == 2:
                target_pose = self.copy_pose_stamped(
                    test["pose_stamped_start"])
                target_pose.pose.position.z += 0.25  # alza il manipolatore
            elif self.fsm_state == 3:
                target_pose = self.copy_pose_stamped(
                    test["pose_stamped_start"])
            elif self.fsm_state == 5:
                target_pose = self.copy_pose_stamped(
                    test["pose_stamped_dest"])
                target_pose.pose.position.z += 0.25  # alza il manipolatore
            elif self.fsm_state == 6:
                target_pose = self.copy_pose_stamped(
                    test["pose_stamped_dest"])

            # plan linear trajectory (just once)
            if self.curr_traj["path"] == []:
                print("[timer_callback] plan trajectory for %s" % curr_color)
                self.compute_trajectory(target_pose)

            # execute trajectory
            curr_pose = PoseStamped()
            time = self.curr_traj["time_idx"]
            curr_pose.pose.position.x = float(self.curr_traj["path"][0][time])
            curr_pose.pose.position.y = float(self.curr_traj["path"][1][time])
            curr_pose.pose.position.z = float(self.curr_traj["path"][2][time])
            curr_pose.pose.orientation = target_pose.pose.orientation

            self.publisher_.publish(curr_pose)

            # print("[timer_callback] current pose: %.3f %.3f %.3f\npublishing %.3f %.3f %.3f" %
            #       (self.ee_actual_pose.pose.position.x, self.ee_actual_pose.pose.position.y, self.ee_actual_pose.pose.position.z,
            #        curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z))

            if self.curr_traj["time_idx"] == self.curr_traj["time"].size - 1:
                print("[timer_callback] reached end of trajectory")

                if self.check_arrived_dest(target_pose, self.position_epsilon):
                    print("### step %d done" % self.fsm_state)
                    self.curr_traj["path"] = []  # reset trajectory
                    self.curr_traj["time"] = []  # reset trajectory
                    self.curr_traj["time_idx"] = 0  # reset trajectory

                    self.fsm_state += 1
            else:
                self.curr_traj["time_idx"] += 1

            self.prev_pose_target = curr_pose

        if self.fsm_state == 4:
            print("### step %d" % self.fsm_state)
            self.close_gripper()
            self.start_timer()
            print("### step %d done" % self.fsm_state)
            self.fsm_state += 1

        if self.fsm_state == 7:
            print("### step %d" % self.fsm_state)
            self.open_gripper()
            self.start_timer()
            print("### step %d done" % self.fsm_state)

            self.cube_idx += 1
            if self.cube_idx == len(self.cube_order):
                print("END OF THE GAME")
                self.cube_idx = 0  # altrimenti devo fare controllo sopra
                self.fsm_state = -1  # fine (-1 non esiste)
            else:
                self.fsm_state = 1  # fai prossimo cubo

    def callback_ee_actual_pose(self, msg):
        # ur5_frame = msg.header.frame_id
        # ur5_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # ur5_orient = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.ee_actual_pose = msg

    def callback_blu_cube_pose(self, msg):
        cube_data["blu"]["pose_stamped_start"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y,
                     msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rots["blu"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["blu"]["pose_stamped_start"].pose.orientation.x = q_new[0]
        cube_data["blu"]["pose_stamped_start"].pose.orientation.y = q_new[1]
        cube_data["blu"]["pose_stamped_start"].pose.orientation.z = q_new[2]
        cube_data["blu"]["pose_stamped_start"].pose.orientation.w = q_new[3]

    def callback_red_cube_pose(self, msg):
        cube_data["red"]["pose_stamped_start"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y,
                     msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rots["red"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["red"]["pose_stamped_start"].pose.orientation.x = q_new[0]
        cube_data["red"]["pose_stamped_start"].pose.orientation.y = q_new[1]
        cube_data["red"]["pose_stamped_start"].pose.orientation.z = q_new[2]
        cube_data["red"]["pose_stamped_start"].pose.orientation.w = q_new[3]

    def callback_green_cube_pose(self, msg):
        cube_data["green"]["pose_stamped_start"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y,
                     msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rots["green"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["green"]["pose_stamped_start"].pose.orientation.x = q_new[0]
        cube_data["green"]["pose_stamped_start"].pose.orientation.y = q_new[1]
        cube_data["green"]["pose_stamped_start"].pose.orientation.z = q_new[2]
        cube_data["green"]["pose_stamped_start"].pose.orientation.w = q_new[3]

    def callback_yellow_cube_pose(self, msg):
        cube_data["yellow"]["pose_stamped_start"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y,
                     msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rots["yellow"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["yellow"]["pose_stamped_start"].pose.orientation.x = q_new[0]
        cube_data["yellow"]["pose_stamped_start"].pose.orientation.y = q_new[1]
        cube_data["yellow"]["pose_stamped_start"].pose.orientation.z = q_new[2]
        cube_data["yellow"]["pose_stamped_start"].pose.orientation.w = q_new[3]

    def callback_yellow_cube_pose_dest(self, msg):
        cube_data["yellow"]["pose_stamped_dest"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y,
                     msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rots["yellow"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["yellow"]["pose_stamped_dest"].pose.orientation.x = q_new[0]
        cube_data["yellow"]["pose_stamped_dest"].pose.orientation.y = q_new[1]
        cube_data["yellow"]["pose_stamped_dest"].pose.orientation.z = q_new[2]
        cube_data["yellow"]["pose_stamped_dest"].pose.orientation.w = q_new[3]

    def callback_blu_cube_pose_dest(self, msg):
        cube_data["blu"]["pose_stamped_dest"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y,
                     msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rots["blu"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["blu"]["pose_stamped_dest"].pose.orientation.x = q_new[0]
        cube_data["blu"]["pose_stamped_dest"].pose.orientation.y = q_new[1]
        cube_data["blu"]["pose_stamped_dest"].pose.orientation.z = q_new[2]
        cube_data["blu"]["pose_stamped_dest"].pose.orientation.w = q_new[3]

    def callback_green_cube_pose_dest(self, msg):
        cube_data["green"]["pose_stamped_dest"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y,
                     msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rots["green"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["green"]["pose_stamped_dest"].pose.orientation.x = q_new[0]
        cube_data["green"]["pose_stamped_dest"].pose.orientation.y = q_new[1]
        cube_data["green"]["pose_stamped_dest"].pose.orientation.z = q_new[2]
        cube_data["green"]["pose_stamped_dest"].pose.orientation.w = q_new[3]

    def callback_red_cube_pose_dest(self, msg):
        cube_data["red"]["pose_stamped_dest"] = msg
        quat_orig = [msg.pose.orientation.x, msg.pose.orientation.y,
                     msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot = self.ee_rots["red"]
        q_new = quaternion_multiply(q_rot, quat_orig)
        cube_data["red"]["pose_stamped_dest"].pose.orientation.x = q_new[0]
        cube_data["red"]["pose_stamped_dest"].pose.orientation.y = q_new[1]
        cube_data["red"]["pose_stamped_dest"].pose.orientation.z = q_new[2]
        cube_data["red"]["pose_stamped_dest"].pose.orientation.w = q_new[3]

    def check_arrived_dest(self, target_pose, thr):
        x_diff = abs(self.ee_actual_pose.pose.position.x -
                     target_pose.pose.position.x)
        y_diff = abs(self.ee_actual_pose.pose.position.y -
                     target_pose.pose.position.y)
        z_diff = abs(self.ee_actual_pose.pose.position.z -
                     target_pose.pose.position.z)
        distance_error = x_diff + y_diff + z_diff
        # print("[check_arrived_dest] ### step %d error: %.3f" %
        #       (self.fsm_state, distance_error))
        # print("[check_arrived_dest] current pose: %.3f %.3f %.3f\ntarget_pose %.3f %.3f %.3f" %
        #       (self.ee_actual_pose.pose.position.x, self.ee_actual_pose.pose.position.y, self.ee_actual_pose.pose.position.z,
        #        target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z))
        return distance_error < thr

    def copy_pose_stamped(self, posestamped):
        new_pose = PoseStamped()
        new_pose.pose.position.x = posestamped.pose.position.x
        new_pose.pose.position.y = posestamped.pose.position.y
        new_pose.pose.position.z = posestamped.pose.position.z
        new_pose.pose.orientation.x = posestamped.pose.orientation.x
        new_pose.pose.orientation.y = posestamped.pose.orientation.y
        new_pose.pose.orientation.z = posestamped.pose.orientation.z
        new_pose.pose.orientation.w = posestamped.pose.orientation.w
        return new_pose

    # GRIPPER
    def close_gripper(self):
        if self.gripper_status == self.GRIPPER_CLOSE:
            return

        print("[close_gripper] called")
        gripper_bool = Bool()
        gripper_bool.data = self.GRIPPER_CLOSE  # close gripper
        self.gripper_pub.publish(gripper_bool)
        self.gripper_status = self.GRIPPER_CLOSE

    def open_gripper(self):
        if self.gripper_status == self.GRIPPER_OPEN:
            return

        print("[open_gripper] called")
        gripper_bool = Bool()
        gripper_bool.data = self.GRIPPER_OPEN  # open gripper
        self.gripper_pub.publish(gripper_bool)
        self.gripper_status = self.GRIPPER_OPEN

    # TIMER
    def start_timer(self):
        self.time_tracker["active"] = True
        self.time_tracker["start_tick"] = self.get_clock().now()

    def timer_finished(self, delta_ms_thr):
        time_now = self.get_clock().now()
        delta_nano = time_now.nanoseconds - \
            self.time_tracker["start_tick"].nanoseconds
        delta_ms = delta_nano / 1000000
        print("[timer_finished] %d / %d\n" % (delta_ms, delta_ms_thr))
        return delta_ms > delta_ms_thr

    def disable_timer(self):
        self.time_tracker["active"] = False

    def timer_status(self):
        return self.time_tracker["active"]

    # TRAJECTORY
    def linear_primitive(self, pi, pf, Ts):
        delta_pos = pf - pi
        delta_pos_norm = np.linalg.norm(delta_pos)

        s = np.arange(0, delta_pos_norm, Ts)
        print(np.shape(s))

        p_x = pi[0] + s * delta_pos[0] / delta_pos_norm
        p_y = pi[1] + s * delta_pos[1] / delta_pos_norm
        p_z = pi[2] + s * delta_pos[2] / delta_pos_norm
        path = [p_x, p_y, p_z]

        # print("linear_primitive")
        # print("pi: %s" % pi)
        # print("pf: %s" % pf)
        # print("traj: %s" % path)

        time = s
        return path, time

    def compute_trajectory(self, target_pose):
        pi = np.array([self.ee_actual_pose.pose.position.x,
                       self.ee_actual_pose.pose.position.y, self.ee_actual_pose.pose.position.z])
        pf = np.array([target_pose.pose.position.x,
                       target_pose.pose.position.y, target_pose.pose.position.z])
        p, time = self.linear_primitive(pi, pf, self.traj_ts)

        self.curr_traj["path"] = p  # fill trajectory path
        self.curr_traj["time"] = time  # fill trajectory time law
        self.curr_traj["time_idx"] = 0  # start traj from start

        print("computed traj")
        print(self.curr_traj["path"])


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
