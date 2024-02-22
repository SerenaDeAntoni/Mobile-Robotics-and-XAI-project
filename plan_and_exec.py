import rclpy
from rclpy.node import Node
import rclpy.qos

from geometry_msgs.msg import Twist
import numpy as np

import clingo

import tf_transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'




def paint(string: str, color: str):
    print(f"{color}{string}{bcolors.ENDC}")


class Turtlebot3PE(Node):

    def __init__(self):
        super().__init__('turtlebot3_plan_and_exec_node')


        # INFO GENERAL
        self.step = 0
        self.step_start = True
        self.step_start_pos = 0.
        self.step_start_orientation = 0.

        # ROS TOPICS
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscriber = self.create_subscription(Odometry,'/odom', self.callback_odom,1)
        self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.laser_callback,
                                                           rclpy.qos.qos_profile_sensor_data)

        # ODOMETRY
        self.odom_msg = Odometry()
        self.ranges = []
        self.print_ranges = False
        self.angle_min = None
        self.angle_increment = None
        self.step_angle = 60

        # MAIN LOOP
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop_2)

        # rotation
        # SIMULATION
        # self.yaw_north = 90
        # self.yaw_west = 180
        # self.yaw_south = 270
        # self.yaw_east = 360
        # SIMULATION

        # REAL
        self.yaw_north = 360
        self.yaw_west = 270
        self.yaw_south = 180
        self.yaw_east = 90
        # REAL

        # ROTATION TWIST
        self.twist_rotate_left = Twist()
        self.twist_rotate_left.angular.z = 0.05

        self.twist_rotate_right = Twist()
        self.twist_rotate_right.angular.z = -0.05

        self.twist_current = None

        # SPACE DISCRETE
        # move foreward --> ((step/len_axes)=(100/cell_in_axes)/100)
        #len_axes = 2 # REAL
        len_axes = 10 # SIMULATION
        #cell_in_axes = 5 # REAL
        cell_in_axes = 10 # SIMULATION
        self.step_foreward = (((100/cell_in_axes)/100)*len_axes)
        # self.step_foreward = self.step_foreward * 2 ## TODO ONLY FOR REAL !!!!!!! 0.40 cm

        # FORWARD TWIST
        self.twist_forward = Twist()
        self.twist_forward.linear.x = 0.03

        # STOP TWIST
        self.twist_stop = Twist()
        self.twist_stop.linear.x = 0.
        self.twist_stop.angular.z = 0.

        # INFO ACTION
        self.action = None
        self.target_orientation = None
        self.reset_cinematic()

        # GENERAL INFO
        self.phase_lidar = True
        self.at = [4, 4] # SIMULATION
        #self.at = [2, 2] # REAL
        # KEEP ALWAYS INT OTHERWISE CLINGO WILL BROKE!!!!
        # self.threshold_sample = 1 # REAL
        self.threshold_sample = 4 # SIMULATION
        # KEEP ALWAYS INT OTHERWISE CLINGO WILL BROKE!!!!
        self.threshold_collision = 0.2

        # GENERAL INFO
        self.near_collision = False
        self.action_plan = None
        self.n_steps = None
        self.add_angle_min_to_target_way = True

        self.real = False




    def callback_odom(self, msg):
        # ODOMETER INFO
        self.odom_msg = msg

    def laser_callback(self, msg):
        # SCAN LASER INFO
        self.ranges = msg.ranges
        # print(self.ranges)
        self.ranges = np.array(self.ranges)
        self.ranges[np.where(self.ranges > self.threshold_sample)] = np.inf

        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment



    def get_odom(self):
        # return current odometry info about position and orientation
        rot = self.odom_msg.pose.pose.orientation
        point = self.odom_msg.pose.pose.position

        self.rot_ = tf_transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        rad_to_deg = np.rad2deg(self.rot_[2]) if np.rad2deg(self.rot_[2]) >= 0 else np.rad2deg(self.rot_[2]) + (180*2)

        if self.add_angle_min_to_target_way:
            # only once set the target orientation
            if rad_to_deg > 90:
                complete_turn_angle = 360
            else:
                complete_turn_angle = 0

            self.yaw_east = rad_to_deg + 270 - complete_turn_angle
            self.yaw_north = rad_to_deg - complete_turn_angle
            self.yaw_west = rad_to_deg + 90 - complete_turn_angle
            self.yaw_south = rad_to_deg + 180 - complete_turn_angle
            self.add_angle_min_to_target_way = False


        return point, rad_to_deg

    def reset_cinematic(self):
        # reset subaction of the agent and increment step action_plan
        self.time_rotation = True
        self.time_move = False
        self.step += 1
        self.step_start = True

    def determine_turn_direction(self, current_orientation, target_orientation):

        # depending on current and target orientation define the best twist to load (right/left)

        differece = target_orientation - current_orientation
        if differece < 180:
            differece += 360
        if differece > 180:
            differece -= 360
        if differece > 0:
            # 'turn left'
            self.twist_current = self.twist_rotate_left
        if differece < 0:
            # 'turn right'
            self.twist_current = self.twist_rotate_right

    def act_details_2(self, p1, p2, current_yaw ):
        # execute subaction to perform between (1) rotate and (2) move foreward

        if self.time_rotation:
            paint(f'TIME ROTATE --> CURRENT YAW: {current_yaw}, TARGET YAW: {self.target_orientation}',bcolors.OKBLUE)
        if self.time_move:
            paint(f'TIME MOVE --> NOW: {np.round(p2,5)}, START: {np.round(p1,5)}, STEP_FOR: {np.round(self.step_foreward,5)}',bcolors.OKCYAN)

        if self.time_rotation and np.abs(current_yaw - self.target_orientation) >= 10:
            # if difference between current orientation and target is major than 10 degrees
            #   -> continuing to rotate
            self.publisher_.publish(self.twist_current)
        else:
            # stop rotate
            self.time_rotation = False
            self.time_move = True

        if self.time_move and np.abs(p2 - p1) < self.step_foreward:
            # if difference between start step position is minor than step_foreward
            #   -> continuing to move
            self.publisher_.publish(self.twist_forward)
        else:
            # stop move
            self.time_move = False

    def get_target_orientation(self,):
        # depending on action select the corresponding angle target

        if self.action == 'north':
            target_orientation = self.yaw_north
        elif self.action == 'south':
            target_orientation = self.yaw_south
        elif self.action == 'east':
            target_orientation = self.yaw_east
        elif self.action == 'west':
            target_orientation = self.yaw_west
        return target_orientation



    def find_non_zero_sequences_with_min(self, arr):
        # extract from lidar signal, sequences of min distances and relative angles
        # based on continuous numbers presence in signal
        non_zero_sequences = []
        min_values = []
        min_positions = []
        current_sequence = []
        current_min_value = np.inf
        current_min_position = -1
        for i, num in enumerate(arr):
            if num != np.inf and not np.isnan(num):
                # if there is a number, start or continue the current sequence
                current_sequence.append(num)
                if num < current_min_value:
                    current_min_value = num
                    current_min_position = i
            elif current_sequence:
                # if a np.inf or a np.nan occurs, end subsequence
                non_zero_sequences.append(current_sequence)
                min_values.append(current_min_value)
                min_positions.append(current_min_position)
                current_sequence = []
                current_min_value = np.inf
                current_min_position = -1
        if current_sequence:
            # extract min values (distance) and corresponding position (angle) from each subsequences
            non_zero_sequences.append(current_sequence)
            min_values.append(current_min_value)
            min_positions.append(current_min_position)

        # print(f'MIN_VALUES: {min_values}, MIN_POSITIONS: {min_positions}')

        return min_values, min_positions

    def get_x_y(self, min_val, min_pos):
        # given distance and angle, compute relative x and y distances from agent
        # the axis are defined based on normal graph with agent in 0,0 coordinates
        pair_x_y = []

        min_pos = [(x * np.rad2deg(self.angle_increment)) + np.rad2deg(self.angle_min) for x in  min_pos ]

        for i, pos in enumerate(min_pos):
            angle = pos
            x = 1
            y = 1
            if angle < 90:
                x = -1
            elif angle >= 90 and angle < 180:
                angle = 180 - angle
                x = -1
                y = -1
            elif angle >= 180 and angle < 270:
                angle = angle - 180
                y = -1
            else:
                angle = 360 - angle
            # print(f'ANG: {pos}, N_ANG: {angle}')
            x *= np.sin(np.radians(angle)) * min_val[i]
            y *= np.cos(np.radians(angle)) * min_val[i]

            pair_x_y.append([x, y])
        return pair_x_y

    def positions_diescrete(self, at, step, col_row):
        # return the position discrete of each obstacle in the grid arena
        disc_pos = []
        at_col = at[0]
        at_row = at[1]

        for i, el in enumerate(col_row):
            new_pos = []
            col = el[0]
            row = el[1]
            # print(f'COL: {col}, ROW: {row}')
            # print(f'STEP {step} STEP')

            at_col_position = (at_col * step) + (step/2)

            if col < 0:
                diff = at_col_position - np.abs(col)
            else:
                diff = at_col_position + col

            real_col = int(diff / step)
            new_pos.append(real_col)

            at_row_position = (at_row * step) + (step/2)

            if row < 0:
                diff = at_row_position + np.abs(row)
            else:
                diff = at_row_position - np.abs(row)

            real_row = int(diff / step)
            new_pos.append(real_row)

            disc_pos.append(new_pos)
        return disc_pos


    def get_zero_and_step(self, ranges, angle_increment_deg, angle_min_deg, step_angle):
        # return 2 array where to pick the position of right and left distances in angles

        len_ranges = len(ranges)
        # create degrees matrix
        matrix = np.full((len_ranges, len_ranges), angle_increment_deg)
        triangular_matrix = np.tril(matrix)
        sum_triangular_matrix = np.sum(triangular_matrix, axis=1)
        degrees_matrix = sum_triangular_matrix + angle_min_deg

        pos_positive = np.array(np.where(degrees_matrix < step_angle ))
        pos_negative = np.array(np.where(degrees_matrix > 360 - step_angle ))

        # print(f'POSITIVE: {pos_positive[0]}')
        # print(f'NEGATIVE: {pos_negative[0]}')

        return pos_positive[0], pos_negative[0]

    def send_clingo_initial_condition(self, rocks_positions, rocks_dist, threshold, agent_loc):

        # return the clingo computed plan given positions of obstacles (rocks), rocks distances,
        # threshold in which obstacles have to be picked and starting agent position in grid space

        asp_code_base = f"""
            rock(0..{len(rocks_dist)-1}).
            cell((0..10,0..10)).

            #show east/1.
            #show south/1.
            #show north/1.
            #show west/1.
            #show sample/2.
        """

        asp_code_step = """
            0 { sample(R,t): at((X,Y),t), loc(R, (X,Y));
                east(t) ; 
                west(t) ; 
                north(t);
                south(t)
                } 1.

            at((X+1,Y),t+1) :- east(t), at((X,Y),t).

            at((X-1,Y),t+1) :- west(t), at((X,Y),t).

            at((X,Y-1),t+1) :- north(t), at((X,Y),t).

            at((X,Y+1),t+1) :- south(t),at((X,Y),t).

            at((X,Y),t+1) :- sample(R, t), at((X,Y),t).
            sampled(R, t+1) :- sample(R,t).
            sampled(R, t+1) :- sampled(R, t).
        """

        asp_code_check = """
            #external query(t).
            :- query(t), not sampled(R,t), rock(R), good(R). %all good rocks must be sampled
        """

        global final_solution
        final_solution = {}

        def on_model(model):

            # fill final_solution dictionary with {id=time,value=action}
            for s in model.symbols(shown=True):
                if s.name == 'sample':
                    final_solution[s.arguments[1].number] = s.name
                else:
                    final_solution[s.arguments[0].number] = s.name

        # this is an arbitrary upper limit set to ensure process terminates
        max_step = 30

        control = clingo.Control()
        control.configuration.solve.models = 1  # find one answer only

        # add each #program
        control.add("base", [], asp_code_base)

        # Add additional constraints to the control program
        for idx, constraint in enumerate(rocks_positions):
            print(f"loc({idx}, ({constraint[0]},{constraint[1]})).")
            print(f"dist({idx}, {int(rocks_dist[idx])}).")
            control.add("base", [], f"loc({idx}, ({constraint[0]},{constraint[1]})).")
            # control.add("base", [], f"dist({idx}, {str(rocks_dist[idx]).replace('.',',')}).")
            control.add("base", [], f"dist({idx}, {int(rocks_dist[idx])}).")

        # add agent start position
        control.add("base", [], f'at(({agent_loc[0]},{agent_loc[1]}),1).')
        control.add("base", [], f'good(R) :- dist(R,D), D<{threshold}.')
        print(f'good(R) :- dist(R,D), D<{threshold}.')

        control.add("step", ["t"], asp_code_step)
        control.add("check", ["t"], asp_code_check)

        # for grounding, we make use of a parts array
        parts = []
        parts.append(("base", []))
        control.ground(parts)
        ret, step = None, 1
        # try solving until max number of steps or until solved
        while step <= max_step and (step == 1 or not ret.satisfiable):
            parts = []
            # handle #external call
            control.release_external(clingo.Function("query", [clingo.Number(step - 1)]))
            parts.append(("step", [clingo.Number(step)]))
            parts.append(("check", [clingo.Number(step)]))
            control.cleanup()  # cleanup previous grounding call, so we can ground again
            control.ground(parts)
            # finish handling #external call
            control.assign_external(clingo.Function("query", [clingo.Number(step)]), True)
            # print(f"Solving step: t={step}")
            ret = control.solve(on_model=on_model)
            # print(f"Returned: {ret}")
            step += 1

        return final_solution

    def control_loop_2(self):

        if self.action_plan != None:
            self.phase_lidar = False

            len_action_plan = len(self.action_plan)

            # truncate action plan to last sample
            if self.action_plan[len_action_plan] == 'sample':
                self.n_steps = len_action_plan
            elif self.action_plan[len_action_plan-1] == 'sample':
                self.n_steps = len_action_plan - 1

            paint(self.action_plan, bcolors.WARNING)

        if self.phase_lidar and len(self.ranges) != 0:
            # PHASE LIDAR START
            min_val, min_pos = self.find_non_zero_sequences_with_min(self.ranges) # compute min distances and angles
            col_row = self.get_x_y(min_val, min_pos) # return coordinates x,y for each obstacle
            real_disc = self.positions_diescrete(self.at, self.step_foreward, col_row) # provides discrete position of obstacles

            print(f'REAL_DISC: {real_disc}')
            print(f'MIN_VAL: {min_val}')
            print(f'AT: {self.at}')

            # action plan computed from clingo
            self.action_plan = self.send_clingo_initial_condition(real_disc, min_val,
                                                                  self.threshold_sample, self.at)

        if self.phase_lidar == False:
            # ACTION PLAN EXECUTION PHASE
            print(f'STEP: {self.step}, N_STEPS: {self.n_steps}')
            if self.step > self.n_steps:
                # exit on termination action plan
                self.publisher_.publish(self.twist_stop)
                exit(0)
            if self.step_start:
                # on start of each action, store the position start, angle start and current action performing
                self.step_start_pos, self.step_start_orientation = self.get_odom()
                self.action = self.action_plan[self.step]

                if self.action != 'sample':
                    # determine turn direction based on current orientation and target
                    self.target_orientation = self.get_target_orientation()
                    self.determine_turn_direction(self.step_start_orientation, self.target_orientation)
                self.step_start = False

            paint(f'@@@ LOOP CALL @@@ --> STEP: {self.step}/{self.n_steps}, ACTION: {self.action}',bcolors.UNDERLINE)

            current_pos, current_yaw = self.get_odom() # get current position and orientation

            # compute min distances to check collision
            angle_increment_deg = np.rad2deg(self.angle_increment)
            angle_min_deg = np.rad2deg(self.angle_min)
            pos_positive, pos_negative = self.get_zero_and_step(self.ranges, angle_increment_deg, angle_min_deg,self.step_angle)
            ranges = np.array(self.ranges)

            is_colliding = np.nanmin(np.concatenate([ranges[pos_positive],ranges[pos_negative]])) < self.threshold_collision

            if is_colliding:
                paint(f'### COLLIDING ###', bcolors.FAIL)
                self.near_collision = True
            else:
                paint(f'### IS NOT COLLIDING ###', bcolors.OKGREEN)

            diff_pos = 0

            # compute the difference from start position and current position
            if self.real:
                if self.action == 'east' or self.action == 'west':
                    diff_pos = np.abs(self.step_start_pos.y - current_pos.y)
                elif self.action == 'north' or self.action == 'south':
                    diff_pos = np.abs(self.step_start_pos.x - current_pos.x)
            else:
                if self.action == 'east' or self.action == 'west':
                    diff_pos = np.abs(self.step_start_pos.x - current_pos.x)
                elif self.action == 'north' or self.action == 'south':
                    diff_pos = np.abs(self.step_start_pos.y - current_pos.y)


            paint(f'COLLISION: {self.near_collision}, NEXT_ACT: {self.action_plan[self.step+1]}, DIFF_POS: {diff_pos}',bcolors.BOLD)

            # increment step in case:
            # (1) rotation and move are end
            # (2) current action is "sample"
            # (3) the agent is near collision, next action is "sample" and the robot at least is moved a little step
            #       away from last obstacle
            if ((self.time_rotation == False and self.time_move == False) or self.action == 'sample'
                    or (self.near_collision and self.action_plan[self.step+1] == 'sample' and diff_pos>(self.step_foreward/8))):
                self.reset_cinematic()
                self.near_collision = False
            else:
                # continuing performing current action
                self.near_collision = False

                # call the real action performing on environment by agent
                if self.real:

                    if self.action == 'east' or self.action == 'west':
                        self.act_details_2(self.step_start_pos.y, current_pos.y, current_yaw)
                    if self.action == 'north' or self.action == 'south':
                        self.act_details_2(self.step_start_pos.x, current_pos.x, current_yaw)
                else:
                    if self.action == 'east' or self.action == 'west':
                        self.act_details_2(self.step_start_pos.x, current_pos.x, current_yaw)
                    if self.action == 'north' or self.action == 'south':
                        self.act_details_2(self.step_start_pos.y, current_pos.y, current_yaw)

                if self.real:
                    paint(f'XXX NOW X: {current_pos.y} XXX', bcolors.BOLD)
                    paint(f'YYY NOW Y: {current_pos.x} YYY', bcolors.BOLD)
                else:
                    paint(f'XXX NOW X: {current_pos.x} XXX', bcolors.BOLD)
                    paint(f'YYY NOW Y: {current_pos.y} YYY', bcolors.BOLD)

    def stop_robot(self):
        self.stop = True


def main(args=None):

    rclpy.init(args=args)
    turtlebot3_pe_node = Turtlebot3PE()
    rclpy.spin(turtlebot3_pe_node)
    turtlebot3_pe_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
