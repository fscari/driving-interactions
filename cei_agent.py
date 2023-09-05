import os
import casadi
import glob
import copy
import warnings
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np
from PyQt5 import QtWidgets, uic

from modules.carlainterface.carlainterface_sharedvariables import CarlaInterfaceSharedVariables
from modules.cei.cei_sharedvariables import CEIAgentSharedVariables
from modules.cei.cei_track_respresentation import SymmetricMergingTrack
from modules.carlainterface.carlainterface_sharedvariables import VehicleSharedVariables

from tools.lowpassfilterbiquad import LowPassFilterBiquad


class CEIAgentSettings:
    def __init__(self):
        self.risk_bounds = (0.1, 0.7)
        self.saturation_time = 2.5
        self.time_horizon = 8
        self.belief_frequency = 4
        self.memory_length = 4
        self.theta = 1.
        self.desired_velocity = 50. / 3.6
        self.expected_acceleration = 1.0
        self.look_ahead_distance = 25.  # todo: fix

        # reference trajectory for steering angles
        self.reference_trajectory_name = ''

        # vehicle to control
        self.vehicle_id = ''
        self.opponent_id = ''

    def as_dict(self):
        return_dict = copy.copy(self.__dict__)
        for key, item in self.__dict__.items():
            if isinstance(item, Enum):
                return_dict[key] = item.value
        return return_dict

    def __str__(self):
        return str('CEI-Agent Settings')

    def set_from_loaded_dict(self, loaded_dict):
        for key, value in loaded_dict.items():
            self.__setattr__(key, value)


class CEIAgentProcess:
    """ This is a simple implementation of a pure pursuit vehicle controller. For more information on this type of controller please use Google, it is pretty common.
    This simple implementation does not use the back axle as the origin of the vehicle frame as it should, so it should not be used when high precision is required.
    """

    def __init__(self, settings: CEIAgentSettings, shared_variables: CEIAgentSharedVariables, other_shared_variables: CEIAgentSharedVariables, carla_interface_shared_variables: CarlaInterfaceSharedVariables, dt,
                 track: SymmetricMergingTrack, vehicle_width, vehicle_length):
        self.settings = settings
        self.shared_variables = shared_variables
        self.carla_interface_shared_variables = carla_interface_shared_variables
        self.other_shared_variables = other_shared_variables

        self._trajectory = None
        self.vehicle_travelled_distance = None
        self.vehicle_velocity = None
        self.last_vehicle_velocity = 0.
        self.vehicle_orientation = None
        self.rear_axle_position = None

        self.last_control_time_stamp = 0.

        self.track = track
        self.dt = dt
        self.vehicle_width = vehicle_width
        self.vehicle_length = vehicle_length
        self.time_of_low_risk = np.inf

        self.lower_bounds = []
        self.upper_bounds = []
        self.collision_probability = []
        self.first_time = True
        self.optimization_failed_flag = False

        self.acceleration_filter = LowPassFilterBiquad(fs=1/dt, fc=2)

        # the action plan consists of the action (constant acceleration) to take at the coming time steps. The position plan is the set of positions along the
        # track where the ego vehicle will end up when taking these actions.
        self.action_plan = 0.0
        self.plan_length = int((1 / dt) * settings.time_horizon)
        self.velocity_plan = np.array([0.0] * self.plan_length)
        self.position_plan = np.array([0.0] * self.plan_length)
        self.action_bounds = [-1.5, 1.5]  # To prevent infeasible solutions. Are clipped to -1, 1.

        # the belief consists of sets of a mean and standard deviation for a distribution over positions at every time step.
        self.belief = []
        self.belief_time_stamps = []
        self.belief_point_contributing_to_risk = []
        for belief_index in range(int(settings.belief_frequency * settings.time_horizon)):
            self.belief.append([0., 0.])

        self.recent_acceleration_observations = np.array([0.] * settings.memory_length * int(1 / self.dt))
        self.other_position = 0.0
        self.other_velocity = 0.0

        self.last_replan_reason = 0
        self.last_replan_time = 0
        self.perceived_risk = 0.

        self._max_steering_angle = self.carla_interface_shared_variables.agents[self.settings.vehicle_id].max_steering_angle

        self._parameters_resistance = np.array([0.000203, 0.04799443, 0.00035266])
        self._max_acceleration = 4.460
        self._min_acceleration = 4.260

        self._is_initialized = False
        self._initialize_optimization()

    def get_ready(self):
        self.load_trajectory()

    def do(self):
        """
        Calculates the vehicle steering angle, throttle and brake based on the current state and desired trajectory.
        At the end of this do function, the calculated values need to be written to shared variables. From there they are automatically used by the connected vehicle.
        Variable to write:
            self.shared_variables.brake = calculated brake between (0.0 , 1.0)
            self.shared_variables.throttle = calculated throttle between (0.0 , 1.0)
            self.shared_variables.steering_angle = calculated steering between (-1.0 , 1.0)
            self.shared_variables.handbrake = False
            self.shared_variables.reverse = False
        """
        if self.first_time:
            print("settings for this vehicle are", )
            print(self.settings)
            self.first_time = False

        self.last_vehicle_velocity = self.vehicle_velocity
        vehicle_transform, self.vehicle_velocity, self.rear_axle_position, time_stamp = self._get_current_state()

        self.vehicle_orientation = vehicle_transform[3:]
        self.vehicle_travelled_distance = self.track.carla_coordinates_to_travelled_distance(vehicle_transform[0:2])

        dt = (time_stamp - self.last_control_time_stamp) * 1e-9

        if dt:
            self._observe_communication()
            if not self.carla_interface_shared_variables.agents[self.settings.vehicle_id].cruise_control:
                if not self._is_initialized:
                    self._initialize_belief()
                    self._update_position_plan()
                    self._update_plan(constraint_risk_bound=0.8 * self.settings.risk_bounds[1])
                    self.perceived_risk = self._evaluate_risk()
                    self._is_initialized = True
                else:
                    self._update_belief()
                    self._update_position_plan()
                    self.perceived_risk = self._evaluate_risk()

                    if self.perceived_risk < self.settings.risk_bounds[0] and not self.optimization_failed_flag:
                        if not np.isfinite(self.time_of_low_risk):
                            self.time_of_low_risk = copy.copy(self.carla_interface_shared_variables.time)

                        if (self.carla_interface_shared_variables.time - self.time_of_low_risk) > (self.settings.saturation_time * 1e9):
                            self.action_plan = 0.5
                            self._update_position_plan()

                            self.last_replan_reason = -1
                            self.last_replan_time = copy.copy(self.carla_interface_shared_variables.time)
                            self.time_of_low_risk = copy.copy(self.carla_interface_shared_variables.time)
                            self._update_plan(constraint_risk_bound=0.8 * self.settings.risk_bounds[1])
                            self.perceived_risk = self._evaluate_risk()
                    elif self.perceived_risk > self.settings.risk_bounds[1] or self.optimization_failed_flag:
                        self.time_of_low_risk = np.inf
                        self.last_replan_reason = 1 if not self.optimization_failed_flag else 3
                        self.optimization_failed_flag = False
                        self.last_replan_time = copy.copy(self.carla_interface_shared_variables.time)
                        self._update_plan(constraint_risk_bound=0.8 * self.settings.risk_bounds[0])
                        self.perceived_risk = self._evaluate_risk()
                    elif self._check_if_crossing_preferred_velocity():
                        self.time_of_low_risk = np.inf
                        self.last_replan_reason = 2
                        self.last_replan_time = copy.copy(self.carla_interface_shared_variables.time)
                        self._update_plan(constraint_risk_bound=0.8 * self.settings.risk_bounds[1])
                        self.perceived_risk = self._evaluate_risk()
                    else:
                        self.time_of_low_risk = np.inf

                if self.action_plan >= 0.:
                    self.shared_variables.throttle = self.action_plan
                    self.shared_variables.brake = 0.
                else:
                    self.shared_variables.throttle = 0.
                    self.shared_variables.brake = - self.action_plan

            # set steering angle
            self.last_control_time_stamp = time_stamp
            closest_way_point_index = self._find_closest_way_point_in_trajectory(vehicle_transform)

            if np.linalg.norm(self._trajectory[closest_way_point_index, 1:3] - self.rear_axle_position[0:2]) > self.settings.look_ahead_distance:
                raise RuntimeError('Pure Pursuit controller to far from path, distance = ' + str(
                    np.linalg.norm(self._trajectory[closest_way_point_index, 1:3] - self.rear_axle_position[0:2])), self._trajectory[closest_way_point_index, 1:3],
                                   self.rear_axle_position[0:2])

            # calculate steering angle using the look ahead distance and trajectory. Steer point is defined as the point where the two intersect.
            first_way_point_outside_look_ahead_circle = self._find_first_way_point_outside_look_ahead_circle(
                self.rear_axle_position, closest_way_point_index)
            steer_point_in_vehicle_frame = self._calculate_steer_point(self.rear_axle_position,
                                                                       self.vehicle_orientation,
                                                                       first_way_point_outside_look_ahead_circle)

            steering_angle = 2 * np.arctan2(steer_point_in_vehicle_frame[1], steer_point_in_vehicle_frame[0])

            # steering_angle in shared velocities is normalized with respect to the maximum steering angle
            self.shared_variables.steering_angle = steering_angle / self._max_steering_angle  # TODO: fix steering gain

            # write other shared variables
            belief = copy.copy(np.array(self.belief))
            self.shared_variables.belief_mu = belief[:, 0]
            self.shared_variables.belief_sd = belief[:, 1]
            self.shared_variables.belief_sigma = belief[:, 0]

            self.shared_variables.desired_velocity = self.settings.desired_velocity

            if self._is_initialized:
                self.shared_variables.upper_bound = self.upper_bounds
                self.shared_variables.lower_bound = self.lower_bounds
                self.shared_variables.risk = self.collision_probability

                self.shared_variables.action_plan = self.action_plan
                self.shared_variables.position_plan = self.position_plan
                self.shared_variables.velocity_plan = self.velocity_plan
                self.shared_variables.perceived_risk = self.perceived_risk
            self.shared_variables.travelled_distance = self.vehicle_travelled_distance
            self.shared_variables.other_travelled_distance = self.other_position
            self.shared_variables.velocity = self.vehicle_velocity
            self.shared_variables.filtered_observed_acceleration = self.recent_acceleration_observations[0]

            self.shared_variables.last_replan_time = self.last_replan_time
            self.shared_variables.last_replan_reason = self.last_replan_reason

    def _find_first_way_point_outside_look_ahead_circle(self, vehicle_transform, closest_way_point_index):
        current_way_point_index = closest_way_point_index
        current_way_point = self._trajectory[current_way_point_index, 1:3]
        while np.linalg.norm(current_way_point - vehicle_transform[0:2]) < self.settings.look_ahead_distance:
            current_way_point_index += 1
            if current_way_point_index < len(self._trajectory[:, 1]):
                current_way_point = self._trajectory[current_way_point_index, 1:3]

        return current_way_point_index

    def _calculate_steer_point(self, rear_axle_position, vehicle_orientation,
                               first_way_point_outside_look_ahead_circle):
        # select the two waypoint that span a line that intersects with the look ahead circle and convert them to vehicle frame
        yaw = np.radians(vehicle_orientation[0])
        rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)],
                                    [np.sin(yaw), np.cos(yaw)]])

        way_point_inside = np.dot(np.linalg.inv(rotation_matrix),
                                  self._trajectory[first_way_point_outside_look_ahead_circle - 1,
                                  1:3] - rear_axle_position[0:2])
        way_point_outside = np.dot(np.linalg.inv(rotation_matrix),
                                   self._trajectory[first_way_point_outside_look_ahead_circle,
                                   1:3] - rear_axle_position[0:2])

        # determine the line y=c1 * x + c2 spanned by the two points

        dx = way_point_outside[0] - way_point_inside[0]
        dy = way_point_outside[1] - way_point_inside[1]

        if abs(dx) <= 1e-2:
            print(
                'WARNING: steering point calculation in pure pursuit controller reached singular point, approximation is used instead')
            return way_point_inside + (way_point_outside - way_point_inside) / 2.

        c1 = dy / dx
        c2 = way_point_outside[1] - c1 * way_point_outside[0]

        # solve 'y=c1 * x + c2' and 'lookahead distance^2 = x^2 + y^2' to find steer point. use abc formula to find solutions for x
        a = 1 + c1 ** 2
        b = 2 * c2 * c1
        c = c2 ** 2 - self.settings.look_ahead_distance ** 2

        d = (b ** 2) - (4 * a * c)
        x1 = (-b - np.sqrt(d)) / (2 * a)
        x2 = (-b + np.sqrt(d)) / (2 * a)

        if x1 > 0.:
            x = x1
            y = c1 * x1 + c2
        elif x2 > 0.:
            x = x2
            y = c1 * x2 + c2
        else:
            raise RuntimeError('NPC controller failed to calculate steer point')

        return np.array([x, y])

    def _observe_communication(self):
        other_shared_variables = self.carla_interface_shared_variables.agents[self.settings.opponent_id]
        self.other_velocity = other_shared_variables.velocities_in_vehicle_frame[0]
        other_position_coordinates = other_shared_variables.transform[0:2]
        self.other_position = self.track.carla_coordinates_to_travelled_distance(other_position_coordinates)

        #TODO: line below is unrealible, therefore used a workaround
        # other_acceleration = other_shared_variables.accelerations_in_vehicle_frame[0]
        other_u = max(min(self.other_shared_variables.throttle - self.other_shared_variables.brake, 1), -1)
        other_acceleration = self._u_to_net_acceleration(other_u, self.other_velocity)

        self.recent_acceleration_observations = np.roll(self.recent_acceleration_observations, 1)
        self.recent_acceleration_observations[0] = other_acceleration

    def _initialize_optimization(self):
        # Optimization based on Casadi (Andersson2018), documentation: https://web.casadi.org/docs/, used the opti() stack
        # Optimization steps based on time horizon of belief points
        N = int(self.settings.time_horizon * self.settings.belief_frequency)

        # Initialize decision variables and parameters
        self.optimizer = casadi.Opti()
        self.x = self.optimizer.variable(2, N + 1)  # The state
        self.u = self.optimizer.variable()  # The control input
        total_cost = 0
        self.risk_bound_agent = self.optimizer.parameter()

        # Save to evaluate later on
        self.mu = []
        self.sd = []
        self.inequality = []

        for i in range(N + 1):
            # Dynamics, use as equality constraint
            x_now = self.x[:, i]
            if i < N:
                x_new = self._dynamics_casadi(x_now, self.u, dt=1 / self.settings.belief_frequency)
                self.optimizer.subject_to(self.x[:, i + 1] == x_new)

            # Cost evaluation
            total_cost += self._cost_function_casadi(x_now, self.u)

            # Inequality constraints
            if i > 0:
                mu_now = self.optimizer.parameter()
                sd_now = self.optimizer.parameter()
                lb, ub = self.track.get_collision_bounds_approximation_casadi(x_now[1])

                inequality_constraint = self._get_normal_probability(mu_now, sd_now, lb, ub) - self.risk_bound_agent
                self.optimizer.subject_to(inequality_constraint <= 0)

                # Save for later evaluation
                self.mu.append(mu_now)
                self.sd.append(sd_now)
                self.inequality.append(inequality_constraint)

        # Set bounds
        self.optimizer.subject_to(self.u >= self.action_bounds[0])
        self.optimizer.subject_to(self.u <= self.action_bounds[1])

        # Set initial x
        self.x_initial = casadi.vertcat(self.optimizer.parameter(), self.optimizer.parameter())
        self.optimizer.subject_to(self.x[:, 0] == self.x_initial)

        # Set cost function
        self.optimizer.minimize(total_cost)

    @staticmethod
    def _get_normal_probability(mu, sigma, lower_bound, upper_bound):
        # Compute the probability of collision
        p = (1 / 2) * (casadi.erf((upper_bound - mu) / (sigma * casadi.sqrt(2))) - casadi.erf((lower_bound - mu) / (sigma * casadi.sqrt(2))))
        return p

    def _u_to_net_acceleration(self, u, v):
        sigmoid_throttle = (casadi.tanh(30 * u)) / 2 + 1 / 2
        sigmoid_no_throttle = (casadi.tanh(30 * -u)) / 2 + 1 / 2

        sigmoid_positive_velocity = (casadi.tanh(500 * (v - 0.01))) / 2 + 1 / 2

        resistance = self._parameters_resistance[0] * v ** 2 + self._parameters_resistance[1] * v + self._parameters_resistance[2]

        input_acceleration = sigmoid_throttle * u * self._max_acceleration + sigmoid_positive_velocity * sigmoid_no_throttle * u * self._min_acceleration

        net_acceleration = input_acceleration - resistance

        return net_acceleration

    def _dynamics_casadi(self, x, u, dt):
        # Equality constraints for the optimization
        x1 = x[0]
        x2 = x[1]
        x1_dot = self._u_to_net_acceleration(u, x1)
        x2_dot = x1
        x1_new = x1 + x1_dot * dt
        x2_new = x2 + x2_dot * dt + 1 / 2 * x1_dot * dt ** 2
        return casadi.vertcat(x1_new, x2_new)

    def _cost_function_casadi(self, x, u):
        # Cost function for the optimization
        desired_velocity = self.settings.desired_velocity
        current_velocity = x[0]
        net_acceleration = self._u_to_net_acceleration(u, current_velocity)
        return (current_velocity - desired_velocity) ** 2 + net_acceleration ** 2

    def _initialize_belief(self):

        upper_velocity_bound = lower_velocity_bound = self.other_velocity
        upper_position_bound = lower_position_bound = self.other_position

        for belief_index in range(len(self.belief)):
            upper_position_bound += upper_velocity_bound * (1 / self.settings.belief_frequency) + (self._max_acceleration / 2.) * (
                    1 / self.settings.belief_frequency) ** 2
            upper_velocity_bound += self._max_acceleration * (1 / self.settings.belief_frequency)

            new_lower_position_bound = lower_position_bound + lower_velocity_bound * (1 / self.settings.belief_frequency) + (
                    -self._max_acceleration / 2.) * (1 / self.settings.belief_frequency) ** 2
            if new_lower_position_bound >= lower_position_bound:
                lower_position_bound = new_lower_position_bound

            lower_velocity_bound -= self._max_acceleration * (1 / self.settings.belief_frequency)

            if lower_velocity_bound < 0.:
                lower_velocity_bound = 0.

            mean = ((upper_position_bound - lower_position_bound) / 2.) + lower_position_bound
            sd = (upper_position_bound - mean) / 3

            self.belief[belief_index][0] = mean
            self.belief[belief_index][1] = sd
            self.belief_time_stamps.append((1 / self.settings.belief_frequency) * (belief_index + 1))

    def _update_belief(self):
        new_belief = []

        data_points = np.count_nonzero(self.recent_acceleration_observations)
        acceleration_mu = sum(self.recent_acceleration_observations) / data_points
        acceleration_sigma = np.sqrt(
            sum((self.recent_acceleration_observations[0:data_points] - acceleration_mu) ** 2) / data_points) + (self.settings.expected_acceleration / 3.)


        for belief_point_index in range(len(self.belief)):
            # update observations
            time_to_point = self.belief_time_stamps[belief_point_index]
            belief_point_mu = ((time_to_point ** 2) / 2) * acceleration_mu + self.other_position + self.other_velocity * time_to_point
            belief_point_sigma = acceleration_sigma * ((time_to_point ** 2) / 2)
            new_belief.append([belief_point_mu, belief_point_sigma])

        self.belief = new_belief

    def _evaluate_risk(self):
        max_risk, risk_per_point = self._get_collision_probability(self.belief, self.position_plan)
        self.belief_point_contributing_to_risk = [bool(p) for p in risk_per_point]
        return max_risk

    def _get_collision_probability(self, belief, position_plan):
        probabilities_over_plan = []
        self.lower_bounds = []
        self.upper_bounds = []

        for belief_index, belief_point in enumerate(belief):
            time_from_now = self.belief_time_stamps[belief_index]
            plan_index = int(time_from_now / self.dt) - 1

            position_plan_point = position_plan[plan_index]
            lower_bound, upper_bound = self.track.get_collision_bounds_approximation(position_plan_point)
            self.lower_bounds.append(lower_bound)
            self.upper_bounds.append(upper_bound)

            if lower_bound and upper_bound:
                collision_probability = self._get_normal_probability(belief_point[0], belief_point[1], lower_bound, upper_bound)
                probabilities_over_plan += [collision_probability]
            else:
                probabilities_over_plan += [0.]

        self.collision_probability = probabilities_over_plan

        return np.amax(probabilities_over_plan), probabilities_over_plan

    def _update_plan(self, constraint_risk_bound):
        # Set initial values
        initial_state = casadi.vertcat(self.vehicle_velocity, self.vehicle_travelled_distance)
        self.optimizer.set_value(self.x_initial, initial_state)

        # Set initial guess
        self.optimizer.set_initial(self.x[:, 0], initial_state)
        plan_indices_for_belief = (np.array(self.belief_time_stamps) / self.dt).astype(int) - 1
        self.optimizer.set_initial(self.x[0, 1:], self.velocity_plan[plan_indices_for_belief])
        self.optimizer.set_initial(self.x[1, 1:], self.position_plan[plan_indices_for_belief])
        self.optimizer.set_initial(self.u, self.action_plan)

        # Give the belief parameters to the solver as parameters
        for belief_index, belief_point in enumerate(self.belief):
            mu, sigma = belief_point
            self.optimizer.set_value(self.mu[belief_index], mu)
            self.optimizer.set_value(self.sd[belief_index], sigma)

        # Set the risk bounds of the agent as a parameter
        self.optimizer.set_value(self.risk_bound_agent, constraint_risk_bound)

        # Solver options
        p_opts = {"expand": True, 'ipopt.print_level': 0, 'print_time': 0}
        s_opts = {"max_iter": 1000, "gamma_theta": 0.1, "constr_viol_tol": 1e-8}
        self.optimizer.solver("ipopt", p_opts, s_opts)

        # Solve, if not feasible, still take solution
        try:
            solution = self.optimizer.solve()
            self.action_plan = solution.value(self.u)

        except RuntimeError as e:
            warnings.warn("Optimization failed produced RuntimeError: ")
            print(e)
            self.optimization_failed_flag = True

            # Make heuristic decision to either fully brake or do full throttle
            if self.other_position > self.vehicle_travelled_distance:
                self.action_plan = -0.8
            else:
                self.action_plan = 0.8

        # Clip the action plan
        self.action_plan = max(min(1, self.action_plan), -1)

        self._update_position_plan()

    def _check_if_crossing_preferred_velocity(self):
        return (self.vehicle_velocity < self.settings.desired_velocity <= self.last_vehicle_velocity) or \
               (self.last_vehicle_velocity <= self.settings.desired_velocity < self.vehicle_velocity)

    def _update_position_plan(self):
        """
        This function updates the current positions and velocity plans. The plan is made in terms of acceleration, but also stored in future positions and
        velocities for convenience. This function uses the current position and velocity and acceleration plan to update these predictions.
        :return:
        """
        previous_position = self.vehicle_travelled_distance
        previous_velocity = self.vehicle_velocity

        for index in range(self.plan_length):
            (previous_velocity, previous_position) = self._dynamics_casadi([previous_velocity, previous_position], self.action_plan, dt=self.dt).elements()

            self.velocity_plan[index] = previous_velocity
            self.position_plan[index] = previous_position

    def _find_closest_way_point_in_trajectory(self, vehicle_transform):
        closest_way_point_index = np.argmin(np.linalg.norm(self._trajectory[:, 1:3] - vehicle_transform[0:2], axis=1))
        return closest_way_point_index

    def load_trajectory(self):
        """
        Load trajectory from csv file
        format = [index, x, y, steering_wheel_angle, throttle, brake, heading, velocity]
        """

        path_trajectory_directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'trajectories')
        self._trajectory = np.loadtxt(os.path.join(path_trajectory_directory, self.settings.reference_trajectory_name),
                                      delimiter=',')
        print('Loaded trajectory = ', self.settings.reference_trajectory_name)

    def _get_current_state(self):
        """
        Definition of transform: [x, y, z, yaw, pitch, roll]
        """

        rear_axle_position = self.carla_interface_shared_variables.agents[self.settings.vehicle_id].rear_axle_position
        vehicle_velocity = self.carla_interface_shared_variables.agents[
            self.settings.vehicle_id].velocities_in_vehicle_frame[0]
        vehicle_transform = self.carla_interface_shared_variables.agents[self.settings.vehicle_id].transform
        time_stamp = self.carla_interface_shared_variables.time

        return vehicle_transform, vehicle_velocity, np.array(rear_axle_position), time_stamp


class CEIAgentSettingsDialog(QtWidgets.QDialog):
    def __init__(self, module_manager, settings: CEIAgentSettings, parent=None):
        super().__init__(parent=parent)
        self.module_manager = module_manager
        self.cei_settings = settings
        uic.loadUi(os.path.join(os.path.dirname(os.path.realpath(__file__)), "cei_agent_settings.ui"), self)

        self.button_box_settings.button(self.button_box_settings.RestoreDefaults).clicked.connect(
            self._set_default_values)
        self._fill_trajectory_combobox()
        self.display_values()

        self.show()

    def accept(self):
        self.cei_settings.risk_bounds = (self.lowerRiskThresholdSpinBox.value(), self.upperRiskThresholdSpinBox.value())
        self.cei_settings.saturation_time = self.saturationTimeSpinBox.value()
        self.cei_settings.time_horizon = self.timeHorizonSpinBox.value()
        self.cei_settings.belief_frequency = self.beliefFrequencySpinBox.value()
        self.cei_settings.memory_length = self.memoryLengthSpinBox.value()
        self.cei_settings.theta = self.thetaSpinBox.value()
        self.cei_settings.desired_velocity = self.desiredVelocitySpinBox.value() / 3.6  # Convert to m/s immediately
        self.cei_settings.expected_acceleration = self.expectedAccelerationDoubleSpinBox.value()

        # reference trajectory for steering angles
        self.cei_settings.reference_trajectory_name = self.trajectoryComboBox.currentText()

        super().accept()

    def show(self):
        self.display_values()
        super().show()

    def display_values(self, settings=None):
        if not settings:
            settings = self.cei_settings

        self.trajectoryComboBox.setCurrentIndex(self.trajectoryComboBox.findText(settings.reference_trajectory_name))

        self.lowerRiskThresholdSpinBox.setValue(settings.risk_bounds[0])
        self.upperRiskThresholdSpinBox.setValue(settings.risk_bounds[1])
        self.saturationTimeSpinBox.setValue(settings.saturation_time)
        self.timeHorizonSpinBox.setValue(settings.time_horizon)
        self.beliefFrequencySpinBox.setValue(settings.belief_frequency)
        self.memoryLengthSpinBox.setValue(settings.memory_length)
        self.thetaSpinBox.setValue(settings.theta)
        self.desiredVelocitySpinBox.setValue(settings.desired_velocity * 3.6)
        self.expectedAccelerationDoubleSpinBox.setValue(self.cei_settings.expected_acceleration)

    def _set_default_values(self):
        self.display_values(CEIAgentSettings())

    def _fill_trajectory_combobox(self):
        self.trajectoryComboBox.addItem(' ')

        path_trajectory_directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'trajectories')
        file_names = glob.glob(os.path.join(path_trajectory_directory, '*.csv'))
        for file in file_names:
            self.trajectoryComboBox.addItem(os.path.basename(file))


if __name__ == '__main__':

    settings = CEIAgentSettings()
    settings.vehicle_id = '1'
    shared = CarlaInterfaceSharedVariables()
    shared.agents[settings.vehicle_id] = VehicleSharedVariables()
    shared.agents[settings.vehicle_id].max_steering_angle = 3.1415

    section_length = 50.
    merge_point = [0.0, np.sqrt((2 * section_length) ** 2 - 12.5 ** 2)]
    end_point = [0.0, merge_point[1] + section_length]

    track = SymmetricMergingTrack(merge_point, end_point, [-12.5, 0.], [12.5, 0.], vehicle_width=1.8, vehicle_length=4.5)
    agent = CEIAgentProcess(settings,
                            shared_variables=CEIAgentSharedVariables(belief_size=int(settings.belief_frequency * settings.time_horizon),
                                                              plan_size=int((1000 / 10) * settings.time_horizon)),
                            other_shared_variables=CEIAgentSharedVariables(belief_size=int(settings.belief_frequency * settings.time_horizon),
                                                                 plan_size=int((1000 / 10) * settings.time_horizon)),
                            carla_interface_shared_variables=shared,
                            dt=0.01, track=track, vehicle_width=2., vehicle_length=4.1)

    net_u = []
    accelerations = [.01 * i - 1 for i in range(201)]
    for u in accelerations:
        net_u.append(agent._u_to_net_acceleration(u, v=10.))

    for point in [[-12.5, 0.], [merge_point[0], -merge_point[1]], [end_point[0], -end_point[1]]]:
        print(track.carla_coordinates_to_travelled_distance(point))

    agent.vehicle_velocity = 10.
    agent.vehicle_travelled_distance = 0.
    agent.other_velocity = 9.
    agent.action_plan = 0.5
    agent._update_position_plan()

    plt.figure()
    plt.plot(agent.position_plan)
    plt.plot(agent.velocity_plan)

    plt.figure()
    plt.plot(accelerations, net_u)
    plt.vlines(0., -100., 100, linestyles='dashed', colors='lightgrey')
    plt.hlines(0., -100., 100, linestyles='dashed', colors='lightgrey')
    plt.xlim((-1.2, 1.2))
    plt.ylim((-6, 5))

    sig = []
    v = [.01 * i - 1 for i in range(201)]

    for index in v:
        sig.append((casadi.tanh(500 * (index - 0.01))) / 2 + 1 / 2)

    # plt.figure()
    # plt.plot(v, sig)

    plt.show()
