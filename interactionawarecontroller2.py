import dynamics, lane, car, feature, world
import carla


desired_v = 1.
# adding Sadighs

dyn = dynamics.CarDynamics(0.1)
# Define cars in exp
simplecar = car.UserControlledCar(dyn, [10, 0, 0, 1], color='yellow')
nestedcar = car.NestedOptimizerCar(dyn,[0, 0, 0, 1], color='yellow')
experiment_env = world2.World()
clane = lane2.StraightLane([0., -1.], [0., 1.], 0.13)
experiment_env.lanes += [clane, clane.shifted(1), clane.shifted(-1)]
experiment_env.roads += [clane]
experiment_env.fences += [clane.shifted(2), clane.shifted(-2)]
simplecar.reward = experiment_env.simple_reward(simplecar, speed=0.5)

nestedcar.human = simplecar
flag = None
r_h = experiment_env.simple_reward([nestedcar.traj], speed_import=.2 if flag else 1., speed=0.8 if flag else 1.)+100.*feature.bounded_control(simplecar.bounds)
@feature.feature
def human_speed(t, x, u):
    return -nestedcar.traj_h.x[t][3]**2
h_speed = human_speed
r_r = 300.*h_speed + experiment_env.simple_reward(nestedcar, speed=0.5)
nestedcar.rewards = (r_h, r_r)

u = nestedcar.control(0.1, 1)

throttle = u[0]
steering = u[1]
# brake = u[2]
#
# # steering_angle in shared velocities is normalized with respect to the maximum steering angle
# self.shared_variables.steering_angle = steering
# self.shared_variables.handbrake = False
# self.shared_variables.reverse = False
# self.shared_variables.desired_velocity = desired_v
# self.shared_variables.throttle = throttle
# # self.shared_variables.brake = brake



class InteractionAwareControllerSettings:
    def __init__(self):
        self.controller_type = NPCControllerTypes.INTERACTION_AWARE_CONTROLLER

        # vehicle to control
        self.vehicle_id = ''
        # other vehicle
        self.opponent_id = ''

    def as_dict(self):
        return_dict = copy.copy(self.__dict__)
        for key, item in self.__dict__.items():
            if isinstance(item, Enum):
                return_dict[key] = item.value
        return return_dict

    def __str__(self):
        return str('Interaction-Aware Controller Settings')

    def set_from_loaded_dict(self, loaded_dict):
        for key, value in loaded_dict.items():
            if key == 'controller_type':
                self.__setattr__(key, NPCControllerTypes(value))
            else:
                self.__setattr__(key, value)


class InteractionAwareControllerDialog(QtWidgets.QDialog):
    # TODO: create a dialog for this controller
    def __init__(self, module_manager, settings: InteractionAwareControllerSettings, parent=None):
        super().__init__(parent=parent)
        self.module_manager = module_manager
        self.pure_pursuit_settings = settings
        uic.loadUi(os.path.join(os.path.dirname(os.path.realpath(__file__)), "ui/pure_pursuit_settings_ui.ui"), self)

        self.button_box_settings.button(self.button_box_settings.RestoreDefaults).clicked.connect(
            self._set_default_values)
        self.dynamicLADCheckBox.stateChanged.connect(self._update_static_dynamic_look_ahead_distance)
        self._fill_trajectory_combobox()
        self.display_values()

        self.show()

    def accept(self):
        self.pure_pursuit_settings.reference_trajectory_name = self.trajectoryComboBox.currentText()
        self.pure_pursuit_settings.static_look_ahead_distance = self.staticLADDoubleSpinBox.value()
        self.pure_pursuit_settings.use_dynamic_look_ahead_distance = self.dynamicLADCheckBox.isChecked()
        self.pure_pursuit_settings.steering_gain = self.steeringGainDoubleSpinBox.value()
        self.pure_pursuit_settings.dynamic_lad_a = self.aDoubleSpinBox.value()
        self.pure_pursuit_settings.dynamic_lad_b = self.bDoubleSpinBox.value()
        self.pure_pursuit_settings.kp = self.kpDoubleSpinBox.value()
        self.pure_pursuit_settings.kd = self.kdDoubleSpinBox.value()

        super().accept()

    def display_values(self, settings=None):
        if not settings:
            settings = self.pure_pursuit_settings

        # self.trajectoryComboBox.setCurrentIndex(self.trajectoryComboBox.findText(settings.reference_trajectory_name))
        # self.staticLADDoubleSpinBox.setValue(settings.static_look_ahead_distance)
        # self.dynamicLADCheckBox.setChecked(settings.use_dynamic_look_ahead_distance)
        # self.steeringGainDoubleSpinBox.setValue(settings.steering_gain)
        # self.aDoubleSpinBox.setValue(settings.dynamic_lad_a)
        # self.bDoubleSpinBox.setValue(settings.dynamic_lad_b)
        # self.kpDoubleSpinBox.setValue(settings.kp)
        # self.kdDoubleSpinBox.setValue(settings.kd)

    def _update_static_dynamic_look_ahead_distance(self):
        use_dynamic = self.dynamicLADCheckBox.isChecked()

        self.staticLADLabel.setEnabled(not use_dynamic)
        self.staticLADDoubleSpinBox.setEnabled(not use_dynamic)
        self.LADExplanationLabel.setEnabled(use_dynamic)
        self.aLabel.setEnabled(use_dynamic)
        self.bLabel.setEnabled(use_dynamic)
        self.aDoubleSpinBox.setEnabled(use_dynamic)
        self.bDoubleSpinBox.setEnabled(use_dynamic)

    def _set_default_values(self):
        self.display_values(NPCControllerTypes.PURE_PURSUIT.settings())

    def _fill_trajectory_combobox(self):
        self.trajectoryComboBox.addItem(' ')

        path_trajectory_directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'trajectories')
        file_names = glob.glob(os.path.join(path_trajectory_directory, '*.csv'))
        for file in file_names:
            self.trajectoryComboBox.addItem(os.path.basename(file))
