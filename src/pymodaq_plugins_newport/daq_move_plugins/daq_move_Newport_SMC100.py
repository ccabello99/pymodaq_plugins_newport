from pymodaq.control_modules.move_utility_classes import DAQ_Move_base, comon_parameters_fun, main, DataActuator
from pymodaq.utils.daq_utils import ThreadCommand  # object used to send info back to the main thread
from pymodaq.utils.parameter import Parameter

from pymodaq_plugins_newport.hardware.smc100 import SMC100

import pyvisa

rm = pyvisa.ResourceManager()
infos = rm.list_resources_info()
ports = [infos[key].interface_board_number for key in infos.keys()]
stage_nb = [1]  # Works with 1 SMC100 controller (1 stage). Not tested with multiple controllers
rm.close()

class DAQ_Move_Newport_SMC100(DAQ_Move_base):
    """ Instrument plugin class for an actuator.

    This object inherits all functionalities to communicate with PyMoDAQ’s DAQ_Move module through inheritance via
    DAQ_Move_base. It makes a bridge between the DAQ_Move module and the Python wrapper of a particular instrument.

    This plugin should be compatible with SMC100 controllers using USB/RS232 connection. Unit should be changed for
    controlling linear actuators (currently set to degrees).

    Tested with SMC100PP (stepper motor) controller and URS150 motorized rotation stage

    Operating System: Windows 11
    PyMoDAQ version: 4.3.0 running in a conda environment with Python 3.11.9

    Installing Newport SMC100 software should install all necessary drivers (testing with manufacturer's software
    should be done in the first place)
    https://www.newport.com/f/smc100-single-axis-dc-or-stepper-motion-controller

    Attributes:
    -----------
    controller: object
        The particular object that allow the communication with the hardware, in general a python wrapper around the
         hardware library.
    """
    _controller_units = 'mm'
    is_multiaxes = False
    _axis_names = ['1']
    _epsilon = 0.01

    params = [{'title': 'COM Port:', 'name': 'com_port', 'type': 'list', 'limits': ports},
              {'title': 'Stage:', 'name': 'stage_nb', 'type': 'list', 'limits': stage_nb}
             ] + comon_parameters_fun(is_multiaxes, axis_names=_axis_names, epsilon=_epsilon)

    # print(params[0]['title'])

    # _epsilon is the initial default value for the epsilon parameter allowing pymodaq to know if the controller reached
    # the target value. It is the developer responsibility to put here a meaningful value

    def ini_attributes(self):
        self.controller: SMC100 = None

    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        pos = self.controller.position
        pos = self.get_position_with_scaling(pos)
        return pos

    def close(self):
        """Terminate the communication protocol"""
        if self.controller is not None:
            self.controller.reset()
            self.controller.close()

    def commit_settings(self, param: Parameter, controller=None):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        controller:
        """

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """

        self.controller = self.ini_stage_init(old_controller=controller,
                                              new_controller=SMC100(port=self.settings['com_port'],
                                                                    dev_number=self.settings['stage_nb']))

        info = "Initializing stage"
        self.controller.homing()  # Turns controller to REFERENCED state (solid green)

        print(f"Connected to Newport stage {self.settings['stage_nb']} on COM{self.settings['com_port']}:"
              f" {self.controller.idn}\n")
        initialized = True
        return info, initialized

    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """

        value = self.check_bound(value)  # if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one

        self.controller.move_abs(value)  # when writing your own plugin replace this line
        self.emit_status(ThreadCommand('Update_Status', ['Moving']))

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        value = self.set_position_relative_with_scaling(value)

        self.controller.move_rel(value)  # when writing your own plugin replace this line
        self.emit_status(ThreadCommand('Update_Status', ['Moving']))

    def move_home(self):
        """Call the reference method of the controller"""

        self.controller.move_abs(.0)  # when writing your own plugin replace this line
        self.emit_status(ThreadCommand('Update_Status', ['Moving to position 0']))

    def stop_motion(self):
        """Stop the actuator and emits move_done signal"""

        self.controller.stop()  # when writing your own plugin replace this line
        self.get_actuator_value()
        self.emit_status(ThreadCommand('Update_Status', ['Motion stopped']))


if __name__ == '__main__':
    main(__file__)