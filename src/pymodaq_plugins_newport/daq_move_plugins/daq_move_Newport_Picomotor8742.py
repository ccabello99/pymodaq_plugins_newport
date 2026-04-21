from typing import Union, List, Dict

from pymodaq.control_modules.move_utility_classes import DAQ_Move_base,\
    comon_parameters_fun, main, DataActuatorType, DataActuator  # common set of parameters for all actuators
from pymodaq.utils.daq_utils import ThreadCommand # object used to send info back to the main thread
from pymodaq.utils.parameter import Parameter
from pylablib.devices import Newport


class DAQ_Move_Newport_Picomotor8742(DAQ_Move_base):
    """Plugin for the Picomotor 8742.

    Attributes:
    -----------
    controller: Newport.Picomotor8742
        The particular object that allow the communication with the hardware, from pylablib

    """
    _controller_units: Union[str, List[str]] = ''
    is_multiaxes = True
    axes_names: Union[List[str], Dict[str, int]] = {'1': 1, '2': 2, '3': 3, '4': 4}
    _epsilon: Union[float, List[float]] = 10.0
    data_actuator_type = DataActuatorType.DataActuator

    devices_info = {"hostnames": ["8742-14296", "8742-14382", "8742-14383", "8742-100628"],
               "ips": ["192.168.178.202", "192.168.178.184", "192.168.178.185", "192.168.178.217"],
               }
    
    
    params = [
                {'title': 'Device Management:', 'name': 'device_manager', 'type': 'group', 'children': [
                    {'title': 'Connected Devices:', 'name': 'connected_devices', 'type': 'list', 'limits': devices_info['hostnames']},
                    {'title': 'Host name:', 'name': 'hostname', 'type': 'str', 'value': "", 'readonly': True},
                    {'title': 'IP address:', 'name': 'ip', 'type': 'str','value': "", 'readonly': True},
                    {'title': 'Selected Device:', 'name': 'selected_device', 'type': 'str', 'value': '', 'readonly': True}
                ]}, 
                {'title': 'Axis parameters: ', 'name': 'axis_p','type': 'group','children':[
                    {'title': 'Velocity (steps/s): ', 'name': 'speed_axis','type': 'int',
                    'value':0,'min':1,'max':2e3},
                    {'title': 'Acceleration (steps/s^2): ', 'name': 'acc_axis', 'type': 'int',
                    'value':0,'min':1,'max':2e5},
                    {'title': 'Type: ', 'name': 'motor', 'type':'str','value':'None'},]}] + \
                    comon_parameters_fun(is_multiaxes, axes_names, epsilon=_epsilon)

    
    def ini_attributes(self):
        self.controller: Newport.Picomotor8742 = None

    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        axis = self.axis_value
        pos = DataActuator(data=self.controller.get_position(axis=axis))
        pos = self.get_position_with_scaling(pos)
        return pos

    def close(self):
        """Terminate the communication """
        self.controller.close()  
        self.controller = None


    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        if param.name() == 'speed_axis' or param.name() == 'acc_axis':
            self.controller.setup_velocity(axis=self.axis_value,
                                           speed=self.settings['axis_p','speed_axis'],
                                           accel=self.settings['axis_p','acc_axis'])
        elif param.name() == 'connected_devices':
            device = self.settings.child('device_manager', 'connected_devices').value()
            param_ = self.settings.child('device_manager', 'hostname')
            index_ = self.devices_info['names'].index(device)
            hostname = self.devices_info['hostnames'][index_]
            param_.setValue(hostname)
            param_.sigValueChanged.emit(param_, hostname)
            param_ = self.settings.child('device_manager', 'ip')
            ip = self.devices_info['ips'][index_]
            param_.setValue(ip)
            param_.sigValueChanged.emit(param_, ip)
        else:
            pass

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
        self.controller = self.ini_stage_init(slave_controller=controller)

        if self.is_master:
            device = self.settings.child('device_manager', 'connected_devices').value()
            param_ = self.settings.child('device_manager', 'hostname')
            index_ = self.devices_info['names'].index(device)
            hostname = self.devices_info['hostnames'][index_]
            param_.setValue(hostname)
            param_.sigValueChanged.emit(param_, hostname)
            param_ = self.settings.child('device_manager', 'ip')
            ip = self.devices_info['ips'][index_]
            param_.setValue(ip)
            param_.sigValueChanged.emit(param_, ip)
            try:
                self.controller = Newport.Picomotor8742(hostname)
            except Exception:
                self.controller = Newport.Picomotor8742(ip)

        try:
            info = self.controller.get_id()
            initialized = True
        except:
            info = ""
            initialized = False

        if initialized:
            try:
                motor_types = self.controller.autodetect_motors()
                axis = self.axis_value
                self.settings['axis_p','motor'] = motor_types[axis-1]
            
                parameters_velocity = self.controller.get_velocity_parameters()
                self.settings['axis_p','speed_axis'] = parameters_velocity[axis-1][0]
                self.settings['axis_p','acc_axis'] = parameters_velocity[axis-1][1]
            except Exception as e:
                print("Error setting axis parameters: ", e)
            
        return info, initialized
    
    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """

        value = self.check_bound(value)  #if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one

        self.controller.move_to(self.axis_value, value.value())  
        self.emit_status(ThreadCommand('Update_Status',
                                       [f'The actuator is moved {value.value()} steps.']))

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        value = self.set_position_relative_with_scaling(value)

        self.controller.move_by(self.axis_value, steps=value.value()) 
        self.emit_status(ThreadCommand('Update_Status',
                    [f'The actuator is moved according to its current position of {value.value()} steps.']))

    def move_home(self):
        """Do nothing"""
        pass
        
    def stop_motion(self):
      """Stop the actuator and emits move_done signal"""
      self.controller.stop(axis=self.axis_value)
      self.emit_status(ThreadCommand('Update_Status', ['The motion of the actuator is stopped.']))


if __name__ == '__main__':
    main(__file__, init=False)
