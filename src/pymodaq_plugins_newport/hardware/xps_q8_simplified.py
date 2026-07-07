from .XPS_Q8_drivers import XPS


class XPSError(Exception):
    """
    Exception related to the Newport XPS system
    """

    pass


class SimpleXPS:
    def __init__(
        self,
        ip: str,
        port: int,
        group: str,
        positioner: str,
    ):
        """
        Parameters
        ----------
        ip: str
            IP address to use for communication with the XPS motion controller. ex: "192.168.0.254"
        port: int
            IP port for communication with the XPS. ex: 5001
        group: str
            name of the group to control. ex: "Group2"
        positioner: str
            name of the positioner. ex: "Pos"
        """

        # init the wrapper given by Newport and some attributes
        self.xps = XPS()  # Instantiate the driver from Newport

        # required to connect via TCP/IP
        self._ip = ip
        self._port = port
        self.socket_id = -1

        # Definition of the stage
        self._group = group
        self._positioner = positioner
        self._full_positioner_name = f"{group}.{positioner}"

        # Some required initialisation steps
        self._init_commands()

    def _init_commands(self):
        """
        Runs some initial commands : connect to the XPS server, group kill, group intialize, move home.
        Some configs could be added here as well
        """
        self.socket_id = self.xps.TCP_ConnectToServer(
            self._ip, self._port, 5
        )  # 5s timeout
        # Check connection passed
        if self.socket_id == -1:
            raise XPSError("XPS_Q8 connection failed. Check ip address and port.")
        else:
            # Group kill to be sure
            [error_code, return_string] = self.xps.GroupKill(
                self.socket_id, self._group
            )
            if error_code != 0:
                self.display_error_and_close(error_code, "GroupKill")

            # Initialize
            [error_code, return_string] = self.xps.GroupInitialize(
                self.socket_id, self._group
            )
            if error_code != 0:
                self.display_error_and_close(error_code, "GroupInitialize")

            # Home search
            self.move_home()

            # Definition of the MotionDone trigger
            # [error_code, return_string] = self.xps.EventExtendedConfigurationTriggerSet(self.socket_id, 'MotionDone',0,0,0,0)
            # if (error_code != 0):
            #     self.display_error_and_close(error_code, 'EventExtendedConfigurationTriggerSet')
            # [error_code, return_string] = self.xps.EventExtendedConfigurationActionSet(self.socket_id, , 0, 0, 0, 0)
            # if (error_code != 0):
            #     self.display_error_and_close(error_code, 'EventExtendedConfigurationActionSet')

    def check_connected(self):
        """Returns true if the connection was successful, else false."""
        return self.socket_id != -1

    def display_error_and_close(self, error_code, api_name):
        """Method to recover an error string based on an error code. Closes the TCPIP connection afterwards"""
        if (error_code != -2) and (error_code != -108):
            [error_code, error_string] = self.xps.ErrorStringGet(
                self.socket_id, error_code
            )
            if error_code != 0:
                raise XPSError(f"{api_name} : ERROR {error_code}")
            else:
                raise XPSError(f"{api_name} : {error_string}")
        else:
            if error_code == -2:
                raise XPSError(f"{api_name} : TCP timeout")
            if error_code == -108:
                raise XPSError(
                    f"{api_name} : The TCP/IP connection was closed by an administrator"
                )
        self.close_tcpip()

    def close_tcpip(self):
        """Call the method to close the socket."""
        self.xps.TCP_CloseSocket(self.socket_id)

    def get_position(self):
        """Returns current the position"""
        if self.check_connected():
            [error_code, current_position] = self.xps.GroupPositionCurrentGet(
                self.socket_id, self._full_positioner_name, 1
            )
            if error_code != 0:
                self.display_error_and_close(error_code, "GroupPositionCurrentGet")
            else:
                return float(current_position)
        else:
            raise XPSError("XPS connection failed")

    def move_absolute(self, value):
        """Moves the stage to the position value."""
        if self.check_connected():
            [error_code, return_string] = self.xps.GroupMoveAbsolute(
                self.socket_id, self._full_positioner_name, [value]
            )
            if error_code != 0:
                self.display_error_and_close(error_code, "GroupMoveAbsolute")
        else:
            raise XPSError("XPS connection failed")

    def move_relative(self, value):
        """Moves the stage to value relative to it's current position."""
        if self.socket_id != -1:
            [error_code, return_string] = self.xps.GroupMoveRelative(
                self.socket_id, self._full_positioner_name, [value]
            )
            if error_code != 0:
                self.display_error_and_close(error_code, "GroupMoveRelative")
        else:
            raise XPSError("XPS connection failed")

    def move_home(self):
        """Moves the stage to it's home"""
        if self.check_connected():
            [error_code, return_string] = self.xps.GroupHomeSearch(
                self.socket_id, self._group
            )
            if error_code != 0:
                self.display_error_and_close(error_code, "GroupHomeSearch")
        else:
            raise XPSError("XPS connection failed")

    def get_group_status(self):
        """Returns (status_code, status_string) for the current group."""
        if not self.check_connected():
            raise XPSError("XPS connection failed")

        [error_code, status_code] = self.xps.GroupStatusGet(self.socket_id, self._group)
        if error_code != 0:
            self.display_error_and_close(error_code, "GroupStatusGet")
            return None

        [error_code, status_string] = self.xps.GroupStatusStringGet(
            self.socket_id, status_code
        )
        if error_code != 0:
            self.display_error_and_close(error_code, "GroupStatusStringGet")
            return status_code, ""

        return status_code, status_string

    def get_following_error(self):
        """Returns the current following error, in controller units, for the positioner."""
        if not self.check_connected():
            raise XPSError("XPS connection failed")

        [error_code, following_error] = self.xps.GroupCurrentFollowingErrorGet(
            self.socket_id, self._group, 1
        )
        if error_code != 0:
            self.display_error_and_close(error_code, "GroupCurrentFollowingErrorGet")
            return None

        return float(following_error)

    def get_positioner_error(self):
        """Reads (and clears) the positioner error code. Returns (code, description)."""
        if not self.check_connected():
            raise XPSError("XPS connection failed")

        [error_code, positioner_error_code] = self.xps.PositionerErrorGet(
            self.socket_id, self._full_positioner_name
        )
        if error_code != 0:
            self.display_error_and_close(error_code, "PositionerErrorGet")
            return None

        if positioner_error_code == 0:
            return 0, "No error"

        [error_code, error_string] = self.xps.PositionerErrorStringGet(
            self.socket_id, positioner_error_code
        )
        if error_code != 0:
            self.display_error_and_close(error_code, "PositionerErrorStringGet")
            return positioner_error_code, ""

        return positioner_error_code, error_string

    def set_group(self, group: str):
        """
        Sets the group to control with the plugin

        Args:
            group: Name of the group
        """
        self._group = group
        self._full_positioner_name = f"{group}.{self._positioner}"

    def set_positioner(self, positioner: str):
        """
        Sets the positioner to control with the plugin

        Args:
            positioner: Name of the positioner
        """
        self._positioner = positioner
        self._full_positioner_name = f"{self._group}.{positioner}"

    def set_ip(self, ip: str):
        """
        Sets the IP address to use for TCPIP communication with the XPS motion controller.

        Args:
            ip: IP address of the XPS device
        """
        self._ip = ip

    def set_port(self, port: int):
        """
        Sets the port to use for TCPIP communication with the XPS motion controller.

        Args:
            port: port to use
        """
        self._port = port

    def retry_connection(self):
        """
        Closes the existing TCPIP connection and tries to reconnect
        """
        self.close_tcpip()
        self._init_commands()
