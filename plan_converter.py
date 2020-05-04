"""
Class to convert QGroundControl .plan file and convert to QGC WPL 120 format
"""

import os
import json
import math


class MavCommand:
    """
	Enum class for MAVLink commands
	Documentation: https://developer.parrot.com/docs/mavlink-flightplan/messages.html
	NOTE: not all commands used/supported in PlanConverter class
	"""

    MAV_CMD_NAV_WAYPOINT = 16
    MAV_CMD_NAV_LAND = 21
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_CMD_NAV_DELAY = 93
    MAV_CMD_CONDITION_DELAY = 112
    MAV_CMD_DO_CHANGE_SPEED = 178
    MAV_CMD_DO_SET_ROI = 201
    MAV_CMD_DO_MOUNT_CONTROL = 205
    MAV_CMD_VIDEO_START_CAPTURE = 2500
    MAV_CMD_VIDEO_STOP_CAPTURE = 2501
    MAV_CMD_PANORAMA_CREATE = 2800
    MAV_CMD_IMAGE_START_CAPTURE = 2000
    MAV_CMD_IMAGE_STOP_CAPTURE = 2001
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
    MAV_CMD_SET_VIEW_MODE = 50000
    MAV_CMD_SET_STILL_CAPTURE_MODE = 50001


class MavViewMode:
    """
	Enum class for MAV_VIEW_MODE_TYPE options
	Documentation: https://developer.parrot.com/docs/mavlink-flightplan/messages.html#mav-view-mode-type
	"""

    VIEW_MODE_TYPE_ABSOLUTE = 0.0
    VIEW_MODE_TYPE_CONTINUE = 1.0
    VIEW_MODE_TYPE_ROI = 2.0


class MavCaptureMode:
    """
	Enum class for MAV_STILL_CAPTURE_MODE_TYPE options
	Documentation: https://developer.parrot.com/docs/mavlink-flightplan/messages.html#mav-still-capture-mode-type
	"""

    STILL_CAPTURE_MODE_TYPE_TIMELAPSE = 0.0
    STILL_CAPTURE_MODE_TYPE_GPS_POSITION = 1.0


class MavImageOptions:
    """
	Enum class for MAV_CMD_IMAGE_START_CAPTURE format options
	Documentation: https://developer.parrot.com/docs/mavlink-flightplan/messages.html#mav-cmd-image-start-capture-message-code-2000
	"""

    SNAPSHOT = 0.0
    JPEG = 12.0
    JPEG_FISHEYE = 13.0
    RAW = 14.0


class PlanConverter:
    def __init__(
        self,
        speed_takeoff=3.0,
        speed_flying=3.0,
        speed_landing=2.0,
        image_mode=MavImageOptions.JPEG,
        gimbal_angle=-90.0,
        initial_wait=20.0,
        waypoint_time=1.0,
        waypoint_radius=2.0,
        track_yaw=False,
    ):
        """
		Class to parse .plan file and convert to QGC WPL 120 format
		"""
        # ensure valid input paramters
        assert speed_takeoff >= 1.0 and speed_takeoff <= 10.0
        assert speed_flying >= 1.0 and speed_flying <= 10.0
        assert speed_landing >= 1.0 and speed_landing <= 10.0
        # NOTE: should be handled by argparse, but let's be sure
        assert image_mode in [
            MavImageOptions.SNAPSHOT,
            MavImageOptions.JPEG,
            MavImageOptions.JPEG_FISHEYE,
            MavImageOptions.RAW,
        ]
        assert gimbal_angle >= -90.0 and gimbal_angle <= 90.0
        assert initial_wait >= 0.0
        assert waypoint_time >= 0.0
        assert waypoint_radius >= 0.0
        self.speed_takeoff = speed_takeoff
        self.speed_flying = speed_flying
        self.speed_landing = speed_landing
        self.image_mode = image_mode
        self.gimbal_angle = gimbal_angle
        self.initial_wait = initial_wait
        self.waypoint_time = waypoint_time
        self.waypoint_radius = waypoint_radius
        self.track_yaw = track_yaw
        # internally tracked objects
        self.current_waypoint = []
        self.home = []
        self.home_flying_altitude = None
        self.current_step = 0

    def convert_plan(self, plan_path, out_path=None, force_home=None):
        """
		Convert a .plan file to QGC WPL format
		"""
        assert plan_path.endswith(".plan")
        if out_path is None:
            out_path = plan_path.split(".plan")[0] + ".txt"
        with open(plan_path) as fp:
            plan_dict = json.load(fp)
        # get home location for initial waypoint from .plan or overriden value
        self.home = self._get_home_location(plan_dict, force_home)
        # TODO: ensure home is close to first waypoint
        cmd_ls = self._set_initial_commands()
        # iterate through plan items
        cmd_ls += self._parse_plan_list(plan_dict)
        # final commands - fly back to home position, set landing speed, land
        cmd_ls += self._set_final_commands()
        # convert cmd list to string, add version 120 at top
        cmd_str = "QGC WPL 120\r\n"
        for item_list in cmd_ls:
            if item_list is not None:
                cmd_str += self._list_to_string(item_list)
        # TODO: line is from inspiration source, determine if needed
        cmd_str = cmd_str.strip() + "\r\n"  # add back the final newline
        # write file to disk
        with open(out_path, "w") as fp:
            fp.write(cmd_str)
        # reset values
        self._reset_values()

    def _get_home_location(self, plan_dict, force_home):
        """
		Ensure home is valid if force_home used, otherwise pull home location from .plan file
		NOTE: use altitude of first waypoint if none is provided by force_home
		"""
        if force_home is not None:
            # ensure force_home set correctly - list of 2 values
            # NOTE: could be set to 3 values if starting/ending altitude differ from flight plan
            try:
                assert type(force_home) == list and (len(force_home) == 2 or len(force_home) == 3)
                home = force_home
                if len(home) == 3:
                    self.home_flying_altitude = home[2]
                    home = home[:2]
            except AssertionError as e:
                raise AssertionError('Invalid input for "force_home," got {}'.format(force_home))
        else:
            # extract home position from plan, fail if not found
            # NOTE: home in order [lat, lon, alt], rearrange as [lon, lat] to remain consistent with force_home - removing alt
            try:
                tmp_home = plan_dict["mission"]["plannedHomePosition"]
                home = [tmp_home[1], tmp_home[0]]
            except KeyError as e:
                raise KeyError(
                    'No "plannedHomePosition" could be found in .plan file, set it in QGC or override with "force_home" value'
                )
        if self.home_flying_altitude is None:
            first_waypoint = self._initial_waypoint(plan_dict)
            self.home_flying_altitude = float(first_waypoint[-1])
        return home

    def _initial_waypoint(self, plan_dict):
        """
		Get initial waypoint from .plan file
		Use to check distance from home and include altitude for initial home waypoint
		"""
        for item in plan_dict["mission"]["items"]:
            # handle for set of items
            if "TransectStyleComplexItem" in item:
                for sub_item in item["TransectStyleComplexItem"]["Items"]:
                    if sub_item["command"] == MavCommand.MAV_CMD_NAV_WAYPOINT:
                        return sub_item["params"][4:]
            # handle for single item
            elif "params" in item:
                if item["command"] == MavCommand.MAV_CMD_NAV_WAYPOINT:
                    return item["params"][4:]
        # NOTE: this exception should theoretically NEVER hit...
        raise KeyError("Could not find any waypoints in .plan file, what the hell did you do?")

    def _item_to_list(self, item):
        """
		Convert a .plan file item to a list for QGC QPL format
		NOTE: forcing None entries to 0.0
		"""
        return [
            self.current_step,
            0,  # HARDCODED as 0
            3,  # HARDCODED as 3 - don't fully understand, different in .plan, all 3's in converted version from app, if using .plan's use item['frame']
            item["command"],
            float(item["params"][0]) if item["params"][0] != None else 0.0,
            float(item["params"][1]) if item["params"][1] != None else 0.0,
            float(item["params"][2]) if item["params"][2] != None else 0.0,
            float(item["params"][3]) if item["params"][3] != None else 0.0,
            float(item["params"][4]) if item["params"][4] != None else 0.0,
            float(item["params"][5]) if item["params"][5] != None else 0.0,
            float(item["params"][6]) if item["params"][6] != None else 0.0,
            1 if item["autoContinue"] else 0,  # always use 1 and just hardcode?
        ]

    def _format_list(self, item_list):
        """
		Format list entry by incrementing step and handling any commands that need further work 
		"""
        self.current_step += 1
        # handle for commands requiring alteration
        cmd = int(item_list[3])
        command_dict = {
            MavCommand.MAV_CMD_IMAGE_START_CAPTURE: self._handle_picture,
            MavCommand.MAV_CMD_NAV_WAYPOINT: self._handle_waypoint,
            MavCommand.MAV_CMD_NAV_RETURN_TO_LAUNCH: self._handle_rth,
        }
        try:
            return command_dict[cmd](item_list)
        except KeyError:
            return item_list

    def _handle_picture(self, item_list):
        """
		Handle for picture commmand
		"""
        item_list[5] = 1.0  # HARDCODED capture count
        item_list[6] = self.image_mode
        return item_list

    def _handle_waypoint(self, item_list):
        """
		Handle for waypoint command
		"""
        item_list[4] = self.waypoint_time
        item_list[5] = self.waypoint_radius
        if self.track_yaw:
            item_list[10] = self._angle_to_next_pt(item_list)
        return item_list

    def _handle_rth(self, item_list):
        """
		Handle explicit return to home (RTH)
		NOTE: we don't understand this command, seems to always fail
		for now just remove since we add an explicit waypoint for home anyways
		"""
        # NOTE: need to decrement otherwise there'll be a missing number near the end
        self.current_step -= 1
        return None

    def _angle_to_next_pt(self, item_list):
        """
		Calculate the angle (in degrees) from self.current_waypoint to next point
		NOTE: not compensating for distortion, angles calculated to the RIGHT (clockwise)	
		"""
        # NOTE: positions swapped since spec requires lat/lon order
        next_pt = [item_list[9], item_list[8]]
        # handle for initial point - just use 0
        if len(self.current_waypoint[0]) == 0:
            yaw = 0
        else:
            x_offset = self.current_waypoint[0] - next_pt[0]
            y_offset = self.current_waypoint[1] - next_pt[1]
            yaw = (360 + math.degrees(math.atan2(x_offset, y_offset))) % 360
        self.current_waypoint = next_pt
        return yaw

    def _set_initial_commands(self):
        """
		Set initial item lists for start of flight plan
		Takeoff speed, wait, gimbal, image type, launch, waypoint of home, flying speed
		"""
        cmd_ls = []
        # set takeoff speed
        speed1 = [
            self.current_step,
            0,
            3,
            MavCommand.MAV_CMD_DO_CHANGE_SPEED,
            1.0,
            self.speed_takeoff,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1,
        ]
        cmd_ls.append(self._format_list(speed1))
        # set initial delay, if there is one
        if self.initial_wait > 0:
            delay = [
                self.current_step,
                0,
                3,
                MavCommand.MAV_CMD_NAV_DELAY,
                self.initial_wait,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1,
            ]
            cmd_ls.append(self._format_list(delay))
        # set image mode
        mode = [
            self.current_step,
            0,
            3,
            MavCommand.MAV_CMD_SET_STILL_CAPTURE_MODE,
            MavCaptureMode.STILL_CAPTURE_MODE_TYPE_GPS_POSITION,
            self.waypoint_radius,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1,
        ]
        cmd_ls.append(self._format_list(mode))
        # set gimbal position
        gimbal = [
            self.current_step,
            0,
            3,
            MavCommand.MAV_CMD_DO_MOUNT_CONTROL,
            self.gimbal_angle,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            2.0,
            1,
        ]
        cmd_ls.append(self._format_list(gimbal))
        # use explicit launch, does not seem to be necessary, but probably better to do
        launch = [
            self.current_step,
            0,
            3,
            MavCommand.MAV_CMD_NAV_TAKEOFF,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1,
        ]
        cmd_ls.append(self._format_list(launch))
        # initial waypoint at home, uses altitude of first wayoint if one not set in force_home
        home1 = [
            self.current_step,
            0,
            3,
            MavCommand.MAV_CMD_NAV_WAYPOINT,
            0.0,
            0.0,
            0.0,
            0.0,
            self.home[1],
            self.home[0],
            self.home_flying_altitude,
            1,
        ]
        cmd_ls.append(self._format_list(home1))
        # set flying speed, if different from takeoff speed
        if self.speed_flying != self.speed_takeoff:
            speed2 = [
                self.current_step,
                0,
                3,
                MavCommand.MAV_CMD_DO_CHANGE_SPEED,
                1.0,
                self.speed_flying,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1,
            ]
            cmd_ls.append(self._format_list(speed2))
        return cmd_ls

    def _parse_plan_list(self, plan_dict):
        """
		Parse .plan dictionary items for items
		"""
        cmd_ls = []
        # iterate through plan items
        for item in plan_dict["mission"]["items"]:
            # handle for set of items
            if "TransectStyleComplexItem" in item:
                for sub_item in item["TransectStyleComplexItem"]["Items"]:
                    cmd_ls.append(self._format_list(self._item_to_list(sub_item)))
            # handle for single item
            elif "params" in item:
                cmd_ls.append(self._format_list(self._item_to_list(item)))
        return cmd_ls

    def _set_final_commands(self):
        """
		Set final item lists for end of flight plan
		Waypoint of home, landing speed, land 
		"""
        cmd_ls = []
        # fly back to home position
        home2 = [
            self.current_step,
            0,
            3,
            MavCommand.MAV_CMD_NAV_WAYPOINT,
            0.0,
            0.0,
            0.0,
            0.0,
            self.home[1],
            self.home[0],
            self.home_flying_altitude,
            1,
        ]
        cmd_ls.append(self._format_list(home2))
        # set landing speed, if different from flying speed
        if self.speed_flying != self.speed_landing:
            speed3 = [
                self.current_step,
                0,
                3,
                MavCommand.MAV_CMD_DO_CHANGE_SPEED,
                1.0,
                self.speed_landing,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1,
            ]
            cmd_ls.append(self._format_list(speed3))
        # give explicit land
        land = [
            self.current_step,
            0,
            3,
            MavCommand.MAV_CMD_NAV_LAND,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1,
        ]
        cmd_ls.append(self._format_list(land))
        return cmd_ls

    def _list_to_string(self, item_list):
        """
		Convert a list representation of command items to string
		"""
        return "\t".join([str(i) for i in item_list]) + "\r\n"

    def _reset_values(self):
        """
		Reset values to convert another .plan file
		"""
        self.current_waypoint = []
        self.home = []
        self.home_flying_altitude = 0
        self.current_step = 0
