"""
Convert QGroundControl .plan file and convert to QGC WPL 120 format required by Parrot Anafi

Helpful sources: 
QGC WPL format: https://mavlink.io/en/file_formats/
Parrot flight plan documentation: https://developer.parrot.com/docs/mavlink-flightplan/messages.html
Mavlink documentation: https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
"""

import argparse
from plan_converter import MavImageOptions, PlanConverter


parser = argparse.ArgumentParser(
    prog="main.py",
    description="Convert a QGroundControl-created .plan file \
    into a QGC WPL 120 flight plan that control the Parrot Anafi",
)

parser.add_argument("plan_path", help="Path to .plan file to convert", type=str)

parser.add_argument(
    "--out_path",
    "--out",
    action="store",
    default=None,
    help="Path to save output file \
    DEFAULT: same name/path as .plan file with .txt extension",
    type=str,
    dest="out_path",
)

parser.add_argument(
    "--force_home",
    "--home",
    action="store",
    default=None,
    help='Override home location set in .plan file to takeoff/land from. \
    This can be a list with either 2 values "lon,lat" OR 3 values "lon,lat,alt" \
    Using 2 values (or the home location set in the .plan file) will set the altitude \
    of the home location waypoints (first/last) to the altitude of the first waypoint \
    from the flight plan. \
    Using 3 values will instead set the altitude of the third value \
    DEFAULT: use home location set in .plan \
    NOTE: will error if no home location set in .plan file and this value is not set \
    Inputs MUST be quoted and without brackets or spaces',
    type=str,
    dest="force_home",
)

parser.add_argument(
    "--speed_takeoff",
    action="store",
    default=3.0,
    help="Set speed for takeoff \
    DEFAULT: 3.0, min=1.0, max=10.0",
    type=float,
    dest="speed_takeoff",
)

parser.add_argument(
    "--speed_flying",
    action="store",
    default=3.0,
    help="Set speed for flying between waypoints \
    DEFAULT: 3.0, min=1.0, max=10.0",
    type=float,
    dest="speed_flying",
)

parser.add_argument(
    "--speed_landing",
    action="store",
    default=2.0,
    help="Set speed for landing \
    DEFAULT: 2.0, min=1.0, max=10.0",
    type=float,
    dest="speed_landing",
)

parser.add_argument(
    "--waypoint_radius",
    action="store",
    default=2.0,
    help="Acceptance radius for arriving at each waypoint \
    DEFAULT: 2.0",
    type=float,
    dest="waypoint_radius",
)

parser.add_argument(
    "--waypoint_time",
    action="store",
    default=1.0,
    help="Time to wait after arriving at each waypoint \
    DEFAULT: 1.0",
    type=float,
    dest="waypoint_time",
)

parser.add_argument(
    "--image_mode",
    action="store",
    default=12.0,
    help="Image capture mode \
    OPTIONS: snapshot (0.0), JPEG rectilinear (12.0), JPEG fisheye (13.0), raw DNG (14.0) \
    DEFAULT: 12.0 \
    NOTE: raw DNG (14.0) does not seem to work properly",
    type=float,
    dest="image_mode",
    choices=[
        MavImageOptions.SNAPSHOT,
        MavImageOptions.JPEG,
        MavImageOptions.JPEG_FISHEYE,
        MavImageOptions.RAW,
    ],
)

parser.add_argument(
    "--gimbal_angle",
    action="store",
    default=-90.0,
    help="Angle to tilt the gimbal (negative values are down) \
    DEFAULT: -90.0, min=-90.0, max=90.0",
    type=float,
    dest="gimbal_angle",
)

parser.add_argument(
    "--initial_wait",
    action="store",
    default=20.0,
    help="Time to wait before takeoff, allowing user to connect with Freeflight app \
    DEFAULT: 20.0",
    type=float,
    dest="initial_wait",
)

parser.add_argument(
    "--track_yaw",
    action="store",
    default=False,
    help="Whether to track changes in yaw and change heading as drone moves (True) \
    or to always face North (False) \
    DEFAULT: False",
    type=bool,
    dest="track_yaw",
)

__doc__ += "\n" + parser.format_help()


def main():
    args = parser.parse_args()
    pc = PlanConverter(
        speed_takeoff=args.speed_takeoff,
        speed_flying=args.speed_flying,
        speed_landing=args.speed_landing,
        image_mode=args.image_mode,
        gimbal_angle=args.gimbal_angle,
        initial_wait=args.initial_wait,
        waypoint_time=args.waypoint_time,
        waypoint_radius=args.waypoint_radius,
        track_yaw=args.track_yaw,
    )
    # convert string home to list of floats
    home = [float(i.strip("")) for i in args.force_home.split(",")] if args.force_home else None
    pc.convert_plan(args.plan_path, out_path=args.out_path, force_home=home)
    print("Done.")


if __name__ == "__main__":
    main()
