# Flightplan Converter

This code converts a .plan file (format created by QGroundControl) to a text file in QGC WPL 120 format, which isrequired to run automated flight plans on the Parrot Anafi, either "IRL" or in Sphinx. 


## QGroundControl

QGroundControl provides a helpful guide to creating flight plans here: https://docs.qgroundcontrol.com/en/PlanView/pattern_survey.html

Be sure to set up your camera parameters!

Remember to set your desired home location in QGroundControl as well. This is not a requirement, as it can be provided/overriden when converting the file, but if one is neither set nor provided, the code will error.

Once you have created your desired flight plan, save it and get the path to it. 


## Script Usage

First, create your desired flight plan in QGroundControl. Then get the path of the file and run `main.py` like so:

```
python3 main.py /path/to/file.plan
```

This will convert the file with all the default parameters, use the home location set in the file (which can be overriden or provided if one was not set), and save the new file with the same path/name as the original, just with a `.txt` extension.

You can change the output file path and provide a new home location like this:

```
python3 main.py /path/to/file.plan /path/to/output.txt --force_home "-118.2,33.3"
```

There are numerous optional paramters that can be set as well, to see them all run:

```
python3 main.py --help
```


## Requirements

You can download/install QGroundControl here: https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html 

The conversion code uses the python3 standard library, no 3rd party libraries required!

Code formatted with black like so:

```
python3 -m black --target-version py37 --line-length 100 .
```

## Unanswered Questions

RTH (20) - how exactly does it work?

Taking pictures - what is snapshot (0), how to make raw dng (14) work properly?

Max altitude/distance from home enforced? 

Flightless plan - can take images without flying?

Diff pic modes/types possible in same flight? - so far seems like no

Need to understand frame value (3rd column) - always make 3?
Set coord frame to 0, use absolute alt?
https://mavlink.io/en/messages/common.html#MAV_FRAME


## Sources

QGC WPL format: https://mavlink.io/en/file_formats/

Parrot docs: https://developer.parrot.com/docs/mavlink-flightplan/messages.html

Mavlink docs: https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml

QGroundControl survey: https://docs.qgroundcontrol.com/en/PlanView/pattern_survey.html

Parser for old format (inspiration): https://github.com/Phrohdoh/mission2waypoint/blob/master/main.py

Forum post about it: https://forum.developer.parrot.com/t/convert-qgc-mission-to-mavlink/4810

QGC issue: https://github.com/mavlink/qgroundcontrol/issues/2243
