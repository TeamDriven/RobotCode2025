import json

INPUT_DIR = "C:\Projects\RobotCode2025\src\main\deploy\choreo\Place 3 left side.traj"
OUTPUT_DIR = "C:\Projects\RobotCode2025\src\main\deploy\choreo\Place 3 right side.traj"

FIELD_LENGTH = 17.548225
FIELD_WIDTH = 8.0518

input = open(INPUT_DIR, "r")

input_json = json.loads(input.read())

output_json = input_json.copy()

for i, waypoint in enumerate(input_json["snapshot"]["waypoints"]):
    newY = FIELD_WIDTH - waypoint["y"]
    output_json["snapshot"]["waypoints"][i].update(y=newY)

    newHeading = waypoint["heading"] * -1
    output_json["snapshot"]["waypoints"][i].update(heading=newHeading)

for i, waypoint in enumerate(input_json["params"]["waypoints"]):
    newY = FIELD_WIDTH - waypoint["y"]["val"]
    output_json["params"]["waypoints"][i]["y"].update(val=newY)
    output_json["params"]["waypoints"][i]["y"].update(exp="{y:.15f} m".format(y=newY))

    newHeading = waypoint["heading"]["val"] * -1
    output_json["params"]["waypoints"][i]["heading"].update(val=newHeading)
    output_json["params"]["waypoints"][i]["heading"].update(exp="{heading:.15f} rad".format(heading=newHeading))

output = open(OUTPUT_DIR, "w")

output.write(json.dumps(output_json, indent=2))