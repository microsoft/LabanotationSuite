#!/usr/bin/env python

# @package docstring
# Provide explanation of this code here.
# This code should be independent from ROS.
#
# More Details.
# Disclaimer and liscensing.
# Author information and email address to contact.
# Date of release.

import json
from labanotationpy import *


# Function to load labanotation json dictionary into a python class object.
#
#  @param  jsonData Json data as a python dictionary.
#  @param  filename Used for printing errors.
#  @param  old      For compatibility with old version.
#  @return Labanotation class object.
def LoadJsonDictionary(jsonData, filename, old=False):

    output = Labanotation()

    # Warn whenever a non-supported element is found
    # Below list MUST match number id in labanotationpy
    # Below body parts are supported as of now
    supportedNames = ['Face',
                      'LeftElbow',
                      'LeftWrist',
                      'LeftHand',
                      'LeftPalm',
                      'RightElbow',
                      'RightWrist',
                      'RightHand',
                      'RightPalm',
                      'WholeTorso',
                      'MotionDuration']
    if old:
        supportedNames[0] = 'Head'
        supportedNames += ['HoldDuration']

    # Below body parts support 'turn' for 'Sign' field in the new version.
    supportedTurns = {'WholeTorso': None}

    # A lot of the parts and signs are not supported in the old version.
    # isValidOld ensures that jsonData is compatible with the old version.
    def isValidOld(keyframeData, name, partid):
        if name not in keyframeData:
            print("[ERROR] labanotationpy: {} does not have field '{}'"
                  .format(filename, name))
            return False
        if 'Sign' in keyframeData[name]:
            print("[ERROR] labanotationpy: 'Sign' is not supported")
            return False
        if 'Direction' not in keyframeData[name] \
           or 'Level' not in keyframeData[name]:
            print("[ERROR] labanotationpy: {} Fields missing for {}!"
                  .format(filename, name))
            return False
        return True

    # Parse sign data on new version.
    def getSigns(keyframeData, name):
        data = keyframeData[name]
        # Turn sign implementation
        if 'Sign' in data:
            if TURN in data['Sign']:
                if name not in supportedTurns:
                    print("[ERROR] labanotationpy: turn invalid for {}"
                          .format(name))
                    turnSign = None
                else:
                    turnSign = data['Sign']
            else:
                print(("[WARN] labanotationpy: {} non-supported "
                       + "value '{}' in Sign, make sure to use lower case")
                      .format(filename, data['Sign']))
        elif (name in supportedTurns
              and supportedTurns[name] is None):
            supportedTurns[name] = TURN_NORMAL
            turnSign = supportedTurns[name]
            # if not None, keep as is with turnSign=None
        else:
            turnSign = None  # None also indicates "keep as is"
        # Check if data has valid direction symbols
        if 'Direction' not in data or 'Level' not in data:
            return None, None, turnSign
        else:
            return data['Direction'], data['Level'], turnSign

    # Set values to the labanotation class object
    # File has a list of keyframes
    timestamp = 0.0
    for k, keyframeData in enumerate(jsonData['Keyframes']):

        for key in keyframeData:
            if key not in supportedNames:
                print("[WARN] labanotationpy: {} in {} is not supported"
                      .format(key, filename))

        timestamp += keyframeData['MotionDuration']  # should finish time
        keyframeObject = LabanotationKeyframe()

        # Note, missing fields are accepted to some extent
        # In the old (deployed) version, ROS treats empty fields as joint hold
        # In the new (upcoming) version, ROS handles empty fields as below:
        # - Angles will be kept if whole move group (e.g. larm, head) is empty
        # - An empty WholeTorso is set to Place High
        # - An empty Turn Sign is treated as Normal until specified
        # - An empty Turn Sign will keep previous settings once specified
        # - Otherwise, empty fields will return random results
        # It is highly recommended to use 'format.py' to auto-fill missing fields

        if old:
            # for compatibility with new format
            holdDuration = 0
            if 'HoldDuration' in keyframeData:
                holdDuration = keyframeData['HoldDuration']
            # load
            keyframeObject.SetTimeInformation(keyframeData['MotionDuration'],
                                              holdDuration,
                                              timestamp)
            timestamp += holdDuration  # start of next
            for i in range(NUMBER_OF_BODY_PARTS):
                if i not in mapOld:  # skip unsupported parts
                    continue
                if isValidOld(keyframeData, supportedNames[i], i):
                    keyframeObject.SetBodyPart(
                        i,
                        keyframeData[supportedNames[i]]['Direction'],
                        keyframeData[supportedNames[i]]['Level'])
            output.Append(keyframeObject)
            continue

        keyframeObject.SetTime(keyframeData['MotionDuration'], timestamp)

        # Fill in the jsonData to the python object
        for i in range(NUMBER_OF_BODY_PARTS):
            # Check missing fields
            name = supportedNames[i]
            if name not in keyframeData:
                print(("[WARN] labanotationpy: {} frame {} "
                       + "does not have field '{}'")
                      .format(filename, k, name))
                continue

            # When field is not missing
            # Get direction symbols (some auto-fill depending on turn fill)
            direction, level, turn = getSigns(keyframeData, name)

            # Make sure there is at least either a hold or turn sign
            if direction is None and turn is None:
                print("[ERROR] labanotationpy: {} frame {} {} fields missing!"
                      .format(filename, k, name))
                continue
            keyframeObject.SetBodyPart(i, direction, level, turn)

        output.Append(keyframeObject)

    return output


# Function to load labanotation json files into a python class object.
#
#  @param  filename     Name of the file. e.g. file.json or /path/file.json
#  @param  printContent True to print content of json file.
#  @param  old          For compatibility with old version.
#  @return Labanotation class object.
def LoadJsonFile(filename, printContent=False, old=False):
    # Read the json file
    with open(filename) as fr:
        jsonData = json.load(fr)
    if printContent:
        print("{}".format(json.dumps(jsonData, indent=4)))  # Show file content
    return LoadJsonDictionary(jsonData, filename, old)


# Function to load labanotation json string into a python class object.
#
#  @param  content Json content in string.
#  @param  old     For compatibility with old version.
#  @return Labanotation class object.
def LoadJsonString(content, old=False):
    jsonData = json.loads(content)
    return LoadJsonDictionary(jsonData, 'string', old)
