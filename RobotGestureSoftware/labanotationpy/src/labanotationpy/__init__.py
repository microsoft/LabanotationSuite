#!/usr/bin/env python

# @package docstring
# Provide explanation of this code here.
# This code should be independent from ROS.
#
# More Details.
# Disclaimer and liscensing.
# Author information and email address to contact.
# Date of release.

# Use constant variables to avoid hard-coded strings in code
# Difficult to detect string typos, easy to detect name typos.
# Names originated from labanotation. Never rename 'Back' to 'Backward'.
PLACE = 'place'
FORWARD = 'forward'
LEFT_FORWARD = 'leftForward'
LEFT = 'left'
LEFT_BACK = 'leftBack'
BACK = 'back'
RIGHT_FORWARD = 'rightForward'
RIGHT = 'right'
RIGHT_BACK = 'rightBack'

HIGH = 'high'
MIDDLE = 'middle'
LOW = 'low'

TURN = 'turn'
TURN_NORMAL = 'turnNormal'
TURN_ONE_EIGHTH_LEFT = 'turn1/8left'
TURN_QUARTER_LEFT = 'turn1/4left'
TURN_THREE_EIGHTHS_LEFT = 'turn3/8left'
TURN_HALF_LEFT = 'turn1/2left'
TURN_ONE_EIGHTH_RIGHT = 'turn1/8right'
TURN_QUARTER_RIGHT = 'turn1/4right'
TURN_THREE_EIGHTHS_RIGHT = 'turn3/8right'
TURN_HALF_RIGHT = 'turn1/2right'

# Below defines the supported body parts
# Body parts are assigned as constant variables for the 'Decompose' method
# Whenever a new body part is added, should set the next ascending integer
HEAD = 0  # For compatibility, not recommended
FACE = 0
LEFT_ELBOW = 1
LEFT_WRIST = 2
LEFT_HAND = 3
LEFT_PALM = 4
RIGHT_ELBOW = 5
RIGHT_WRIST = 6
RIGHT_HAND = 7
RIGHT_PALM = 8
WHOLE_TORSO = 9

# Below value should match the number of defined body part constants
NUMBER_OF_BODY_PARTS = 10
NUMBER_OF_BODY_PARTS_OLD = 5  # For compatibility with old structure

# Map to map index from new structure to old structure
mapOld = {HEAD: 0,
          LEFT_ELBOW: 1, LEFT_WRIST: 2,
          RIGHT_ELBOW: 3, RIGHT_WRIST: 4}


# Class that holds information on each body part.
class LabanotationBodyPart:

    # The constructor.
    def __init__(self):

        # Set default values
        self._empty = True
        self._direction = None
        self._level = None
        self._turn = None
        # Create list of valid values
        self._validDirections = [PLACE,
                                 FORWARD,
                                 LEFT_FORWARD,
                                 LEFT,
                                 LEFT_BACK,
                                 BACK,
                                 RIGHT_FORWARD,
                                 RIGHT,
                                 RIGHT_BACK]
        self._validLevels = [HIGH,
                             MIDDLE,
                             LOW]
        self._validTurns = [TURN_NORMAL,
                            TURN_ONE_EIGHTH_LEFT,
                            TURN_QUARTER_LEFT,
                            TURN_THREE_EIGHTHS_LEFT,
                            TURN_HALF_LEFT,
                            TURN_ONE_EIGHTH_RIGHT,
                            TURN_QUARTER_RIGHT,
                            TURN_THREE_EIGHTHS_RIGHT,
                            TURN_HALF_RIGHT]

    # Set the bodypart information.
    #  @param self      The object pointer
    #  @param direction Direction (horizontal facing direction) of the part.
    #  @param level     Level (vertical facing direction) of the part.
    def SetDirectionSymbol(self, direction, level):
        # make sure input value is valid
        if type(direction) == float:  # labanotationpy allows -pi ~ pi
            if direction > 3.15 or direction < -3.15:
                print("[WARN] labanotationpy: {} is not a valid direction"
                      .format(direction))
                return False
        else:  # usual string labanotation
            if direction not in self._validDirections:
                print("[WARN] labanotationpy: {} is not a valid direction"
                      .format(direction))
                return False
        if type(level) == float:  # labanotationpy allows -pi/2 ~ pi/2
            if level > 1.58 or level < -1.58:
                print("[WARN] labanotationpy: {} is not a valid level"
                  .format(level))
                return False
        else:  # usual string labanotation
            if level not in self._validLevels:
                print("[WARN] labanotationpy: {} is not a valid level"
                      .format(level))
                return False
        self._empty = False
        self._direction = direction
        self._level = level
        return True

    # Set the turn information. Supported in new version only.
    #  @param self  The object pointer
    #  @param turn  Turn amount of the part.
    def SetTurn(self, turn):
        # make sure input value is valid
        if turn not in self._validTurns:
            print("[WARN] labanotationpy: {} is not a valid turn"
                  .format(turn))
            return False
        self._empty = False
        self._turn = turn
        return True

    # Check if value was filled.
    #  @param self The object pointer.
    #  @return True if filled.
    def isEmpty(self):
        return self._empty

    # Get direction.
    #  @param self The object pointer.
    #  @return Direction.
    def GetDirection(self):
        return self._direction

    # Get level.
    #  @param self The object pointer.
    #  @return Level.
    def GetLevel(self):
        return self._level

    # Get turn.
    #  @param self The object pointer.
    #  @return Turn.
    def GetTurn(self):
        return self._turn

    # Print for debugging.
    def info(self):
        print("    BodyPart printing ...")
        print("      direction: {}".format(self._direction))
        print("      level:     {}".format(self._level))
        print("      turn:      {}".format(self._turn))

    # @var _empty
    #  whether the value was filled

    # @var _direction
    #  horizontal facing direction of the body part

    # @var _level
    #  vertical facing direction of the body part

    # @var _turn
    #  turning amount of the body part (new version only)

    # @var _validDirections
    #  a list of valid values for direction

    # @var _validLevels
    #  a list of valid values for level

    # @var _validTurns
    #  a list of valid values for turn sign


# Class that holds information on each keyframe.
class LabanotationKeyframe:

    # The constructor.
    def __init__(self):
        self._timeFromStart = 0
        self._motionDuration = 0
        self._holdDuration = 0  # to be deprecated
        self._parts = [LabanotationBodyPart()
                       for n in range(NUMBER_OF_BODY_PARTS)]

    # Set the time information.
    #  @param self           The object pointer
    #  @param motionDuration Seconds to complete the motion.
    #  @param timeFromStart  Timestamp of the start of the motion (optional)
    def SetTime(self, motionDuration, timeFromStart=0):
        # Expected unit is seconds, warn if input might be milliseconds
        if motionDuration > 1000:
            print("[WARN] labanotationpy: detected a large motionDuration {}!"
                  .format(motionDuration), " mistaken sec as millisec?")
        self._motionDuration = motionDuration
        self._timeFromStart = timeFromStart

    # Set the time information. (to be deprecated)
    #  @param self           The object pointer
    #  @param motionDuration Seconds to complete the motion.
    #  @param holdDuration   Seconds to hold the motion after finishing.
    #  @param timeFromStart  Timestamp of the start of the motion (optional)
    def SetTimeInformation(self, motionDuration, holdDuration,
                           timeFromStart=0):
        print("[WARN] labanotationpy: SetTimeInformation will be deprecated.")
        # Expected unit is seconds, warn if input might be milliseconds
        if motionDuration > 1000:
            print("[WARN] labanotationpy: detected a large motionDuration {}!"
                  .format(motionDuration), " mistaken sec as millisec?")
        self._motionDuration = motionDuration
        self._holdDuration = holdDuration
        self._timeFromStart = timeFromStart

    # Set the bodypart information.
    #  @param self      The object pointer
    #  @param partId    The body part (defined constant variable name) to save.
    #  @param direction Horizontal facing direction of the body part.
    #  @param level     Vertical facing direction of the body part.
    #  @param turn      Rotation of the body part (for FACE, WholeTorso only).
    #  @return Whether set was successful.
    def SetBodyPart(self, partId, direction, level, turn=None):
        if partId >= len(self._parts) or partId < 0:
            print("[ERROR] labanotationpy: part with id {} as does not exist"
                  .format(partId))
            return False
        if direction is not None and level is not None:
            self._parts[partId].SetDirectionSymbol(direction, level)
        # New version supports turn sign for some body parts
        if turn is not None:
            # Whether a turn field is valid is decided explicitly
            self._parts[partId].SetTurn(turn)
        return True

    # Get the duration of the motion.
    #  @return Duration of the motion.
    def GetMotionDuration(self):
        return self._motionDuration

    # Get the holding duration of the motion.
    #  @return Holding duration of the motion.
    def GetHoldDuration(self):
        return self._holdDuration

    # Get the timestamp when the motion is supposed to finish.
    #  @return Timestamp of when the motion is supposed to finish.
    def GetTimeFromStart(self):
        return self._timeFromStart

    # Get the bodypart information.
    #  @param self   The object pointer
    #  @param partId The body part (defined constant variable name) to same.
    #  @return The bodypart.
    def GetBodyPart(self, partId):
        if partId >= len(self._parts) or partId < 0:
            print("[ERROR] labanotationpy: part with id {} as does not exist"
                  .format(partId))
            return None
        return self._parts[partId]

    # Decomposes body array to direction and level arrays.
    #  @param self The object pointer
    #  @param old  For compatibility with codes not supporting new style.
    #  @return Direction and level arrays.
    def Decompose(self, old=False):
        directions = [PLACE for n in range(NUMBER_OF_BODY_PARTS)]
        levels = [MIDDLE for n in range(NUMBER_OF_BODY_PARTS)]
        for i in range(NUMBER_OF_BODY_PARTS):
            directions[i] = self._parts[i].GetDirection()
            levels[i] = self._parts[i].GetLevel()
        # hard-coded compatibility for old labanotation structures
        if old:
            directions = [directions[FACE],
                          directions[LEFT_ELBOW],
                          directions[LEFT_WRIST],
                          directions[RIGHT_ELBOW],
                          directions[RIGHT_WRIST]]
            levels = [levels[FACE],
                      levels[LEFT_ELBOW],
                      levels[LEFT_WRIST],
                      levels[RIGHT_ELBOW],
                      levels[RIGHT_WRIST]]
        return directions, levels

    # Print for debugging.
    def info(self):
        print("  Keyframe printing ...")
        print("    timeFromStart:  {}".format(self._timeFromStart))
        print("    motionDuration: {}".format(self._motionDuration))
        for part in self._parts:
            part.info()

    # @var _motionDuration
    #  time duration of the motion

    # @var _holdDuration
    #  holding duration of the motion after finishing the motion

    # @var _timeFromStart
    #  timestamp of when to finish the motion (excluding _holdDuration)

    # @var _parts
    #  list of body part information in the current frame


# Class representing the score i.e. an array of labanotation keyframes.
class Labanotation:

    # The constructor.
    def __init__(self):
        self._keyframes = []

    # Get the number of keyframes in score.
    #  @param self The object pointer.
    #  @return Number of keyframes.
    def Length(self):
        return len(self._keyframes)

    # Append a new keyframe to the labanotation score.
    #  @param self     The object pointer.
    #  @param keyframe The keyframe to append.
    #  @return Whether append was successful.
    def Append(self, keyframe):
        if not isinstance(keyframe, LabanotationKeyframe):
            print("[ERROR] labanotationpy: wrong object type in Append")
            return False
        self._keyframes.append(keyframe)
        return True

    # Get the specified keyframe.
    #  @param self     The object pointer.
    #  @param frameNum The frame number to get.
    #  @return The specified keyframe.
    def GetKeyframe(self, frameNum):
        if frameNum >= len(self._keyframes) \
           or frameNum <= -len(self._keyframes):
            print("[WARN] labanotationpy: frame {} exceeds size {}".
                  format(frameNum, len(self._keyframes)))
            raise ValueError(frameNum)
        return self._keyframes[frameNum]

    # Print for debugging.
    def info(self):
        print("labanotationpy printing ...")
        for keyframe in self._keyframes:
            keyframe.info()

    # @var _keyframes
    #  an array of labanotation keyframes
