import enum
##-------------enum actionflag
class Action_Type(enum.IntEnum):
    SetVel = 0
    Delay = 1
    PtoP = 2
    Line = 3
##------------enum Arm feedback
class Arm_feedback_Type(enum.IntEnum):
    Idle = 0
    Isbusy = 1
    Error = 2
    shutdown = 6
##--------enum grip action
class Grip_Cmd(enum.IntEnum):
    STOP = 0
    CATCH =1
    CATCH01 = 2
    CATCH02 = 3
    CATCH_LOOSEN = 6
    CATCH_BACKHOME = 7
##---------enum CTRL Mode
class Ctrl_Mode(enum.IntEnum):
    CTRL_POS = 0
    CTRL_EULER = 1
    CTRL_BOTH = 2
##----------enum RA
class RA(enum.IntEnum):
    REL = 0
    ABS = 1
