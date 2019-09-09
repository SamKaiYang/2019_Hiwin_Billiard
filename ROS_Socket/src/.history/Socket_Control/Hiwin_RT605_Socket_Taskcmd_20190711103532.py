import enum
##-------------enum actionflag
class Action_Type(enum.IntEnum):
    SetVel = 0
    Delay = 1
    PtoP = 2
    Line = 3
    Mode = 4
    Check = 5
    Suction = 6
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
    WID_1 = 2
    WID_2 = 3
    CATCH_OPEN = 5
    CATCH_LOOSEN = 6
##---------enum CTRL Mode
class Ctrl_Mode(enum.IntEnum):
    CTRL_POS = 0
    CTRL_EULER = 1
    CTRL_BOTH = 2
##----------enum RA
class RA(enum.IntEnum):
    REL = 0
    ABS = 1
##----------enum Suction
class Suction(enum.IntEnum):
    Close = 0
    Open = 1
