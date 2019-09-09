#!/usr/bin/env python3
# license removed for brevity
import RobotArm5_socket_Taskcmd as Taskcmd
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

def SetPtoP(GripCmd,RA,CtrlMode,x,y,z,pitch,roll,yaw,setvel ):
    Cmd = "# " + str(int(Taskcmd.Action_Type.PtoP))+" "\
    +str(int(GripCmd))+" "\
    +str(int(RA))+" "\
    +str(int(CtrlMode))+ " "\
    +str(float(x))+ " "\
    +str(float(y))+ " "\
    +str(float(z))+ " "\
    +str(float(pitch))+ " "\
    +str(float(roll))+ " "\
    +str(float(yaw))+" "\
    +str(float(setvel))+" *"
    return Cmd

def SetLine(GripCmd,RA,CtrlMode,x,y,z,pitch,roll,yaw,setvel ):
    Cmd = "# " + str(int(Taskcmd.Action_Type.Line))+" "\
    +str(int(GripCmd))+" "\
    +str(int(RA))+" "\
    +str(int(CtrlMode))+ " "\
    +str(float(x))+ " "\
    +str(float(y))+ " "\
    +str(float(z))+ " "\
    +str(float(pitch))+ " "\
    +str(float(roll))+ " "\
    +str(float(yaw))+" "\
    +str(float(setvel))+" *"
    return Cmd
def SetVel(GripCmd,Vel):
    Cmd = "# " + str(Taskcmd.Action_Type.SetVel)+" "\
    +str(int(GripCmd))+" "\
    +str(int(Vel))+" *"

    return Cmd
def SetDelay(GripCmd,Delay):
    Cmd = "# " + str(int(Taskcmd.Action_Type.Delay))+" "\
    +str(int(GripCmd))+" "\
    +str(int(Delay))+ " *"

    return Cmd

def Is_busy(Cmd):
    ###test 0321
    feedback = Cmd
    return feedback
    ###test 0321