#!/usr/bin/env python

import argparse
import sys

import rospy
from sawyer_metc_gripper.msg import GripperControl

import intera_interface
from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Lights,
    Cuff,
    RobotParams,
    CHECK_VERSION,
)

class GripperConnect(object):
    def __init__(self, arm, lights=True):
        self._arm = arm
        self._cuff = Cuff(limb=arm)

        # connect callback fns to signals
        self._lights = None
        if lights:
            self._lights = Lights()
            self._cuff.register_callback(self._light_action,
                                         '{0}_cuff'.format(arm))
        try:
            self._gripper = get_current_gripper_interface()
            self._is_clicksmart = isinstance(self._gripper, SimpleClickSmartGripper)

            if self._is_clicksmart:
                if self._gripper.needs_init():
                    self._gripper.initialize()
            else:
                if not (self._gripper.is_calibrated() or
                        self._gripper.calibrate() == True):
                    raise
            self._cuff.register_callback(self._close_action,
                                         '{0}_button_upper'.format(arm))
            self._cuff.register_callback(self._open_action,
                                         '{0}_button_lower'.format(arm))

            rospy.loginfo("{0} Cuff Control initialized...".format(
                          self._gripper.name))
        except:
            self._gripper = None
            self._is_clicksmart = False
            msg = ("{0} Gripper is not connected to the robot."
                   " Running cuff-light connection only.").format(arm.capitalize())
            rospy.logwarn(msg)

    def _open_action(self, value):
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper open triggered")
            if self._is_clicksmart:
                self._gripper.set_ee_signal_value('grip', False)
            else:
                self._gripper.open()
            if self._lights:
                self._set_lights('red', False)
                self._set_lights('green', True)

    def _close_action(self, value):
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper close triggered")
            if self._is_clicksmart:
                self._gripper.set_ee_signal_value('grip', True)
            else:
                self._gripper.close()
            if self._lights:
                self._set_lights('green', False)
                self._set_lights('red', True)

    def _light_action(self, value):
        if value:
            rospy.logdebug("cuff grasp triggered")
        else:
            rospy.logdebug("cuff release triggered")
        if self._lights:
            self._set_lights('red', False)
            self._set_lights('green', False)
            self._set_lights('blue', value)

    def _set_lights(self, color, value):
        self._lights.set_light_state('head_{0}_light'.format(color), on=bool(value))
        self._lights.set_light_state('{0}_hand_{1}_light'.format(self._arm, color),
                                                                 on=bool(value))
    
    def _handle_message(self, msg):
        flag_close = msg.status
        rospy.loginfo(rospy.get_caller_id() + " got message : %i", flag_close)
        if flag_close:
            print('action - close gripper')
            self._close_action(1)
        else:
            print('action - open gripper')
            self._open_action(1)            

def clean_shutdown():
    print("\nExiting sawyer_metc_gripper.")

def main():
    rp = RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    if len(valid_limbs) > 1:
        valid_limbs.append("all_limbs")

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument('-g', '--gripper', dest='gripper', default=valid_limbs[0],
                        choices=[valid_limbs],
                        help='gripper limb to control (default: both)')
    parser.add_argument('-n', '--no-lights', dest='lights',
                        action='store_false',
                        help='do not trigger lights on cuff grasp')
    parser.add_argument('-v', '--verbose', dest='verbosity',
                        action='store_const', const=rospy.DEBUG,
                        default=rospy.INFO,
                        help='print debug statements')
    parser.add_argument("-l", "--limb", dest="limb", default=valid_limbs[0],
                        choices=valid_limbs,
                        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('sawyer_metc_gripper'.format(args.gripper), log_level=args.verbosity)

    arms = (args.gripper,) if args.gripper != 'all_limbs' else valid_limbs[:-1]
    grip_ctrls = [GripperConnect(arm, args.lights) for arm in arms]

    rs = intera_interface.RobotEnable(CHECK_VERSION)
    rs.enable()

    rospy.on_shutdown(clean_shutdown)

    rospy.Subscriber("chatter", GripperControl, grip_ctrls[0]._handle_message)
  
    rospy.spin()
    return 0

if __name__ == '__main__':
    sys.exit(main())
