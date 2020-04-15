from dual_modular_test import Dual_modular_test
import time
import pdb

def main():
    eds_file = 'Copley.eds'
    dual_motor = Dual_modular_test(eds_file)
    #pdb.set_trace()
    dual_motor.start()

    print 'position mode'
    dual_motor.set_mode(1)
    position = [0.2,0.2]
    dual_motor.sent_joint_position(position)
    while dual_motor.get_position()[0] < 0.2 or dual_motor.get_position()[1] < 0.2:
        print 'actula motor_I1 position: %f rad' % dual_motor.get_position()[0]
        print 'actula motor_T2 position: %f rad' % dual_motor.get_position()[1]
        dual_motor.sent_joint_position(position)

    position = [0,0]
    dual_motor.sent_joint_position(position)
    while dual_motor.get_position()[0] > 0 or dual_motor.get_position()[1] > 0:
        pass
        dual_motor.sent_joint_position(position)
    time.sleep(5)

    print 'velocity mode'
    dual_motor.set_mode(3)
    velocity = [0.02,0.02]
    dual_motor.sent_joint_velocity(velocity)
    while(dual_motor.get_position()[0] < 0.5 and dual_motor.get_position()[1]<0.5):
        print 'actula motor_I1 current: %f mA' % dual_motor.get_current()[0]
        print 'actula motor_T2 current: %f mA' % dual_motor.get_current()[1]
        dual_motor.sent_joint_velocity(velocity)
        #print 'current{0} - {1}'.format(dual_motor.get_current()[0],dual_motor.get_current()[1])

    velocity = [-0.03,-0.03]
    dual_motor.sent_joint_velocity(velocity)
    while(dual_motor.get_position()[0] > 0 and dual_motor.get_position()[1] > 0):
        dual_motor.sent_joint_velocity(velocity)
    velocity = [0,0]
    dual_motor.sent_joint_velocity(velocity)
    time.sleep(5)
    
    print 'torque mode'
    dual_motor.set_mode(4)
    torque = [200,200]
    dual_motor.sent_joint_torque(torque)
    while(dual_motor.get_position()[0]<1 and dual_motor.get_position()[1]<1):
        pass
    torque = [-200,-200]
    dual_motor.sent_joint_torque(torque)
    while(dual_motor.get_position()[0] > 0 and dual_motor.get_position()[1] > 0):
        pass
    torque = [0,0]
    dual_motor.sent_joint_torque(torque)
    time.sleep(5)

    dual_motor.stop()

main()