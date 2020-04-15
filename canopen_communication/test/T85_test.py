import sys
sys.path.append("./../modular")
import  pdb
from modular_T85 import T85

def main():
    eds_file = './Copley_T85.eds'
    motor = T85(5,eds_file)
    motor.start()

    ######### position mode #############
    motor.set_mode(1)
    print "position mode !!!/n/n/n"
    position = -0.3
    motor.sent_position(position) # 0.4rad
    while (motor.get_position() > position):
        print 'actual motor position: %f rad' % motor.get_position()
        print 'actula motor velocity: %f rad/s' % motor.get_velocity()
        print 'actual motor torque: %f mN.m' % motor.get_torque()
        print 'actual motor current: %f mA' % motor.get_current()

    position = 0
    motor.sent_position(position)
    while (motor.get_position() < position):
        print 'actual motor position: %f rad' % motor.get_position()
        print 'actula motor velocity: %f rad/s' % motor.get_velocity()
        print 'actual motor torque: %f mN.m' % motor.get_torque()
        print 'actual motor current: %f mA' % motor.get_current()
    ####################################


    ########## velocity mode ###########
    motor.set_mode(3)
    print "velocity mode !!!/n/n/n"
    #pdb.set_trace()
    motor.sent_velocity(0.02) # 0.02rad/s
    while motor.get_position() < 0.5:
        print 'actual motor position: %f rad' % motor.get_position()
        print 'actula motor velocity: %f rad/s' % motor.get_velocity()
        print 'actual motor torque: %f mN.m' % motor.get_torque()
        print 'actual motor current: %f mA' % motor.get_current()

    motor.sent_velocity(-0.02) # -0.02rad/s
    while motor.get_position() > 0:
        print 'actual motor position: %f rad' % motor.get_position()
        print 'actula motor velocity: %f rad/s' % motor.get_velocity()
        print 'actual motor torque: %f mN.m' % motor.get_torque()
        print 'actual motor current: %f mA' % motor.get_current()
ss    motor.sent_velocity(0) # 0 rad/s
    ####################################

    ######### torque mode ##############
    #pdb.set_trace()
    motor.set_mode(4)
    print 'torque_mode !!!/n/n/n'
    #pdb.set_trace()
    motor.sent_torque(600) # 600mN.m
    while motor.get_position() < 0.4:
        print 'actual motor position: %f rad' % motor.get_position()
        print 'actula motor velocity: %f rad/s' % motor.get_velocity()
        print 'actual motor torque: %f mN.m' % motor.get_torque()
        print 'actual motor current: %f mA' % motor.get_current()

    motor.sent_torque(-600) # -600 mN.m
    while motor.get_position() > 0:
        print 'actual motor position: %f rad' % motor.get_position()
        print 'actula motor velocity: %f rad/s' % motor.get_velocity()
        print 'actual motor torque: %f mN.m' % motor.get_torque()
        print 'actual motor current: %f mA' % motor.get_current()
    motor.sent_torque(0)
    #####################################

    motor.stop()

if __name__ == '__main__':
    main()
    sys.exit(0)
