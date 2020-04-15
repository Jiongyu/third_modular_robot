import sys
sys.path.append("./../robot")
import  pdb
from climbot5d import Climbot5d
import time

def main():
    eds_file = './Copley.eds'
    climbot = Climbot5d()
    climbot.start()
    climbot.set_mode(1)

    climbot.sent_G0_torque(50)
    while 1:
        if(climbot.get_velocity[0]<1e-3):
            time.sleep(1)
            climbot.sent_G0_torque(0)
        else:
            print 'actual gripper torque:{0} mN.m.'.format(climbot.get_torque[0])

    climbot.stop()

if __name__ == '__main__':
    main()


