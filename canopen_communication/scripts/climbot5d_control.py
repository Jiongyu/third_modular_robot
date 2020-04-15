from climbot5d import Climbot5d

def main(eds_file):

    climbot = Climbot5d(eds_file)
    climbot.start()
    climbot.set_mode(1)


    position = []
    climbot.sent_joint_position()

    climbot.sent_G0_torque(...)
    climbot.sent_G6_torque(...)


    climbot.stop()
