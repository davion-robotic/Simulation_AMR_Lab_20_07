#!/usr/bin/env python3


import sys
import time
import roslibpy

HOST = 'localhost'   # doi thanh IP robot/Jetson khi test that
PORT = 9090

LINEAR_STEP = 0.1     # m/s moi lan tang/giam
ANGULAR_STEP = 0.2    # rad/s moi lan tang/giam
MAX_LINEAR = 1.0
MAX_ANGULAR = 1.5


class FakeWebController:
    def __init__(self, host=HOST, port=PORT):
        self.client = roslibpy.Ros(host=host, port=port)
        self.client.run()
        print(f'Da ket noi rosbridge: ws://{host}:{port}')

        self.cmd_vel_topic = roslibpy.Topic(
            self.client, '/cmd_vel', 'geometry_msgs/msg/Twist'
        )

        self.linear_x = 0.0
        self.angular_z = 0.0

    def publish(self):
        msg = {
            'linear': {'x': self.linear_x, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': self.angular_z},
        }
        self.cmd_vel_topic.publish(roslibpy.Message(msg))
        print(f'  -> gui: linear.x={self.linear_x:.2f}  angular.z={self.angular_z:.2f}')

    def stop(self):
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.publish()

    def close(self):
        self.stop()
        self.cmd_vel_topic.unadvertise()
        self.client.terminate()


# ---------------------------------------------------------------------
def run_interactive(ctrl: FakeWebController):
    print("""
Che do dieu khien nhu joystick:
  w = tang toc tien       s = tang toc lui
  a = quay trai            d = quay phai
  x = dung khan cap         q = thoat
""")
    while True:
        key = input('Nhap lenh > ').strip().lower()

        if key == 'w':
            ctrl.linear_x = min(ctrl.linear_x + LINEAR_STEP, MAX_LINEAR)
        elif key == 's':
            ctrl.linear_x = max(ctrl.linear_x - LINEAR_STEP, -MAX_LINEAR)
        elif key == 'a':
            ctrl.angular_z = min(ctrl.angular_z + ANGULAR_STEP, MAX_ANGULAR)
        elif key == 'd':
            ctrl.angular_z = max(ctrl.angular_z - ANGULAR_STEP, -MAX_ANGULAR)
        elif key == 'x':
            ctrl.stop()
            continue
        elif key == 'q':
            break
        else:
            print('  lenh khong hop le, dung w/s/a/d/x/q')
            continue

        ctrl.publish()

    ctrl.close()
    print('Da dung va ngat ket noi.')


# ---------------------------------------------------------------------
def run_auto_test(ctrl: FakeWebController):
    print('Chay kich ban tu dong: tien -> quay -> lui -> dung (moi buoc 3s)\n')

    steps = [
        ('Tien thang', 0.3, 0.0),
        ('Quay phai tai cho', 0.0, -0.5),
        ('Lui', -0.2, 0.0),
        ('Dung', 0.0, 0.0),
    ]

    try:
        for name, lin, ang in steps:
            print(f'>> {name}')
            ctrl.linear_x = lin
            ctrl.angular_z = ang
            ctrl.publish()
            time.sleep(3)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.close()
        print('Da dung va ngat ket noi.')


# ---------------------------------------------------------------------
def main():
    ctrl = FakeWebController()

    print('Chon che do:')
    print('  1 = Nhap tay (interactive)')
    print('  2 = Auto test (tu dong chay kich ban)')
    mode = input('> ').strip()

    if mode == '1':
        run_interactive(ctrl)
    elif mode == '2':
        run_auto_test(ctrl)
    else:
        print('Lua chon khong hop le.')
        ctrl.close()
        sys.exit(1)


if __name__ == '__main__':
    main()