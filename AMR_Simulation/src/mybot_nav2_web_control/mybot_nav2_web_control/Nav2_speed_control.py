#!/usr/bin/env python3
"""
nav2_speed_control.py
------------------------
Gia lap web goi service set_parameters cua Nav2 qua rosbridge
de chinh toc do toi da (max_vel_x) trong luc robot dang chay autonomous.

LUU Y: day la phan mo rong, chi dung khi ban muon chinh dong luc runtime.
Neu ban da set toc do co dinh san trong nav2_params.yaml truoc khi launch
thi KHONG can dung script nay.

Chay:
    pip3 install roslibpy
    python3 nav2_speed_control.py
"""

import time
import roslibpy

HOST = 'localhost'
PORT = 9090

# Ten service va ten tham so - PHAI khop voi ten thuc te trong nav2_params.yaml
SERVICE_NAME = '/controller_server/set_parameters'
PARAM_NAME = 'FollowPath.desired_linear_vel'   # doi theo plugin ban dung (DWB/RPP/MPPI...)

# Cac ma type theo chuan rcl_interfaces/msg/ParameterType
# PARAMETER_NOT_SET=0 BOOL=1 INTEGER=2 DOUBLE=3 STRING=4
# BOOL_ARRAY=5 INTEGER_ARRAY=6 DOUBLE_ARRAY=7 STRING_ARRAY=8 BYTE_ARRAY=9
PARAMETER_DOUBLE = 3


def set_nav2_speed(client: roslibpy.Ros, speed: float):
    """Goi service set_parameters de doi toc do toi da cua Nav2."""
    service = roslibpy.Service(client, SERVICE_NAME, 'rcl_interfaces/srv/SetParameters')

    request = roslibpy.ServiceRequest({
        'parameters': [
            {
                'name': PARAM_NAME,
                'value': {
                    'type': PARAMETER_DOUBLE,
                    'double_value': speed
                }
            }
        ]
    })

    print(f'Dang gui: {PARAM_NAME} = {speed} m/s ...')

    def on_result(result):
        results = result.get('results', [])
        if results and results[0].get('successful'):
            print(f'  -> Thanh cong: {PARAM_NAME} da doi thanh {speed} m/s')
        else:
            reason = results[0].get('reason', 'khong ro ly do') if results else 'khong co phan hoi'
            print(f'  -> That bai: {reason}')

    def on_error(err):
        print(f'  -> Loi goi service: {err}')

    service.call(request, callback=on_result, errback=on_error)


def main():
    client = roslibpy.Ros(host=HOST, port=PORT)
    client.run()
    print(f'Da ket noi rosbridge: ws://{HOST}:{PORT}')

    print("""
Nhap toc do toi da (m/s), vi du: 0.3
Nhap 'q' de thoat.
""")
    try:
        while True:
            raw = input('toc do > ').strip()
            if raw.lower() == 'q':
                break
            try:
                speed = float(raw)
            except ValueError:
                print('  gia tri khong hop le, nhap so thuc vi du 0.3')
                continue

            set_nav2_speed(client, speed)
            time.sleep(0.5)  # doi phan hoi service truoc khi nhap tiep

    finally:
        client.terminate()
        print('Da ngat ket noi.')


if __name__ == '__main__':
    main()