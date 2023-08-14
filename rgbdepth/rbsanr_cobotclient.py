import time
from pymycobot import MyCobot
from pymycobot.genre import Angle
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# 소켓을 사용하기 위해서는 socket을 import해야 한다.
import socket

mc = MyCobot('/dev/ttyAMA0',1000000)

usleep = lambda x: time.sleep(x/1000000.0)
servo_move = lambda x: usleep(int(2000-(x-4)*43))#41.667
zero_point = [100, 0, 175, 180, 0, -90]  #184.1, -36.7, 176, 180, 0, -180
place_point = [50, -200, 190, 180, 0, 0]  #184.1, -36.7, 176, 180, 0, -180
servo_grip = 3
pickup_offset = 30
mc.gpio_init()
mc.gpio_output(servo_grip, 0)

class MinimalSubscriber(Node):
    def __init__(self):
        pass

    def is_moving():
        return mc.is_moving
    
    def wait_move(self):
        time.sleep(0.5)
        while mc.is_moving() == 1:
            mc.set_color(255, 0, 0)
        time.sleep(0.1)
        while mc.is_moving() == 1:
            mc.set_color(255, 0, 0)
        time.sleep(0.1)
        while mc.is_moving() == 1:
            mc.set_color(255, 0, 0)
        mc.set_color(0, 255, 0)
        time.sleep(0.5)
    
    def get_axles(self):
        while True:
            axle = mc.get_coords()
            if len(axle) == 6:
                return axle

    def griper_move(self, langth):
        for i in range(0, 10):
            if langth > 45:
                mc.gpio_output(3, 1)
                servo_move(45)
                mc.gpio_output(3, 0)
            elif langth < 1:
                mc.gpio_output(3, 1)
                servo_move(1)
                mc.gpio_output(3, 0)
            else:
                mc.gpio_output(3, 1)
                servo_move(langth)
                mc.gpio_output(3, 0)
            time.sleep(0.1)

    def Just_move(self, data):
            data = list(map(float, data))
            for i in range(0, 6):
                data[i] = data[i] + zero_point[i]
            mc.send_coords(data[:6], int(data[7]), 0)
            self.wait_move()

            buff = data.copy()
            buff_update = data.copy()

            mc.gpio_output(3, 1)    
            servo_move(data[6])
            mc.gpio_output(3, 0)

    def Pickup_move(self, data):
        data = list(map(float, data))
        for i in range(0, 6):
            data[i] = data[i]# + zero_point[i]
        buff = data.copy()
        buff[2] = buff[2] + pickup_offset

        mc.send_coords(buff[:6], int(data[7]), 0) # X
        print(buff[:6])
        self.wait_move()

        self.griper_move(40)# O

        mc.send_coords(data[:6], int(data[7]), 1) # O
        print(data[:6])
        self.wait_move()

        self.griper_move(data[6])
        
        mc.send_coords(buff[:6], int(data[7]), 1) # X
        print(buff[:6])
        self.wait_move()

        buff = place_point.copy()
        buff[2] = buff[2] + pickup_offset

        mc.send_coords(buff[:6], int(data[7]), 0)
        print(buff[:6])
        self.wait_move()

        mc.send_coords(place_point[:6], int(data[7]), 1)
        print(place_point[:6])
        self.wait_move()

        self.griper_move(40)

        mc.send_coords(buff[:6], int(data[7]), 1)
        print(buff[:6])
        self.wait_move()

        mc.send_coords(zero_point, 10)
        self.wait_move()

    def listener_callback(self, msg):
        data = msg
        #print(data)
        data = data.replace(',', ' ')
        #print(data)
        list_axle = data.split()
        #print(list_axle)
        if list_axle[0] == "is_moving":
            self.is_moving()
        elif len(list_axle) == 8:
            self.Pickup_move(list_axle)
            #self.Just_move(list_axle)
        else:
            print("Wrong Protocol")

def main(args = None):
    HOST = '192.168.0.14'  
    # port는 위 서버에서 설정한 9999로 접속을 한다.
    PORT = 9999       
    # 소켓을 만든다.
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # connect함수로 접속을 한다. 
    client_socket.connect((HOST, PORT))

    rclpy.init(args = args)
    time.sleep(1)
    mc.send_angles([0, 0, 0, 0, 0, 0], 10)
    sub = MinimalSubscriber()
   
    
    mc.send_coords(zero_point, 10)
    
    

    while True:
        # server로 부터 전송받`을 데이터 길이를 받는다.
        data = client_socket.recv(4)
        # 데이터 길이는 리틀 엔디언 형식으로 int를 변환한다.
        length = int.from_bytes(data, "little")
        # 데이터 길이를 받는다.
        data = client_socket.recv(length)
        # 데이터를 수신한다.
        msg = data.decode()
        # 데이터를 출력한다.
        print('Received from : ', msg)

        sub.listener_callback(msg)

        # 메시지를 바이너리(byte)형식으로 변환한다.
        data = msg.encode()
        # 메시지 길이를 구한다.
        length = len(data)
        # server로 리틀 엔디언 형식으로 데이터 길이를 전송한다.
        client_socket.sendall(length.to_bytes(4, byteorder="little"))
        # 데이터를 전송한다.
        client_socket.sendall(data)

if __name__ == "__main__":
    main()
