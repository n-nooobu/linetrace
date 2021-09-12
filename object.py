import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Car:
    def __init__(self, car_state_init, duty_init, field, interval, Kp, Ki, Kd):
        self.led_pos_car = [[80, 20.5],
                            [80, 5],
                            [80, -5],
                            [80, -20.5]]

        self.wheel_pos_car = [[0, 40],
                              [0, -40]]
        self.d = 40

        self.car_state = car_state_init
        self.field = field
        self.sensor = [0, 0, 0, 0]
        self.sensor_weight = [2, 1, -1, -2]

        self.duty = duty_init
        self.a = 5.680899591979651e-05
        self.b = 41.1064425770308
        self.dt = interval / 1000
        self.v = [0, 0]
        self.cal_v()
        self.dL = [v * self.dt for v in self.v]
        self.turn_state = [0, 0, 0]

        self.diff = [0, 0]
        self.intergral = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.update_obj_pos()
        self.update_car_image()


    def update_sensor(self):
        self.sensor_val = 0
        cnt = 0
        for i, led in enumerate(self.led_pos):
            x = round(led[0])
            y = round(led[1])
            if 0 <= -y + 420 < 840 and 0 <= x + 600 < 1200:
                if self.field.field_image[-y + 420, x + 600, 0] != 255:
                    self.sensor[i] = 1
                    self.sensor_val += self.sensor_weight[i]
                    cnt += 1
                else:
                    self.sensor[i] = 0
            else:
                self.sensor[i] = 0

        if cnt:
            self.sensor_val /= cnt

    def update_duty(self):
        """16ms * 285 = 4.56
        if self.sensor == [0, 0, 0, 1]:
            self.duty = [60, 30]
        elif self.sensor == [0, 0, 1, 1]:
            self.duty = [60, 40]
        elif self.sensor == [1, 0, 0, 0]:
            self.duty = [30, 60]
        elif self.sensor == [0, 1, 1, 0]:
            self.duty = [75, 75]"""

        """16ms * 215 = 3.44
        if self.sensor == [0, 0, 0, 1]:
            self.duty = [90, 30]
        elif self.sensor == [0, 0, 1, 1]:
            self.duty = [90, 40]
        elif self.sensor == [1, 0, 0, 0]:
            self.duty = [30, 90]
        elif self.sensor == [0, 1, 1, 0]:
            self.duty = [90, 90]"""

        """16ms * 195 = 3.12
        if self.sensor == [0, 0, 0, 1]:
            self.duty = [100, 30]
        elif self.sensor == [0, 0, 1, 1]:
            self.duty = [100, 40]
        elif self.sensor == [1, 0, 0, 0]:
            self.duty = [30, 100]
        elif self.sensor == [0, 1, 1, 0]:
            self.duty = [100, 100]"""

        """if self.sensor == [0, 0, 0, 1]:
            self.duty = [100, 65]
        elif self.sensor == [0, 0, 1, 1]:
            self.duty = [100, 65]
        elif self.sensor == [0, 1, 1, 0]:
            self.duty = [100, 100]"""

        """if self.sensor == [0, 1, 0, 0]:
            self.duty = [100, 95]
        elif self.sensor == [0, 1, 1, 0]:
            self.duty = [100, 80]
        elif self.sensor == [0, 0, 1, 0]:
            self.duty = [100, 65]
        elif self.sensor == [0, 0, 1, 1]:
            self.duty = [100, 65]
        elif self.sensor == [0, 0, 0, 1]:
            self.duty = [100, 50]
        elif self.sensor == [1, 0, 0, 0]:
            self.duty = [100, 100]"""



        """if self.sensor == [0, 0, 0, 1]:
            self.duty = [50, 35]
        elif self.sensor == [0, 0, 1, 1]:
            self.duty = [50, 35]
        elif self.sensor == [1, 0, 0, 0]:
            self.duty = [35, 50]
        elif self.sensor == [1, 1, 0, 0]:
            self.duty = [35, 50]
        elif self.sensor == [0, 1, 1, 0]:
            self.duty = [50, 50]"""

        """if self.sensor == [0, 0, 0, 1]:
            self.duty = [55, 25]
        elif self.sensor == [0, 0, 1, 1]:
            self.duty = [55, 35]
        elif self.sensor == [1, 0, 0, 0]:
            self.duty = [30, 50]
        elif self.sensor == [0, 1, 1, 0]:
            self.duty = [65, 65]"""

        self.pid()

    def pid(self):
        self.diff[0] = self.diff[1]
        self.diff[1] = self.sensor_val - 0
        self.intergral += (self.diff[1] + self.diff[0]) / 2 * self.dt

        p = self.Kp * self.diff[1]
        i = self.Ki * self.intergral
        d = self.Kd * (self.diff[1] - self.diff[0]) / self.dt

        dtheta = min(max(p + i + d, -0.75), 0.75)
        print(dtheta)
        if dtheta != 0:
            lo = self.turn_state[0] / dtheta
            dL = [(lo - self.d) * dtheta, (lo + self.d) * dtheta]
        else:
            dL = [self.turn_state[0], self.turn_state[0]]
        v = [dL[0] / self.dt, dL[1] / self.dt]
        self.duty = [self.a * v[0] ** 2 + self.b, self.a * v[1] ** 2 + self.b]

    def cal_v(self):
        # vを計算
        for i in range(2):
            if self.duty[i] >= 50:
                self.v[i] = np.sqrt((self.duty[i] - self.b) / self.a)
            elif self.duty[i] == 40:
                self.v[i] = 1120 / 3.40
            elif self.duty[i] == 35:
                self.v[i] = 1120 / 4.30
            elif self.duty[i] == 30:
                self.v[i] = 1120 / 6.38
            elif self.duty[i] == 25:
                self.v[i] = 1120 / 8.50
            elif self.duty[i] == 20:
                self.v[i] = 1120 / 15.16
            elif self.duty[i] == 15:
                self.v[i] = 1120 / 44.55

    def update_car_pos(self):
        self.cal_v()

        # dLを計算
        self.dL = [v * self.dt for v in self.v]

        # turn_stateを計算
        self.turn_state[0] = (self.dL[1] + self.dL[0]) / 2
        self.turn_state[1] = (self.dL[1] - self.dL[0]) / (2 * self.d)
        if self.turn_state[1] != 0:
            self.turn_state[2] = self.turn_state[0] / self.turn_state[1]

        # car_stateを計算
        if self.turn_state[1] == 0:
            self.car_state[0] += self.turn_state[0] * np.cos(self.car_state[2])
            self.car_state[1] += self.turn_state[0] * np.sin(self.car_state[2])
        else:
            self.car_state[0] += 2 * self.turn_state[2] * np.sin(self.turn_state[1] / 2) * np.cos(self.car_state[2] + self.turn_state[1] / 2)
            self.car_state[1] += 2 * self.turn_state[2] * np.sin(self.turn_state[1] / 2) * np.sin(self.car_state[2] + self.turn_state[1] / 2)
            self.car_state[2] += self.turn_state[1]

    def update_obj_pos(self):
        self.led_pos = [[self.led_pos_car[led][0] * np.cos(self.car_state[2]) - self.led_pos_car[led][1] * np.sin(self.car_state[2]) + self.car_state[0],
                         self.led_pos_car[led][0] * np.sin(self.car_state[2]) + self.led_pos_car[led][1] * np.cos(self.car_state[2]) + self.car_state[1]] for led in range(4)]
        self.wheel_pos = [[self.wheel_pos_car[led][0] * np.cos(self.car_state[2]) - self.wheel_pos_car[led][1] * np.sin(self.car_state[2]) + self.car_state[0],
                           self.wheel_pos_car[led][0] * np.sin(self.car_state[2]) + self.wheel_pos_car[led][1] * np.cos(self.car_state[2]) + self.car_state[1]] for led in range(2)]

    def update_car_image(self):
        self.car_image = np.ones((840, 1200, 3), dtype=int)

        for i, led in enumerate(self.led_pos):
            if self.sensor[i]:
                color = [1, 0, 0]
                r = 6
            else:
                color = [0, 0, 0]
                r = 4
            for x in range(round(led[0]) - r, round(led[0]) + r):
                for y in range(round(led[1]) - r, round(led[1]) + r):
                    if np.sqrt((x - led[0]) ** 2 + (y - led[1]) ** 2) <= r:
                        if -y + 420 < 0 or -y + 420 >= 840 or x + 600 < 0 or x + 600 >= 1200:
                            continue
                        self.car_image[-y + 420, x + 600, :] = color

        for wheel in self.wheel_pos:
            r = 10
            for x in range(round(wheel[0]) - r, round(wheel[0]) + r):
                for y in range(round(wheel[1]) - r, round(wheel[1]) + r):
                    if np.sqrt((x - wheel[0]) ** 2 + (y - wheel[1]) ** 2) <= r:
                        if -y + 420 < 0 or -y + 420 >= 840 or x + 600 < 0 or x + 600 >= 1200:
                            continue
                        self.car_image[-y + 420, x + 600, :] = [0, 0, 0]


class Field:
    def __init__(self):
        self.rectangle = [[-280, 280, 300, 320],
                          [-280, 280, -320, -300],
                          [-510, -490, -90, 90],
                          [490, 510, -90, 90]]
        self.circle = [[280, 90, 280, 510, 90, 320],
                       [-280, 90, -510, -280, 90, 320],
                       [-280, -90, -510, -280, -320, -90],
                       [280, -90, 280, 510, -320, -90]]

        self.field_image = self.get_field_image()

    def get_field_image(self):
        field_image = np.ones((840, 1200, 3), dtype=int) * 255

        for rec in self.rectangle:
            for x in range(rec[0], rec[1]):
                for y in range(rec[2], rec[3]):
                    field_image[-y + 420, x + 600, :] = [216, 216, 216]

        for cir in self.circle:
            for x in range(cir[2], cir[3]):
                for y in range(cir[4], cir[5]):
                    if 210 <= np.sqrt((x - cir[0]) ** 2 + (y - cir[1]) ** 2) <= 230 and abs(x) >= 280 and abs(y) >= 90:
                        field_image[-y + 420, x + 600, :] = [216, 216, 216]
        return field_image


if __name__ == '__main__':
    car_state_init = [80, -310, np.pi]

    """5s
    duty_init = [50, 50]
    interval = 100
    Kp = 0.2
    Ki = 0.1
    Kd = 0.0001"""

    """4.5s
    duty_init = [65, 65]
    interval = 100
    Kp = 0.2
    Ki = 0.1
    Kd = 0.0001"""

    """#3.8s
    duty_init = [75, 75]
    interval = 100
    Kp = 0.2
    Ki = 0.15
    Kd = 0.0001"""

    #2.85s
    duty_init = [100, 100]
    interval = 50
    Kp = 0.2
    Ki = 0.1
    Kd = 0.0001

    field = Field()
    car = Car(car_state_init, duty_init, field, interval, Kp, Ki, Kd)

    fig = plt.figure()

    ims = []
    for t in range(57):  # 16ms * 100 = 1.6s   50ms * 100 = 5s   100ms * 100 = 10s
        car.update_sensor()
        car.update_duty()
        car.update_car_pos()
        car.update_obj_pos()
        car.update_car_image()
        image = car.car_image * car.field.field_image
        if t % 1 == 0:
            im = plt.imshow(image, vmin=0, vmax=255, animated=True)
            ims.append([im])

    anime = animation.ArtistAnimation(fig, ims, interval=50)
    plt.show()

    """def plot(data):
        plt.cla()
        car.update_car_pos()
        car.update_obj_pos()
        car.update_car_image()
        image = car.car_image * field.field_image
        im = plt.imshow(image, vmin=0, vmax=255, cmap='gray', animated=True)
    ani = animation.FuncAnimation(fig, plot, interval=50)
    plt.show()"""

    #image = car.car_image * field.field_image
    #plt.imshow(field.field_image, vmin=0, vmax=255, animated=True)

    """
    Duty 40 : 3.40
    Duty 35 : 4.30
    Duty 30 : 6.38
    Duty 25 : 8.50
    Duty 20 : 15.16
    Duty 15 : 44.55
    """
