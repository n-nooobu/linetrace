import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Car:
    def __init__(self, car_pos_init, car_dir_init):
        self.led_pos_car = [[-20.5, 80],
                            [-5, 80],
                            [5, 80],
                            [20.5, 80]]

        self.wheel_pos_car = [[-40, 0],
                              [40, 0]]

        self.car_pos = car_pos_init
        self.car_dir = car_dir_init

        self.update_obj_pos()
        self.update_car_image()

        self.wheel_duty = [50, 25]
        self.speed = [0, 0]

        self.a = 5.680899591979651e-05
        self.b = 41.1064425770308

        self.interval = 50

    def update_obj_pos(self):
        self.led_pos = [[self.led_pos_car[led][0] * np.cos(self.car_dir) - self.led_pos_car[led][1] * np.sin(self.car_dir) + self.car_pos[0],
                         self.led_pos_car[led][0] * np.sin(self.car_dir) + self.led_pos_car[led][1] * np.cos(self.car_dir) + self.car_pos[1]] for led in range(4)]
        self.wheel_pos = [[self.wheel_pos_car[led][0] * np.cos(self.car_dir) - self.wheel_pos_car[led][1] * np.sin(self.car_dir) + self.car_pos[0],
                           self.wheel_pos_car[led][0] * np.sin(self.car_dir) + self.wheel_pos_car[led][1] * np.cos(self.car_dir) + self.car_pos[1]] for led in range(2)]

    def update_car_image(self):
        self.car_image = np.ones((840, 1200), dtype=int)

        for obj_pos, r in [[self.led_pos, 3], [self.wheel_pos, 10]]:
            for obj in obj_pos:
                for x in range(round(obj[0]) - r, round(obj[0]) + r):
                    for y in range(round(obj[1]) - r, round(obj[1]) + r):
                        if np.sqrt((x - obj[0]) ** 2 + (y - obj[1]) ** 2) <= r:
                            if -y + 420 < 0 or -y + 420 >= 840 or x + 600 < 0 or x + 600 >= 1200:
                                continue
                            self.car_image[-y + 420, x + 600] = 0

    def update_car_pos(self):
        for i in range(2):
            if self.wheel_duty[i] >= 50:
                self.speed[i] = np.sqrt((self.wheel_duty[i] - self.b) / self.a)
            elif self.wheel_duty[i] == 25:
                self.speed[i] = 1120 / 8.50

        if self.wheel_duty[0] == self.wheel_duty[1]:
            self.car_pos[0] += -self.speed[1] * self.interval / 1000 * np.sin(self.car_dir)
            self.car_pos[1] += self.speed[1] * self.interval / 1000 * np.cos(self.car_dir)
        else:
            if self.wheel_duty[0] > self.wheel_duty[1]:
                r = self.speed[1] * self.interval / 1000 * 80 / (self.speed[0] * self.interval / 1000 - self.speed[1] * self.interval / 1000)
                theta = -self.speed[1] * self.interval / 1000 / r
            else:
                r = self.speed[0] * self.interval / 1000 * 80 / (self.speed[1] * self.interval / 1000 - self.speed[0] * self.interval / 1000)
                theta = self.speed[0] * self.interval / 1000 / r

            car_pos_p = self.car_pos
            self.car_pos[0] = car_pos_p[0] * np.cos(theta) - car_pos_p[1] * np.sin(theta)
            self.car_pos[1] = car_pos_p[0] * np.sin(theta) + car_pos_p[1] * np.cos(theta)
            self.car_dir += theta


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
        field_image = np.ones((840, 1200), dtype=float) * 255

        for rec in self.rectangle:
            for x in range(rec[0], rec[1]):
                for y in range(rec[2], rec[3]):
                    field_image[-y + 420, x + 600] = 216

        for cir in self.circle:
            for x in range(cir[2], cir[3]):
                for y in range(cir[4], cir[5]):
                    if 210 <= np.sqrt((x - cir[0]) ** 2 + (y - cir[1]) ** 2) <= 230 and abs(x) >= 280 and abs(y) >= 90:
                        field_image[-y + 420, x + 600] = 216
        return field_image


if __name__ == '__main__':
    car_pos_init = [400, -310]
    car_dir_init = np.pi / 2

    car = Car(car_pos_init, car_dir_init)
    field = Field()

    fig = plt.figure()

    ims = []
    for t in range(51):  # 16ms * 100
        car.update_car_pos()
        car.update_obj_pos()
        car.update_car_image()
        image = car.car_image * field.field_image
        im = plt.imshow(image, vmin=0, vmax=255, cmap='gray', animated=True)
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
    #plt.imshow(image, vmin=0, vmax=255, cmap='gray', animated=True)

    """
    Duty 40 : 3.40
    Duty 35 : 4.30
    Duty 30 : 6.38
    Duty 25 : 8.50
    Duty 20 : 15.16
    Duty 15 : 44.55
    """
