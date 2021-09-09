import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from tqdm import tqdm


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

        self.wheel_duty = [50, 10]

    def update_obj_pos(self):
        self.led_pos = [[self.led_pos_car[led][0] * np.cos(self.car_dir) - self.led_pos_car[led][1] * np.sin(self.car_dir) + self.car_pos[0],
                         self.led_pos_car[led][0] * np.sin(self.car_dir) + self.led_pos_car[led][1] * np.cos(self.car_dir) + self.car_pos[1]] for led in range(4)]
        self.wheel_pos = [[self.wheel_pos_car[led][0] * np.cos(self.car_dir) - self.wheel_pos_car[led][1] * np.sin(self.car_dir) + self.car_pos[0],
                           self.wheel_pos_car[led][0] * np.sin(self.car_dir) + self.wheel_pos_car[led][1] * np.cos(self.car_dir) + self.car_pos[1]] for led in range(2)]

    def update_car_image(self):
        self.car_image = np.ones((840, 940), dtype=int)

        for obj_pos, r in [[self.led_pos, 3], [self.wheel_pos, 10]]:
            for obj in obj_pos:
                for x in range(round(obj[0]) - r, round(obj[0]) + r):
                    for y in range(round(obj[1]) - r, round(obj[1]) + r):
                        if np.sqrt((x - obj[0]) ** 2 + (y - obj[1]) ** 2) <= r:
                            if -y + 420 < 0 or -y + 420 >= 840 or x + 470 < 0 or x + 470 >= 940:
                                continue
                            self.car_image[-y + 420, x + 470] = 0

    def update_car_pos(self):
        if self.wheel_duty[0] == self.wheel_duty[1]:
            self.car_pos[0] += -self.wheel_duty[0] * 0.158 * np.sin(self.car_dir)
            self.car_pos[1] += self.wheel_duty[0] * 0.158 * np.cos(self.car_dir)
        else:
            if self.wheel_duty[0] > self.wheel_duty[1]:
                r = self.wheel_duty[1] * 20 / (self.wheel_duty[0] - self.wheel_duty[1])
                theta = -self.wheel_duty[1] * 0.158 / r

                car_pos_p = self.car_pos
                self.car_pos[0] = car_pos_p[0] * np.cos(theta) - car_pos_p[1] * np.sin(theta)
                self.car_pos[1] = car_pos_p[0] * np.sin(theta) + car_pos_p[1] * np.cos(theta)
                self.car_dir += theta
            else:
                r = self.wheel_duty[0] * 20 / (self.wheel_duty[1] - self.wheel_duty[0])
                theta = self.wheel_duty[0] * 0.158 / r

                car_pos_p = self.car_pos
                self.car_pos[0] = car_pos_p[0] * np.cos(theta) - car_pos_p[1] * np.sin(theta)
                self.car_pos[1] = car_pos_p[0] * np.sin(theta) + car_pos_p[1] * np.cos(theta)
                self.car_dir += theta


class Field:
    def __init__(self):
        self.rectangle = [[-200, 200, 300, 320],
                          [-200, 200, -320, -300],
                          [-370, -350, -150, 150],
                          [350, 370, -150, 150]]
        self.circle = [[200, 150], [-200, 150], [-200, -150], [200, -150]]

        self.field_image = self.get_field_image()

    def get_field_image(self):
        field_image = np.ones((840, 940), dtype=float) * 255

        for rec in self.rectangle:
            for x in range(rec[0], rec[1]):
                for y in range(rec[2], rec[3]):
                    field_image[-y + 420, x + 470] = 216

        for cir in self.circle:
            for x in range(cir[0] - 170, cir[0] + 170):
                for y in range(cir[1] - 170, cir[1] + 170):
                    if 150 <= np.sqrt((x - cir[0]) ** 2 + (y - cir[1]) ** 2) <= 170 and abs(x) >= 200 and abs(y) >= 150:
                        field_image[-y + 420, x + 470] = 216
        return field_image


if __name__ == '__main__':
    car_pos_init = [300, -310]
    car_dir_init = np.pi / 2

    car = Car(car_pos_init, car_dir_init)
    field = Field()

    fig = plt.figure()

    ims = []
    for t in tqdm(range(10)):  # 50ms * 100
        car.update_car_pos()
        car.update_obj_pos()
        car.update_car_image()
        image = car.car_image * field.field_image
        im = plt.imshow(image, vmin=0, vmax=255, cmap='gray', animated=True)
        ims.append([im])

    anime = animation.ArtistAnimation(fig, ims, interval=1000)
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
