from Ax12 import Ax12
import time

# motor objects
all_motors = Ax12(254)
ankleL = Ax12(1)
kneeL = Ax12(2)
hipL = Ax12(3)
hipR = Ax12(4)
kneeR = Ax12(5)
ankleR = Ax12(6)

default_speed = 40
max_speed = 1023
half_speed = 512

#angle range for servos = [-150,150]
#required range = [-90, 90]

def map_val(x_in, x_min, x_max, y_min, y_max):
    """Linearly maps x to y; returns corresponding y value"""
    m = ((y_max - y_min) / (x_max - x_min))
    y_out = m * (x_in - x_min) + y_min
    return y_out


def deg_to_pos(obj, deg):
    pos = map_val(deg, -150, 150, obj.MIN_POS_VAL, obj.MAX_POS_VAL)
    return pos


def check_limit(motor_object, deg):
    if deg > 90:
        return 90
    elif deg < -90:
        return -90
    else:
        return deg


def update_servo_pos(target1, target2, target3, target4, target5, target6):
    #if (leg == 'l'):
    # print("Sensed Motor positions: ", get_angle(hipL), get_angle(kneeL), get_angle(ankleL))
    set_angle(hipL, target1)
    set_angle(kneeL, -target2)
    set_angle(ankleL, target3)
    
   # elif (leg == 'r'):
     #   print("Sensed Motor positions: ", get_angle(hipR), get_angle(kneeR), get_angle(ankleR))
    set_angle(hipR, -target4)
    set_angle(kneeR, target5)
    set_angle(ankleR, -target6)


def set_angle(self, input_deg):
    """Sets motor to specified input angle."""
    deg = check_limit(self, input_deg)
    dxl_goal_position = int(deg_to_pos(self, input_deg))
    self.set_position(dxl_goal_position)


def get_angle(self):
    """Returns present angle."""
    dxl_present_position = self.get_position()
    dxl_angle = int(map_val(dxl_present_position, self.MIN_POS_VAL, self.MAX_POS_VAL, -150, 150))
    return dxl_angle


def rest_position():
    update_servo_pos(0, 0, 0, 'l')
    update_servo_pos(0, 0, 0, 'r')
  #  time.sleep(5)


def turn_on():
    Ax12.open_port()
    Ax12.set_baudrate()
    print('Connection Successful')
    all_motors.set_moving_speed(default_speed)


def turn_off():
    all_motors.disable_torque()
    Ax12.close_port()

