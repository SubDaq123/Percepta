from gpiozero import Servo, DistanceSensor, PWMOutputDevice, OutputDevice, Buzzer
from signal import pause
import time
import threading

PAN_SERVO_PIN = 18
TILT_SERVO_PIN = 17

SENSOR_RIGHT_TRIG = 23
SENSOR_RIGHT_ECHO = 24
SENSOR_LEFT_TRIG = 14
SENSOR_LEFT_ECHO = 21

MOTOR_LEFT_ENABLE = 25
MOTOR_RIGHT_ENABLE = 8
MOTOR_LEFT_SPEED = 12
MOTOR_RIGHT_SPEED = 19

BUZZER_LEFT_PIN = 16
BUZZER_RIGHT_PIN = 20

HAPTIC_LEFT_PIN = 5
HAPTIC_RIGHT_PIN = 6

DETECTION_DISTANCE = 30
MIN_DISTANCE = 5
MAX_DISTANCE = 30

TILT_MEAN_VALUE = 0.85
TILT_DOWN_VALUE = 1.0
TILT_STEP = 0.01
TILT_STEP_DELAY = 0.08

PAN_SWEEP_STEP = 2
PAN_SWEEP_DELAY = 0.12

REST_SETTLE_DELAY = 0.2
DETECTION_REQUIRED = 2

pan_servo = Servo(PAN_SERVO_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
tilt_servo = Servo(TILT_SERVO_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
tilt_servo.value = None

sensor_right = DistanceSensor(echo=SENSOR_RIGHT_ECHO, trigger=SENSOR_RIGHT_TRIG, max_distance=4)
sensor_left = DistanceSensor(echo=SENSOR_LEFT_ECHO, trigger=SENSOR_LEFT_TRIG, max_distance=4)

motor_left_enable = OutputDevice(MOTOR_LEFT_ENABLE)
motor_right_enable = OutputDevice(MOTOR_RIGHT_ENABLE)
motor_left_speed = PWMOutputDevice(MOTOR_LEFT_SPEED, frequency=1000)
motor_right_speed = PWMOutputDevice(MOTOR_RIGHT_SPEED, frequency=1000)

buzzer_left = Buzzer(BUZZER_LEFT_PIN)
buzzer_right = Buzzer(BUZZER_RIGHT_PIN)

haptic_left = PWMOutputDevice(HAPTIC_LEFT_PIN, frequency=200)
haptic_right = PWMOutputDevice(HAPTIC_RIGHT_PIN, frequency=200)

servo_sweep_active = False
sweep_thread = None


def map_distance_to_pwm(distance):
    if distance <= MIN_DISTANCE:
        return 0.0
    elif distance >= MAX_DISTANCE:
        return 1.0
    return (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE)


def map_distance_to_haptic(distance):
    if distance <= MIN_DISTANCE:
        return 1.0
    elif distance >= MAX_DISTANCE:
        return 0.0
    return 1.0 - ((distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE))


def sweep_servo_left():
    for angle in range(90, 181, PAN_SWEEP_STEP):
        if not servo_sweep_active:
            return
        pan_servo.value = (angle - 90) / 90
        time.sleep(PAN_SWEEP_DELAY)
    for angle in range(180, 89, -PAN_SWEEP_STEP):
        if not servo_sweep_active:
            return
        pan_servo.value = (angle - 90) / 90
        time.sleep(PAN_SWEEP_DELAY)


def sweep_servo_right():
    for angle in range(90, -1, -PAN_SWEEP_STEP):
        if not servo_sweep_active:
            return
        pan_servo.value = (angle - 90) / 90
        time.sleep(PAN_SWEEP_DELAY)
    for angle in range(0, 91, PAN_SWEEP_STEP):
        if not servo_sweep_active:
            return
        pan_servo.value = (angle - 90) / 90
        time.sleep(PAN_SWEEP_DELAY)


def continuous_sweep(direction):
    global servo_sweep_active
    while servo_sweep_active:
        if direction == "left":
            sweep_servo_left()
        elif direction == "right":
            sweep_servo_right()


def tilt_down_up_down_cycles(cycles=3):
    tilt_servo.value = TILT_MEAN_VALUE
    time.sleep(0.1)
    for _ in range(cycles):
        value = TILT_MEAN_VALUE
        while value < TILT_DOWN_VALUE:
            value = min(value + TILT_STEP, TILT_DOWN_VALUE)
            tilt_servo.value = value
            time.sleep(TILT_STEP_DELAY)
        while value > TILT_MEAN_VALUE:
            value = max(value - TILT_STEP, TILT_MEAN_VALUE)
            tilt_servo.value = value
            time.sleep(TILT_STEP_DELAY)
    tilt_servo.value = None


def check_sensors():
    global servo_sweep_active, sweep_thread
    tilt_cycle_active = False
    tilt_thread = None
    left_hits = 0
    right_hits = 0

    while True:
        try:
            left_distance = sensor_left.distance * 100
            right_distance = sensor_right.distance * 100
        except:
            left_distance = 999
            right_distance = 999

        left_hits = left_hits + 1 if left_distance < DETECTION_DISTANCE else 0
        right_hits = right_hits + 1 if right_distance < DETECTION_DISTANCE else 0

        left_detected = left_hits >= DETECTION_REQUIRED
        right_detected = right_hits >= DETECTION_REQUIRED

        if left_detected:
            motor_left_enable.on()
            motor_right_enable.on()

            pwm_value = map_distance_to_pwm(left_distance)
            motor_left_speed.value = pwm_value
            motor_right_speed.value = pwm_value

            haptic_left.value = map_distance_to_haptic(left_distance)
            haptic_right.value = 0

            if not servo_sweep_active:
                servo_sweep_active = True
                sweep_thread = threading.Thread(target=continuous_sweep, args=("left",))
                sweep_thread.start()

            if not tilt_cycle_active:
                tilt_cycle_active = True
                tilt_thread = threading.Thread(target=tilt_down_up_down_cycles, args=(3,))
                tilt_thread.start()

            buzzer_left.on()

        elif right_detected:
            motor_left_enable.on()
            motor_right_enable.on()

            pwm_value = map_distance_to_pwm(right_distance)
            motor_left_speed.value = pwm_value
            motor_right_speed.value = pwm_value

            haptic_right.value = map_distance_to_haptic(right_distance)
            haptic_left.value = 0

            if not servo_sweep_active:
                servo_sweep_active = True
                sweep_thread = threading.Thread(target=continuous_sweep, args=("right",))
                sweep_thread.start()

            if not tilt_cycle_active:
                tilt_cycle_active = True
                tilt_thread = threading.Thread(target=tilt_down_up_down_cycles, args=(3,))
                tilt_thread.start()

            buzzer_right.on()

        else:
            motor_left_enable.off()
            motor_right_enable.off()
            motor_left_speed.value = 0
            motor_right_speed.value = 0

            haptic_left.value = 0
            haptic_right.value = 0

            if servo_sweep_active:
                servo_sweep_active = False
                if sweep_thread:
                    sweep_thread.join()
                pan_servo.value = 0
                tilt_servo.value = TILT_MEAN_VALUE
                time.sleep(REST_SETTLE_DELAY)
                pan_servo.value = None
                tilt_servo.value = None

            if tilt_thread and not tilt_thread.is_alive():
                tilt_cycle_active = False
                tilt_thread = None

            buzzer_left.off()
            buzzer_right.off()

        time.sleep(0.2)


if __name__ == "__main__":
    try:
        check_sensors()

    except KeyboardInterrupt:
        print("\nShutting down...")
        servo_sweep_active = False
        motor_left_enable.off()
        motor_right_enable.off()
        motor_left_speed.value = 0
        motor_right_speed.value = 0
        buzzer_left.off()
        buzzer_right.off()
        haptic_left.value = 0
        haptic_right.value = 0
        pan_servo.value = 0
        tilt_servo.value = TILT_MEAN_VALUE
        time.sleep(REST_SETTLE_DELAY)
        pan_servo.value = None
        tilt_servo.value = None
