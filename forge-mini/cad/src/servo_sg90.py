from cadgen import step

from lib.geo import SG90_BLUE, paint
from lib.servos import servo_body


@step(out="../STEP/purchased/servo_sg90.step")
def servo_sg90():
    return paint(servo_body("sg90"), SG90_BLUE, "servo_sg90")


if __name__ == "__main__":
    servo_sg90()
