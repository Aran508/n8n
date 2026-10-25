from cadgen import step

from lib.geo import SERVO_BLACK, paint
from lib.servos import servo_body


@step(out="../STEP/purchased/servo_mg996r.step")
def servo_mg996r():
    return paint(servo_body("mg996r"), SERVO_BLACK, "servo_mg996r")


if __name__ == "__main__":
    servo_mg996r()
