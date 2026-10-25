from cadgen import step

from lib.geo import SERVO_BLACK, paint
from lib.servos import servo_body


@step(out="../STEP/purchased/servo_mg90s.step")
def servo_mg90s():
    return paint(servo_body("mg90s"), SERVO_BLACK, "servo_mg90s")


if __name__ == "__main__":
    servo_mg90s()
