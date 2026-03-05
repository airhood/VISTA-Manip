import raspberry_utils.pwm as pwm

def init_status_led():
    pwm.init_pwm(16, 2, 50)