import RPi.GPIO as GPIO

_pwm_objects: dict[int, GPIO.PWM] = {}


def init_pwm(pin: int, freq: float, duty: float):
    if freq <= 0:
        raise ValueError(f"freq must be positive: {freq}")
    if not (0.0 <= duty <= 100.0):
        raise ValueError(f"duty must be between 0 and 100: {duty}")

    if pin in _pwm_objects:
        _pwm_objects[pin].stop()

    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, freq)
    pwm.start(duty)
    _pwm_objects[pin] = pwm

def set_duty(pin: int, duty: float):
    if not (0.0 <= duty <= 100.0):
        raise ValueError(f"duty must be between 0 and 100: {duty}")
    _get_pwm(pin).ChangeDutyCycle(duty)

def set_freq(pin: int, freq: float):
    if freq <= 0:
        raise ValueError(f"freq must be positive: {freq}")
    _get_pwm(pin).ChangeFrequency(freq)

def stop_pwm(pin: int):
    pwm = _pwm_objects.pop(pin, None)
    if pwm is not None:
        pwm.stop()

def stop_all_pwm():
    for pwm in _pwm_objects.values():
        pwm.stop()
    _pwm_objects.clear()


def _get_pwm(pin: int) -> GPIO.PWM:
    if pin not in _pwm_objects:
        raise KeyError(f"Pin {pin} is not initialized. Call init_pwm() first.")
    return _pwm_objects[pin]