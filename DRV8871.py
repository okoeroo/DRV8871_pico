from machine import Pin, PWM


class DRV8871(object):
    def __init__(self, in1, in2, freq=5000, tolerance_buffer=0):
        self._in1     = in1
        self._in2     = in2
        self._in1_pin = Pin(in1)
        self._in2_pin = Pin(in2)
        
        self._freq = freq

        self._pwm1 = PWM(self._in1_pin)
        self._pwm2 = PWM(self._in2_pin)

        self._pwm1.freq(self._freq)
        self._pwm2.freq(self._freq)

        self.MIN = 0
        self.MAX = 2 ** 16 - 1
        
        self._tolerance_buffer = tolerance_buffer
        
    
    def _map_min_max(self, x: int,  in_min: int, in_max: int, out_min: int, out_max: int) -> int:
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def forward(self, val_u16):
        self._pwm1.duty_u16(self.MAX)
        self._pwm2.duty_u16(val_u16)

    def backward(self, val_u16):
        self._pwm1.duty_u16(val_u16)
        self._pwm2.duty_u16(self.MAX)

    def zero(self):
        self._pwm1.duty_u16(self.MIN)
        self._pwm2.duty_u16(self.MIN)

    # Set a tolerance buffer
    def set_tolerance_buffer(self, tol_buf=200):
        self._tolerance_buffer = tol_buf
        
    def tolerance_buffer_remove(self):
        self._tolerance_buffer = 0

    # Divide the value 50-50.
    # The lower end will move backward, otherwise forward. Both using interpolation.
    def value(self, value, scale_min=0, scale_max=65535):
        if value > scale_max:
            raise ValueError("Value overflow")
        
        if value < scale_min:
            raise ValueError("Value underflow")
        
        balance = int(self._map_min_max(value, scale_min, scale_max, -65535, 65535) - 0.1)
        output  = int(self._map_min_max(value, scale_min, scale_max,      0, 65535) - 0.1)
        
        if balance < 0:
            self.backward(output)
            direction = "backward"
        elif balance == 0:
            self.zero()
            direction = "to zero"
        elif balance > 0 and balance < self._tolerance_buffer:
            self.zero()
            direction = "to zero (within upper balance)"
        elif balance < 0 and balance > self._tolerance_buffer:
            self.zero()
            direction = "to zero (within lower balance)"        
        else:
            self.forward(output)
            direction = "forward"
            
        return 'Moving {} value: {} balance: {} output: {}'.format(direction, value, balance, output)




### MAIN
if __name__ == '__main__':
    from machine import ADC, Pin
    import utime
    
    adc0 = ADC(26)
    drv0 = DRV8871(10, 11)
    
    drv0.value(0)
    drv0.value(2 ** 14)
    drv0.value(2 ** 15)
    drv0.value(2 ** 15 + 2 ** 14)
    drv0.value(2 ** 16 - 1)
    
    # forward
    r0 = adc0.read_u16()
    print("forward...", r0)
    drv0.forward(r0)

    utime.sleep(3)

    # backward
    r0 = adc0.read_u16()
    print("backward...", r0)
    drv0.backward(r0)

    utime.sleep(3)
    
    drv0.set_tolerance_buffer(3000)

    # test value
    while True:
        r0 = adc0.read_u16()
        feedback = drv0.value(r0)
        print(feedback)
        
        utime.sleep(3)
