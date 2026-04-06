import board
import usb_host
import usb.core
import digitalio
import array
import pwmio
import time

print("BOOT STARTED")

usb_host.Port(board.GP12, board.GP13)

time.sleep(2)


pwm1 = pwmio.PWMOut(board.GP2, frequency=20000, duty_cycle=0)
pwm2 = pwmio.PWMOut(board.GP3, frequency=20000, duty_cycle=0)

def send_value(v1, v2):
    pwm1.duty_cycle = int(v1 / 32767 * 65535)
    pwm2.duty_cycle = int(v2 / 32767 * 65535)

usb_host.Port(board.GP12, board.GP13)

print("Scanning USB devices...")

device = None
endpoint = 0x81

for d in usb.core.find(find_all=True):

    print(f"Device found: {d.idVendor:04x}:{d.idProduct:04x}")

    try:
        print("Manufacturer:", d.manufacturer)
        print("Product:", d.product)
    except:
        pass

    device = d
    device.set_configuration()
    break


if device is None:
    print("No USB device detected")
    while True:
        pass


buf = array.array("B", [0]*8)

print("Touchscreen ready")

while True:

    try:
        count = device.read(endpoint, buf, timeout=20)

        x = buf[2] | (buf[3] << 8)
        y = buf[4] | (buf[5] << 8)

        touching = buf[1] & 1

        if touching:
            send_value(x,y)
            print(x,y)
        else:
            send_value(0,0)
            

    except usb.core.USBTimeoutError:
        continue
