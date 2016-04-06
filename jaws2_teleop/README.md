# Jaws 2 Teleop

*Contains nodes for interpreting joystick input to Jaws 2.*

## Connecting a PS3 Controller

### Initial Pairing

Connect the controller using a USB cable, then type:

    sudo sixpair

### Starting Paired

Without the controller connected to a USB cable, type:

    sixad -s

Press the PS button, wait for rumbling.

## Troubleshooting

### Sixpair Fails

For:

    Current Bluetooth master: 00:22:b0:d0:5a:09
    Unable to retrieve local bd_addr from `hcitool dev`.

Try:

    sudo hciconfig hci0

If DOWN:

    sudo hciconfig hci0 up

Else:

    sudo hciconfig hci0 reset

### Sixad Fails

If controller LEDs blink quickly, reset the controller. (Pen-tip button on underside of controller.)
