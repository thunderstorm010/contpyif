from contpyif import ros_init, Controller, ControllerSend # type: ignore

ros_init()

controller = Controller()
sender = ControllerSend()

while True:
    data = controller.read()

    # Check data
    print(data)

    # You can also send data to the User interface right here
    # properties: x, y, z, r, buttons (tuple u8), gain
    # the rest are ints

    sender.send(data) # Send data to vehicle