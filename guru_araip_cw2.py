from controller import Robot, Camera

# ================= CONSTANTS =================
MAX_SPEED = 6.28
MULTIPLIER = 0.5
OBSTACLE_DISTANCE = 0.02

# Color detection margins
DOMINANCE_MARGIN  = 40
DOMINANCE_MARGIN2 = 20
DOMINANCE_MARGIN3 = 5

MIN_INTENSITY   = 120
BLUE_INTENSITY  = 25
GREEN_INTENSITY = 20

# Dog colour — golden Labrador (warm tan/yellow fur)
DOG_R     = 210
DOG_G     = 170
DOG_B     = 100
TOLERANCE = 45


# ================= DISTANCE SENSORS =================
def get_distance_values(distance_sensors, distance_values):
    for i in range(8):
        val = distance_sensors[i].getValue() / 4096.0
        distance_values[i] = min(val, 1.0)


def front_obstacle(distance_values):
    avg = (distance_values[0] + distance_values[7]) / 2.0
    return avg > OBSTACLE_DISTANCE


# ================= MOVEMENT =================
def move_forward(left_motor, right_motor):
    left_motor.setVelocity(MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(MAX_SPEED * MULTIPLIER)


def move_backward(left_motor, right_motor, robot, timestep):
    left_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    wait(robot, timestep, 0.3)


def turn_left(left_motor, right_motor, robot, timestep):
    left_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(MAX_SPEED * MULTIPLIER)
    wait(robot, timestep, 0.3)


def turn_right(left_motor, right_motor, robot, timestep):
    left_motor.setVelocity(MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    wait(robot, timestep, 0.3)


def wait(robot, timestep, sec):
    start = robot.getTime()
    while robot.getTime() < start + sec:
        robot.step(timestep)


# ================= CAMERA =================
def get_camera_rgb(camera, interval, state):
    width  = camera.getWidth()
    height = camera.getHeight()
    image  = camera.getImage()

    if state["camera_interval"] >= interval:
        r = g = b = 0
        for x in range(width):
            for y in range(height):
                r += camera.imageGetRed(image, width, x, y)
                g += camera.imageGetGreen(image, width, x, y)
                b += camera.imageGetBlue(image, width, x, y)
        state["camera_interval"] = 0
        return (
            int(r / (width * height)),
            int(g / (width * height)),
            int(b / (width * height)),
        )
    else:
        state["camera_interval"] += 1
        return (0, 0, 0)


# ================= DOG DETECTION =================
def is_dog_in_frame(camera):
    width  = camera.getWidth()
    height = camera.getHeight()
    image  = camera.getImage()
    dog_pixels = 0

    for x in range(width):
        for y in range(height):
            r = camera.imageGetRed(image,   width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image,  width, x, y)

            if (abs(r - DOG_R) < TOLERANCE and
                abs(g - DOG_G) < TOLERANCE and
                abs(b - DOG_B) < TOLERANCE and
                r > g > b and
                r > 150):
                dog_pixels += 1

    total_pixels = width * height
    return (dog_pixels / total_pixels) > 0.02


# ================= IMAGE CAPTURE =================
def capture_image(camera, image_id):
    filename = f"dog_capture_{image_id}.png"
    camera.saveImage(filename, 100)
    print(f"📸 Image saved: {filename}")


# ================= MAIN ROBOT =================
def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())

    # Distance sensors
    sensor_names = ("ps0","ps1","ps2","ps3","ps4","ps5","ps6","ps7")
    distance_sensors = []
    distance_values  = [0.0] * 8
    for name in sensor_names:
        s = robot.getDevice(name)
        s.enable(timestep)
        distance_sensors.append(s)

    # Camera
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    camera_state = {"camera_interval": 0}

    # Motors
    left_motor  = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

    # Tracking
    encountered   = []
    image_counter = 0
    captured      = False

    # ================= MAIN LOOP =================
    while robot.step(timestep) != -1:

        get_distance_values(distance_sensors, distance_values)

        red, green, blue = get_camera_rgb(camera, 5, camera_state)

        # -------- COLOUR BLOCK DETECTION --------
        if not (red == 0 and green == 0 and blue == 0):

            if red > MIN_INTENSITY and red - max(green, blue) > DOMINANCE_MARGIN:
                if "Red" not in encountered:
                    print("I see Red")
                    encountered.append("Red")
                    print("Summary:", " + ".join(encountered))

            if blue > BLUE_INTENSITY and blue - max(red, green) > DOMINANCE_MARGIN2:
                if "Blue" not in encountered:
                    print("I see Blue")
                    encountered.append("Blue")
                    print("Summary:", " + ".join(encountered))

            if green > GREEN_INTENSITY and green - max(red, blue) > DOMINANCE_MARGIN3:
                if "Green" not in encountered:
                    print("I see Green")
                    encountered.append("Green")
                    print("Summary:", " + ".join(encountered))

        # -------- DOG DETECTION & CAPTURE --------
        if is_dog_in_frame(camera):
            print("🐶 DOG DETECTED")

            left_motor.setVelocity(0)
            right_motor.setVelocity(0)

            if not captured:
                wait(robot, timestep, 2.0)      # wait 2 sec for clean frame
                capture_image(camera, image_counter)
                image_counter += 1
                captured = True

                wait(robot, timestep, 1.0)      # wait 1 sec before resuming
                print("Resuming exploration...")
                move_forward(left_motor, right_motor)

        else:
            captured = False

            # -------- OBSTACLE AVOIDANCE --------
            if front_obstacle(distance_values):
                move_backward(left_motor, right_motor, robot, timestep)
                right_side = (distance_values[2] + distance_values[3]) / 2.0
                left_side  = (distance_values[4] + distance_values[5]) / 2.0
                if left_side < right_side:
                    turn_left(left_motor, right_motor, robot, timestep)
                else:
                    turn_right(left_motor, right_motor, robot, timestep)
            else:
                move_forward(left_motor, right_motor)


# ================= ENTRY POINT =================
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)