import random

DEGREES_PER_STEP = 1.8
MICROSTEP_UPPER_BOUND = 6

def stepper_tick(current_position, target_position):
    pos_dir = True
    angle_diff = target_position-current_position
    if (angle_diff < 0):
        pos_dir = False
        # make angle diff positive
        angle_diff *= -1

    if angle_diff > 180:
        angle_diff = 360-angle_diff
        pos_dir = not pos_dir

    # determine largest step possible
    divisor = 1
    mode_bits = 0
    while mode_bits < MICROSTEP_UPPER_BOUND:
        if DEGREES_PER_STEP/divisor <= angle_diff:
            break
        divisor *= 2
        mode_bits+=1

	# close enough that we don't need to step
    if mode_bits == MICROSTEP_UPPER_BOUND:
        return current_position

    # update current angle
    if pos_dir:
        current_position += DEGREES_PER_STEP/divisor
    else:
        current_position -= DEGREES_PER_STEP/divisor

    # constrain position from [0, 360)
    if current_position >= 360:
        current_position -= 360
    elif current_position < 0:
        current_position += 360

    return current_position

def run_random_test_single():
    start_position = random.uniform(0, 360)
    target_position = random.uniform(0, 360)

    i = 1
    current_position = start_position
    while True:
        updated_position = stepper_tick(current_position, target_position)
        if updated_position == current_position:
            break
        else:
            current_position = updated_position
            i+=1

    status = "\033[91mFAIL\033[0m"
    if abs(target_position-current_position) <= DEGREES_PER_STEP/(2**(MICROSTEP_UPPER_BOUND-1)) and i < 105:
        status = "\033[92mPASS\033[0m"
    
    return f"{status}: start position: {start_position}, target position: {target_position}, reached {current_position} after {i-1} iterations"


def run_random_test_many(iterations):
    for i in range(iterations):
        string = f"{i+1}/{iterations} "
        string += run_random_test_single()
        print(string)


def main():
    run_random_test_many(10)


if __name__ == "__main__":
    main()