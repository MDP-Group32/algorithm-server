import math
from shared.models import LiveResponsePosition

def get_command_one(
    segments
):
    result = []

    result = process_segments(segments)
    resultCombined = combine_results(result)
    
    return update_results(resultCombined)

def create_position_entry(direction, movement, distance, segment):
    return [
        f"{direction},-1,{movement},{distance}",
        LiveResponsePosition(
            x=segment.pos.x // 10,
            y=segment.pos.y // 10,
            d=convertThetatoNumericBearing(segment.pos.theta)
        )
    ]

def process_segments(segments):
    result = []
    for segment in segments:
        if segment.v == 1:
            if segment.s == -1:
                result.append(create_position_entry("left", "forward", "-1", segment))
            elif segment.s == 0:
                result.append(create_position_entry("center", "forward", str(int(segment.d)), segment))
            elif segment.s == 1:
                result.append(create_position_entry("right", "forward", "-1", segment))
        elif segment.v == -1:
            if segment.s == -1:
                result.append(create_position_entry("left", "reverse", "-1", segment))
            elif segment.s == 0:
                result.append(create_position_entry("center", "reverse", str(int(segment.d)), segment))
            elif segment.s == 1:
                result.append(create_position_entry("right", "reverse", "-1", segment))
    return result

def combine_results(result):
    result_combined = []
    for entry in result:
        command, pos = entry
        command_parts = command.split(',')

        if not result_combined:
            result_combined.append(entry)
        else:
            prev_command, prev_pos = result_combined[-1]
            prev_parts = prev_command.split(',')

            if command_parts[0] == "center" and prev_parts[:3] == command_parts[:3]:
                new_distance = str(int(prev_parts[3]) + int(command_parts[3]))
                result_combined[-1] = [f"{prev_parts[0]},{prev_parts[1]},{prev_parts[2]},{new_distance}", pos]
            else:
                result_combined.append(entry)
    return result_combined

def truncate(direction: str, distance: int) -> int:
    if distance == 0:
        return 0
    elif direction == "reverse":
        if distance < 50:
            return distance
        else:
            return distance+2
    elif direction == "forward":
        return distance-1
    
def process_result(result):
    split_result = result.split(",")

    # Skip if command is not a "center" command
    if split_result[0] != "center":
        return result

    # Update the direction and distance
    direction, distance = split_result[2], int(split_result[3])
    split_result[3] = str(truncate(direction, distance))
    return ",".join(split_result)

def update_results(resultCombined):
    return [[process_result(item[0])] for item in resultCombined]

def convertThetatoNumericBearing(theta):
    # North
    if math.pi / 4 <= theta and theta <= 3 * math.pi / 4:
        return 1
    # South
    elif (-3 * math.pi / 4 <= theta and theta <= -math.pi / 4):
        return 2
    # East
    elif -math.pi / 4 <= theta and theta < math.pi / 4:
        return 3
    # West
    elif (3 * math.pi / 4 < theta and theta <= math.pi) or (-math.pi <= theta and theta < -3 * math.pi / 4):
        return 4
    else:
        return 1

def getFinalStmCommand(originalCommand):
    if originalCommand.upper().startswith("ST"):
        return originalCommand
    else:
        originalCommandArray = originalCommand.split(",")
        if originalCommandArray[0] == 'center' and originalCommandArray[2] == 'forward':
            return 'FF' + originalCommandArray[3].zfill(3)
        elif originalCommandArray[0] == 'center' and originalCommandArray[2] == 'reverse':
            return 'RR' + originalCommandArray[3].zfill(3)
        elif originalCommandArray[0] == 'left' and originalCommandArray[2] == 'forward':
            return 'FL000'
        elif originalCommandArray[0] == 'right' and originalCommandArray[2] == 'forward':
            return 'FR000'
        elif originalCommandArray[0] == 'left' and originalCommandArray[2] == 'reverse':
            return 'BL000'
        elif originalCommandArray[0] == 'right' and originalCommandArray[2] == 'reverse':
            return 'BR000'
    print('Unrecognized original STM Command: ', originalCommand)
    return originalCommand
