import math
from typing import List
from shared.models import AlgorithmOutputLivePosition
from shared.types import Position
from pathfinding.astar import Node

def convert_segments_to_commands(
    segments: List["Node"]
) -> list[list[str | Position]]:
    '''
        Converts Path Segments to Commands

        Returns:
            [command, AlgoOutputLIvePosition (end_position of robot after executing the command)]
    '''
    result = []

    GRID_CELL_CM = 10
    #TODO: Tune w STM Team forward left, forward right, reverse left, reverse right values to input
    # for each node in path
    # The segment are nodes, and it contains information on how this node is reached
    # Is it through a forward motion, backward motion, left turn, right turn, etc.
    for segment in segments:
        if segment.v == 1:
            # PREVIOUSLY: robot move forward, and steered left (FORWARD LEFT)
            if segment.s == -1:
                result.append([
                    "left,77,forward,0",
                    AlgorithmOutputLivePosition(
                        x = segment.pos.x // GRID_CELL_CM,
                        y = segment.pos.y // GRID_CELL_CM,
                        d = convertThetatoNumericDirection(segment.pos.theta)
                    )
                ])
            # PREVIOUSLY: robot move forward, and steered straight (FORWARD)
            elif segment.s == 0:
                result.append([
                    "center,0,forward," + str(int(segment.d)),
                    AlgorithmOutputLivePosition(
                        x = segment.pos.x // GRID_CELL_CM,
                        y = segment.pos.y // GRID_CELL_CM,
                        d = convertThetatoNumericDirection(segment.pos.theta)
                    )
                ])
            # The value here shold move the robot from (0,7,d=1) to (4,12,d=3)
            # PREVIOUSLY: robot move forward, and steered right (FORWARD RIGHT)
            elif segment.s == 1:
                result.append([
                    "right,102,forward,0",
                    AlgorithmOutputLivePosition(
                        x = segment.pos.x // GRID_CELL_CM,
                        y = segment.pos.y // GRID_CELL_CM,
                        d = convertThetatoNumericDirection(segment.pos.theta)
                    )
                ])
        elif segment.v == -1:
            # PREVIOUSLY: robot move backward, and steered left (REVERSE LEFT)
            if segment.s == -1:
                result.append([
                    "left,111,reverse,0",
                    AlgorithmOutputLivePosition(
                        x = segment.pos.x // GRID_CELL_CM,
                        y = segment.pos.y // GRID_CELL_CM,
                        d = convertThetatoNumericDirection(segment.pos.theta)
                    )
                ])
            # PREVIOUSLY: robot move backward, and steered straight (REVERSE)
            elif segment.s == 0:
                result.append([
                    "center,0,reverse," + str(int(segment.d)),
                    AlgorithmOutputLivePosition(
                        x = segment.pos.x // GRID_CELL_CM,
                        y = segment.pos.y // GRID_CELL_CM,
                        d = convertThetatoNumericDirection(segment.pos.theta)
                    )
                ])
            # PREVIOUSLY: robot move backward, and steered right (REVERSE RIGHT)
            elif segment.s == 1:
                result.append([
                    "right,71,reverse,0",
                    AlgorithmOutputLivePosition(
                        x = segment.pos.x // GRID_CELL_CM,
                        y = segment.pos.y // GRID_CELL_CM,
                        d = convertThetatoNumericDirection(segment.pos.theta)
                    )
                ])

    # [Merge Commands] Combine similar commands together to reduce the number of commands (to improve Robot Execution time)
    resultCombined = []
    n = 0
    for i in range(len(result)):
        string = result[i][0].split(',')
        if i == 0:
            resultCombined.append(result[i])
            n = 0
        elif string[0] != "center":
            resultCombined.append(result[i])
            n += 1
        else:
            prevstr = resultCombined[n][0].split(',')
            if string[0] == prevstr[0] and string[2] == prevstr[2]:
                new = string[0]+','+string[1]+','+string[2]+','+str(int(string[3])+int(prevstr[3]))
                resultCombined[n] = [new, result[i][1]]
            else:
                resultCombined.append(result[i])
                n += 1
    
    def _get_translated_straight_distance(direction: str, distance: int) -> int:
        """
            To stop the robot's straight movement slightly later to get the actual desired distance to be travelled by the robot.
            Value (in cm) is measured via trial and error

            Parameter:
                direction: Robot's direction
                distance: Distance that the Robot should travel physically.
        """
        # For Stop Command
        if distance == 0:
            return 0
        
        if direction == "forward":
            return distance-1
        elif direction == "reverse":
            if distance < 50:
                return distance
            else:
                return distance+2

    # Get translated straight distance to early/late stop the robot to get desired distance travelled
    for i in range(len(resultCombined)):
        result = resultCombined[i][0]
        split_result = result.split(",")

        # Skip if command is a turn command
        if split_result[0] != "center":
            continue

        direction, distance = split_result[2], int(split_result[3])
        split_result[3] = str(_get_translated_straight_distance(direction, distance))
        resultCombined[i][0] = ",".join(split_result)

    return resultCombined
    
def convertThetatoNumericDirection(theta):
    '''
        Converts Robot's Theta to Numeric Representation of Robot's Direction

        Returns:
            1: North
            2: South
            3: East
            4: West
    '''
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