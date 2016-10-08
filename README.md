# Pololu Zumo Robot - Search and Rescue

In this assignment I was required to program a Pololu Zumo robot to perform a simulated search and rescue operation.

The scenario motivating this assignment is to imagine that the robot is trying to find/rescue people trapped in a building which is filled with smoke. People are to be discovered in 'rooms', and the robot moves through corridors. When the robot discovers a 'person' it signals back to ‘base’ so that a search party can come in to rescue that person. The robot, however, continues to search, signalling as and when it finds people in other rooms. When the search-bot reaches the end of the corridor, it turns around and returns to base.

For the Zumo this meant that we needed to make use of its line sensor (to stay within the maze), add an ultrasonic sensor (to detect objects in rooms) and add a Xbee Bluetooth device to communicate with a remote laptop acting as the ‘base’.

The ‘building’ consist of a corridor with corners and adjoining rooms. The boarders of the building are set out with black lines on a white background. The diagram below gives an example of a building layout.

![Building Examle](https://github.com/kevinchar93/University_ZumoRobot_SearchAndResue/blob/master/building_example_2.png "An example building layout")

## Video of Robot in Action
<a href="http://www.youtube.com/watch?feature=player_embedded&v=9lQy158vBtM
" target="_blank"><img src="http://img.youtube.com/vi/9lQy158vBtM/0.jpg" 
alt="Zumo Robot in Maze" width="640" height="400" border="10" /></a>

## Quick Operation Overview
The way in which my robot completes the course, is by building up an “image” of its
current surroundings step by step using the line sensor. The robot is driven forward
for a short distance, stopping when a timer has expired or a line is hit, it then turns
left and right driving up to the wall on each side to ensure that it remains within the
walls. This is repeated as the robot makes its way through the map.
The robot uses a data structure to store what kind of wall it just sensed to the left,
right and in front of it (eg a partial wall, full wall, no wall) ­ this data structure is then
examined to estimate where the robot currently is in the map, the previous position
estimate is also used to calculate the new position estimate in some situations.

Examples of position estimates are : in a right room, in a left room , in corridor , in
right corner, end of maze.
The estimate is then used to decide what to do next, this table specifies how the
robot acts with a given position estimate.

| Positon Estimate          | Behaviour                                                                                                                                               |
|---------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| In Corridoor              | Move forward and search, then turn and search left and right for walls                                                                                  |
| In Right Room             | If the room hasn’t been searched move into the room and search it for people                                                                            |
| In Left Room              | If the room hasn’t been searched move into the room and search it for people                                                                            |
| In Right Corner           | Turn right then continue as normal move forward and search left / right                                                                                 |
| In Left Corner            | Turn left then continue as normal move forward and search left / right                                                                                  |
| End of Maze               | Turn the robot around 180 degrees and do the maze in reverse                                                                                            |
| Partial wall on the left  | Move twice as far forward as normal to get past the partial wall quicker, then search left / right as normal                                            |
| Partial wall on the right | Move twice as far forward as normal to get past the partial wall quicker, then search left / right as normal                                            |
| Uncertain of position     | Continue as normal, move forward and search left right, hopefully we will get enough sensory data to have a firm position estimate on the next movement |
| At the start              | Move forward and search left and right as normal.                                                                                                       |
With these estimates the robot is fully equipped with instructions of how to handle
each situation it may encounter in the maze ­ fulfilling its job of going through the
entire maze and finding people in rooms.

## Acknowledgement and Sources
My robot makes use of libraries and ideas from three sources, Pololu the robot
manufacturer , Texas Instruments, who had interns who wrote libraries for working
with the Zumo robot using TI hardware / Energia and Tim Eckel the developer of the
NewPing library. See the [Report](https://github.com/kevinchar93/University_ZumoRobot_SearchAndResue/blob/master/Report.pdf) for details.
