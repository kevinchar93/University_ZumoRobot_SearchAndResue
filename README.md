# Pololu Zumo Robot - Search and Rescue

In this assignment I was required to program a Pololu Zumo robot to perform a simulated search and rescue operation.

The scenario motivating this assignment is to imagine that the robot is trying to find/rescue people trapped in a building which is filled with smoke. People are to be discovered in 'rooms', and the robot moves through corridors. When the robot discovers a 'person' it signals back to ‘base’ so that a search party can come in to rescue that person. The robot, however, continues to search, signalling as and when it finds people in other rooms. When the search-bot reaches the end of the corridor, it turns around and returns to base.

For the Zumo this meant that we needed to make use of its line sensor (to stay within the maze), add an ultrasonic sensor (to detect objects in rooms) and add a Xbee Bluetooth device to communicate with a remote laptop acting as the ‘base’.

The ‘building’ consist of a corridor with corners and adjoining rooms. The boarders of the building are set out with black lines on a white background. The diagram below gives an example of a building layout.

![Building Examle](https://github.com/kevinchar93/University_ZumoRobot_SearchAndResue/blob/master/building_example_2.png "")

<p align="center">
<img src="https://github.com/kevinchar93/University_ZumoRobot_SearchAndResue/blob/master/building_example_2.png" 
alt="An example building layout" width="704" height="366" border="10" />
</p>

## Video of Robot in Action
<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=9lQy158vBtM
" target="_blank"><img src="http://img.youtube.com/vi/9lQy158vBtM/0.jpg" 
alt="Zumo Robot in Maze" width="640" height="400" border="10" /></a>
</p>

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

##Key Issues / Challenges
###Staying within the walls: 
This issue is key to being able to complete the course at
all, originally I thought it would be best to make the robot zig zag hitting each wall
through the course until it detected rooms on either sides of the wall, but the problem
with this is that it is difficult to differentiate between corners and rooms, I could not
guarantee that the robot would not zig zag into a corner, in a way that would appear
to be a room to the robot's sensors ­ or the opposite situation.

The solution that I thought up of to keep the robot within the walls was to have it
move forward then rotate left and right driving up to the walls on each side. This
ensures that the robot is still within the walls after each movement and has the
added benefit of allowing us to differentiate between different parts of the map
(corners , rooms, corridor) in a consistent way by checking what walls are
surrounding the robot.


###Turning Using the Gyro ​(motion.ino : rotateToAngle()​) ​: 
With my decision to turn the robot left and right to each of the surrounding walls, I needed a method to
accurately rotate a given amount of degrees. Since this robot has no encoders it is
impossible to accurately tell how much the wheels have spun in a given span of time,
if this was available it could be used to calculate how much to spin each wheel to
spin to a specific angle. Instead the robot does have a built in gyroscope that can be
used to measure angular velocity and convert this into a measurement of where the
robot is facing ­ my robot uses the gyroscope to make turns by doing the following:

1. Zeros the gyroscope so that wherever it is facing is considered zero degrees
2. Adds the target angle to the current heading to calculate where the heading
should be at the end of the turn (wrapping this between ­180 to +180 degrees
(the operating range of the gyro)
3. The current heading of the robot is then read to calculate how much error
there is between the current heading and the target angle
4. This error value is then used in a Proportional Integral control algorithm to
calculate speeds for the motors to rotate the robot
5. The robot keeps checking the current heading and adjusting the motor speeds
until the current heading is within 1 degree of the target angle, stopping the
robot spinning when it is.


###Correcting the Robot after a turn: 
After each turn that the robot makes it has the
potential to be off by about 1 or 2 degrees, this makes the robot turn fairly accurately
but over time these errors can accumulate with the result being the robot facing in an
unexpected direction after about 10 turns.

To fix this I had the robot correct its orientation after each turn to the left and right by
using each wall as a reference point to correct against, the robot slowly drives up to
wards each wall (when facing left / right) with the aim being to get the reflectance
sensors on the right and left edges of the sensor array on the wall.

When a reflectance sensor hits the wallthe motor on that side would set its speed to
zero, this had the effect of making the robot parallel with the wall on each left and
right turn. So if the right reflectance sensor reached the wall before the left, the right
motor would set its speed to zero letting the left motor keep driving forward until the
left reflectance sensor hit the wall and vice versa. By correcting the robot’s
orientation on each of its turns any errors that were created during a turn are wiped
out, preventing them from accumulating and causing issues.


###Detecting Partial Walls​: 
Implementing the correction feature described above
allows the robot to stop accumulated errors from turns causing issues but it also
creates a potential problem. Should the one of the reflectance sensors on the edge
of the reflectance sensor array hit a wall ,the robot will begin to correct by moving the
opposite motor, but if there is a gap in the wall (for a corner or room) to robot will
over correct and end up facing the wrong way.

To solve this issue, I implemented a feature that times how long on average it takes
the robot to perform a normal correction and should a correction that the robot is performing
take longer the 1.5 times the average correction time ­ it stops correcting and signals
that a partial wall has been detected. This prevents the robot from over correcting
and facing the wrong way.


###Detecting Rooms and Corners: ​
To detect rooms and corners a similar timing
feature as described above is used. It times how long on average it takes for the
robot to encounter a wall after turning left or right and driving forward. Should the
robot take longer than twice the average time it takes to travel to a left or right wall it
stops moving turns around to keep travelling as normal and signals that no wall has
been detected on that side.

To detect corners the robot combines this feature with detecting if there is a wall in
front of the robot to determine if it is a corner and whether the corner is a right or left
turn.

When the robot hits a wall in front of it, it is probable that the robot did not hit the line
straight on and may be misaligned with it, the robot uses a function (motion.ino :
**performReverseCorrections**​) that reverses the robot off the line until the robot is
parallel with the line it drove into, this is done so that the robot is not too close to the
wall it drove into when we perform further movement, particularly when taking
corners or reaching the end of a corridor.


###Detecting objects: ​
Once the robot has determined that it is in a room it checks to
see if has searched the room by consulting a boolean flag “roomSearched”, the flag
is reset to false when the robot is not in a room and true once it has scanned the
room it is in. This ensures that it only searches a room once.

The robot searches a room by performing a 180 degree sweep at the mouth of the
room and polling to see if it detects anything within 30cm of its ultrasonic sensor. If
an object is detected the robot makes a tick sound as it detects objects in real time,
leaves the room then sounds a siren to signal that a person is found ­ it also sends
an Xbee message of this occurrence.


###Driving Straight Using the Gyro ​(motion.ino : driveForwardFor()​): 
To move the robot forward through the course I needed functionality that could move the robot
forward in a straight line without it drifting to the left and right, as we established
earlier we cannot track the rotations made by each wheel as there are no encoders
on the wheels.

However we can use the gyro to drive in a fairly straight line by keeping the robot's
current heading constant, the functionality works as described:
1. Zeros the gyroscope so that wherever it is facing is considered zero degrees
2. The target heading is set to zero
3. The current heading of the robot is read and the amount of error from the
target heading is calculated from this
4. A Proportional Integral control algorithm is used to calculate speeds for each
motor based on the amount of error from the target heading, these speeds
adjust to keep the robot heading at zero
5. The robots keeps adjusting the speeds of the motors to keep the heading at
zero until the travel time exceeds the set amount or the robot hits a line in
front

## Acknowledgement and Sources
My robot makes use of libraries and ideas from three sources, Pololu the robot
manufacturer , Texas Instruments, who had interns that wrote libraries for working
with the Zumo robot using TI hardware / Energia and Tim Eckel the developer of the
NewPing library. See the [Report](https://github.com/kevinchar93/University_ZumoRobot_SearchAndResue/blob/master/Report.pdf) for details.

## License

Copyright © 2016 Kevin Charles

Distributed under the MIT License
