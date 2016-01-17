/* Enums defining all the possible types of wall that can be sensed by the reflectance
   sensor array in from of the robot */
typedef enum _WALL_SENSE
{
    WS_NIL,
    WS_FULL_WALL,
    WS_PARTIAL_WALL,
    WS_NO_WALL,
    WS_NO_WALL_AHEAD,
    WS_HIT_WALL,
    WS_WALL_AHEAD,
} WALL_SENSE;

/* Enums describing all the directions the robot can face during operation */
typedef enum _DRIVE_DIRECTION
{
    LEFT,
    RIGHT,
    FORWARD,
} DRIVE_DIRECTION;

/* A struct used to store information about the robots current surroundings */
typedef struct _WALL_INFO
{
    WALL_SENSE lastLeftWall;
    WALL_SENSE lastRightWall;
    WALL_SENSE lastForwardWall;
} WALL_INFO;

/* A type used to represent all the possible places that are in the map
   that the robot could possibly be in */
typedef enum _POSITION_ESTIMATE
{
    PE_NIL,
    PE_CORRIDOOR,
    PE_RIGHT_ROOM,
    PE_LEFT_ROOM,
    PE_RIGHT_CORNER,
    PE_LEFT_CORNER,
    PE_END_OF_MAZE,
    PE_PARTIAL_LEFT,
    PE_PARTIAL_RIGHT,
    PE_UNCERTAIN,
    PE_AT_START
} POSITION_ESTIMATE;
