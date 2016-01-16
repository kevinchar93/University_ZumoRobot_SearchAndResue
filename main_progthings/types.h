typedef enum _WALL_SENSE
{
    WS_NIL,             // 0
    WS_FULL_WALL,       // 1
    WS_PARTIAL_WALL,    // 2
    WS_NO_WALL,         // 3
    WS_NO_WALL_AHEAD,   // 4
    WS_HIT_WALL,        // 5
    WS_WALL_AHEAD,      // 6
} WALL_SENSE;


typedef enum _DRIVE_DIRECTION
{
    LEFT,
    RIGHT,
    FORWARD,
} DRIVE_DIRECTION;

typedef struct _WALL_INFO
{
    WALL_SENSE lastLeftWall;
    WALL_SENSE lastRightWall;
    WALL_SENSE lastForwardWall;
} WALL_INFO;

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
