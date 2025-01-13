#ifndef COMMANDS_H
#define COMMANDS_H

// Command definitions
const char CMD_STATUS_REQ = 'S';
const char CMD_STOP = 'X';
const char CMD_X_NEW_POS = 'X';
const char CMD_Y_NEW_POS = 'Y';
const char CMD_Z_NEW_POS = 'Z';
const char CMD_CLOSE = 'C';
const char CMD_OPEN = 'O';
const char CMD_CALIBRATE = 'H';
const char CMD_DOUBLE_CALIBRATE = 'D';

// Command terminator
const char TERMINATOR = '\n';

// Command prefix
const char AT = '@';

#endif // COMMANDS_H
