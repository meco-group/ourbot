#ifndef GAMEPAD_CMD_H
#define GAMEPAD_CMD_H

#include "GamePad_protocol.h"

// Commands
//-----------------------------------------
                        /* function               3rd arg   */
#define JSIOCGAXES      /* get number of axes     char      */
#define JSIOCGBUTTONS   /* get number of buttons  char      */
#define JSIOCGVERSION   /* get driver version     int       */
#define JSIOCGNAME(len) /* get identifier string  char      */
#define JSIOCSCORR      /* set correction values  &js_corr  */
#define JSIOCGCORR      /* get correction values  &js_corr  */

#endif //GAMEPAD_CMD_H
