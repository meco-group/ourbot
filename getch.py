class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()

####################################################################################

KEY_UP = 65
KEY_DOWN = 66
KEY_RIGHT = 67
KEY_LEFT = 68
USER_QUIT = 100

def getArrow():

    myKey = getch()

    # do not stop unless user quits with 'q'
    while(myKey != 'q'):
        print "ENTER ARROW KEY OR QUIT WITH 'q'"
        if (ord(myKey) == 27):
            myKey = getch()
            if(ord(myKey) == 91):
                myKey = getch()
                if ord(myKey) == KEY_UP:
                    print "increase wheel velocities\n"
                    return KEY_UP
                elif ord(myKey) == KEY_DOWN:
                    print "decrease wheel velocities\n"
                    return KEY_DOWN
                elif ord(myKey) == KEY_LEFT:
                    print "turn left\n"
                    return KEY_LEFT
                elif ord(myKey) == KEY_RIGHT:
                    print "turn right\n"
                    return KEY_RIGHT
                else:
                    myKey = getch()
            else:
                myKey = getch()
        else:
            myKey = getch()

    if(myKey == 'q'):
        print "user wishes to quit\n"
    return USER_QUIT
