import fcntl
import os
import sys
import termios

# Executed after Popen creates a new session; no preexec hook in a threaded parent
fcntl.ioctl(0, termios.TIOCSCTTY, 0)
os.execvpe(sys.argv[1], sys.argv[1:], os.environ)
