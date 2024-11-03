import rclpy
import std_msgs.msg

import sys
import threading

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
This node takes keypresses from the keyboard and publishes them
as ByteMultiArray messages.
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x   


q/e : clock and cclock wise spin
z : Air bearings on/off
x : stop

anything else : stop

CTRL-C to quit
"""

moveBindings = {
    'q': (1, 1, 0, 1, 0, 1, 0, 1, 0),
    'w': (1, 0, 1, 0, 1, 1, 0, 1, 0),
    'e': (1, 0, 1, 0, 1, 0, 1, 0, 1),
    'a': (1, 1, 0, 0, 1, 0, 1, 1, 0),
    's': (1, 1, 0, 1, 0, 0, 1, 0, 1),
    'd': (1, 0, 1, 1, 0, 1, 0, 0, 1),
    'x': (1, 0, 0, 0, 0, 0, 0, 0, 0),
    'z': (0, 0, 0, 0, 0, 0, 0, 0, 0),
    'Q': (1, 1, 0, 1, 0, 1, 0, 1, 0),
    'W': (1, 0, 1, 0, 1, 1, 0, 1, 0),
    'E': (1, 0, 1, 0, 1, 0, 1, 0, 1),
    'A': (1, 1, 0, 0, 1, 0, 1, 1, 0),
    'S': (1, 1, 0, 1, 0, 0, 1, 0, 1),
    'D': (1, 0, 1, 1, 0, 1, 0, 0, 1),
    'X': (1, 0, 0, 0, 0, 0, 0, 0, 0),
    'Z': (0, 0, 0, 0, 0, 0, 0, 0, 0),
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('unilu_FP_Keyboard_node')

    ByteMultiArrayMsg = std_msgs.msg.ByteMultiArray
    pub = node.create_publisher(ByteMultiArrayMsg, '/omniFPS/Robots/FloatingPlatform/thrusters/input', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    byte_msg = ByteMultiArrayMsg()
    
    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                cmd = moveBindings[key]
                byte_msg.data = [bytes([value]) for value in cmd]
            else:
                cmd = moveBindings['x']
                byte_msg.data = [bytes([value]) for value in cmd]
                if (key == '\x03'):
                    break
            
            pub.publish(byte_msg)
    except Exception as e:
        print(e)

    finally:
        pub.publish(byte_msg)
        rclpy.shutdown()
        spinner.join()
        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()