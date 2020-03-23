from pynput.keyboard import Key, Listener
import rospy
from std_msgs.msg import String

rospy.init_node('tele_operation', anonymous=True)
pub = rospy.Publisher('/data', String, queue_size=10)

def on_release(key):
    """ Press the following keys for desired effect :
            1. Space - moves the drone forward by 1m/s.
            2. Up - increase height by 0.5 m/s
            3. Down - decrease height by 0.5m/s
            4. Shift - go back to initial start location
            5. Esc - quit recording keys
    """
    print('{0} release'.format(key))
    if key == Key.space:
        pub.publish("FORWARD")
    elif key == Key.up:
        pass
    elif key == Key.down:
        pass
    elif key == Key.shift_r or key == Key.shift_l:
        pub.publish("HOVER")
    elif key == Key.esc:
        # Stop listener
        return False

def on_press(key):
    pass

print(""" Press the following keys for desired effect :
                1. Space - moves the drone forward by 1m/s.
                2. Up - increase height by 0.5 m/s
                3. Down - decrease height by 0.5m/s
                4. Shift - go back to initial start location
                5. Esc - quit recording keys    
        """)

# listen for keypress
with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()
