# Created by hhj on 2021/1/20.
# Author's Homepage: https://github.com/Harry-hhj
# Last Modified: 2021/1/20

# to import the class you have defined in cpp, use
# import Message_<class_name>, in which <class_name> is declared in UMT_EXPORT_MESSAGE_ALIAS(...)
# Don't try to use `from Message_<class_name> import *`, since the Publisher/Subsccriber of all classes is title
# However, you can try this: from Message_<class_name> import <class_name> since all classed is distinguishable
import Message_A
from Message_A import A
# In Python, to use thread, you have two alternatives, one is thread.run(), the other is thead.start()
# To make sure that the thread won't interfere each other, just use start()
import threading

# declare a Subscriber
sub = Message_A.Subscriber("pub-A1")
# declare a Publisher
pub = Message_A.Publisher("pub-B1")

# get msg from pub2 in c++
# wait until it gets the msg
def func1():
    global sub
    while True:
        a: A = sub.pop()
        print(a.str)

# send msg to sub2 in c++
def func2():
    global pub
    t = time.time()
    while True:
        if time.time() - t < 0.5:
            pass
        else:
            pub.push(A("PubB1: hello, world."))
            t = time.time()


if __name__ == '__main__':
    # make sure that the subscriber in python is defined after the publisher in c++
    import time
    time.sleep(0.5)

    thread2 = threading.Thread(target=func2)
    thread2.setDaemon(True)
    thread2.start()
    thread1 = threading.Thread(target=func1)
    thread1.setDaemon(True)
    thread1.start()
    thread1.join()
