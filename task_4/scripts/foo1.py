from threading import Thread
import time
class ThreadTest():

    def loop1(self):
        for i in range(0, 100, 5):
            print(i)
            time.sleep(2)

    def loop2(self):
        for i in range(100, 210, 11):
            print(i)
            time.sleep(2)

if __name__ == '__main__':
    T1 = Thread(target=ThreadTest().loop1, args=())
    T2 = Thread(target=ThreadTest().loop2, args=())
    T1.start()
    T2.start()
    T1.join()
    T2.join()