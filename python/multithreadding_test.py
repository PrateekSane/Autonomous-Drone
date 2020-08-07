import multiprocessing as mp
import time

class mptest:
    def __init__ (self):
        self.a = 0
        self.b = 0
        
    def methodA(self, val):
        print('enter A')
        time.sleep(1)
        a = val
        self.a = 1
        
    def methodB(self, val):
        print('enter B')
        time.sleep(1)
        b = val
        
        
    def multiprocessing(self):
        a = 0
        b = 0
        p1 = mp.Process(target=self.methodA, args=[1])
        p2 = mp.Process(target=self.methodB, args=[2])
        p1.start()
        p2.start()
        p1.join()
        p2.join()
        #print(a, b)
if __name__ == '__main__':
    start = time.perf_counter()
    test = mptest()
    test.multiprocessing()
    finish = time.perf_counter() 
    runtime = round(finish - start, 4)
    print(runtime, test.a, test.b)
