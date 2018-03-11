'''
Created on 4 mar 2018

@author: arazu
'''

from queue import Queue
import threading
import time


print_lock = threading.Lock()

def exJob(worker):
    time.sleep(0.5)
    
    with print_lock:
        print(threading.current_thread().name,  worker)

#threading operation
def threader():
    print('im now in threader')
    while True:
        worker = q.get()
        exJob(worker)
        q.task_done()

q = Queue()

for  x in range(10):
    t = threading.Thread(target= threader)
    
    t.deamon = True # makes thread to die after its done
    t.start()
    
start = time.time()

print('job about to start')
for worker in range(20):
    q.put(worker)
    print(worker, ' in queue')
    
q.join() # waiting for thread to terminate
print('entire job took:' , time.time() - start)


