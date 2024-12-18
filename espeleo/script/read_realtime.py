#! /usr/bin/pyhthon]

import time
import io
import Queue
import threading

node_id = 1
default_search_str = "JointStateSubscriber receive -- called device{}/set_joint_state".format(node_id)
log_file = '/var/log/syslog'
max_timeout_s = 3
start_time = time.time()
q = Queue.Queue()

def monitor_log(fname, search_str):
    f = open(fname, 'r')

    while True:
        line = ''
        while len(line) == 0 or line[-1] != '\n':
            tail = f.readline()
            if tail == '':
                time.sleep(0.01)          # avoid busy waiting
                f.seek(0, io.SEEK_CUR)    # appears to be unneccessary
                continue
            line += tail

        if search_str in line:
            q.put(True)


if __name__ == "__main__":
    t = threading.Thread(target=monitor_log, args=(log_file, default_search_str))
    t.daemon = True
    t.start()

    while True:
        try:
            q.get(True, 1)
        except Exception as e:
            timestamp = time.asctime( time.localtime(time.time()) )
            print("{} ERROR - NODE_ID:{} not found".format(timestamp, node_id))
