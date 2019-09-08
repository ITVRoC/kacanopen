#! /usr/bin/pyhthon]

import time

node_id = 1
search_str = "JointStateSubscriber receive -- called device{}/set_joint_state".format(node_id)
max_timeout_s = 5
start_time = time.time()

f = open('/tmp/roslauch.log', 'r')
while True:
    line = ''
    while len(line) == 0 or line[-1] != '\n':
        tail = f.readline()
        if tail == '':
            time.sleep(0.1)          # avoid busy waiting
            # f.seek(0, io.SEEK_CUR) # appears to be unneccessary
            continue
        line += tail
    #print(line)

    if search_str not in line:
    	elapsed_time = time.time() - start_time
    	if elapsed_time > max_timeout_s:
    		timestamp = time.asctime( time.localtime(time.time()) )
    		print("{} ERROR - NODE_ID:{} not found".format(timestamp, node_id))
    		time.sleep(10)
    else:
    	start_time = time.time()