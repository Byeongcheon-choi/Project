import websocket
import RRT
import math


try:
    import thread
except ImportError:
    import _thread as thread
import time


once = 0
state = [0,0,0]  # (X,Y,H)
targetPoint = (100,450,(3*math.pi)/2)
obstacleList = [((200,700),(300,650)),((225,450),(325, 350)),((125,350),(225,200))]
vertices = []
edges = []
path = []
path_counter = 1
# commands = [(0, 0, 500), (180, 180, 500)]  # two instructions, (leftPWM, rightPWM, timeToExecuteInMillis)
error_threshold = 2000000
is_done = False

def on_message(ws, message):
    def sendCommand(*args):
        # time.sleep(1)
        global vertices, edges, path, path_counter, obstacleList, is_done
        if not path:
            print("Generating a new tree since there was no path")
            (vertices, edges, path) = RRT.RRTPlanner(tuple(state), targetPoint, obstacleList)
            print("Done generating new tree")
            # thread.start_new_thread(RRT.plotCSpace, (obstacleList, edges, vertices,path))
            print("Just spawned the plot thread")
            print("Path:")
            print(path)
            path_counter = 1
            is_done = False
        else:  # already generated path, let's see if we're still close enough to the trajectory
            dest = path[path_counter]
            dx = dest[0] - state[0]
            dy = dest[1] - state[1]
            if(math.sqrt((dx**2) + (dy**2)) > error_threshold):  # just look at the geometric distance, ignoring error in theta
                print("Generating a new tree since " + str(math.sqrt((dx**2) + (dy**2))) + " > " + str(error_threshold))
                (vertices, edges, path) = RRT.RRTPlanner(state, targetPoint, obstacleList)
                print("Done generating new tree")
                # thread.start_new_thread(RRT.plotCSpace, (obstacleList, edges, vertices,path))
                print("Just spawned the plot thread")
                path_counter = 1
                is_done = False

        # print(path)
        # print(path_counter)
        # print("State: " + str(state))
        # print("path[path_counter]" + str(path[path_counter]))
        startandachieved, commands = RRT.genAchievableTraj(state, path[path_counter])
        print("Start/Achieved: " + str(startandachieved))
        path_counter = path_counter + 1
        if path_counter >= len(path):  # We finished the path
            is_done = True

        to_send = "&\n" 
        for command in commands:
            to_send = to_send + "{:7.4f}\n".format(command[0]) + "{:7.4f}\n".format(command[1]) + "{:7.4f}\n".format(command[2] * 1000)
        ws.send(to_send)
        print("Just sent " + to_send)
        print("thread terminating...")
    
    if(message[0] == '%'):
        # print(message[5:])
        data = message[2:]
        lines = data.splitlines()
        global state
        for line_num in range(len(lines)):
            if line_num == 2:
                state[2] = float(lines[line_num]) % (2*math.pi)  # have to change the numbers around as the paperbot stores it as (H,X,Y)
            elif line_num == 3:
                state[0] = float(lines[line_num])
            elif line_num == 4:
                state[1] = float(lines[line_num])
        print(state)
        if not is_done:
            thread.start_new_thread(sendCommand, ())
        else:
            ws.close()
            RRT.plotCSpace(obstacleList, edges, vertices, path)
    else:
        print(message)


def on_error(ws, error):
    print(error)

def on_close(ws):
    print("### closed ###")

def on_open(ws):
    def run(*args):
        # for i in range(30000):
        #     time.sleep(1)
        #     ws.send("#C")
        time.sleep(1)
        # ws.close()
        print("thread terminating...")
    # thread.start_new_thread(run, ())


if __name__ == "__main__":
    websocket.enableTrace(True)
    address = "192.168.4.1"
    ws = websocket.WebSocketApp("ws://" + address + ':81/',
                              on_message = on_message,
                              on_error = on_error,
                              on_close = on_close)
    ws.on_open = on_open
    ws.run_forever()