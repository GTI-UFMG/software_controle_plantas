import sys
import threading 

from PySide6 import QtWidgets

import numpy as np

# The Communication Agent that takes care of the communication between 
# the GUI and the embedded code in the ESP32.
from communication_agent import CommunicationAgent

# The Graphical User Interface class.
from motorDC_GUI import GUIWindow 

if __name__ == "__main__":

    # Start the Communication Agent thread 
    comm_agent = CommunicationAgent()
    CommTask = threading.Thread(target=comm_agent.comm_loop)
    CommTask.daemon = True
    CommTask.start()
    
    # Setup the GUI (in the main thread).
    myGUI_app = QtWidgets.QApplication(sys.argv)
    #      In Python, objects from custom classes are passed by reference.
    window = GUIWindow(comm_agent)
    window.show()
    
    # Start the GUI event loop.
    myGUI_app.exec()

# Stop the Communication Handler: 
# 1. Signal that it should stop.
# 2. Wait until the thread is terminated.
CommTask.do_run = False
CommTask.join()

# Some statistics at end of the program.
print("\
\n\
****************\n\
Some statistics:\n\
*****************")

if (comm_agent.get_data_count() == 0):
    print("No data was received.")
else:
    data_array = comm_agent.get_data_array(np.arange(0,comm_agent.get_data_count()))
    print("Sampling time = %2.3g"%(1.0e3*np.mean(np.diff(data_array[:,0]))),"ms")
    print("Sampling time standard deviation = %2.3e"%(1000.0*np.std(np.diff(data_array[:,0]))),"ms.")

# import matplotlib.pyplot as plt
print("data count =",comm_agent.get_data_count())
# print(data_array[0:10,:])
# # plt.plot(data_array[0:data_count-2,0],np.diff(data_array[0:data_count-1,0]),'b-o')
# plt.hist(np.diff(data_array[0:data_count-1,0]),20)
# plt.show()
