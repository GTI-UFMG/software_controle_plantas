import numpy as np

import threading 

import struct

import serial
import serial.tools.list_ports   

import queue

import time

import sys

# Maximum amount of data that can be stored to be saved later.
# The idea is to pre-allocate a numpy vector to speed up the data storage process.
# If the sampling time = 2ms -> 500Hz, the amount 500*1800 corresponds
# to 30 minutes collecting data.
_data_maxsize = 500*1800

# Serial port communication thread sampling time. 
_comm_handler_ts = 0.05

# Serial port baudrate.
_serial_baudrate = 115200

# Dictionary of Command messages.
# The numerical values must correspond to those defined in the C++ code
# for the ESP32 platform.
cmd_messages = {
    "start"     : int(0),
    "stop"      : int(1),
    "store_ref" : int(2),
    "set_ref"   : int(4),
    "test_comm" : int(8),
    "set_ts"    : int(16),
    "set_ctrl_code" : int(32),
    "set_ctrl_variable" : int(36),
    "set_ctrl_sys_param" : int(48),
    "set_deadzone_comp" : int(64)}

# Dictionary of controller code IDs.
# The numerical values must correspond to those defined in the C++ code
# for the ESP32 platform.
ctrl_code = {
    "ctrl_open_loop" : int(0),
    "ctrl_pid_ct"    : int(1),
    "ctrl_ft_ct"     : int(2),
    "ctrl_pid_dt"    : int(4),
    "ctrl_ft_dt"     : int(8),
    "ctrl_custom"    : int(16)}

ctrl_var = {
    "speed"    : int(0),
    "position" : int(1)}

# Dictionary of Pre-programmed Controllers parameters. It should match
# the values defined in the C++ code for the ESP32 platform.   
ctrl_sys_params = {
    "ctrl_sys_param_kp" : int(0),
    "ctrl_sys_param_ti" : int(1),
    "ctrl_sys_param_td" : int(2)}

# Maximum length for the command queue.
_cmd_queue_maxsize = 10

# The overall idea is to have a thread that takes care of the communication as
# one of the class member functions. 
# The unique Communication Agent object is created in the main thread,
# and its reference is passed to the GUI class object.
# The GUI thread can then call the methods of the Communication Agent object
# to send commands and receive data.
class CommunicationAgent:
    
    def __init__(self):
        
        # The structure of the Data Message corresponds to 
        # the msgData_package_t type declared in the C++ code 
        # for the ESP32 platform.
        self.str_msgDataFormat = 'fffff'
        
        # The structure of the Command Message corresponds to 
        # the msgCmd_package_t type declared in the C++ code 
        # for the ESP32 platform.
        self.str_msgCmdFormat = 'iff'
        
        # Memory area for the data that 
        # will be an array whose access is controlled by a Mutex.
        self.data_lock = threading.Lock()
        # Data array that stores experimental data.
        self.data_array = np.zeros((_data_maxsize,5))
        # Number of received data points in each experiment.
        self.data_count = 0

        # Initialize the communication parameters and its lock.
        self.comm_lock = threading.Lock()
        self.comm_state = 'comm_not_ok'
        
        self.serial_portname = 'auto' # automatic detection of the ESP32 port.
        self.serialPort = serial.Serial()
        self.serial_baudrate = _serial_baudrate

        # Initialize the command and response queues.
        self.cmd_queue = queue.Queue()
        self.resp_queue = queue.Queue()
        
        # Find out the operating system name.
        self.os_name = self.get_os_name()
        print("\n")
        print(f"* Operating System: {self.os_name}\n")
        if self.os_name == 'Unknown':
            raise EnvironmentError("Unknown operating system. I don't know how to handle serial ports in this OS...")   
        
        # Internal auxiliary buffer for the serial data.
        self.serialData = b''
        
        # Flag for the communication test response.
        self.test_comm_ok_flag = False
        
    def set_serial_portname(self,portname):
        with self.comm_lock:
            if self.serial_portname != portname:                
                # Check if the serial port is already open.
                if self.comm_state == 'comm_ok':
                    # If it is open, close it first.
                    self.serialPort.close()
                    
                self.serial_portname = portname
                self.comm_state = 'comm_not_ok' # Force to try to open the port again.  
                
    def get_serial_portname(self):
        with self.comm_lock:
            portname = self.serial_portname
        return portname # In Python, strings are immutable, so this is safe because it is passed by value.
            
    def find_esp32_port(self):
        """
        Gemini LLM generated code:
        Searches for a connected ESP32 board by checking common descriptions and hardware IDs.
        Returns:
            str: The name of the serial port if an ESP32 board is found, otherwise None.
        """
        print("Searching for connected ESP32 development board...")

        # A list of common strings found in the hardware ID or description of ESP32 boards
        # and their USB-to-UART bridge chips (e.g., CP210x, CH340).
        esp32_identifiers = [
            "CP210", # Common identifier for Silicon Labs CP210x chips
            "CH340", # Common identifier for WCH CH340 chips
            "USB-SERIAL",
            "SERIAL",
            "FTDI", # Some boards might use FTDI chips
        ]

        # Use serial.tools.list_ports to get a list of all available serial ports.
        ports = serial.tools.list_ports.comports()

        if not ports:
            print("No serial ports found.")
            return None

        for port in ports:
            print(f"Checking port: {port.device} - Description: {port.description}")

            # Convert all identifiers and descriptions to lowercase for a case-insensitive search.
            port_description_lower = port.description.lower()
            port_hwid_lower = port.hwid.lower()

            # Check if any of the common identifiers are in the port's description or hardware ID.
            if any(identifier.lower() in port_description_lower or identifier.lower() in port_hwid_lower for identifier in esp32_identifiers):
                # A potential ESP32 board has been found.
                print(f"Found a potential ESP32 board on port: {port.device}")
                # The hardware ID often provides more specific details, so we'll prefer it.
                # On some systems, the device itself can be a clear indicator.
                return port.device

        print("ESP32 development board not found on any serial port.")
        return None

    # Check if the response to the communication test command was received.
    def test_comm_ok(self):
        
        with self.comm_lock:
            flag = self.test_comm_ok_flag
            
        return flag

    def send_command(self,cmd_id,cmd_val1=0.0,cmd_val2=0.0):
        
        if cmd_id is cmd_messages['test_comm']:
            with self.comm_lock:
                self.test_comm_ok_flag = False
            cmd_val1 = -123.0 # Special value to identify the communication test command.
        elif cmd_id is cmd_messages['start']:
            with self.data_lock:
                self.data_count = 0 # Reset data count at start of new experiment.
        
        cmdMsg = [int(cmd_id),float(cmd_val1),float(cmd_val2)]       
             
        self.cmd_queue.put_nowait(cmdMsg)

    def receive_message_from_serial(self):

        # Read and store all available incoming bytes at once.
        try:
            c = self.serialPort.read(self.serialPort.in_waiting)
            self.serialData += c
        except:
            c = b''
            with self.comm_lock:
                self.comm_state = 'comm_not_ok'

        # Data message processing. Here is where processing time
        # becomes critical, according to my analysis.
        # The method 'split' was crucial to speed it up.
        r = self.serialData.split(b'end')
        
        n = struct.calcsize(self.str_msgDataFormat)
        for i in range(0,len(r)-1):
            if len(r[i]) == n:
                data = struct.unpack(self.str_msgDataFormat,r[i])

                # Verify if the data message corresponds to the special data package
                # used to test the communication with the ESP32 hardware.
                if data[0] == -123.0:
                    with self.comm_lock:
                        self.test_comm_ok_flag = True
                else:
                    with self.data_lock:
                        self.data_array[self.data_count,:] = np.asarray(data,dtype=np.float32)
                        self.data_count += 1

        # Keep in the serial data buffer the characters that do not
        # form a message terminated with "end".
        self.serialData = r[-1]

    def send_message_to_serial(self):
        try:
            cmdMsg = self.cmd_queue.get_nowait()
            cmd = struct.pack(self.str_msgCmdFormat,cmdMsg[0],cmdMsg[1],cmdMsg[2])
            cmd += b'end'
            try:
                self.serialPort.write(cmd)
                # self.serialPort.flush()
            except:
                print("Serial port: Could not send commands to the system.")
                with self.comm_lock:
                    self.comm_state = 'comm_not_ok'

        except queue.Empty:
            pass

    def stop(self):
        # Thread termination actions.
        if self.comm_state == 'comm_ok':
            self.serialPort.close()
            with self.comm_lock:
                self.comm_state = 'comm_not_ok'            
            print("Serial port closed.\n")
            
    def comm_loop(self):

        n = struct.calcsize(self.str_msgDataFormat)
        
        # Current thread reference.
        task = threading.current_thread()
            
        while getattr(task,"do_run",True):
            
            with self.comm_lock:
                state = self.comm_state
                            
            match state:
                case 'comm_not_ok':

                    self.open_comm_port()

                case 'comm_ok':       

                    ##########################
                    ## Receive data and response messages
                    ## and store them in the respective queues:
                    ##########################
                    self.receive_message_from_serial()
        
                    ##########################
                    ## Send command messages to the embedded system by 
                    ## retrieving them from the command queue:
                    ##########################
                    self.send_message_to_serial()
                    
            # Sleep for a while to avoid overloading the CPU.
            time.sleep(_comm_handler_ts)
        
        # If we exit the loop, close the serial port.
        self.stop()
            
    def open_comm_port(self):
        
        # Try to open the serial port.
        with self.comm_lock:
            if self.comm_state == 'comm_ok':
                return
            else:
                if self.serial_portname == 'auto':
                    self.serial_portname = self.find_esp32_port()
                    
            if self.serial_portname != None:
                try:
                    self.serialPort.port = self.serial_portname
                    self.serialPort.baudrate = self.serial_baudrate
                    
                    # Non-blocking mode:
                    self.serialPort.write_timeout = 0
                    self.serialPort.timeout = 0
                    
                    # Prevents ACM/CDC devices to wait for DSR signal to reset the ESP32
                    self.serialPort.rtscts = False
                    self.serialPort.dsrdtr = False
                    
                    self.serialPort.open()
                    time.sleep(1.0) # Wait for the serial port to stabilize.
                    
                    self.serialPort.reset_output_buffer()
                    self.serialPort.reset_input_buffer()
                    
                    self.comm_state = 'comm_ok'
                        
                    print("Serial port opened successfully.")
                    
                except serial.SerialException as e:
                    self.comm_state = 'comm_not_ok'
                    self.serial_portname = 'auto' # Revert to automatic detection of the ESP32 port.
                    print(f"Could not open the serial port.\nException: {e}\n")
                    print("Reverting to automatic detection of the ESP32 port.\n")                    
            else:
                self.serial_portname = 'auto' # Revert to automatic detection of the ESP32 port.
                time.sleep(1.0) # Wait a bit before trying again.
    
    def get_data_count(self):
        with self.data_lock:
            count = self.data_count
        return count # In Python, integers are immutable, so this is safe because it is passed by value.
            
    def get_data_array(self,rows=None):
        with self.data_lock:
            if rows is None:
                data = self.data_array[0:self.data_count,:]
            else:
                data = self.data_array[rows,:]
                
        return data
    
    def get_os_name(self):
        """
        Gemini LLM generated code:
        Determines the name of the operating system.

        Returns:
            str: A string indicating the operating system ('Linux', 'Windows', 'macOS', or 'Unknown').
        """
        platform_name = sys.platform
        if platform_name.startswith('linux'):
            return 'Linux'
        elif platform_name == 'win32':
            return 'Windows'
        elif platform_name == 'darwin':
            return 'macOS'
        else:
            return 'Unknown'
        