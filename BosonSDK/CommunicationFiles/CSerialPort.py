# -*- coding: utf-8 -*-
"""
Serial port supported by C serial library (DLL files)
"""

import ctypes, os, sys
import _ctypes
import serial
import platform
try:
    import serial.tools.list_ports as portList
    HAS_SERIAL_LIST = True
except ImportError:
    print("Serial port list not available!")
    HAS_SERIAL_LIST = False
from .PortBase import PortBase

class CSerialPort(PortBase):
    def __init__(self, portID, baudrate=None, dllPath=None):
        if baudrate == None:
            baudrate = 921600
        super().__init__(baudrate=int(baudrate))
        self.portOpen = False
        self.readBufferSize = 2048
        self.__library = None
        self.__dllHandle = None
        
        # Determine the correct library file based on platform
        if platform.system() == "Windows":
            dll_name = "FSLP_64.dll"
        elif platform.system() == "Darwin":  # macOS
            dll_name = "FSLP_64.dylib"
        else:  # Linux and others
            dll_name = "FSLP_64.so"

        # Try to find the library file
        if dllPath:
            # If path is explicitly provided, use it
            loadpath = os.path.join(dllPath, dll_name)
        else:
            # Search in multiple possible locations
            possible_paths = [
                # Current directory
                os.path.join(os.path.dirname(__file__), "FSLP_Files", dll_name),
                # Parent directory
                os.path.join(os.path.dirname(os.path.dirname(__file__)), "FSLP_Files", dll_name),
                # Two levels up
                os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "FSLP_Files", dll_name),
                # Specific to the SDK structure
                os.path.join(os.path.dirname(os.path.dirname(__file__)), "BosonSDK", "FSLP_Files", dll_name),
            ]
            
            # Try each path until we find the library
            loadpath = None
            for path in possible_paths:
                if os.path.exists(path):
                    loadpath = path
                    break
            
            # If we didn't find it, use the default path (which will likely fail, but with a clearer error)
            if loadpath is None:
                loadpath = os.path.join(os.path.dirname(__file__), "FSLP_Files", dll_name)
                print(f"Warning: Could not find {dll_name} in any of the expected locations.")
                print(f"Tried: {possible_paths}")
                print(f"Falling back to: {loadpath}")
        
        try:
            print(f"Loading library from: {loadpath}")
            self.__library = ctypes.cdll.LoadLibrary(loadpath)
            self.__dllHandle = self.__library._handle
            self.camsend = self.__library.__getattr__("FSLP_send_to_camera")
            self.camread = self.__library.__getattr__("FSLP_read_frame")
            self.camunframed = self.__library.__getattr__("FSLP_read_unframed")
            self.port_open = self.__library.__getattr__("FSLP_open_port")
            self.port_close = self.__library.__getattr__("FSLP_close_port")
            self.lookup_port_name = self.__library.__getattr__("FSLP_lookup_port_id")
        except OSError as e:
            print("Error loading library:")
            print(f"dllPath = {dllPath}")
            print(f"filePath = {os.path.dirname(__file__)}")
            print(f"loadpath = {loadpath}")
            print(f"dllName = {dll_name}")
            raise e
        self.setPortID(portID)

    def setPortID(self, portID):
        """Set the port ID for this serial port.
        
        Args:
            portID: Can be an integer port number or a string port name
        """
        try:
            # First try to convert to integer (for Windows COM port numbers)
            portNum = int(portID)
            self.portID = portNum
            print(f"Using numeric port ID: {portNum}")
        except ValueError:
            # If it's not an integer, it's a string port name (like /dev/ttyUSB0)
            print(f"Using string port name: {portID}")
            # Convert the port name to bytes for the C function
            portBuffer = (ctypes.c_uint8 * 128)()  # Increased buffer size to 128
            
            # Convert the port name to bytes and copy to the buffer
            port_bytes = portID.encode('ascii')
            if len(port_bytes) > 127:  # Check if port name is too long
                raise ValueError(f"Port name too long (max 127 chars): {portID}")
                
            for i, dat in enumerate(port_bytes):
                portBuffer[i] = dat
            
            # Look up the port ID using the C function
            portNum = self.lookup_port_name(portBuffer, len(port_bytes))
            if portNum < 0:
                raise ValueError(f"Port {portID} not found or not registered")
            
            self.portID = portNum
            print(f"Port {portID} registered as ID: {portNum}")
    
    def open(self):
        """Open the serial port (lowercase method required by PortBase)."""
        if self.portOpen:
            return True
        
        print(f"Opening port ID: {self.portID} at {self.baudrate} baud")
        ret = self.port_open(ctypes.c_int32(self.portID), ctypes.c_int32(self.baudrate))
        if ret == 0:
            self.portOpen = True
            print(f"Port opened successfully")
            return True
        else:
            raise IOError(f"Failed to open port #{self.portID} with error {ret}!")
    
    def Open(self):
        """Alias for open() for backward compatibility."""
        return self.open()
    
    def close(self):
        """Close the serial port (lowercase method required by PortBase)."""
        if not self.portOpen:
            return
        
        print(f"Closing port ID: {self.portID}")
        self.port_close(ctypes.c_int32(self.portID))
        self.portOpen = False
        print(f"Port closed")
    
    def Close(self):
        """Alias for close() for backward compatibility."""
        return self.close()
    
    def __del__(self):
        """Destructor to ensure port is closed."""
        self.close()
        if self.__library != None:
            try:
                _ctypes.FreeLibrary(self.__dllHandle)
            except:
                pass
            self.__library = None

    def isOpen(self):
        return self.portOpen

    def isAvailable(self):
        if HAS_SERIAL_LIST:
            return self.portName in [portFromList[0] for portFromList in portList.comports()]
        else:
            return None

    def write(self, channel_ID, sendBytes, sendBuffer):
        self.camsend(ctypes.c_int32(self.portID), channel_ID, sendBytes, sendBuffer)

    def read(self, channel_ID, start_byte_ms, receiveBytes):
        receiveBuffer = (ctypes.c_uint8*self.readBufferSize)(*[0xFF]*self.readBufferSize)
        self.camread(ctypes.c_int32(self.portID), channel_ID, start_byte_ms, ctypes.byref(receiveBytes), receiveBuffer)
        return receiveBuffer

    def readUnframed(self, start_byte_ms, receiveBytes):
        receiveBuffer = (ctypes.c_uint8*self.readBufferSize)(*[0xFF]*self.readBufferSize)
        self.camunframed(ctypes.c_int32(self.portID), start_byte_ms, ctypes.byref(receiveBytes), receiveBuffer)
        return receiveBuffer
