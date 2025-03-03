import os
import sys
import time
import numpy as np
import cv2
import serial.tools.list_ports
import inspect

# Import the Boson SDK
try:
    from BosonSDK.ClientFiles_Python import Client_API as CamAPI
    from BosonSDK.ClientFiles_Python.EnumTypes import FLR_ENABLE_E, FLR_CAPTURE_SRC_E, FLR_COLORLUT_ID_E
    from BosonSDK.ClientFiles_Python.ReturnCodes import FLR_RESULT
except ImportError:
    print("ERROR: Boson SDK not found or not installed properly.")
    print("Make sure the BosonSDK package is in your Python path.")
    sys.exit(1)

def get_camera_resolution(camera):
    """Get the camera resolution from various potential sources."""
    
    # Option 1: Check if camera has direct resolution methods
    resolution_methods = [method for method in dir(camera) 
                         if any(term in method.lower() for term in ["resolution", "dimension", "width", "height"])]
    
    if resolution_methods:
        print(f"Found potential resolution methods: {resolution_methods}")
        
    # Option 2: Get sensor info which might contain resolution
    try:
        # Get camera info that might contain resolution
        _, camera_pn = camera.bosonGetCameraPN()
        print(f"Camera part number: {camera_pn}")
        
        # Many Boson cameras have resolution in the part number
        # Common resolutions: 320x256, 640x512
        if hasattr(camera_pn, 'value') and '640' in str(camera_pn.value):
            return 640, 512
        elif hasattr(camera_pn, 'value') and '320' in str(camera_pn.value):
            return 320, 256
    except Exception as e:
        print(f"Error getting camera info: {e}")
    
    # Option 3: Get resolution from gaoSetOption
    try:
        # These are commonly used options for Boson cameras
        resolution_options = camera.LookupAPI("gaoGetOption")
        if resolution_options:
            print(f"Found GAO options: {resolution_options}")
            # Try to get height and width options if available
            # Common option IDs for resolution are around 0x0100-0x0103
    except Exception as e:
        print(f"Error accessing options: {e}")
    
    # Option 4: Try other SDK-specific info methods
    try:
        # Check for methods related to sensor info
        sensor_methods = camera.LookupAPI("sensor")
        if sensor_methods:
            print(f"Found sensor methods: {sensor_methods}")
            for method in sensor_methods:
                if "size" in method.lower() or "resolution" in method.lower():
                    result = getattr(camera, method)()
                    print(f"Result of {method}: {result}")
                    # Parse result for width/height
    except Exception as e:
        print(f"Error with sensor methods: {e}")
    
    # If all else fails, you might need to try the most common resolutions
    # based on the Boson camera models
    print("Could not determine resolution automatically, using default 640x512")
    return 640, 512  # Most common Boson resolution as fallback

def connect_camera():
    """Connect to the camera and return the camera object."""
    try:
        # Use the specific port provided by the user
        specific_port = "/dev/cu.usbmodem3870193"
        print(f"Using specified port: {specific_port}")
        
        # Import the Boson SDK classes
        from BosonSDK.ClientFiles_Python import Client_API as CamAPI
        from BosonSDK.ClientFiles_Python.EnumTypes import FLR_ENABLE_E
        
        # Connect to camera using the specific port
        try:
            camera = CamAPI.pyClient(manualport=specific_port, manualbaud=921600)
            print(f"Connected to camera on port {specific_port}")
        except Exception as e:
            print(f"Failed to connect on port {specific_port}: {e}")
            return None
        
        # Get camera resolution
        width, height = get_camera_resolution(camera)
        print(f"Camera resolution: {width}x{height}")
        
        # Store resolution with the camera object for later use
        camera.width = width
        camera.height = height
        
        return camera
    except Exception as e:
        print(f"Error connecting to camera: {e}")
        return None

def capture_frames(camera):
    """Capture frames from the camera using OpenCV's VideoCapture and cycle through colorLUTs."""
    if not camera:
        print("No camera connected")
        return

    try:
        # Run FFC (Flat Field Correction)
        camera.bosonRunFFC()
        print("FFC completed")
        
        # Enable colorLUT
        camera.colorLutSetControl(FLR_ENABLE_E.FLR_ENABLE)
        print("ColorLUT enabled")
        
        # Available colorLUTs from the SDK
        colorluts = [
            (FLR_COLORLUT_ID_E.FLR_COLORLUT_WHITEHOT, "WHITEHOT"),
            (FLR_COLORLUT_ID_E.FLR_COLORLUT_BLACKHOT, "BLACKHOT"),
            (FLR_COLORLUT_ID_E.FLR_COLORLUT_RAINBOW, "RAINBOW"),
            (FLR_COLORLUT_ID_E.FLR_COLORLUT_RAINBOW_HC, "RAINBOW HC"),
            (FLR_COLORLUT_ID_E.FLR_COLORLUT_IRONBOW, "IRONBOW"),
            (FLR_COLORLUT_ID_E.FLR_COLORLUT_LAVA, "LAVA"),
            (FLR_COLORLUT_ID_E.FLR_COLORLUT_ARCTIC, "ARCTIC"),
            (FLR_COLORLUT_ID_E.FLR_COLORLUT_GLOBOW, "GLOBOW"),
            (FLR_COLORLUT_ID_E.FLR_COLORLUT_GRADEDFIRE, "GRADEDFIRE"),
            (FLR_COLORLUT_ID_E.FLR_COLORLUT_HOTTEST, "HOTTEST")
        ]
        
        current_lut_index = 0
        frame_count = 0
        
        # Set initial colorLUT
        lut_id, lut_name = colorluts[current_lut_index]
        camera.colorLutSetId(lut_id)
        print(f"Set initial ColorLUT to {lut_name}")
        
        # Initialize video capture
        cap = cv2.VideoCapture(0)  # Try default camera
        if not cap.isOpened():
            print("Failed to open camera with index 0, trying index 1")
            cap = cv2.VideoCapture(1)  # Try alternative camera index
            
        if not cap.isOpened():
            print("Failed to open camera with OpenCV")
            return
        
        print("Camera opened successfully with OpenCV")
        
        while True:
            # Capture frame from OpenCV
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            
            # Increment frame counter
            frame_count += 1
            
            # Change colorLUT every 30 frames
            if frame_count % 30 == 0:
                current_lut_index = (current_lut_index + 1) % len(colorluts)
                lut_id, lut_name = colorluts[current_lut_index]
                camera.colorLutSetId(lut_id)
                print(f"Changed ColorLUT to {lut_name}")
            
            # Add text showing current colorLUT
            cv2.putText(frame, f"ColorLUT: {colorluts[current_lut_index][1]}", 
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Display the frame
            cv2.imshow("Boson Camera", frame)
            
            # Break loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        
    except Exception as e:
        print(f"Error during frame capture: {e}")
        import traceback
        traceback.print_exc()

def get_frame_direct(camera, width, height):
    """Get frame directly from camera with focused approach for code 518 (DATA_SIZE_ERROR)."""
    try:
        # First get the capture size to confirm dimensions
        result = camera.memGetCaptureSize()
        print(f"memGetCaptureSize returned: {result}")
        
        # If result has 4 values, update the resolution
        if isinstance(result, tuple) and len(result) >= 4:
            returnCode, size, actual_height, actual_width = result
            print(f"Camera reports resolution: {actual_width}x{actual_height}, size: {size} bytes")
            
            # Update width and height if different from what's provided
            if width != actual_width or height != actual_height:
                print(f"Updating resolution from {width}x{height} to {actual_width}x{actual_height}")
                width, height = actual_width, actual_height
        
        # Import the necessary enumeration for capture source
        from BosonSDK.ClientFiles_Python.ReturnCodes import FLR_RESULT
        
        # Step 1: Capture a single frame first
        if hasattr(camera, 'captureSingleFrame'):
            print("Capturing single frame...")
            returnCode = camera.captureSingleFrame()
            if hasattr(returnCode, 'value') and returnCode.value == 0:
                print("Frame capture succeeded")
                import time
                time.sleep(0.5)  # Wait for capture to complete
            else:
                print(f"Frame capture failed with code: {returnCode}")
        
        # Step 2: Try to read the frame with exact parameters from reference
        if hasattr(camera, 'memReadCapture'):
            buffer_num = 0    # Buffer number (UCHAR, 1 byte)
            offset = 0        # Offset (UINT_32, 4 bytes)
            
            # IMPORTANT: sizeInBytes is UINT_16 (2 bytes), so max is 65535
            # This might be causing our DATA_SIZE_ERROR if we're asking for too much
            
            # We might need multiple reads if the frame is large
            # Start with a small read to test
            max_size_per_read = 65000  # Just under UINT_16 max
            
            if size <= max_size_per_read:
                # We can read it all at once
                print(f"Reading {size} bytes in one operation")
                returnCode, data = camera.memReadCapture(buffer_num, offset, size)
                
                if hasattr(returnCode, 'value') and returnCode.value == 0 and data is not None:
                    print(f"Successfully read {len(data)} bytes of image data")
                    return process_frame_data(data, width, height)
                else:
                    print(f"Full read failed with code: {returnCode}")
            else:
                # We need multiple reads
                print(f"Frame size {size} exceeds max read size, using multiple reads")
                all_data = bytearray()
                
                for current_offset in range(0, size, max_size_per_read):
                    bytes_to_read = min(max_size_per_read, size - current_offset)
                    print(f"Reading {bytes_to_read} bytes at offset {current_offset}")
                    
                    returnCode, chunk = camera.memReadCapture(buffer_num, current_offset, bytes_to_read)
                    
                    if hasattr(returnCode, 'value') and returnCode.value == 0 and chunk is not None:
                        print(f"Read {len(chunk)} bytes successfully")
                        all_data.extend(chunk)
                    else:
                        print(f"Partial read failed with code: {returnCode}")
                        break
                
                if len(all_data) == size:
                    print(f"Successfully read complete frame ({len(all_data)} bytes)")
                    return process_frame_data(all_data, width, height)
                else:
                    print(f"Incomplete read: got {len(all_data)} of {size} bytes")
            
            # If standard read failed, try with different size values
            if hasattr(FLR_RESULT, 'FLR_DATA_SIZE_ERROR'):
                error_518 = FLR_RESULT.FLR_DATA_SIZE_ERROR.value
                print(f"Checking if error is DATA_SIZE_ERROR (518): {error_518}")
                
                # Try different sizes in case it's a sizing issue
                test_sizes = [
                    width * height * 2,  # 16-bit data
                    width * height,      # 8-bit data
                    512,                 # Max size mentioned in reference
                    256,                 # Half max size
                    65000,               # Just under UINT_16 max
                    32768                # Half of UINT_16 max
                ]
                
                for test_size in test_sizes:
                    if test_size > 65535:  # Skip if too large for UINT_16
                        continue
                        
                    print(f"Trying with size: {test_size}")
                    returnCode, data = camera.memReadCapture(buffer_num, offset, test_size)
                    
                    if hasattr(returnCode, 'value') and returnCode.value == 0 and data is not None:
                        print(f"Success with size {test_size}, read {len(data)} bytes")
                        # If we got less than a full frame, attempt to build the rest
                        if len(data) < size:
                            all_data = bytearray(data)
                            remaining = size - len(data)
                            print(f"Attempting to read remaining {remaining} bytes")
                            
                            for current_offset in range(len(data), size, test_size):
                                bytes_to_read = min(test_size, size - current_offset)
                                returnCode, chunk = camera.memReadCapture(buffer_num, current_offset, bytes_to_read)
                                
                                if hasattr(returnCode, 'value') and returnCode.value == 0 and chunk is not None:
                                    all_data.extend(chunk)
                                else:
                                    break
                            
                            if len(all_data) >= width * height:
                                return process_frame_data(all_data, width, height)
                        else:
                            return process_frame_data(data, width, height)
                    else:
                        print(f"Test size {test_size} failed with code: {returnCode}")
                        
        # Try alternative methods if memReadCapture failed
        print("Trying other methods to get the frame...")
        other_methods = [
            'captureGetCapturedBuffer',
            'dvoGetFrame',
            'bosonGetFrameData',
            'memRead'
        ]
        
        for method_name in other_methods:
            if hasattr(camera, method_name):
                try:
                    print(f"Trying method: {method_name}")
                    method = getattr(camera, method_name)
                    
                    # Different methods may take different parameters
                    if method_name == 'captureGetCapturedBuffer':
                        result = method(size)
                    elif method_name == 'memRead':
                        result = method(0, size)
                    else:
                        result = method()
                    
                    if isinstance(result, tuple) and len(result) >= 2:
                        returnCode, frameData = result[:2]
                        if hasattr(returnCode, 'value') and returnCode.value == 0 and frameData is not None:
                            return process_frame_data(frameData, width, height)
                except Exception as e:
                    print(f"Method {method_name} error: {e}")
                
    except Exception as e:
        print(f"Exception during frame capture: {e}")
        import traceback
        traceback.print_exc()
    
    return None

def process_frame_data(data, width, height):
    """Process raw frame data into a numpy array with better type handling."""
    print(f"Processing data of type: {type(data)}, length: {len(data) if hasattr(data, '__len__') else 'unknown'}")
    
    if data is None:
        print("Received None data")
        return None
        
    if isinstance(data, (bytes, bytearray)):
        if len(data) >= width * height * 2:  # 16-bit data
            print(f"Creating 16-bit image with shape ({height}, {width})")
            return np.frombuffer(data, dtype=np.uint16).reshape((height, width))
        elif len(data) >= width * height:  # 8-bit data
            print(f"Creating 8-bit image with shape ({height}, {width})")
            return np.frombuffer(data, dtype=np.uint8).reshape((height, width))
        else:
            print(f"Data length {len(data)} is insufficient for {width}x{height} image")
            return None
            
    elif isinstance(data, np.ndarray):
        if data.size >= width * height:
            print(f"Reshaping numpy array to ({height}, {width})")
            return data.reshape((height, width))
        else:
            print(f"Array size {data.size} is insufficient for {width}x{height} image")
            return None
            
    print(f"Unknown data type: {type(data)}")
    return None

def main():
    # Connect to camera
    camera = connect_camera()
    
    # Start capturing frames
    if camera:
        capture_frames(camera)

if __name__ == "__main__":
    main()
