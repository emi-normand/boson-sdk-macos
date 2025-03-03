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
    
    # If all else fails, use the most common Boson resolution
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

def main():
    # Connect to camera
    camera = connect_camera()
    
    # Start capturing frames
    if camera:
        capture_frames(camera)

if __name__ == "__main__":
    main()
