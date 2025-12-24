# VR Teleoperation Module

VR teleoperation module to get controller data for robot control

## Connection Flow

```
Quest 3/3S Headset
       ↓
   WebXR Browser
       ↓
HTTPS Server (port 8881)
       ↓
   WebSocket (/ws)
       ↓
  Controller Data
  (90Hz streaming)
       ↓
   dimos Streams
       ↓
  Robot Control
```

**Data Flow**: Quest controllers → WebXR API → Three.js → WebSocket → dimos streams → robot

## Quick Start

### Standalone Server
```bash
source .venv/bin/activate
python dimos/vr/run_vr_server.py
```

### DimOS Module Integration
```python
import dimos.core as core
from dimos.vr.modules import MetaQuestModule

dimos = core.start(1)
quest = dimos.deploy(MetaQuestModule, port=8881)
quest.generate_certificate()
quest.start()

# Subscribe to controller streams
quest.controller_left.observable().subscribe(lambda data: handle_left_controller(data))
quest.controller_right.observable().subscribe(lambda data: handle_right_controller(data))
quest.controller_both.observable().subscribe(lambda frame: handle_both_controllers(frame))
```

### With Camera Streaming
```python
import dimos.core as core
from dimos.vr.modules import MetaQuestModule
from dimos.hardware.camera import CameraModule, Webcam

dimos = core.start(1)

# Setup camera
camera = dimos.deploy(CameraModule, hardware=Webcam)
camera.start()

# Deploy VR with camera feeds
quest = dimos.deploy(
    MetaQuestModule,
    port=8881,
    camera_streams={
        'front_camera': camera.image.observable()
    }
)
quest.generate_certificate()
quest.start()

# Subscribe to controller streams as usual
quest.controller_both.observable().subscribe(lambda frame: handle_controllers(frame))
```

### With Stereo Camera (ZED)
```python
import dimos.core as core
from dimos.vr.modules import MetaQuestModule
from dimos.hardware.camera.zed import ZEDModule

dimos = core.start(1)

# Setup ZED camera
zed = dimos.deploy(ZEDModule, camera_id=0)
zed.start()

# Deploy VR with stereo camera feeds
quest = dimos.deploy(
    MetaQuestModule,
    port=8881,
    camera_streams={
        'left_camera': zed.color_image.observable(),
        # Can add depth stream as well
        'depth': zed.depth_image.observable()
    }
)
quest.generate_certificate()
quest.start()
```

## Reference Output

The server logs complete controller data at 90Hz. Example output:

```
============================================================
Frame #18.344
============================================================

LEFT CONTROLLER: Not connected

RIGHT CONTROLLER:
  Connected: True
  Position (m): x=0.123, y=1.456, z=-0.789
  Rotation (quat): x=0.123, y=0.456, z=0.789, w=0.987
  Rotation (deg): x=15.2, y=-23.4, z=45.6
  Buttons:
    Trigger: 0.85 PRESSED
    Grip: 0.00 released
    Menu: 0.00 released
    Thumbstick: 0.00 released
    X/A: 1.00 PRESSED
    Y/B: 0.00 released
  Thumbstick: x=0.234, y=-0.567
  Raw button values: ['0.85', '0.00', '0.00', '0.00', '1.00', '0.00']
```

**Frame Rate**: 90Hz (11ms intervals)  
**Data Fields**: Position, rotation (quaternion + euler), 6 buttons, thumbstick axes  
**Connection Status**: Per-controller connect/disconnect detection

## Data Access Guide

### Complete Controller Frame
```python
def handle_frame(frame: ControllerFrame):
    timestamp = frame.timestamp  # Unix timestamp
    
    # Check which controllers are connected
    if frame.left:
        print("Left controller connected")
    if frame.right: 
        print("Right controller connected")
```

### Position Data (Meters)
```python
def get_position(controller: ControllerData):
    x, y, z = controller.position
    print(f"Position: {x:.3f}m, {y:.3f}m, {z:.3f}m")
    
    # Individual access
    x_pos = controller.position[0]
    y_pos = controller.position[1] 
    z_pos = controller.position[2]
```

### Rotation Data
```python
def get_rotation(controller: ControllerData):
    # Quaternion (for 3D math)
    qx, qy, qz, qw = controller.rotation
    
    # Euler angles (degrees, human readable)
    pitch, yaw, roll = controller.rotation_euler
    print(f"Rotation: pitch={pitch:.1f}°, yaw={yaw:.1f}°, roll={roll:.1f}°")
```

### Button States
```python
def get_buttons(controller: ControllerData):
    buttons = controller.buttons
    
    # Individual button access
    trigger_value = buttons.trigger.value      # 0.0 to 1.0
    trigger_pressed = buttons.trigger.pressed  # True/False
    trigger_touched = buttons.trigger.touched  # True/False
    
    # All buttons
    if buttons.trigger.pressed:
        print(f"Trigger pressed: {buttons.trigger.value:.2f}")
    if buttons.grip.pressed:
        print("Grip pressed")
    if buttons.menu.pressed:
        print("Menu pressed")
    if buttons.thumbstick.pressed:
        print("Thumbstick clicked")
    if buttons.x_or_a.pressed:
        print("X/A button pressed")
    if buttons.y_or_b.pressed:
        print("Y/B button pressed")
        
    # Raw button array (same order)
    raw_values = controller.button_values  # [trigger, grip, menu, thumbstick, x_or_a, y_or_b]
```

### Thumbstick Input
```python
def get_thumbstick(controller: ControllerData):
    x_axis = controller.axes.thumbstick_x  # -1.0 to 1.0
    y_axis = controller.axes.thumbstick_y  # -1.0 to 1.0
    
    # Movement detection
    if abs(x_axis) > 0.1 or abs(y_axis) > 0.1:
        print(f"Thumbstick: x={x_axis:.3f}, y={y_axis:.3f}")
```

## Integration Examples

### Robot Movement Control
```python
def control_robot(frame: ControllerFrame):
    if frame.right:  # Use right controller
        right = frame.right
        
        # Forward/backward from thumbstick
        forward_speed = right.axes.thumbstick_y
        
        # Turn left/right from thumbstick
        turn_speed = right.axes.thumbstick_x
        
        # Boost with trigger
        boost = right.buttons.trigger.value
        
        # Apply movement
        move_robot(forward_speed * boost, turn_speed * boost)
```

### Object Manipulation
```python
def manipulate_object(frame: ControllerFrame):
    if frame.left:
        left = frame.left
        
        # Get 3D position for target
        target_x, target_y, target_z = left.position
        
        # Grip control
        if left.buttons.grip.pressed:
            grasp_object(target_x, target_y, target_z)
        else:
            release_object()
```

### Menu Navigation
```python
def handle_menu(frame: ControllerFrame):
    if frame.right:
        right = frame.right
        
        if right.buttons.menu.pressed:
            toggle_menu()
        if right.buttons.x_or_a.pressed:
            select_item()
        if right.buttons.y_or_b.pressed:
            back_button()
```

## Connection Setup

1. **Start Server**
   ```bash
   source .venv/bin/activate
   python dimos/vr/run_vr_server.py
   ```

2. **Connect Quest 3**
   - Note server IP from terminal output
   - Open Quest browser
   - Navigate to `https://<server_ip>:8881`
   - Accept SSL certificate warning
   - Press "Enter VR" button

3. **Controller Activation**
   - **First-time setup**: If controllers don't appear immediately, press the Meta button to exit VR mode, then re-enter by pressing "Enter VR" again. This refreshes the WebXR controller detection (a known Meta browser bug).
   - Controllers will automatically connect once VR mode is active

4. **Data Streaming**
   - Controllers automatically detected at 90Hz
   - Full data logged to terminal
   - Available on dimos streams for module integration

## Output Streams

- `quest.controller_left` → `ControllerData` for left hand
- `quest.controller_right` → `ControllerData` for right hand
- `quest.controller_both` → `ControllerFrame` with both controllers

## Camera Streaming

The VR module supports streaming camera feeds to the Quest 3 headset for display in VR.

### Features

- **Multi-camera support**: Stream multiple cameras simultaneously
- **Monocular & stereo**: Works with Webcam and ZED cameras
- **Low latency**: MJPEG streaming over HTTPS
- **VR display**: Cameras appear as 3D video planes in VR space
- **Interactive control**: Toggle camera visibility with Y/B controller button

### Camera Feed Setup

Pass camera observables to `MetaQuestModule` via the `camera_streams` parameter:

```python
quest = dimos.deploy(
    MetaQuestModule,
    port=8881,
    camera_streams={
        'camera_name': camera.image.observable(),
        'another_camera': camera2.image.observable()
    },
    jpeg_quality=85  # Optional: adjust JPEG compression (1-100)
)
```

### VR Display

- Cameras appear as video planes in VR at eye level
- Multiple cameras are arranged horizontally
- Default position: 2.0m in front, 1.5m height
- Semi-transparent black backdrop for contrast

**Controls:**
- **On-screen buttons**: "VR Controls" panel (visible in browser)
- **Y/B button**: Toggle camera visibility (in VR)
- **X/A button**: Toggle passthrough mode (in VR)

### API Endpoints

- `GET /camera/list` - List available camera streams
- `GET /camera_feed/{camera_key}` - MJPEG stream for specific camera
- `GET /health` - Server health check (includes camera list)

### Performance Tuning

```python
quest = dimos.deploy(
    MetaQuestModule,
    camera_streams=cameras,
    jpeg_quality=75  # Lower = smaller files, faster, lower quality
)
```

Recommended quality settings:
- **85** (default): High quality, balanced
- **75**: Good quality, better performance
- **60**: Lower quality, maximum performance

### Passthrough Mode

Quest 3 passthrough is enabled by default using WebXR AR mode:

- **Default mode**: `immersive-ar` - Passthrough ON, see real environment with virtual overlays
- **VR mode**: `immersive-vr` - Passthrough OFF, black background with virtual scene only
- **Implementation**: Uses `navigator.xr.requestSession('immersive-ar')` for passthrough
- **On-screen controls**: Use "VR Controls" panel on the right side of screen
- **Toggle methods**:
  - Click "Toggle AR/VR Mode" button on screen (restarts session with new mode)
  - Press X/A button on controller (in VR - restarts session)

### On-Screen Controls

A control panel is available on the right side of the screen:

- **Passthrough Toggle**: Switch between MR and VR modes
- **Camera Toggle**: Show/hide camera feeds (when available)
- **Status Indicators**: Green = ON, Gray = OFF
- **Controller Hints**: Shows VR button mappings

All controls work both before entering VR and during VR session.

## Files

- `models.py` - Data models (ControllerFrame, ControllerData, ButtonState)
- `generate_cert.py` - SSL certificate generator
- `run_vr_server.py` - Standalone server with full logging
- `modules/metaquest.py` - DimOS module integration
- `static/` - WebXR interface files

## Commands

- `quest.start()` - Start HTTPS server
- `quest.stop()` - Stop server  
- `quest.get_stats()` - Frame count and error stats
- `quest.generate_certificate()` - Create SSL certificates

## Requirements

- Quest 3/3S headset with WebXR support
- Quest connected to the local network as server
- HTTPS enabled (auto-configured) - Required by WebXR