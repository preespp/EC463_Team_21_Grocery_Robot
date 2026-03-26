# ViperX Modular Leaf Node Refactor - Implementation Guide

## Overview
This refactor transforms the ViperX arm control into a modular, service-based architecture where each behavior tree leaf node performs a single, well-defined action instead of combining detect→move→grab into one monolithic action.

## Changes Made

### 1. Blackboard Enhancements (`blackboard.py`)

Added new blackboard variables for basket management and shelf configuration:

```python
# Basket management (3 bottle slots + 1 random slot)
bb.basket_poses = [None, None, None, None]  # Bottle slot 1, 2, 3, Random items slot
bb.basket_bottle_count = 0  # Track how many bottles picked
bb.basket_pose = None  # Currently selected basket pose

# Shelf height configuration (3 levels)
bb.shelf_height = None  # Height parameter (1, 2, or 3)
bb.shelf_pose = None  # Shelf pose with x,y empty (only z/height used)

# Object detection results
bb.detected_object_pose = None  # Pose from vision service
```

**Usage Pattern:**
- `bb.basket_bottle_count` increments as each bottle is picked (max 3)
- `bb.shelf_height` set during testing to indicate which shelf level (1-3)
- `bb.detected_object_pose` filled by `VerifyViperXPosition` node

### 2. Modified Nodes

#### `VerifyViperXPosition` - Service-Based Detection
**Old behavior:** Subscribed to camera topic and processed detections in real-time
**New behavior:** Calls `robot_vision` service to get object detection

```python
class VerifyViperXPosition(py_trees.behaviour.Behaviour):
    """
    Requests object detection from robot_vision service.
    Stores result in bb.detected_object_pose for next node to use.
    """
```

**Usage:**
```python
bt_nodes.VerifyViperXPosition(service_name="/robot_vision/detect_object")
```

**Expected Service Response Format:**
```python
# Response from /robot_vision/detect_object service
response.x              # Object x coordinate
response.y              # Object y coordinate  
response.z              # Object z coordinate
response.distance_m     # Distance to object
```

### 3. New Leaf Nodes

#### `SelectBasketSlot` - Smart Basket Position Selection
Selects basket slot based on item type with automatic bottle tracking.

```python
class SelectBasketSlot(py_trees.behaviour.Behaviour):
    """
    Logic:
    - For bottles: Use slots 0, 1, 2 sequentially, increment bb.basket_bottle_count
    - For non-bottles (random items): Use slot 3
    
    Sets bb.basket_pose to selected position.
    """
```

**Preconditions:**
- `bb.current_item` must be set (by `SetCurrentItem`)
- `bb.basket_poses[slot]` must have valid pose coordinates

**Item Classification:**
- **Bottle:** Item name contains "bottle" or "water"
- **Other:** Everything else

#### `MoveToDetectedPose` - Move to Vision-Detected Object
Executes arm movement to the object pose detected by `VerifyViperXPosition`.

```python
class MoveToDetectedPose(py_trees.behaviour.Behaviour):
    """
    Precondition:
    - bb.detected_object_pose must be populated by VerifyViperXPosition
    
    Uses PickArm action to move arm to detected object pose.
    """
```

**Preconditions:**
- `bb.detected_object_pose` must be set by `VerifyViperXPosition`

## Updated Tree Structure (`customer_viperx.py`)

### Workflow
```
Customer Picking Task
├── For each item:
│   ├── SetCurrentItem (load next item from order)
│   ├── NavigateToGoalPose (go to shelf location)
│   │
│   ├── DetectAndApproach (Sequence)
│   │   ├── RepositionViperXArm (go to observation pose from bb.pose)
│   │   └── VerifyViperXPosition (detect object → bb.detected_object_pose)
│   │
│   ├── MoveToDetectedPose (move to detected object)
│   ├── MoveViperXGripper (close - grab object)
│   │
│   ├── PlaceInBasket (Sequence)
│   │   ├── SelectBasketSlot (pick slot based on item)
│   │   ├── RepositionViperXArm (move to basket)
│   │   └── MoveViperXGripper (open - release item)
│   │
│   ├── RepositionViperXArm (return to home pose)
│   └── ChangeInventory (update database)
│
├── SetHome (prepare for navigation home)
├── NavigateToGoalPose (return to home location)
└── Complete
```

## Configuration & Testing Guide

### Before First Run

**1. Initialize Basket Poses in Blackboard:**
```python
# During robot calibration/startup
bb.basket_poses = [
    {"x": 0.3, "y": 0.2, "z": 0.15, "frame_id": "vx300/base_link"},  # Bottle slot 1
    {"x": 0.3, "y": 0.0, "z": 0.15, "frame_id": "vx300/base_link"},   # Bottle slot 2
    {"x": 0.3, "y": -0.2, "z": 0.15, "frame_id": "vx300/base_link"},  # Bottle slot 3
    {"x": 0.1, "y": 0.3, "z": 0.15, "frame_id": "vx300/base_link"},   # Random items slot
]
```

**2. Set Shelf Heights:**
```python
# Adjust z-coordinate based on shelf level (3 levels)
shelf_configs = {
    1: 0.5,   # Bottom shelf
    2: 0.8,   # Middle shelf  
    3: 1.1,   # Top shelf
}
bb.shelf_height = shelf_configs[current_level]
```

**3. Set Default Poses:**
```python
bb.home_pose = {"x": -0.1, "y": -0.3, "z": 0.3, "frame_id": "vx300/base_link"}
bb.pose = observation_pose_from_shelf  # Observation point to view shelf
```

### Running the Tree

```python
from robot_task_manager.trees.customer_viperx import create_customer_viperx_tree
from robot_task_manager.blackboard import setup_blackboard

bb = setup_blackboard()
# Configure all poses above
tree = create_customer_viperx_tree(bb)

# In your BT executor
tree.setup_with_descendants()
while True:
    tree.tick_once()
    if tree.status != py_trees.common.Status.RUNNING:
        break
```

## Service Interface Requirements

Your `robot_vision` and `robot_manipulation` servers need these endpoints:

### Robot Vision Service
**Service Name:** `/robot_vision/detect_object`

**Request:**
```python
# Define in your .srv file
geometry_msgs/Pose current_arm_pose  # Optional: current arm position
# or can be empty if camera always looks at same location
```

**Response:**
```python
float x              # Object x in arm frame
float y              # Object y in arm frame
float z              # Object z in arm frame
float distance_m     # Distance to object center
```

### Robot Manipulation Services
These already exist but confirming expected interface:
- `PickArm` action with `/pick_viperx` endpoint: ✓ Supported
- Gripper control via `planning_group = "open_gripper"/"close_gripper"`: ✓ Supported

## Integration Checklist

- [ ] Blackboard variables initialized
- [ ] Basket poses calibrated and set
- [ ] Shelf heights configured
- [ ] Home pose configured
- [ ] Observation pose (bb.pose) configured
- [ ] `/robot_vision/detect_object` service running
- [ ] `/pick_viperx` action server running
- [ ] Navigation to shelf location works
- [ ] Test single item pickup workflow
- [ ] Verify bottle count tracking (should increment 0→1→2→3)
- [ ] Verify random items slot used for non-bottles
- [ ] Update inventory API endpoint verified

## Troubleshooting

### VerifyViperXPosition Returns Failure
- Check `/robot_vision/detect_object` service is running
- Verify object is visible in camera frame
- Check detection response format matches expected fields

### SelectBasketSlot Fails with "All bottle slots full"
- This is expected after 3 bottles picked
- Remaining items should auto-select slot 3 (random items)
- Check item names contain "bottle"/"water" if miscategorized

### MoveToDetectedPose Never Receives Pose
- Ensure VerifyViperXPosition runs successfully first
- Check `bb.detected_object_pose` is being populated
- Verify response mapper correctly extracts x,y,z from service

### Basket Poses Return None
- Check `bb.basket_poses` initialized before tree execution
- Verify poses have all required fields (x, y, z, frame_id)
- Confirm poses are within arm workspace

## Key Design Patterns

### Leaf Node Communication via Blackboard
Each leaf node reads/writes to blackboard sequentially:
1. `SetCurrentItem` → reads order, writes `bb.current_item`
2. `NavigateToGoalPose` → reads `bb.nav_goal`, writes location
3. `VerifyViperXPosition` → writes `bb.detected_object_pose`
4. `MoveToDetectedPose` → reads `bb.detected_object_pose`
5. `SelectBasketSlot` → reads `bb.current_item`, writes `bb.basket_pose`

### Retry & Fallback Pattern
If needed, wrap with retry decorator:
```python
py_trees.decorators.Retry(
    name="DetectWithRetry",
    num_failures=3,
    child=bt_nodes.VerifyViperXPosition(),
)
```

## Future Enhancements

1. **Confidence Scoring:** Add confidence threshold to object detection
2. **Multi-object Handling:** If multiple objects detected, pick largest
3. **Adaptive Gripper Position:** Adjust gripper close position based on object size
4. **Fallback Poses:** If detection fails, use hardcoded shelf position
5. **Telemetry:** Log all arm movements and detection results for analysis
