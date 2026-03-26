# Architecture Diagram - ViperX Modular Design

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│         Behavior Tree (customer_viperx.py)              │
│                                                         │
│  For each item:                                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │ SetCurrentItem                                   │   │
│  │ ├─ Read item from order                          │   │
│  │ └─ Set bb.current_item                           │   │
│  └──────────────────────────────────────────────────┘   │
│           ↓                                              │
│  ┌──────────────────────────────────────────────────┐   │
│  │ NavigateToGoalPose (shelf)                       │   │
│  │ └─ Uses BB: nav_goal                             │   │
│  └──────────────────────────────────────────────────┘   │
│           ↓                                              │
│  ┌──────────────────────────────────────────────────┐   │
│  │ VerifyViperXPosition   [SERVICE-BASED]           │   │
│  │ ├─ Call /robot_vision/detect_object              │   │
│  │ └─ Set BB: detected_object_pose                  │   │
│  └──────────────────────────────────────────────────┘   │
│           ↓                                              │
│  ┌──────────────────────────────────────────────────┐   │
│  │ MoveToDetectedPose     [NEW NODE]                │   │
│  │ ├─ Read BB: detected_object_pose                 │   │
│  │ └─ Execute PickArm action to object              │   │
│  └──────────────────────────────────────────────────┘   │
│           ↓                                              │
│  ┌──────────────────────────────────────────────────┐   │
│  │ MoveViperXGripper (close)                        │   │
│  │ └─ Execute PickArm gripper action                │   │
│  └──────────────────────────────────────────────────┘   │
│           ↓                                              │
│  ┌──────────────────────────────────────────────────┐   │
│  │ SelectBasketSlot        [NEW NODE]               │   │
│  │ ├─ Check item type (is_bottle?)                  │   │
│  │ ├─ If bottle: slot = basket_bottle_count (0-2)  │   │
│  │ ├─ If other: slot = 3                           │   │
│  │ ├─ Increment basket_bottle_count                │   │
│  │ └─ Set BB: basket_pose                          │   │
│  └──────────────────────────────────────────────────┘   │
│           ↓                                              │
│  ┌──────────────────────────────────────────────────┐   │
│  │ RepositionViperXArm (to basket)                  │   │
│  │ ├─ Read BB: basket_pose                          │   │
│  │ └─ Execute PickArm action                        │   │
│  └──────────────────────────────────────────────────┘   │
│           ↓                                              │
│  ┌──────────────────────────────────────────────────┐   │
│  │ MoveViperXGripper (open)                         │   │
│  │ └─ Execute PickArm gripper action                │   │
│  └──────────────────────────────────────────────────┘   │
│           ↓                                              │
│  ┌──────────────────────────────────────────────────┐   │
│  │ ChangeInventory                                  │   │
│  │ └─ POST to inventory API                         │   │
│  └──────────────────────────────────────────────────┘   │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## Data Flow - Blackboard Communication

```
SetCurrentItem
  ├─ Reads: Order items[]
  ├─ Writes: current_item, nav_goal, rack_goal
  └─ Validates: item exists in order

      ↓

NavigateToGoalPose (handled by Nav2)
  ├─ Reads: nav_goal = (x, y, 0.0)
  └─ Result: At shelf location

      ↓

VerifyViperXPosition [NEW SERVICE PATTERN]
  ├─ Reads: (nothing - internal)
  ├─ Calls: /robot_vision/detect_object
  ├─ Gets: Response {x, y, z, distance_m}
  └─ Writes: detected_object_pose = {x, y, z, frame_id, distance}

      ↓

MoveToDetectedPose [NEW]
  ├─ Reads: detected_object_pose
  ├─ Calls: /pick_viperx action
  └─ Result: Arm at object location

      ↓

MoveViperXGripper(close)
  └─ Calls: /pick_viperx action with planning_group="close_gripper"

      ↓

SelectBasketSlot [NEW]
  ├─ Reads: current_item.name, basket_bottle_count, basket_poses[]
  ├─ Logic: 
  │   if("bottle" in name.lower()):
  │       slot = basket_bottle_count  # 0, 1, or 2
  │       basket_bottle_count++
  │   else:
  │       slot = 3  # random items slot
  ├─ Writes: basket_pose = basket_poses[slot]
  └─ Validates: basket_pose is not None

      ↓

RepositionViperXArm(basket_pose)
  ├─ Reads: basket_pose from blackboard
  └─ Calls: /pick_viperx action

      ↓

MoveViperXGripper(open)
  └─ Calls: /pick_viperx action with planning_group="open_gripper"

      ↓

ChangeInventory
  ├─ Reads: current_item.product_id, current_item.qty
  └─ Calls: POST /api/inventory/decrement
```

## Service Interface

### Vision Service (New)
```
Service: /robot_vision/detect_object
├─ Request: {} (or optional current arm pose)
└─ Response:
   ├─ x: float        (meters, in vx300/base_link frame)
   ├─ y: float        (meters, in vx300/base_link frame)
   ├─ z: float        (meters, in vx300/base_link frame)
   └─ distance_m: float (meters, from camera to object)
```

### Manipulation Action (Existing)
```
Action: /pick_viperx (PickArm)
├─ Goal:
│  ├─ planning_group: "arm" | "open_gripper" | "close_gripper"
│  ├─ target_pose: PoseStamped
│  └─ gripper_close_position: float
└─ Result:
   └─ status: GoalStatus (SUCCEEDED/FAILED)
```

## Bottle Counting Example

```
Order: [Water Bottle, Apple, Water Bottle, Water Bottle, Chips]

Processing:
1. Water Bottle #1
   ├─ SelectBasketSlot checks: "water bottle".contains("bottle")? YES
   ├─ Line: slot = basket_bottle_count (now 0)
   ├─ Action: basket_bottle_count++ → 1
   └─ Result: Store in basket_poses[0]

2. Apple
   ├─ SelectBasketSlot checks: "apple".contains("bottle")? NO
   ├─ Line: slot = 3
   └─ Result: Store in basket_poses[3]

3. Water Bottle #2
   ├─ SelectBasketSlot checks: "water bottle".contains("bottle")? YES
   ├─ Line: slot = basket_bottle_count (now 1)
   ├─ Action: basket_bottle_count++ → 2
   └─ Result: Store in basket_poses[1]

4. Water Bottle #3
   ├─ SelectBasketSlot checks: "water bottle".contains("bottle")? YES
   ├─ Line: slot = basket_bottle_count (now 2)
   ├─ Action: basket_bottle_count++ → 3
   └─ Result: Store in basket_poses[2]

5. Chips
   ├─ SelectBasketSlot checks: "chips".contains("bottle")? NO
   ├─ Line: slot = 3
   └─ Result: Store in basket_poses[3] (shares with Apple)

Final State:
├─ basket_poses[0] = Water Bottle #1
├─ basket_poses[1] = Water Bottle #2
├─ basket_poses[2] = Water Bottle #3
├─ basket_poses[3] = Apple & Chips (stacked)
└─ basket_bottle_count = 3 (would reject 4th bottle)
```

## Configuration Setup

```python
# 1. Initialize blackboard in startup
bb = setup_blackboard()

# 2. Set basket physical locations (from calibration)
bb.basket_poses = [
    {
        "frame_id": "vx300/base_link",
        "x": 0.35,    # Bottle slot 1
        "y": 0.25,
        "z": 0.20,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
    },
    {
        "frame_id": "vx300/base_link",
        "x": 0.35,    # Bottle slot 2
        "y": 0.0,
        "z": 0.20,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
    },
    {
        "frame_id": "vx300/base_link",
        "x": 0.35,    # Bottle slot 3
        "y": -0.25,
        "z": 0.20,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
    },
    {
        "frame_id": "vx300/base_link",
        "x": 0.15,    # Random items slot
        "y": 0.35,
        "z": 0.15,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
    }
]

# 3. Set arm poses
bb.home_pose = {...}  # When arm is "at home"
bb.pose = {...}       # Observation point to view objects

# 4. Set shelf heights (will use during testing)
bb.shelf_height = None  # To be set per order

# 5. Reset counter for new order
bb.basket_bottle_count = 0
```

## Decision Tree - Item Classification

```
SelectBasketSlot()
├─ Get current_item.name
├─ Convert to lowercase
├─ Check: "bottle" in name OR "water" in name?
│  ├─ YES: Use bottle slots
│  │   ├─ Is basket_bottle_count < 3?
│  │   │  ├─ YES: slot = basket_bottle_count, increment
│  │   │  └─ NO: FAILURE (all slots full)
│  │   └─ Result: basket_pose = basket_poses[0-2]
│  │
│  └─ NO: Use random items slot
│      └─ Result: basket_pose = basket_poses[3]
│
└─ Validate basket_pose is not None
```

