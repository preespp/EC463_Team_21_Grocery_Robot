import py_trees
import robot_task_manager.bt_nodes as bt_nodes


def create_restock_viperx_tree(bb):
    fixed_restock_motion = py_trees.composites.Sequence(
        "FixedRestockMotion",
        memory=True,
        children=[
            bt_nodes.MoveViperXGripper(command="open", position=0.057, bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="restock_pick_ready_pose", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="restock_pick_approach_pose", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="restock_pick_pose", bb=bb),
            bt_nodes.MoveViperXGripper(command="close", position=0.041, bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="restock_post_pick_pose", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="restock_transfer_pose", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="restock_place_pose", bb=bb),
            bt_nodes.MoveViperXGripper(command="open", position=0.057, bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="restock_pick_ready_pose", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="restock_home_pose", bb=bb),
        ],
    )

    process_one_item = py_trees.composites.Sequence(
        "ProcessRestockItem",
        memory=True,
        children=[
            bt_nodes.SetCurrentItem(bb),
            fixed_restock_motion,
            bt_nodes.ChangeInventory(bb, mode="restock"),
        ],
    )

    num_items = max(1, len(getattr(bb, "items", [])))
    repeat_items = py_trees.decorators.Repeat(
        name="RepeatUntilAllItemsDone",
        child=process_one_item,
        num_success=num_items,
    )

    root = py_trees.composites.Sequence(
        "EmployeeRestockFlow",
        memory=True,
        children=[
            bt_nodes.DebugPrint("Restock ViperX hard-coded tree selected"),
            repeat_items,
            bt_nodes.DebugPrint("Restock ViperX sequence complete"),
        ],
    )

    return root
