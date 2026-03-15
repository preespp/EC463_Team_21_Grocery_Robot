import py_trees
import robot_task_manager.bt_nodes as bt_nodes
from robot_task_manager.bt_nodes.navigation_nodes import MoveDistanceForCurrentItem


def create_restock_viperx_tree(bb):

    process_one_item = py_trees.composites.Sequence("ProcessRestockItem", memory=True, children=[
        bt_nodes.SetCurrentItem(bb),
        ### Delete Later Only for Feature 1 Demo Need to Migrate to real auto nav script)
        MoveDistanceForCurrentItem(bb),
        ###############################################

        # bt_nodes.NavigateToGoalPose(goal_key="nav_goal"),
        # bt_nodes.RepositionViperXArm(goal_key="basket_pose"),

        # bt_nodes.VerifyViperXPosition(),
        # py_trees.decorators.Retry(
        #     name="AlignUntilReady",
        #     num_failures=3,
        #     child=py_trees.composites.Sequence("AlignStep", memory=True, children=[
        #         bt_nodes.RepositionViperXArm(goal_key="pose"),
        #         bt_nodes.VerifyViperXPosition(),
        #     ]),
        # ),
        # bt_nodes.RepositionViperXArm(goal_key="home_pose"),

        # bt_nodes.RepositionRackToGoalLevel(goal_key="rack_goal"),

        # bt_nodes.RepositionViperXArm(goal_key="shelf_pose"),
        # bt_nodes.VerifyViperXPosition(),
        # py_trees.decorators.Retry(
        #     name="AlignUntilReady",
        #     num_failures=3,
        #     child=py_trees.composites.Sequence("AlignStep", memory=True, children=[
        #         bt_nodes.RepositionViperXArm(goal_key="pose"),
        #         bt_nodes.VerifyViperXPosition(),
        #     ]),
        # ),
        # bt_nodes.MoveViperXGripper(command="open"),
        # bt_nodes.RepositionViperXArm(goal_key="home_pose"),
        # bt_nodes.RepositionRackToGoalLevel(goal_key=1),

        bt_nodes.ChangeInventory(bb, mode="restock"),
    ])

    num_items = len(getattr(bb, "items", []))

    repeat_each_item = py_trees.decorators.Repeat(
        name="RepeatForEachItem",
        child=process_one_item,
        num_success=num_items,
    )

    root = py_trees.composites.Sequence("RestockRoot", memory=True, children=[
        bt_nodes.DebugPrint("Restock/Employee tree selected"),
        bt_nodes.DebugPrint(lambda: f"Items in order: {len(getattr(bb, 'items', []))}"),
        repeat_each_item,
        # bt_nodes.SetHome(bb),
        # bt_nodes.NavigateToGoalPose(goal_key="nav_goal"),
    ])

    return root
