import py_trees
import robot_task_manager.bt_nodes as bt_nodes
from robot_task_manager.bt_nodes.navigation_nodes import MoveDistanceForCurrentItem


def create_customer_tree(bb):

    process_one_item = py_trees.composites.Sequence(
        "ProcessCustomerItem",
        memory=True,
        children=[
            bt_nodes.SetCurrentItem(bb),
            ### Delete Later Only for Feature 1 Demo Need to Migrate to real auto nav script)
            MoveDistanceForCurrentItem(bb),
            ###############################################

            # bt_nodes.NavigateToGoalPose(goal_key="nav_goal"),
            # bt_nodes.RepositionRackToGoalLevel(goal_key="rack_goal"),
            # bt_nodes.RepositionArmToGoalPose(goal_key="shelf_pose"),

            # py_trees.composites.Selector("GrabWithRecovery", memory=True, children=[
            #     py_trees.composites.Sequence("GrabSequence", memory=True, children=[
            #         bt_nodes.VerifyPosition(),
            #         bt_nodes.AdjustOrientation(),
            #         bt_nodes.MoveGripper(command="close"),
            #     ]),
            #     bt_nodes.RetryGrab(max_retries=3),
            # ]),

            # bt_nodes.RepositionArmToGoalPose(goal_key="home_pose"),
            # bt_nodes.RepositionRackToGoalLevel(goal_key=1),
            # bt_nodes.RepositionArmToGoalPose(goal_key="basket_pose"),
            # bt_nodes.MoveGripper(command="open"),
            # bt_nodes.RepositionArmToGoalPose(goal_key="home_pose"),

            bt_nodes.ChangeInventory(bb, mode="customer"),
        ],
    )

    num_items = len(getattr(bb, "items", []))

    repeat_each_item = py_trees.decorators.Repeat(
        name="RepeatForEachItem",
        child=process_one_item,
        num_success=num_items,
    )

    root = py_trees.composites.Sequence(
        "CustomerRoot",
        memory=True,
        children=[
            bt_nodes.DebugPrint("Customer tree selected"),
            bt_nodes.DebugPrint(lambda: f"Items in order: {len(getattr(bb, 'items', []))}"),
            repeat_each_item,
            # bt_nodes.SetHome(),
            # bt_nodes.NavigateToGoalPose(goal_key="nav_goal"),
        ],
    )

    return root
