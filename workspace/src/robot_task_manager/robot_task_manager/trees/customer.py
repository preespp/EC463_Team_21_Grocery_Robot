import py_trees
import robot_task_manager.bt_nodes as bt_nodes


def create_customer_tree(bb):

    process_one_item = py_trees.composites.Sequence(
        "ProcessCustomerItem",
        memory=True,
        children=[
            bt_nodes.SetCurrentItem(bb),
            bt_nodes.ResolveCurrentItemSemanticTarget(bb),
            bt_nodes.NavigateToGoalPose(goal_key="nav_goal", bb=bb),
            # bt_nodes.RepositionRackToGoalLevel(goal_key="rack_goal"),
            # bt_nodes.VerifyPosition(),
            # py_trees.decorators.Retry(
            #     name="AlignUntilReady",
            #     num_failures=3,
            #     child=py_trees.composites.Sequence("AlignStep", memory=True, children=[
            #         bt_nodes.RepositionArmToGoalPose(goal_key="pose"),
            #         bt_nodes.VerifyPosition(),
            #     ]),
            # ),
            # bt_nodes.MoveGripper(command="close"),
            # bt_nodes.RepositionArmToGoalPose(goal_key="basket_pose"),
            # bt_nodes.RepositionRackToGoalLevel(goal_key=1),
            # bt_nodes.MoveGripper(command="open"),
            # bt_nodes.RepositionArmToGoalPose(goal_key="home_pose"),

            #bt_nodes.ChangeInventory(bb, mode="customer"),
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
            bt_nodes.SetHome(bb),
            bt_nodes.NavigateToGoalPose(goal_key="nav_goal", bb=bb),
        ],
    )

    return root
