import py_trees

def create_customer_tree(ros_node, bt_nodes, bb):
    
    process_one_item = py_trees.composites.Sequence("ProcessCustomerItem", children=[
        bt_nodes.SetCurrentItem(),

        bt_nodes.NavigateToGoalPose(goal_key="nav_goal"),

        bt_nodes.RepositionRackToGoalLevel(goal_key="rack_goal"),

        bt_nodes.RepositionArmToGoalPose(goal_key="shelf_pose"),

        py_trees.composites.Selector("GrabWithRecovery", children=[
            py_trees.composites.Sequence("GrabSequence", children=[
                bt_nodes.VerifyPosition(),
                bt_nodes.AdjustOrientation(),
                bt_nodes.MoveGripper(command="close"),
            ]),
            bt_nodes.RetryGrab(max_retries=3),
        ]),

        bt_nodes.RepositionArmToGoalPose(goal_key="home_pose"),

        bt_nodes.RepositionRackToGoalLevel(goal_key=1),

        bt_nodes.RepositionArmToGoalPose(goal_key="basket_pose"),

        bt_nodes.MoveGripper(command="open"),

        bt_nodes.RepositionArmToGoalPose(goal_key="home_pose"),

        bt_nodes.ChangeInventory(),
    ])

    repeat_until_done = py_trees.decorators.Repeat(
        name="RepeatUntilAllItemsDone",
        child=py_trees.composites.Selector("LoopBody", children=[
            py_trees.composites.Sequence("IfMoreItemsThenProcess", children=[
                bt_nodes.HasMoreItems(),
                process_one_item
            ]),
            bt_nodes.AllItemsDone()
        ]),
        num_success=-1
    )

    root = py_trees.composites.Sequence("CustomerRoot", children=[
        bt_nodes.DebugPrint("Customer tree selected"),
        bt_nodes.DebugPrint(lambda: f"Items in order: {len(bb.items)}"),

        repeat_until_done,

        bt_nodes.SetHome(),
        bt_nodes.NavigateToGoalPose(goal_key="nav_goal"),
    ])

    return root
