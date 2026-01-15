import py_trees
from rclpy.node import Node

def create_customer_tree(Node, bt_nodes, bb):
    root = py_trees.composites.Sequence("CustomerRoot")

    root.add_children([
        bt_nodes.DebugPrint("Customer tree selected"),
        bt_nodes.DebugPrint(f"Items in order: {len(bb.items)}"),

        py_trees.decorators.Repeat(
            child=py_trees.composites.Sequence("ProcessItem", children=[
                bt_nodes.SetCurrentItem(),
                bt_nodes.NavigateToShelf(),
                bt_nodes.PickItem(),
                bt_nodes.PlaceItemToBasket(),
                bt_nodes.ChangeInventory(),
            ]),
            num_success=-1
        ),

        bt_nodes.NavigateToHome(),
        bt_nodes.ClearTaskToIdle(),
    ])

    return root
