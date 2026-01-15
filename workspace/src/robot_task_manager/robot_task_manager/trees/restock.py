import py_trees
from rclpy.node import Node

def create_restock_tree(Node, bt_nodes, bb):
    root = py_trees.composites.Sequence("RestockRoot")

    root.add_children([
        bt_nodes.DebugPrint("Resotck/Employee tree selected"),
        bt_nodes.DebugPrint(f"Items in order: {len(bb.items)}"),

        py_trees.decorators.Repeat(
            child=py_trees.composites.Sequence("RestockItem", children=[
                bt_nodes.SetCurrentItem(),
                bt_nodes.NavigateToShelf(),
                bt_nodes.PickItemFromBasket(),
                bt_nodes.PlaceItemToShelf(),
                bt_nodes.ChangeInventory(),
            ]),
            num_success=-1
        ),

        bt_nodes.NavigateToHome(),
        bt_nodes.ClearTaskToIdle(),
    ])

    return root
