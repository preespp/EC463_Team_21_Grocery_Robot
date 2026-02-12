import py_trees

class DebugPrint(py_trees.behaviour.Behaviour):
    def __init__(self, msg):
        super().__init__("DebugPrint")
        self.msg = msg

    def update(self):
        out = self.msg() if callable(self.msg) else self.msg
        print(out)
        return py_trees.common.Status.SUCCESS
