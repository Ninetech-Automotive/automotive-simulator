from Navigation.EdgeStatus import EdgeStatus

class Edge:
    def __init__(self):
        self.status = EdgeStatus.UNKNOWN
        self.length = 1

    def get_weight(self) -> int:
        return 0
    
    def set_status(self, status):
        self.status = status