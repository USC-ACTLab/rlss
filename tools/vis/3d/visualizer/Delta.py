class Delta:
    def __init__(self):
        self.changes = []

    def forward(self):
        markers = []
        for change in self.changes:
            if change["action"] == "add":
                change["marker"].action = change["marker"].ADD
                markers.append(change["marker"])
            elif change["action"] == "update":
                change["new_marker"].action = change["new_marker"].ADD
                markers.append(change["new_marker"])
            elif change["action"] == "remove":
                change["removed_marker"].action = change["removed_marker"].DELETE
                markers.append(change["removed_marker"])

        return markers

    def backward(self):
        markers = []
        for change in reversed(self.changes):
            if change["action"] == "add":
                change["marker"].action = change["marker"].DELETE
                markers.append(change["marker"])
            elif change["action"] == "update":
                change["old_marker"].action = change["old_marker"].ADD
                markers.append(change["old_marker"])
            elif change["action"] == "remove":
                change["removed_marker"].action = change["removed_marker"].ADD
                markers.append(change["removed_marker"])

        return markers
    def addMarker(self, _marker):
        _id = _marker.id
        _marker.action = _marker.ADD

        change = {
            "action": "add",
            "marker_id": _id,
            "marker": _marker
        }

        self.changes.append(change)


    def updateMarker(self, _old_marker, _new_marker):
        assert(_old_marker.id == _new_marker.id)
        _new_marker.action = _new_marker.ADD
        _id = _new_marker.id

        change = {
            "action": "update",
            "marker_id": _id,
            "old_marker": _old_marker,
            "new_marker": _new_marker
        }

        self.changes.append(change)

    def removeMarker(self, _removed_marker):
        _id = _removed_marker.id
        _removed_marker.action = _removed_marker.DELETE

        change = {
            "action": "remove",
            "marker_id": _id,
            "removed_marker": _removed_marker
        }

        self.changes.append(change)

    def addMarkerArray(self, _markers):
        for marker in _markers:
            self.addMarker(marker)

    def removeMarkerArray(self, _markers):
        for marker in _markers:
            self.removeMarker(marker)