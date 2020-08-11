from objects.SegmentList import SegmentList

class SegmentListContainer:
    def __init__(self):
        self.segment_lists = {}
        self.last_markers = {}

    def addPt(self, rid, pt):
        if not rid in self.segment_lists:
            self.segment_lists[rid] = SegmentList()

        self.segment_lists[rid].addPt(pt)

    def toMarkerArray(self, colors, _frame_id = "map"):
        markers = []

        for rid, lst in self.segment_lists.items():
            markers.append(lst.toMarker(_frame_id, colors[rid]))

        self.last_markers = {}
        for marker in markers:
            self.last_markers[marker.id] = marker

        return markers