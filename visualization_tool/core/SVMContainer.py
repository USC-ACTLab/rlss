from SVM import SVM

class SVMContainer(object):

    def __init__(self,copy=None,numRobots=None,config=None):
        self.config = config
        if copy:
            self.svms = [x for x in copy.svms]
        else:
            if numRobots==None: raise ValueError("SVMContainer cannot initialized without numrobots")
            self.svms = [None for x in xrange(numRobots)]

    def update(self,svm_hyperplanes_key):
        for el in svm_hyperplanes_key:
            rid = el["robot_id"]
            self.svms[rid] = SVM(el["hyperplanes"])

    def get_all_markers(self):
        if self.config==None:
            pc = 1
        else:
            pc = self.config["max_pieces"]
        res = []
        for r in self.svms:
            if r==None: continue
            res.extend(r.get_markers(pc))
        return res


