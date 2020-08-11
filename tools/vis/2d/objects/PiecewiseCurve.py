from objects.BezierCurve import BezierCurve
import numpy as np

class PiecewiseCurve:

    @staticmethod
    def fromDictionary(_dict):
        pieces = []
        for _dd in _dict:
            if _dd["type"] == "bezier":
                pieces.append(BezierCurve.fromDictionary(_dd))
            else:
                raise Exception("not recognized curve type")

        return PiecewiseCurve(pieces)

    def __init__(self, _pieces):
        assert(type(_pieces) == list)
        for piece in _pieces:
            assert(type(piece) == BezierCurve)

        self.pieces = _pieces
        self.max_parameter = sum([piece.max_parameter for piece in _pieces])

    def eval(self, param):
        i = 0
        while param > self.pieces[i].max_parameter:
            param -= self.pieces[i].max_parameter
            i += 1

        return self.pieces[i].eval(param)

    def pts(self, _param_from):
        pts = np.ndarray(shape = (2, 0))
        i = 0
        while i < len(self.pieces) and _param_from > self.pieces[i].max_parameter:
            _param_from -= self.pieces[i].max_parameter
            i += 1


        while i < len(self.pieces):
            bpts = self.pieces[i].pts(_param_from)
            pts = np.concatenate((pts, bpts), axis = 1)
            _param_from = 0
            i += 1

        return pts

    def draw(self, _ax, _param_from):
        i = 0
        while i < len(self.pieces) and _param_from > self.pieces[i].max_parameter:
            _param_from -= self.pieces[i].max_parameter
            i += 1


        while i < len(self.pieces):
            self.pieces[i].draw(_ax, _param_from)
            _param_from = 0
            i += 1


if __name__ == "__main__":
    duration = 4.78978
    control_points= [
        (-3.51892,1.88924,0.75),
        (-3.51892,1.88924,0.75),
        (-3.51892,1.88924,0.75),
        (-3.51892,1.88924,0.75),
        (-2.2442192974,1.37519264943,0.673804284093),
        (-1.79448197019,1.15927033917,0.812013024357),
        (-1.20297981168,0.880590763017,0.968507627773),
        (-0.611415798577,0.601931825628,1.1250072342)
    ]
    piece1 = BezierCurve(duration, control_points)

    duration = 4.81254
    control_points = [
        (-0.611505,0.601895,1.125),
        (-0.0172071854228,0.321883547651,1.28223668189),
        (0.577090629154,0.0418720953028,1.43947336378),
        (1.02760307657,-0.174474089868,1.57816455423),
        (2.29590940091,-0.685450298368,1.4999997525),
        (2.2959012435,-0.685449101747,1.49999949992),
        (2.29587699784,-0.685452982346,1.50000027797),
        (2.29582118084,-0.685492687181,1.50000654961)
    ]
    piece2 = BezierCurve(duration, control_points)

    traj = PiecewiseCurve([piece1, piece2])


    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1)
    traj.draw(ax, 0)

    plt.show()