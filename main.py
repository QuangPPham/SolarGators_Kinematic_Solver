import numpy as np
from kinSolve import kinSolve

if __name__ == "__main__":
    coords = np.array([[0.0, 435.0, 278.5], # FG: initial wheel center (x,y,z)
                      [214.55, 222.75, 586.94], # SP_in: spring chassis attachment
                      [24.94, 222.75, 586.94], # SP_out: spring rocker attachment
                      [-2.19, 222.75, 560.1], # R1: rocker axis 1
                      [-2.19, 194.92, 560.1], # R2: rocker axis 2
                      [-8.97, 222.75, 517.78], # PR_in: push rod rocker attachment
                      [-8.97, 365.37, 194.74], # PR_out: push rod knuckle attachment
                      [120.78, 324.77, 458.43], # TR_in: tie rod inboard attachment
                      [120.78, 200, 458.43], # TR_out: tie rod upright attachment
                      [0.0, 372.68, 278.5], # SA: spindle axis (x,y,z)
                      [11.39, 348.05, 406.4], # UB: upper ball joint (x,y,z)
                      [-8.97, 391.9, 177.8], # LB: lower ball joint (x,y,z)
                      [0.0, 435.0, 0.0], # WB: wheel base/contact patch (x,y,z)
                      [-67.17, 216.81, 398.39], # UCF: upper front (x,y,z)
                      [69.50, 216.81, 398.39], # UCR: upper rear (x,y,z)
                      [-170.08, 219.08, 177.8], # LCF: lower front (x,y,z)
                      [68.31, 219.08, 177.8]]) # LCR: lower rear (x,y,z)

    springList = np.arange(189.0, 180.0, -0.01)
    solver = kinSolve(coords, springList, True)
    solver.solve()