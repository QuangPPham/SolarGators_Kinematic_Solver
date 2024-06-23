import numpy as np
from kinSolve import kinSolve

if __name__ == "__main__":
    coords = np.array([[0.0, 435.0, 278.5], # FG: initial wheel center (x,y,z)
                      [222.05, 226.8, 577.42], # SP_in: spring chassis attachment
                      [13.26, 226.8, 577.42], # SP_out: spring rocker attachment
                      [41.06, 251.66, 505.21], # R1: rocker axis 1
                      [41.06, 278.53, 514.46], # R2: rocker axis 2
                      [-8.97, 248.79, 513.55], # PR_in: push rod rocker attachment
                      [-8.97, 364.4, 177.8], # PR_out: push rod knuckle attachment
                      [115.32, 183.33, 435.14], # TR_in: tie rod inboard attachment
                      [115.33, 322.9, 444.07], # TR_out: tie rod upright attachment
                      [0.0, 372.68, 278.5], # SA: spindle axis (x,y,z)
                      [11.39, 348.05, 406.4], # UB: upper ball joint (x,y,z)
                      [-8.97, 391.9, 177.8], # LB: lower ball joint (x,y,z)
                      [0.0, 435.0, 0.0], # WB: wheel base/contact patch (x,y,z)
                      [-76.2, 200, 397.45], # UCF: upper front (x,y,z)
                      [76.2, 200, 397.45], # UCR: upper rear (x,y,z)
                      [-114.3, 200, 177.8], # LCF: lower front (x,y,z)
                      [76.2, 200, 177.8]]) # LCR: lower rear (x,y,z)

    solver = kinSolve(coords, True)
    solver.solve()