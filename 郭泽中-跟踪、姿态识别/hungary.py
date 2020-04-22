import numpy as np
import copy
from scipy.optimize import linear_sum_assignment

def hungary(speedMatrix):
    row=len(speedMatrix)
    col=len(speedMatrix[0])
    matrix=copy.copy(speedMatrix)

    while row!=col:
        if row<col:
            sub_row=np.array([100.0]*col)
            matrix=np.append(matrix,[sub_row],axis=0)
            row+=1
        else:
            sub_col=np.array([100.0]*row)
            matrix=np.append(matrix.T,[sub_col],axis=0).T
            col+=1

    row_ind,col_ind=linear_sum_assignment(matrix)

    return row_ind,col_ind