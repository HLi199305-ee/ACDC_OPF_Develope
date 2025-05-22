import numpy as np
from scipy.sparse import csr_matrix

def makeYbus(baseMVA, bus, branch):

    nb = bus.shape[0]
    nl = branch.shape[0]

    # define named indices into bus, branch matrices
    BUS_I, GS, BS = 0, 4, 5
    F_BUS, T_BUS, BR_R, BR_X, BR_B, TAP, SHIFT, BR_STATUS = 0, 1, 2, 3, 4, 8, 9, 10

    if not np.array_equal(bus[:, BUS_I], np.arange(1, nb + 1)):
        raise ValueError("makeYbus: buses must be numbered consecutively in bus matrix")
    
    # for each branch, compute the elements of the branch admittance matrix where
    #   | If |   | Yff  Yft |   | Vf |
    #   |    | = |          | * |    |
    #   | It |   | Ytf  Ytt |   | Vt |
    stat = branch[:, BR_STATUS]  
    Ys = stat / (branch[:, BR_R] + 1j * branch[:, BR_X])  
    Bc = stat * branch[:, BR_B]  

    # calucate ratio
    tap = np.ones(nl, dtype=np.complex128)
    idx = np.where(branch[:, TAP] != 0)
    tap[idx] = branch[idx, TAP]

    shifts = branch[:, SHIFT].astype(float) 
    phase_shifts = 1j * np.pi / 180 * shifts 
    tap *= np.exp(phase_shifts)

    # calculate admittance matrix 
    Ytt = Ys + 1j * Bc / 2
    Yff = Ytt / (tap * np.conj(tap))
    Yft = -Ys / np.conj(tap)
    Ytf = -Ys / tap

    # compute shunt admittance
    Ysh = (bus[:, GS] + 1j * bus[:, BS]) / baseMVA

    # bus indices
    f = branch[:, F_BUS].astype(int) - 1   
    t = branch[:, T_BUS].astype(int) - 1   

    # build Yf and Yt
    i = np.hstack((np.arange(nl), np.arange(nl)))
    Yf = csr_matrix((np.hstack((Yff, Yft)), (i, np.hstack((f, t)))), shape=(nl, nb))
    Yt = csr_matrix((np.hstack((Ytf, Ytt)), (i, np.hstack((f, t)))), shape=(nl, nb))

    Ybus = csr_matrix(
        (np.hstack((Yff, Yft, Ytf, Ytt)), 
         (np.hstack((f, f, t, t)), np.hstack((f, t, f, t)))),
        shape=(nb, nb)
    ) + csr_matrix((Ysh, (np.arange(nb), np.arange(nb))), shape=(nb, nb))

    return Ybus

