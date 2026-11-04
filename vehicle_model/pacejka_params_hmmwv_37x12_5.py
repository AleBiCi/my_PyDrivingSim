from dataclasses import dataclass

@dataclass
class PacejkaParamHMMWV37x12_5:

    ## Tyre 37x12.5R16.5, ADAMS PAC2002 file "HMMWV_Pac02Tire.tir" from Project Chrono (BSD-3), data/vehicle/hmmwv/tire/.
    ## The file itself says it was "gained from ADAMS/Car truck example file 315/80 R 22.5 by scaling".
    ## NOT IN SLIDES: external source, see rock_crawler_params/README.md, set "cherokee_37in".
    ## Converted from HMMWV_Pac02Tire.tir by tir2pacejka.py
    ## alpha-sign mirrored (ISO .tir -> sim convention): pKy1, pHy1, pHy2, pHy3, pEy3, pEy4, qHz1, qHz2, qHz3, qHz4, qEz4, qEz5, rBy3, rHx1
    ## pCx1 = PCX1 / 1.3 to cancel the factor applied in pacejka_model.py (Cx = pCx1*1.3)
    ## Fz0 = FNOMIN * LFZO; coefficients absent from PAC2002 set to 0: rBy4

    rBy1 : float = 13.271
    rBy2 : float = 5.2405
    rBy3 : float = -1.1547e-05
    rBy4 : float = 0.0
    rCy1 : float = 1.01
    rEy1 : float = 0.010513
    rEy2 : float = 5.9816e-05
    rHy1 : float = 0.028005
    rHy2 : float = -4.8794e-05
    rVy1 : float = 0.0066878
    rVy2 : float = -0.042813
    rVy3 : float = -0.16227
    rVy4 : float = -0.019796
    rVy5 : float = 1.9
    rVy6 : float = -7.8097
    rBx1 : float = 11.13
    rBx2 : float = -12.494
    rCx1 : float = 0.97505
    rEx1 : float = -0.37196
    rEx2 : float = 0.0017379
    rHx1 : float = -0.0045181
    pCy1 : float = 1.5874
    pDy1 : float = 0.73957
    pDy2 : float = -0.075004
    pDy3 : float = -8.0362
    pEy1 : float = 0.37562
    pEy2 : float = -0.069325
    pEy3 : float = -0.29168
    pEy4 : float = -11.559
    pHy1 : float = -0.0056509
    pHy2 : float = 0.0020257
    pHy3 : float = 0.038716
    pKy1 : float = 10.289
    pKy2 : float = 3.3343
    pKy3 : float = -0.25732
    pVy1 : float = 0.015216
    pVy2 : float = -0.010365
    pVy3 : float = -0.31373
    pVy4 : float = -0.055766
    qBz1 : float = 5.8978
    qBz2 : float = -0.1535
    qBz3 : float = -2.0052
    qBz4 : float = 0.62731
    qBz5 : float = -0.92709
    qBz9 : float = 10.637
    qBz10 : float = 0.0
    qCz1 : float = 1.4982
    qDz1 : float = 0.085549
    qDz2 : float = -0.025298
    qDz3 : float = 0.21486
    qDz4 : float = -3.9098
    qDz6 : float = -0.0013373
    qDz7 : float = 0.0013869
    qDz8 : float = -0.053513
    qDz9 : float = 0.025817
    qEz1 : float = -0.0084519
    qEz2 : float = 0.0097389
    qEz3 : float = 0.0
    qEz4 : float = -4.3583
    qEz5 : float = 645.04
    qHz1 : float = -0.0085657
    qHz2 : float = 0.0042922
    qHz3 : float = -0.14763
    qHz4 : float = 0.29999
    R0 : float = 0.4699
    pCx1 : float = 1.3233846153846154
    pDx1 : float = 0.77751
    pDx2 : float = -0.24431
    pDx3 : float = -0.00015908
    pEx1 : float = 0.46659
    pEx2 : float = 0.393
    pEx3 : float = 0.076024
    pEx4 : float = 2.6509e-06
    pHx1 : float = -0.00088873
    pHx2 : float = -0.00067818
    pKx1 : float = 14.848
    pKx2 : float = -9.8161
    pKx3 : float = 0.15818
    pVx1 : float = -5.5714e-07
    pVx2 : float = 6.2972e-06
    Fz0 : float = 20331.66961
