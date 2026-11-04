from dataclasses import dataclass

@dataclass
class PacejkaParamPolaris29x9_14:

    ## Tyre 29x9-14 61M, ADAMS PAC2002 file "Polaris_Pac02Tire.tir" from Project Chrono (BSD-3),
    ## data/vehicle/Polaris/. NOT IN SLIDES: external source, see rock_crawler_params/README.md, set "polaris_utv".
    ## Converted from Polaris_Pac02Tire.tir by tir2pacejka.py
    ## alpha-sign mirrored (ISO .tir -> sim convention): pKy1, pHy1, pHy2, pHy3, pEy3, pEy4, qHz1, qHz2, qHz3, qHz4, qEz4, qEz5, rBy3, rHx1
    ## pCx1 = PCX1 / 1.3 to cancel the factor applied in pacejka_model.py (Cx = pCx1*1.3)
    ## Fz0 = FNOMIN * LFZO; coefficients absent from PAC2002 set to 0: rBy4

    rBy1 : float = 6.2238
    rBy2 : float = 3.0734
    rBy3 : float = -0.016076
    rBy4 : float = 0.0
    rCy1 : float = 1.0051
    rEy1 : float = 0.019749
    rEy2 : float = -0.0020691
    rHy1 : float = -0.0010319
    rHy2 : float = 7.4123e-06
    rVy1 : float = 0.02962
    rVy2 : float = -0.011053
    rVy3 : float = -0.0009317
    rVy4 : float = 11.842
    rVy5 : float = 1.9
    rVy6 : float = 0.0
    rBx1 : float = 5.9945
    rBx2 : float = -8.2609
    rCx1 : float = 1.07816
    rEx1 : float = 1.644
    rEx2 : float = -0.0064359
    rHx1 : float = -0.008847
    pCy1 : float = 1.3223
    pDy1 : float = 1.0141
    pDy2 : float = -0.12274
    pDy3 : float = -1.0426
    pEy1 : float = -0.63772
    pEy2 : float = -0.050782
    pEy3 : float = 0.27333
    pEy4 : float = 8.3143
    pHy1 : float = -0.0011453
    pHy2 : float = 6.6688e-05
    pHy3 : float = -0.044112
    pKy1 : float = 19.797
    pKy2 : float = 1.7999
    pKy3 : float = 0.0095418
    pVy1 : float = 0.031305
    pVy2 : float = -0.0085749
    pVy3 : float = -0.092912
    pVy4 : float = -0.27907
    qBz1 : float = 7.5088
    qBz2 : float = -1.9428
    qBz3 : float = 0.61681
    qBz4 : float = 0.12231
    qBz5 : float = 0.50016
    qBz9 : float = 5.5144
    qBz10 : float = 0.0
    qCz1 : float = 1.2237
    qDz1 : float = 0.062582
    qDz2 : float = 0.00052585
    qDz3 : float = -0.60661
    qDz4 : float = 8.634
    qDz6 : float = -0.0048467
    qDz7 : float = 0.0034983
    qDz8 : float = -0.11032
    qDz9 : float = 0.021277
    qEz1 : float = -5.3971
    qEz2 : float = 1.1207
    qEz3 : float = 0.0
    qEz4 : float = -0.14942
    qEz5 : float = 1.1429
    qHz1 : float = 0.00069905
    qHz2 : float = -0.0055192
    qHz3 : float = -0.065953
    qHz4 : float = -0.11393
    R0 : float = 0.3683
    pCx1 : float = 1.4146153846153846
    pDx1 : float = 1.1387
    pDx2 : float = -0.11999
    pDx3 : float = -2.2142e-05
    pEx1 : float = 0.62727
    pEx2 : float = -0.12336
    pEx3 : float = -0.03448
    pEx4 : float = -1.5066e-05
    pHx1 : float = -0.00033912
    pHx2 : float = -8.5877e-06
    pKx1 : float = 18.886
    pKx2 : float = -3.988
    pKx3 : float = 0.21542
    pVx1 : float = -4.638e-06
    pVx2 : float = 1.9874e-05
    Fz0 : float = 1489.8550719999998
