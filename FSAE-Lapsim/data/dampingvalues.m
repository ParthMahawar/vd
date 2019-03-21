%% dampingvalues
%this script is used to record physical damper curve values with respect to
%actual damper setting values. Data was collected from the Ohlins TTX25
%MkII dyno curves off the Ohlins website using webplot digitizer. This
%script complies the webplot output and creates an easy array for other
%scrips to work with. cdamp is for compression and rdamp is for rebound.
%the data is recorded as low speed setting (ls) followed by high speed
%setting (hs). 
% LSC-HSC, LSR_HSR
% LS ticks from fully clockwise
% HS turns from fully couter clockwise
% C12/R12 valving
% data is recorded as [velocity (in per sec), lbs]
ls0hs4_3 = [0.12463, 26.830
0.16442, -34.192
0.14651, 35.760
0.21575, 46.843
0.28108, -43.848
0.30766, 54.384
0.37690, -50.930
0.41510, 61.413
0.50224, -55.014
0.53646, 71.471
0.64190, -61.466
0.68236, 74.712
0.77747, -65.185
0.81350, 76.514
0.90333, -68.358
0.96335, 80.440
1.0418, 85.284
1.0582, -71.931
1.1791, 88.240
1.1917, -74.143
1.3354, 91.533
1.3283, -74.994
1.4739, 94.902
1.4662, -75.377
1.6076, 99.076
1.6042, -76.313
1.7282, 99.880
1.7262, -77.802
1.8620, 101.24
1.8893, -83.099
1.9898, 103.90
2.0159, -85.485
2.1230, 103.50
2.1579, -86.353
2.2662, 108.75
2.2892, -87.885
2.4026, 110.57
2.4265, -89.416
2.5530, 112.51
2.5638, -90.948
2.7058, 114.51
2.8383, 116.19
2.8907, -98.350
3.0189, -98.398
3.1547, 124.00
3.1696, -99.323
3.2962, 123.72
3.2966, -99.626
3.4268, 124.63
3.4398, -100.26
3.5799, -100.47
3.6336, 125.60
3.7091, -100.86
3.7739, 126.94
3.8545, -101.91
3.9169, 127.18
4.0308, 132.47
4.0261, -106.07
4.1693, 130.25
4.1980, -108.48
4.3573, -109.52
4.3245, 133.04
4.4677, 133.80
4.5322, -110.78
4.5990, 134.57
4.6916, -111.81
4.7423, 135.33
4.8664, -113.08
4.8736, 136.10
5.0064, -114.78
5.0109, 137.22
5.1482, 137.63
5.2007, -115.38
5.2854, 138.40
5.3463, -117.06
5.4227, 139.16
5.5600, 139.93
5.6359, -121.65
5.7032, 140.69
5.7842, -122.16
5.8345, 141.46
5.9509, -123.03
6.0867, -122.82
6.1712, 146.44
6.2557, -123.22
6.3132, 147.13
6.4108, -123.69
6.4559, 147.50
6.5642, -124.11
6.5821, 148.07
6.6854, -124.34
6.7074, 148.74
6.7895, -124.45
6.8396, 147.97
6.9832, 148.39
6.9832, -125.01
7.1271, 148.86
7.1271, -125.28
7.2506, 149.21
7.2849, -125.73
7.3738, 149.29
7.4384, -125.77
7.5117, 149.50
7.5526, -125.95
7.6370, 149.88
7.6703, -126.61
7.7767, 150.15
7.8221, -127.18
7.9056, 150.12
7.9839, -126.90
8.0276, 150.61
8.1391, -127.79
8.1635, 151.03
8.2709, -126.38
8.3088, 151.25
8.3732, -128.26
8.4523, 153.56
8.5171, -128.23
8.5204, 153.71
8.6696, 154.48
8.8367, 155.24
8.8666, -133.45
9.0038, 156.22
9.0277, -134.43
9.1710, 156.78
9.2127, -134.98
9.3381, 157.54
9.3858, -135.74
9.5052, 158.31
9.5589, -136.51
9.6783, 159.07
9.7320, -137.28
9.8567, 160.01
9.9275, -138.86
9.9898, 161.68];
lsc0hsc4_3 = [0,0];
lsr0hsr4_3 = [0,0];
for i = 1:length(ls0hs4_3) %this separates compression and rebound
    if ls0hs4_3(i,2) < 0
        lsr0hsr4_3 = [lsr0hsr4_3;ls0hs4_3(i,:)];
    else
        lsc0hsc4_3 = [lsc0hsc4_3;ls0hs4_3(i,:)];
    end
end
%%
ls2hs4_3 = [0.51827, 17.243
0.65981, 22.188
0.65981, -14.929
0.79112, 25.355
0.79470, -19.426
0.92740, 29.309
0.92242, -22.621
1.0537, 35.855
1.0537, -25.299
1.1850, 40.380
1.1850, -29.582
1.3163, 45.358
1.3163, -33.620
1.4477, 49.662
1.4477, -37.553
1.5790, 52.911
1.7167, 56.078
1.7103, -46.916
1.8416, 62.866
1.8416, -49.736
1.9749, 66.512
1.9749, -52.636
2.1042, 70.523
2.1042, -54.354
2.2355, 73.203
2.2355, -56.732
2.3668, 75.083
2.3668, -59.656
2.4981, 76.893
2.4981, -61.535
2.6294, 78.633
2.6294, -63.520
2.7672, 80.200
2.7607, -65.469
2.8729, -69.672
2.8920, 84.968
2.9995, -70.155
3.0214, 87.028
3.1308, -71.978
3.1547, 90.315
3.2621, -73.370
3.2860, 92.846
3.3934, -74.727
3.4173, 95.979
3.5247, -76.097
3.5486, 98.044
3.6560, -77.303
3.6799, 99.970
3.7873, -80.088
3.8112, 101.71
3.9186, -82.455
3.9465, 103.78
4.0499, -83.951
4.0738, 104.81
4.1813, -85.448
4.2051, 107.65
4.3126, -86.771
4.3364, 109.68
4.4439, -87.989
4.4727, 111.06
4.5752, -89.381
4.5990, 112.09
4.7065, -90.669
4.7375, 115.12
4.8378, -91.957
4.9737, -92.393
5.1021, 118.89
5.1004, -94.927
5.2198, 121.26
5.2317, -97.027
5.3511, 121.95
5.3630, -97.851
5.4824, 123.09
5.4943, -98.988
5.6137, 124.34
5.6256, -99.858
5.7514, 123.82
5.7569, -100.89
5.8763, 126.69
5.8883, -101.87
6.0066, 127.68
6.0176, -103.17
6.1389, 128.23
6.1509, -104.38
6.2702, 129.00
6.2822, -105.43
6.4016, 129.01
6.4135, -106.23
6.5329, 130.12
6.5448, -107.03
6.6642, 132.10
6.6761, -107.86
6.7955, 132.79
6.8074, -108.66
6.9268, 133.52
6.9437, -109.86
7.0581, 134.95
6.9984, 133.42
7.0700, -110.37
7.1894, 134.95
7.2013, -111.17
7.3207, 135.82
7.3326, -112.01
7.4520, 136.55
7.4640, -112.81
7.5833, 137.25
7.5953, -113.64
7.7146, 137.94
7.7266, -114.51
7.8459, 138.71
7.8579, -115.31
7.9782, 139.34
7.9892, -115.82
8.1086, 140.10
8.1205, -116.95
8.2399, 140.87
8.2518, -117.75
8.3712, 141.56
8.3831, -119.28
8.5025, 142.26
8.5144, -120.38
8.6338, 142.99
8.6457, -121.28
8.7651, 144.59
8.7770, -121.81
8.8964, 146.12
8.9083, -122.46
9.0257, 146.24
9.0367, -122.82
9.1590, 147.20
9.1710, -123.88
9.2903, 147.62
9.3023, -124.54
9.4216, 148.04
9.4336, -125.09
9.5529, 148.65
9.5649, -125.72
9.6843, 149.12
9.6962, -126.54
9.8156, 149.61
9.8275, -127.04
9.9469, 150.23
9.9662, -127.87
10.030, 149.69
10.030, -128.66];
lsc2hsc4_3 = [0,0];
lsr2hsr4_3 = [0,0];
for i = 1:length(ls2hs4_3)
    if ls2hs4_3(i,2) < 0
        lsr2hsr4_3 = [lsr2hsr4_3;ls2hs4_3(i,:)];
    else
        lsc2hsc4_3 = [lsc2hsc4_3;ls2hs4_3(i,:)];
    end
end
%%
ls4hs4_3 = [0.89855, -9.8119
0.95824, 15.719
1.0299, -11.309
1.1373, 20.958
1.1850, -13.432
1.2686, 23.290
1.3163, -15.311
1.3999, 25.343
1.4511, -18.200
1.5312, 27.536
1.5790, -20.567
1.6625, 29.799
1.7103, -21.925
1.7938, 34.533
1.8416, -23.387
1.9251, 37.596
1.9729, -24.675
2.0610, 42.400
2.0714, -28.538
2.1072, -26.241
2.1925, 46.315
2.1340, 42.103
2.2355, -29.130
2.3191, 47.307
2.3668, -31.740
2.4504, 49.558
2.4981, -34.107
2.5817, 51.588
2.6294, -36.474
2.7130, 53.711
2.7607, -38.841
2.8489, 55.695
2.8920, -41.626
2.9756, 60.648
3.0214, -45.228
3.0726, 63.811
3.1547, -47.056
3.1905, 65.650
3.2896, 70.717
3.2860, -48.935
3.4173, 71.846
3.4173, -50.606
3.5486, 73.969
3.5486, -52.346
3.6799, 76.023
3.6799, -53.808
3.8112, 78.216
3.8158, -55.870
3.9489, 79.895
3.9465, -60.086
4.0670, -63.380
4.0738, 84.620
4.0667, -62.078
4.2051, 86.465
4.2051, -63.694
4.3364, 88.936
4.3364, -65.225
4.4677, 90.224
4.4677, -68.462
4.5990, 92.104
4.5990, -69.854
4.7304, 93.983
4.7304, -70.748
4.8617, 97.151
4.8617, -72.291
4.9930, 98.087
4.9939, -73.767
5.1243, 99.692
5.1243, -74.310
5.2556, 102.93
5.2556, -75.389
5.3944, 105.32
5.3869, -76.503
5.5238, 106.42
5.5182, -77.756
5.6435, 106.70
5.6495, -78.568
5.7808, 108.18
5.7808, -79.809
5.8507, 111.38
5.9121, -83.255
5.9738, 111.76
6.0395, -84.718
6.1031, 112.71
6.1747, -85.622
6.2344, 113.65
6.3061, -86.666
6.3657, 114.73
6.4374, -87.780
6.4970, 115.74
6.5687, -88.720
6.6284, 116.64
6.7000, -89.729
6.7689, 119.41
6.8910, 120.01
6.9626, -92.965
7.0203, 120.47
7.0939, -94.776
7.1536, 121.85
7.2252, -95.473
7.2849, 122.18
7.3565, -96.447
7.4162, 123.02
7.4878, -97.004
7.5475, 123.93
7.6191, -97.700
7.6788, 124.51
7.7504, -98.536
7.8101, 125.31
7.8818, -99.301
7.9454, 125.88
8.0131, -100.10
8.0727, 126.82
8.1444, -100.66
8.2041, 127.58
8.2757, -101.49
8.3354, 128.14
8.4070, -102.23
8.4667, 129.19
8.5383, -102.75
8.5980, 129.81
8.6696, -105.32
8.7293, 130.36
8.8009, -106.26
8.8646, 131.28
8.9322, -107.24
8.9929, 133.85
9.0635, -108.32
9.1232, 135.06
9.1948, -109.36
9.2545, 135.96
9.3261, -110.27
9.3858, 137.04
9.4574, -111.45
9.5171, 137.98
9.5888, -112.42
9.6484, 138.88
9.7201, -113.61
9.7797, 139.79
9.8514, -114.62
9.9111, 140.90
9.9854, -115.26
10.018, 141.29];
lsc4hsc4_3 = [0,0];
lsr4hsr4_3 = [0,0];
for i = 1:length(ls4hs4_3)
    if ls4hs4_3(i,2) < 0
        lsr4hsr4_3 = [lsr4hsr4_3;ls4hs4_3(i,:)];
    else
        lsc4hsc4_3 = [lsc4hsc4_3;ls4hs4_3(i,:)];
    end
end
%%
ls6hs4_3 = [0.92242, -3.5640
1.0179, 9.9702
1.0537, -4.4854
1.1492, 11.734
1.1804, -6.6768
1.2805, 13.196
1.3163, -6.7071
1.4118, 14.727
1.4477, -7.1492
1.5432, 16.154
1.5790, -7.5146
1.6745, 17.651
1.7103, -9.5033
1.8416, -10.346
1.9421, 21.257
1.9729, -11.406
2.0631, -13.407
2.0684, 23.278
2.1868, -13.832
2.1997, 24.682
2.2924, -20.861
2.3310, 25.993
2.4265, -22.447
2.4623, 27.304
2.5578, -23.387
2.5936, 28.592
2.6891, -24.466
2.8204, -25.522
2.8562, 35.472
2.9582, -27.082
2.9901, 36.010
3.0830, -27.297
3.1188, 39.823
3.2143, -28.875
3.2502, 42.086
3.3496, -31.729
3.3815, 44.453
3.4770, -34.734
3.5128, 48.339
3.6083, -36.509
3.6441, 49.302
3.7396, -38.284
3.7754, 50.405
3.8709, -40.025
3.9067, 51.437
4.0013, -40.689
4.0316, 52.532
4.1335, -43.784
4.1733, 53.717
4.2648, -47.671
4.3006, 59.385
4.3961, -48.657
4.4319, 61.578
4.5274, -49.341
4.5632, 63.666
4.6587, -50.154
4.6995, 66.225
4.7900, -51.018
4.8068, 71.304
4.9214, -52.037
4.9383, 71.638
5.0444, -53.286
5.0646, 73.401
5.1933, -54.009
5.1959, 75.048
5.3153, -59.099
5.3272, 76.023
5.4466, -60.770
5.4585, 76.742
5.5779, -62.545
5.5898, 77.879
5.7092, -64.320
5.8405, -65.991
5.8524, 84.794
5.9764, -66.360
5.9865, 86.431
6.1031, -69.054
6.1151, 88.484
6.2344, -70.980
6.2546, 87.990
6.3657, -73.242
6.4970, -73.695
6.6164, 96.014
6.6284, -74.345
6.7477, 97.441
6.7597, -74.832
6.8790, 97.928
6.8910, -75.342
7.0103, 98.958
7.0172, -75.211
7.1536, -76.433
7.1775, 99.796
7.2849, -76.909
7.3088, 100.67
7.4162, -77.872
7.4401, 102.27
7.5558, -79.069
7.5714, 102.34
7.6794, -79.352
7.7027, 103.07
7.8101, -84.334
7.8340, 103.94
7.9414, -85.557
7.9765, 104.96
8.0727, -86.666
8.0966, 109.19
8.2041, -87.850
8.2279, 110.59
8.3354, -88.998
8.3592, 111.98
8.4667, -90.182
8.4905, 113.34
8.5980, -91.365
8.6218, 114.69
8.7293, -92.549
8.7532, 116.05
8.8606, -93.697
8.8819, 119.95
8.9945, -93.559
9.0132, 121.48
9.1232, -97.735
9.1471, 122.07
9.2545, -98.559
9.2784, 123.33
9.3858, -99.035
9.4097, 123.82
9.5171, -99.719
9.5410, 124.61
9.6484, -100.31
9.6723, 125.24
9.7797, -100.94
9.8036, 125.97
9.9111, -101.65
9.9399, 126.47
10.005, -100.04];
lsc6hsc4_3 = [0,0];
lsr6hsr4_3 = [0,0];
for i = 1:length(ls6hs4_3)
    if ls6hs4_3(i,2) < 0
        lsr6hsr4_3 = [lsr6hsr4_3;ls6hs4_3(i,:)];
    else
        lsc6hsc4_3 = [lsc6hsc4_3;ls6hs4_3(i,:)];
    end
end
%%
ls10hs4_3 = [1.1714, -4.1779
1.1933, 6.3078
1.3192, 6.3956
1.3114, -5.4901
1.4477, 8.3546
1.4477, -5.5349
1.5790, 9.2532
1.5790, -6.7953
1.7103, 11.212
1.9636, -10.421
2.1042, 10.806
2.2355, 12.233
2.3668, 14.240
2.3668, -10.891
2.4981, -10.740
2.6303, 18.346
2.6294, -12.422
2.6861, 20.119
2.7607, -13.188
2.7942, 20.994
2.8920, -13.954
2.9328, 21.633
3.0234, -14.597
3.0552, 22.831
3.1905, 23.209
3.2860, -16.182
3.3218, 23.986
3.4173, -17.574
3.4531, 24.682
3.5660, -21.023
3.5844, 25.552
3.7038, -20.695
3.7157, 26.353
3.8351, -21.356
3.8470, 26.933
3.9684, -22.689
3.9811, 27.926
4.1017, -22.150
4.1096, 28.801
4.2290, -23.137
4.2419, 29.223
4.3603, -23.839
4.3722, 33.384
4.4916, -24.501
4.5036, 34.707
4.6229, -25.266
4.6349, 36.099
4.7542, -25.858
4.7662, 37.491
4.8855, -26.595
4.8975, 38.883
5.0168, -28.107
5.0242, 40.246
5.1673, 42.992
5.1840, -30.070
5.3004, 44.719
5.3153, -31.114
5.4227, 46.518
5.4466, -32.228
5.5540, 47.539
5.5779, -33.307
5.6853, 48.305
5.7092, -34.386
5.8166, 49.186
5.8405, -35.465
5.9562, 50.671
5.9718, -36.335
6.0792, 51.020
6.1867, -38.493
6.2106, 51.925
6.3180, -39.781
6.3419, 55.034
6.4493, -40.999
6.4732, 58.811
6.5806, -43.349
6.6045, 58.201
6.7358, 59.977
6.7358, -45.856
6.8671, 61.543
6.8671, -46.677
6.9984, 63.470
6.9984, -47.406
7.1297, 64.815
7.1297, -48.292
7.2610, 67.112
7.2610, -49.133
7.3923, -49.979
7.4281, 71.567
7.5236, -50.757
7.5594, 72.310
7.6549, -51.549
7.6908, 73.250
7.7863, -52.216
7.8270, 72.753
7.9176, -53.103
8.0489, -53.597
8.1046, 75.462
8.1855, -55.293
8.2399, 77.380
8.3055, -56.751
8.3712, 78.076
8.4428, -59.934
8.5025, 81.662
8.6338, 83.646
8.7054, -62.336
8.7651, 85.247
8.8367, -63.694
8.8964, 86.883
8.9757, -64.613
9.0226, 89.626
9.0993, -66.095
9.1590, 90.155
9.2306, -68.549
9.2943, 92.239
9.3619, -70.973
9.4216, 95.654
9.4933, -71.706
9.5529, 96.896
9.6246, -72.653
9.6843, 97.986
9.7559, -73.270
9.8156, 98.903
9.8872, -73.962
9.9551, 100.09
10.011, -74.822];
lsc10hsc4_3 = [0,0];
lsr10hsr4_3 = [0,0];
for i = 1:length(ls10hs4_3)
    if ls10hs4_3(i,2) < 0
        lsr10hsr4_3 = [lsr10hsr4_3;ls10hs4_3(i,:)];
    else
        lsc10hsc4_3 = [lsc10hsc4_3;ls10hs4_3(i,:)];
    end
end
%%
ls15hs4_3 = [0.98211, -1.2957
1.1134, -1.6949
1.2447, 3.9187
1.3760, 5.0657
1.3760, -3.9302
1.5073, 5.9558
1.5073, -3.1869
1.6387, 5.6147
1.6387, -3.0222
1.7580, -4.0339
1.9013, 6.6519
1.9013, -3.3482
2.0326, 7.1044
2.0379, -5.8708
2.1639, 7.7773
2.1667, -6.4661
2.2952, 9.3669
2.2936, -6.4758
2.4265, 9.1928
2.4265, -6.2361
2.5578, 9.4075
2.5578, -5.8439
2.6891, 9.4829
2.6891, -6.3544
2.8204, 9.6917
2.8316, -8.1860
2.9517, 10.641
2.9573, -7.8585
3.0830, 10.516
3.0830, -7.3522
3.2143, 11.803
3.2143, -7.5146
3.3457, 12.082
3.3457, -7.4798
3.4770, 15.145
3.4770, -8.3500
3.6083, 15.702
3.6083, -8.5589
3.7396, 16.328
3.7396, -8.6076
3.8709, 17.024
3.8709, -8.9069
4.0022, 17.679
4.0012, -10.596
4.1335, 18.486
4.1335, -11.657
4.2648, 19.960
4.2648, -12.214
4.3961, 21.909
4.3961, -12.713
4.5274, 22.391
4.5274, -11.854
4.6587, 23.011
4.6587, -12.179
4.7900, 23.568
4.7900, -13.641
4.9214, 24.090
4.9214, -15.137
5.0527, 25.413
5.0527, -15.694
5.1840, 25.135
5.1959, -16.321
5.3153, 25.726
5.3272, -17.539
5.4466, 26.353
5.5779, 26.829
5.7092, 27.432
5.7211, -20.892
5.8405, 27.977
5.8524, -21.356
5.9764, 28.629
5.9865, -22.108
6.1071, 29.096
6.1151, -22.099
6.2407, 29.115
6.2464, -22.656
6.3657, 34.150
6.3777, -23.021
6.5090, -23.572
6.6284, 36.517
6.6403, -24.083
6.7597, 37.700
6.7716, -24.605
6.8910, 38.883
6.9029, -25.162
7.0172, 40.287
7.0342, -26.241
7.1563, 41.882
7.1655, -26.032
7.2825, 43.124
7.2968, -26.577
7.4162, 46.042
7.4281, -27.018
7.5475, 47.121
7.5594, -28.596
7.6788, 47.666
7.6908, -29.838
7.8101, 48.328
7.8221, -30.766
7.9497, 50.035
7.9534, -31.891
8.0727, 53.075
8.0847, -32.332
8.2041, 50.718
8.2160, -33.098
8.3354, 51.275
8.3473, -33.864
8.4697, 52.047
8.4786, -34.664
8.5980, 52.946
8.6099, -35.430
8.7293, 53.526
8.7412, -36.196
8.8646, 53.909
8.8725, -37.031
9.0038, -37.859
9.1232, 58.217
9.1351, -38.563
9.2665, -39.328
9.3978, -40.094
9.6604, -41.881
9.9091, 68.033
10.048, 69.479];
lsc15hsc4_3 = [0,0];
lsr15hsr4_3 = [0,0];
for i = 1:length(ls15hs4_3)
    if ls15hs4_3(i,2) < 0
        lsr15hsr4_3 = [lsr15hsr4_3;ls15hs4_3(i,:)];
    else
        lsc15hsc4_3 = [lsc15hsc4_3;ls15hs4_3(i,:)];
    end
end
%%
ls25hs4_3 = [0.91049, -0.39771
1.0418, -0.48357
1.1731, -0.71562
1.3044, -0.71562
1.4357, -0.97087
1.8296, -0.97087
1.9685, -0.60527
2.0923, -0.86065
2.2236, -0.74463
2.3549, -0.97087
3.4053, 8.6011
3.5366, 8.9492
3.6680, 9.3321
3.7993, 9.6105
3.9306, 9.7846
4.0619, 10.133
4.1932, 10.724
4.3245, 11.072
4.4558, 11.525
4.4558, -8.0716
4.5871, 11.873
4.5871, -8.2804
4.7184, 12.325
4.7184, -8.6285
4.8497, 12.674
4.8497, -8.9069
4.9810, 12.917
4.9810, -9.4290
5.1123, 13.474
5.1123, -9.3942
5.2437, 13.927
5.2437, -9.7423
5.3750, 14.275
5.3750, -9.9511
5.5063, 14.727
5.5063, -10.230
5.6376, 15.075
5.6376, -10.543
5.7689, 15.493
5.7689, -10.787
5.9002, 15.876
6.0269, 15.022
6.0269, -10.469
6.1628, 16.850
6.1628, -11.726
6.2941, 17.373
6.2941, -12.074
6.4254, 17.929
6.4254, -12.457
6.5567, 18.452
6.5567, -12.840
6.6880, 19.008
6.6880, -13.153
6.8193, 19.496
6.8193, -13.571
6.9507, 20.053
6.9609, -13.157
7.0820, 20.784
7.0820, -14.267
7.2133, 21.549
7.2133, -14.615
7.3446, 22.072
7.3446, -14.998
7.4759, 23.023
7.4759, -15.346
7.6072, 23.742
7.6072, -15.729
7.7385, 23.974
7.7385, -16.042
7.8698, 24.299
7.8698, -16.460
8.0011, 24.786
8.0002, -15.975
8.1324, 24.891
8.1324, -17.852
8.2637, 25.343
8.3640, -21.302
8.3950, 25.587
8.4905, -20.950
8.5264, 25.854
8.6218, -21.681
8.6577, 26.248
8.7532, -21.171
8.7890, 26.504
8.8845, -22.134
8.9203, 26.794
9.0132, -22.568
9.0353, 29.454
9.1471, -22.168
9.2208, 28.982
9.2784, -22.424
9.4097, -22.633
9.4694, 30.495
9.5410, -22.714
9.6007, 30.982
9.6723, -22.969
9.7320, 31.574
9.8036, -23.352
9.9399, -23.401
10.030, -21.391
10.066, 31.574];
lsc25hsc4_3 = [0,0];
lsr25hsr4_3 = [0,0];
for i = 1:length(ls25hs4_3)
    if ls25hs4_3(i,2) < 0
        lsr25hsr4_3 = [lsr25hsr4_3;ls25hs4_3(i,:)];
    else
        lsc25hsc4_3 = [lsc25hsc4_3;ls25hs4_3(i,:)];
    end
end
%%
ls0hs3 = [0.60240, -50.067
0.67787, -51.982
0.76194, -54.145
0.84601, -55.912
0.92244, 82.317
0.93008, -57.680
1.0065, 83.524
1.0112, -59.735
1.0906, 82.798
1.0982, -60.230
1.1746, 83.335
1.1823, -61.192
1.2587, 84.118
1.2664, -62.042
1.3428, 84.677
1.3504, -63.071
1.4269, 85.438
1.4345, -63.966
1.5109, 86.176
1.5186, -64.973
1.5950, 86.601
1.6026, -65.868
1.6791, 87.541
1.6867, -66.920
1.7631, 88.078
1.7708, -67.725
1.8472, 88.749
1.8548, -68.799
1.9284, 89.471
1.9389, -69.649
2.0153, 89.562
2.0230, -70.454
2.0994, 90.539
2.1071, -70.678
2.1835, 91.076
2.1911, -70.678
2.2675, 91.479
2.2752, -71.484
2.3516, 91.881
2.3593, -72.684
2.4357, 92.463
2.4433, -72.796
2.5198, 93.000
2.5274, -73.087
2.6038, 93.224
2.6115, -73.341
2.6879, 93.805
2.6955, -73.318
2.7720, 94.342
2.7796, -73.497
2.8560, 94.902
2.8637, -73.691
2.9401, 95.170
2.9477, -73.974
3.0242, 95.707
3.0318, -74.093
3.1159, -74.370
3.1235, 97.788
3.2000, -74.482
3.2076, 97.944
3.2840, -74.571
3.2917, 97.892
3.3681, -74.772
3.3757, 98.683
3.4522, -75.018
3.4598, 99.063
3.5362, -75.332
3.5439, 99.160
3.6203, -75.540
3.6279, 99.578
3.7044, -75.593
3.7120, 99.876
3.7884, -75.697
3.7961, 100.23
3.8725, -75.913
3.8802, 100.41
3.9598, -76.410
3.9668, 100.84
4.0406, -76.398
4.0483, 101.56
4.1247, -76.674
4.1324, 101.19
4.2088, -76.778
4.2164, 101.46
4.2929, -76.950
4.3005, 101.49
4.3769, -76.890
4.3846, 101.62
4.4610, -76.808
4.4686, 102.13
4.5451, -77.300
4.5527, 102.34
4.6291, -77.465
4.6368, 102.42
4.7132, -77.629
4.7208, 103.02
4.7973, -77.860
4.8049, 102.89
4.8813, -77.852
4.8890, 104.37
4.9680, -77.866
4.9750, 103.97
5.0495, -78.725
5.0571, 104.60
5.1336, -80.030
5.1412, 105.53
5.2176, -80.343
5.2253, 105.71
5.3017, -80.522
5.3093, 105.93
5.3858, -80.970
5.3934, 106.06
5.4698, -81.059
5.4775, 106.56
5.5539, -81.507
5.5615, 107.01
5.6380, -81.507
5.6456, 107.39
5.7220, -81.507
5.7297, 107.50
5.8061, -81.864
5.8137, 107.68
5.8902, -81.999
5.8978, 108.03
5.9784, -82.269
5.9848, 108.17
6.0583, -82.625
6.0660, 108.88
6.1424, -82.983
6.1500, 109.13
6.2265, -82.983
6.2341, 109.40
6.3105, -82.983
6.3182, 109.73
6.3946, -83.296
6.4022, 110.14
6.4787, -83.475
6.4863, 110.58
6.5627, -83.878
6.5704, 110.94
6.6468, -83.968
6.6544, 111.01
6.7309, -84.415
6.7385, 111.44
6.8149, -84.504
6.8226, 111.88
6.8990, -84.952
6.9067, 112.33
6.9831, -84.880
6.9907, 112.51
7.0671, -84.952
7.0748, 112.84
7.1512, -85.176
7.1589, 113.14
7.2353, -85.444
7.2429, 113.58
7.3194, -85.757
7.3270, 114.03
7.4034, -85.936
7.4111, 114.39
7.4875, -86.317
7.4951, 114.41
7.5716, -86.429
7.5792, 114.88
7.6556, -86.652
7.6633, 115.33
7.7397, -86.719
7.7473, 115.78
7.8238, -86.921
7.8314, 116.20
7.9078, -87.077
7.9155, 116.36
7.9919, -87.333
7.9996, 116.84
8.0760, -87.659
8.0836, 117.03
8.1600, -87.905
8.1677, 117.43
8.2441, -88.218
8.2518, 117.83
8.3282, -88.397
8.3358, 118.08
8.4123, -88.397
8.4199, 118.33
8.4963, -88.442
8.5040, 118.77
8.5804, -88.889
8.5880, 119.18
8.6645, -88.979
8.6721, 119.60
8.7485, -89.382
8.7562, 119.80
8.8326, -89.561
8.8402, 120.03
8.9167, -89.874
8.9243, 121.03
9.0007, -89.809
9.0066, 121.47
9.0848, -90.120
9.0925, 122.65
9.1689, -90.299
9.1765, 122.46
9.2529, -90.366
9.2606, 123.14
9.3370, -90.814
9.3447, 123.34
9.4211, -90.858
9.4287, 123.72
9.5052, -91.350
9.5128, 123.87
9.5892, -91.440
9.5969, 123.96
9.6733, -91.843
9.6809, 124.06
9.7574, -91.843
9.7650, 124.36
9.8414, -91.843
9.8491, 124.71
9.9255, -92.111
9.9331, 124.97
10.003, -93.500
10.018, 125.51
10.059, -94.427];
lsc0hsc3 = [0,0];
lsr0hsr3 = [0,0];
for i = 1:length(ls0hs3)
    if ls0hs3(i,2) < 0
        lsr0hsr3 = [lsr0hsr3;ls0hs3(i,:)];
    else
        lsc0hsc3 = [lsc0hsc3;ls0hs3(i,:)];
    end
end
%%
ls0hs2 = [0.26922, -37.590
0.35640, -37.929
0.36998, 51.123
0.42676, -39.846
0.45305, 53.532
0.51738, -41.871
0.54031, 55.815
0.62438, 56.719
0.63202, -42.298
0.70844, 57.651
0.71609, -43.433
0.79251, 58.390
0.80016, -44.055
0.87658, 59.105
0.88423, -44.592
0.96594, 59.722
0.97241, -45.401
1.0447, 59.866
1.0524, -45.733
1.1288, 60.179
1.1364, -46.068
1.2129, 60.672
1.2205, -46.359
1.2969, 61.208
1.3046, -47.433
1.3810, 61.499
1.3886, -47.985
1.4651, 61.835
1.4727, -48.283
1.5491, 62.372
1.5568, -48.216
1.6332, 62.797
1.6409, -48.470
1.7173, 63.155
1.7249, -48.828
1.8013, 63.580
1.8090, -48.939
1.8854, 64.005
1.8931, -49.312
1.9695, 64.609
1.9801, -50.002
2.0536, 64.833
2.0612, -49.558
2.1376, 65.213
2.1453, -49.760
2.2217, 65.504
2.3058, 65.817
2.3134, -49.894
2.3898, 66.198
2.3975, -50.140
2.4739, 66.645
2.4815, -50.185
2.5580, 66.869
2.5656, -50.625
2.6420, 67.115
2.6497, -50.506
2.7261, 67.473
2.7338, -50.849
2.8102, 67.920
2.8178, -50.998
2.8942, 68.233
2.9019, -51.192
2.9813, 68.030
2.9877, -51.471
3.0624, 68.905
3.0700, -51.192
3.1465, 69.106
3.1541, -51.199
3.2305, 69.598
3.2382, -51.512
3.3146, 70.471
3.3222, -51.721
3.3987, 70.836
3.4063, -51.796
3.4827, 71.239
3.4904, -52.444
3.5668, 71.918
3.5744, -52.512
3.6509, 72.611
3.6585, -51.885
3.7349, 72.865
3.7426, -52.303
3.8190, 72.820
3.8267, -52.474
3.9031, 73.133
3.9107, -52.303
3.9878, 73.517
3.9948, -52.885
4.0712, 73.745
4.0814, -52.993
4.1553, 73.849
4.1616, -53.582
4.2394, 74.222
4.2427, -53.749
4.3234, 74.177
4.3294, -53.269
4.4075, 74.267
4.4151, -55.174
4.4916, 74.535
4.4992, -55.174
4.5756, 74.736
4.5833, -55.398
4.6597, 74.856
4.6673, -55.420
4.7438, 75.147
4.7514, -55.532
4.8278, 75.348
4.8355, -55.912
4.9119, 75.549
4.9196, -55.912
4.9960, 75.646
5.0036, -56.225
5.0801, 75.728
5.0877, -56.404
5.1673, 75.911
5.1718, -56.404
5.2464, 77.256
5.2558, -56.628
5.3323, 76.235
5.3399, -56.651
5.4163, 76.489
5.4240, -56.718
5.5004, 76.765
5.5080, -56.897
5.5845, 77.004
5.5921, -56.897
5.6685, 77.071
5.6762, -57.187
5.7526, 76.922
5.7602, -57.389
5.8367, 77.175
5.8443, -57.389
5.9207, 77.294
5.9284, -57.859
6.0042, 78.014
6.0107, -57.424
6.0889, 78.010
6.0965, -57.971
6.1730, 78.256
6.1806, -58.127
6.2570, 78.100
6.2647, -58.127
6.3411, 77.966
6.3487, -58.463
6.4252, 78.316
6.4328, -58.619
6.5145, 77.871
6.5169, -58.619
6.5974, 77.834
6.6009, -58.865
6.6806, 79.524
6.6850, -58.865
6.7614, 80.673
6.7691, -59.044
6.8455, 81.075
6.8532, -59.358
6.9296, 81.165
6.9372, -59.358
7.0136, 81.368
7.0184, -60.024
7.0977, 81.568
7.1054, -59.850
7.1818, 81.903
7.1894, -59.872
7.2659, 82.149
7.2735, -60.096
7.3499, 82.395
7.3576, -60.096
7.4340, 82.574
7.4416, -60.275
7.5181, 82.641
7.5257, -60.588
7.6021, 82.888
7.6098, -60.588
7.6862, 83.022
7.6938, -60.767
7.7703, 83.380
7.7779, -60.834
7.8543, 83.603
7.8620, -60.879
7.9384, 83.872
7.9461, -61.326
8.0206, 83.589
8.0276, -61.504
8.1065, 84.140
8.1142, -61.505
8.1906, 84.610
8.1983, -61.819
8.2747, 84.655
8.2823, -61.796
8.3588, 84.856
8.3664, -61.975
8.4428, 85.102
8.4505, -62.065
8.5269, 85.349
8.5345, -62.087
8.6110, 85.684
8.6186, -62.311
8.6950, 85.841
8.7027, -62.311
8.7791, 86.087
8.7867, -62.535
8.8632, 86.154
8.8708, -62.803
8.9472, 86.579
8.9581, -63.158
9.0288, 86.242
9.0390, -63.122
9.1154, 86.825
9.1230, -63.295
9.1995, 87.138
9.2071, -63.318
9.2835, 87.317
9.2912, -63.541
9.3676, 87.742
9.3752, -63.541
9.4517, 87.832
9.4593, -63.765
9.5357, 88.056
9.5434, -64.033
9.6198, 88.168
9.6274, -64.033
9.7039, 88.302
9.7115, -64.235
9.7879, 88.593
9.7956, -64.280
9.8720, 88.794
9.8796, -64.347
9.9593, 89.002
9.9690, -65.491
10.039, -65.577];
lsc0hsc2 = [0,0];
lsr0hsr2 = [0,0];
for i = 1:length(ls0hs2)
    if ls0hs2(i,2) < 0
        lsr0hsr2 = [lsr0hsr2;ls0hs2(i,:)];
    else
        lsc0hsc2 = [lsc0hsc2;ls0hs2(i,:)];
    end
end
%%
ls0hs1 = [0.028247, 7.9156
0.057969, -5.7459
0.11041, 26.038
0.14471, -15.457
0.22218, 26.832
0.22101, -16.296
0.31676, 28.787
0.31103, -17.410
0.39510, -18.736
0.44095, 30.894
0.47057, -19.797
0.52502, 31.885
0.53266, -19.982
0.60909, 32.750
0.64730, -20.966
0.69316, 33.366
0.73137, -21.563
0.77723, 33.489
0.81544, -21.906
0.89951, -22.644
0.94537, 34.339
0.98485, -22.880
1.0269, 34.844
1.0313, -21.704
1.0762, -23.550
1.1135, 34.898
1.1517, -23.569
1.1976, 35.211
1.2358, -23.718
1.2816, 35.435
1.3199, -23.793
1.3657, 35.637
1.4039, -24.001
1.4498, 35.972
1.4880, -24.233
1.5339, 36.263
1.5721, -24.367
1.6179, 36.621
1.6561, -24.568
1.7020, 36.822
1.7402, -24.613
1.7861, 36.934
1.8243, -24.658
1.8701, 37.113
1.9083, -24.837
1.9574, 37.047
1.9930, -24.670
2.0383, 37.667
2.0765, -25.060
2.1223, 38.098
2.1606, -25.105
2.2064, 38.165
2.2446, -25.127
2.2905, 38.344
2.3287, -25.195
2.3745, 38.590
2.4128, -25.396
2.4586, 38.836
2.4968, -25.500
2.5427, 39.015
2.5809, -25.567
2.6268, 39.082
2.6650, -25.754
2.7108, 39.485
2.7490, -25.829
2.7949, 39.574
2.8331, -25.791
2.8790, 40.066
2.9172, -26.037
2.9662, 40.440
3.0007, -25.938
3.0471, 40.312
3.0853, -26.149
3.1312, 40.402
3.1694, -26.067
3.2152, 40.559
3.2535, -26.246
3.2993, 40.849
3.3375, -26.157
3.3834, 41.051
3.4216, -26.268
3.4674, 41.386
3.5057, -26.433
3.5515, 41.543
3.5897, -26.433
3.6356, 41.946
3.6738, -26.559
3.7197, 42.058
3.7579, -26.947
3.8037, 42.281
3.8419, -26.776
3.8878, 42.326
3.9260, -27.052
3.9738, 42.584
4.0088, -27.259
4.0559, 42.527
4.0941, -26.850
4.1400, 43.020
4.1782, -27.231
4.2241, 43.020
4.2623, -27.700
4.3081, 43.176
4.3464, -27.477
4.3922, 43.512
4.4304, -27.588
4.4763, 43.512
4.5145, -27.685
4.5604, 43.691
4.5986, -27.395
4.6444, 43.758
4.6826, -27.395
4.7285, 43.780
4.7667, -27.395
4.8126, 44.026
4.8508, -27.290
4.8966, 44.004
4.9348, -27.514
4.9820, 44.104
5.0170, -28.007
5.0648, 44.496
5.1030, -27.633
5.1488, 44.496
5.1900, -28.040
5.2329, 44.876
5.2711, -29.602
5.3170, 44.988
5.3552, -29.602
5.4042, 45.234
5.4393, -29.602
5.4729, 47.203
5.5240, -29.785
5.5539, 47.047
5.6093, -29.867
5.6380, 46.950
5.6915, -30.094
5.7220, 46.718
5.7773, -29.693
5.8061, 46.718
5.8596, -30.385
5.8902, 46.972
5.9437, -30.564
5.9784, 46.539
6.0277, -30.953
6.0583, 47.218
6.1118, -30.564
6.1424, 47.718
6.1959, -30.765
6.2265, 47.718
6.2800, -30.810
6.3154, 49.350
6.3640, -30.810
6.3946, 48.053
6.4481, -30.877
6.4787, 48.068
6.5322, -31.302
6.5627, 47.785
6.6162, -31.302
6.6468, 47.994
6.7003, -31.302
6.7309, 48.270
6.7844, -31.504
6.8149, 48.322
6.8684, -31.795
6.8990, 48.725
6.9525, -31.795
6.9860, 49.471
7.0366, -31.821
7.0671, 48.971
7.1206, -31.996
7.1512, 49.060
7.2047, -32.041
7.2353, 49.150
7.2888, -32.041
7.3194, 49.329
7.3729, -32.063
7.4034, 49.433
7.4569, -32.287
7.4875, 49.433
7.5410, -32.287
7.5716, 49.433
7.6251, -32.287
7.6556, 49.731
7.7091, -32.510
7.7397, 49.731
7.7932, -32.779
7.8238, 49.933
7.8773, -32.779
7.9078, 50.089
7.9645, -32.887
7.9925, 50.266
8.0454, -33.115
8.0760, 50.403
8.1295, -33.271
8.1600, 50.425
8.2135, -33.271
8.2441, 50.649
8.2976, -33.271
8.3282, 50.649
8.3817, -33.517
8.4123, 50.917
8.4658, -33.517
8.4963, 51.007
8.5498, -33.517
8.5804, 51.074
8.6339, -33.674
8.6645, 51.074
8.7180, -34.009
8.7485, 51.215
8.8020, -34.009
8.8326, 51.409
8.8861, -34.009
8.9167, 51.409
8.9743, -33.517
9.0007, 51.520
9.0542, -34.256
9.0848, 51.745
9.1383, -34.256
9.1689, 52.125
9.2224, -34.256
9.2529, 52.215
9.3064, -34.748
9.3370, 51.991
9.3905, -34.748
9.4211, 52.080
9.4746, -34.748
9.5052, 52.155
9.5587, -34.904
9.5892, 52.282
9.6427, -35.240
9.6733, 52.416
9.7268, -35.240
9.7574, 52.841
9.8109, -35.240
9.8414, 52.752
9.8949, -35.397
9.9255, 53.110
9.9790, -35.382
10.003, 51.460
10.043, -34.895];
lsc0hsc1 = [0,0];
lsr0hsr1 = [0,0];
for i = 1:length(ls0hs1)
    if ls0hs1(i,2) < 0
        lsr0hsr1 = [lsr0hsr1;ls0hs1(i,:)];
    else
        lsc0hsc1 = [lsc0hsc1;ls0hs1(i,:)];
    end
end
%%
ls0hs0 = [0.031304, -0.63240
0.12178, -2.9296
0.19851, -2.9463
0.21294, 7.2839
0.25896, -4.9185
0.28861, 9.4954
0.37981, 10.327
0.41802, -7.0728
0.46388, 11.206
0.50209, -7.6880
0.54795, 12.056
0.58616, -7.9006
0.63202, 12.511
0.67023, -7.8335
0.71609, 12.928
0.75430, -8.0796
0.80016, 12.906
0.83837, -8.6165
0.88423, 13.331
0.92244, -8.7507
0.96830, 13.174
1.0052, -8.6510
1.0524, 13.488
1.0906, -8.9073
1.1364, 13.532
1.1746, -9.0639
1.2205, 13.734
1.2587, -9.1534
1.3046, 13.734
1.3428, -9.1534
1.3886, 13.734
1.4269, -9.1534
1.4727, 13.801
1.5109, -9.4667
1.5568, 13.980
1.5950, -9.6456
1.6409, 13.980
1.6791, -9.6456
1.7249, 13.980
1.7631, -9.6456
1.8090, 14.114
1.8472, -10.004
1.8931, 14.472
1.9313, -10.138
1.9801, 14.258
2.0124, -9.9738
2.0612, 14.472
2.0994, -10.138
2.1453, 14.472
2.1835, -10.138
2.2293, 14.920
2.2675, -10.138
2.3134, 14.964
2.3516, -10.518
2.3975, 14.964
2.4357, -10.630
2.4815, 14.964
2.5198, -10.630
2.5656, 15.009
2.6038, -10.630
2.6497, 15.456
2.6879, -10.630
2.7338, 15.456
2.7720, -10.630
2.8178, 15.456
2.8560, -10.630
2.9019, 15.456
2.9401, -10.630
2.9877, 14.933
3.0201, -10.507
3.0700, 15.703
3.1082, -10.876
3.1541, 15.703
3.1923, -10.876
3.2382, 15.703
3.2764, -10.876
3.3222, 15.703
3.3605, -10.876
3.4063, 15.837
3.4445, -10.876
3.4904, 15.949
3.5286, -10.876
3.5744, 15.949
3.6127, -10.876
3.6585, 15.949
3.6967, -11.077
3.7426, 15.949
3.7808, -11.122
3.8267, 16.307
3.8649, -11.122
3.9107, 16.441
3.9489, -11.122
3.9948, 16.453
4.0330, -10.949
4.0789, 16.441
4.1171, -11.122
4.1629, 16.441
4.2011, -11.122
4.2470, 16.642
4.2852, -11.547
4.3311, 16.933
4.3693, -11.614
4.4151, 16.933
4.4534, -11.614
4.4992, 16.933
4.5374, -11.614
4.5833, 16.933
4.6215, -11.614
4.6673, 16.933
4.7056, -11.614
4.7514, 17.045
4.7896, -12.017
4.8355, 17.179
4.8737, -12.107
4.9196, 17.179
4.9610, -12.009
5.0030, 17.022
5.0418, -12.107
5.0877, 17.179
5.1259, -12.107
5.1718, 17.179
5.2100, -12.107
5.2558, 17.425
5.2940, -12.330
5.3399, 17.671
5.3781, -12.353
5.4240, 17.671
5.4622, -12.353
5.5080, 17.671
5.5463, -12.353
5.5921, 17.671
5.6303, -12.353
5.6762, 17.671
5.7144, -12.353
5.7602, 17.806
5.7985, -12.755
5.8443, 17.917
5.8825, -12.845
5.9284, 17.917
5.9719, -12.397
6.0107, 17.559
6.0507, -12.845
6.0965, 17.917
6.1347, -12.845
6.1806, 18.186
6.2188, -13.069
6.2647, 18.410
6.3029, -13.091
6.3487, 18.410
6.3869, -13.091
6.4328, 18.410
6.4710, -13.091
6.5169, 18.678
6.5551, -13.538
6.6009, 18.902
6.6392, -13.583
6.6850, 18.902
6.7232, -13.583
6.7691, 18.902
6.8073, -13.583
6.8532, 19.058
6.8914, -14.053
6.9372, 19.148
6.9795, -13.519
7.0184, 19.145
7.0595, -14.075
7.1054, 19.148
7.1436, -14.075
7.1894, 19.327
7.2276, -14.322
7.2735, 19.394
7.3117, -14.322
7.3576, 20.304
7.3958, -14.322
7.4416, 19.607
7.4799, -14.322
7.5257, 19.730
7.5639, -14.568
7.6098, 19.886
7.6480, -14.568
7.6938, 19.886
7.7321, -14.568
7.7779, 19.886
7.8161, -14.612
7.8620, 20.222
7.9002, -15.060
7.9461, 20.378
7.9843, -15.060
8.0301, 20.440
8.0683, -15.060
8.1142, 20.378
8.1524, -15.105
8.1983, 20.759
8.2365, -15.552
8.2823, 20.871
8.3205, -15.552
8.3664, 20.871
8.4046, -15.552
8.4505, 20.871
8.4887, -15.574
8.5345, 21.050
8.5728, -16.044
8.6186, 21.117
8.6568, -16.044
8.7027, 21.117
8.7409, -16.044
8.7867, 21.117
8.8250, -16.067
8.8708, 21.318
8.9090, -16.290
8.9549, 21.309
8.9937, -16.231
9.0390, 21.335
9.0772, -16.290
9.1230, 21.363
9.1612, -16.335
9.2071, 21.766
9.2453, -16.536
9.2912, 21.855
9.3294, -16.536
9.3752, 23.108
9.4134, -16.536
9.4593, 22.951
9.4975, -16.648
9.5434, 23.265
9.5816, -17.029
9.6274, 23.421
9.6657, -17.029
9.7115, 23.287
9.7497, -17.029
9.7956, 23.153
9.8338, -17.163
9.8796, 23.145
9.9179, -17.521
10.006, -16.363];
lsc0hsc0 = [0,0];
lsr0hsr0 = [0,0];
for i = 1:length(ls0hs0)
    if ls0hs0(i,2) < 0
        lsr0hsr0 = [lsr0hsr0;ls0hs0(i,:)];
    else
        lsc0hsc0 = [lsc0hsc0;ls0hs0(i,:)];
    end
end
%%
% here are the two output variables
% it is important to note the order of the variables inside the cell array
% and to look at the dyno curves for reference
cdamp = {lsc0hsc0,lsc0hsc1,lsc0hsc2,lsc0hsc3,lsc0hsc4_3,lsc2hsc4_3,lsc4hsc4_3,lsc6hsc4_3,lsc10hsc4_3,lsc15hsc4_3,lsc25hsc4_3};
rdamp = {lsr0hsr0,lsr0hsr1,lsr0hsr2,lsr0hsr3,lsr0hsr4_3,lsr2hsr4_3,lsr4hsr4_3,lsr6hsr4_3,lsr10hsr4_3,lsr15hsr4_3,lsr25hsr4_3};
%% MR
%B18 front Wingeo motion ratio
MR18Front = [-0.90531, 0.96078
-0.83294, 0.95419
-0.73221, 0.93943
-0.66740, 0.93433
-0.61946, 0.92811
-0.52993, 0.91901
-0.46514, 0.91323
-0.39729, 0.90693
-0.31770, 0.89990
-0.24628, 0.89463
-0.17179, 0.88826
-0.12404, 0.88294
-0.052416, 0.88005
0.017695, 0.87510
0.10675, 0.86924
0.18220, 0.86410
0.25266, 0.85989
0.32562, 0.85588
0.39857, 0.85138
0.47152, 0.84763
0.57100, 0.84199
0.64396, 0.83872
0.71857, 0.83514
0.78986, 0.83121
0.86282, 0.82812
0.91588, 0.82579];

MRx = MR18Front(:,1);% displacement
MR18F = MR18Front(:,2);% motion ratio