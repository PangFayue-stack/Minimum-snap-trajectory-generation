clear;
clc;
path1 = [0 0 0; ...
         0.0 0.0 1.0 ; ...
         1.0 1.0 1.0 ; ...
         -1.0 2.0 1.0 ; ...
         1.0 3.0 1.0 ; ...
         -1.0 4.0 1.0 ; ...
         1.0 5.0 1.0 ; ...
         -1.0 6.0 1.0 ; ...
         1.0 7.0 1.0 ; ...
         -1.0 8.0 1.0 ; ...
         1.0 9.0 1.0 ; ...
         0.0 10.0 1.0 ; ];
arrangeT(path1)
M = getM(11,7,arrangeT(path1));