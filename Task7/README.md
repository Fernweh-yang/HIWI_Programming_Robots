# Problems not solved

**Miss a valid desired pose**:

1. The desired pose I used in task7_AIK.cpp is :

       0.232   0.316   -0.311   -0.32
       -0.434   0.063   0.252   -0.249
       0.22    -0.31    0.452    0.285
       0       0       0        1

   I generate this matrix by random numbers. But this matrix seems to be invalid, because the result are all **NAN**.

2. The desired pose I used in task7_pinocchio.cpp is:

   ```
   1 0 0 1
   0 1 0 1
   0 0 1 1
   0 0 0 1
   ```

   This matrix seems to be also invalid, because the error between the desired pose and the current one can't converge. 