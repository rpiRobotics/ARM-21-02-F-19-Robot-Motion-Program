# The Input Curve

The scope of the project is to take in a curve and find the best relative pose to the robots in term of traveling speed and speed uniformity, and then robot command generation and optimization. The generation of such curve is relied on each user. However, we here provided the specification of such curve and tips to generate such curve.

## Curve Specification

The curve (path) is discretized to `N` points. Each point is described by a position `(x,y,z)` and the normal direction `(i,j,k)` which is opposite to the `z-direction` of the robot (that hold the tool, e.g. spray gun) tcp. The cartesian frame of the curve can be `arbitrary` because it will be resolve in the redundancy resolution step. 

An portion of such curve is like this. The first three number is `(x,y,z)` and the last three number is `(i,j,k)`. 
```
...
1.9316380763831253,0.001054979200265463,25.39999998904546,-0.0,-2.076730708736155e-05,-0.9999999997843596
1.952187417621244,0.001077544961782589,25.39999998857182,-0.0,-2.121151499094843e-05,-0.9999999997750358
1.9727367588593623,0.001100349514426261,25.39999998808298,-0.0,-2.1660423507246714e-05,-0.999999999765413
1.9932861000974806,0.0011233928581964457,25.39999998757862,-0.0,-2.21140326362551e-05,-0.9999999997554847
2.0138354413355986,0.001146674993093109,25.399999987058422,-0.0,-2.2572342377972192e-05,-0.9999999997452446
...
```

Example files are [curve_1](https://github.com/rpiRobotics/ARM-21-02-F-19-Robot-Motion-Program/blob/main/data/curve_1/Curve_dense.csv) and [curve_2](https://github.com/rpiRobotics/ARM-21-02-F-19-Robot-Motion-Program/blob/main/data/curve_2/Curve_dense.csv). We discretized the curves to `N=50000` points, namely, there are 50000 lines in the files.

## Tips on generate the curve
1. Try to discretize the curve with equally space points.
2. The larger the `N`, the more accurate the algorithm can be. However, it will be more time consuming as well. The tested curves are 1 meter and around 1.7 meter repectively. Therefore, if you have no where to start, you can sample the curve every `1/50000`~`1.7/50000` meter.