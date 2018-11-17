function crossarray=crossarrayfun(X)%定义函数，求列阵的交叉反对称阵
crossarray=[0     -X(3)  X(2);
            X(3)  0      -X(1);
            -X(2) X(1)   0    ];
