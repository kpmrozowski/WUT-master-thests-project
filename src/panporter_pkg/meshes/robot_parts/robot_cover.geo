//+
SetFactory("OpenCASCADE");
Cylinder(1) = {-0.2, 0, 0, 0.4, 0, 0, 0.3, 2*Pi};
Rotate {{0, 0, 1}, {0, 0, 0}, Pi/2} { Volume{1};  }
//+
Box(2) = {-0.25, -0.5, -0.05, 0.5, 1, 0.01};
//+
Box(3) = {-0.5, -0.18, -0.05, 1, 0.36, 0.01};
//+
BooleanUnion{ Volume{2}; Delete; }{ Volume{3}; Delete; }
BooleanDifference{ Volume{1}; Delete; }{ Volume{2}; Delete; }
