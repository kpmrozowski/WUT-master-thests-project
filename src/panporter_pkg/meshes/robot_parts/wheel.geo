// Gmsh project created on Fri Dec 10 15:26:12 2021
SetFactory("OpenCASCADE");
//+
SetFactory("OpenCASCADE");
//+
SetFactory("OpenCASCADE");
//+
SetFactory("Built-in");
//+
SetFactory("OpenCASCADE");
Torus(1) = {0, 0, 0, 0.33, 0.02, 2*Pi};
//+
Rotate {{1, 0, 0}, {0, 0, 0}, Pi/2} {
  Volume{1}; 
}
//+
Box(2) = {-0.33, -0.01, -0.002, 0.66, 0.02, 0.004};
Box(3) = {-0.33, -0.01, -0.002, 0.66, 0.02, 0.004};
Box(4) = {-0.33, -0.01, -0.002, 0.66, 0.02, 0.004};
Box(5) = {-0.33, -0.01, -0.002, 0.66, 0.02, 0.004};
//+
Rotate {{0, 1, 0}, {0, 0, 0}, Pi/4} { Volume{3}; }
Rotate {{0, 1, 0}, {0, 0, 0}, Pi/2} { Volume{4}; }
Rotate {{0, 1, 0}, {0, 0, 0}, 3*Pi/4} { Volume{5}; }
//+
BooleanUnion{ Volume{1}; }{ Volume{2}; Delete; }
BooleanUnion{ Volume{1}; }{ Volume{3}; Delete; }
BooleanUnion{ Volume{1}; }{ Volume{4}; Delete; }
BooleanUnion{ Volume{1}; }{ Volume{5}; Delete; }
//+
BooleanUnion{ Surface{45}; }{ Surface{50}; Delete; }
//+
BooleanUnion{ Surface{47}; }{ Surface{52}; Delete; }
