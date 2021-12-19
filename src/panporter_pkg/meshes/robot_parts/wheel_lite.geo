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
Torus(1) = {0, 0, 0, 330, 20, 2*Pi};
//+
Rotate {{1, 0, 0}, {0, 0, 0}, Pi/2} {
  Volume{1}; 
}
