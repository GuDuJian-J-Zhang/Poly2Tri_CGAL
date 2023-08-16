Poly2Tri Based On CGAL

#Usage
```
#include <poly2tri/inc/cdt.h>
p2t_cgal::SetLoggingCallback([](const std::string& info)->void {
		GWRENDERLOGD(info.c_str());
	});

std::vector<p2t_cgal::Point> tPolyline;
// ... prepare your polygon //
p2t_cgal::CDT tCdt(tPolyline);
// optional include several holes
std::vector<p2t_cgal::Point> tHole;
tCdt.Triangulate();

// after that, you can get all triangles by:
const auto& ts = tCdt.GetTriangles();
// of writ out the result mesh into *.STL
tCdt.WriteToSTL("test.stl");
```

![mesh1](https://github.com/GuDuJian-J-Zhang/Poly2Tri_CGAL/blob/main/img/mesh1.jpg)
