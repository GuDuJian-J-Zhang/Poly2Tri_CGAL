#pragma once

#include <vector>
#include <memory>
#include <string>
#include "export.h"

namespace p2t_cgal 
{

struct Point
{
    double x{ 0.0f };
    double y{ 0.0f };
    double z{ 0.0f };
};

struct Triangle
{
    Point mPoints[3];
};

struct CDTContext;

class CDT
{
public:
    /**
   * Constructor - add polyline with non repeating points
   *
   * @param polyline
   */
    POLY2TRI_API CDT(const std::vector<Point>& polyline);

    /**
    * Destructor - clean up memory
    */
    POLY2TRI_API ~CDT();

    /**
     * Add a hole
     *
     * @param polyline
     */
    POLY2TRI_API void AddHole(const std::vector<Point>& polyline);

    /**
     * Add a steiner point
     *
     * @param point
     */
    POLY2TRI_API void AddPoint(const Point& point);

    /**
     * Triangulate - do this AFTER you've added the polyline, holes, and Steiner points
     */
    POLY2TRI_API void Triangulate();

    /**
     * Get CDT triangles
     */
    POLY2TRI_API const std::vector<Triangle>& GetTriangles();

    POLY2TRI_API bool WriteToSTL(const std::string& filePath) const;

private:
    std::unique_ptr<CDTContext> mpContext{ nullptr };
};

}