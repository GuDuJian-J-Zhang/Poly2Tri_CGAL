#include <cdt.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/Polygon_2.h>

namespace p2t_cgal
{
    struct CDTContext
    {
        typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
        typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
        typedef CGAL::Constrained_triangulation_face_base_2<K>            Fb;
        typedef CGAL::Triangulation_data_structure_2<Vb, Fb>              TDS;
        typedef CGAL::Exact_predicates_tag                                Itag;
        typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CGAL_CDT;
        typedef CGAL_CDT::Face_handle                                          Face_handle;
        typedef CGAL_CDT::Point                                                Point;
        typedef CGAL::Polygon_2<K>                                        Polygon_2;

        CGAL_CDT mCdt;

        std::vector<p2t_cgal::Point> mPloygon;
        std::vector<std::vector<p2t_cgal::Point>> mHoles;

        std::vector<p2t_cgal::Triangle> mMesh;

        void Triangulate()
        {
            mCdt.clear();
            mMesh.clear();

            //construct two non-intersecting nested polygons
            CDTContext::Polygon_2 polygon1;
            for (size_t i = 0; i < mPloygon.size(); ++i)
            {
                polygon1.push_back(CDTContext::Point(mPloygon[i].x, mPloygon[i].y));
            }
            mCdt.insert_constraint(polygon1.begin(), polygon1.end(), true);

            for (size_t i = 0; i < mHoles.size(); ++i)
            {
                CDTContext::Polygon_2 polygon1;
                for (size_t j = 0; j < mHoles[i].size(); ++j)
                {
                    polygon1.push_back(CDTContext::Point(mHoles[i][j].x, mHoles[i][j].y));
                }
                mCdt.insert_constraint(polygon1.begin(), polygon1.end(), true);
            }

            std::unordered_map<Face_handle, bool> in_domain_map;
            boost::associative_property_map< std::unordered_map<Face_handle, bool> >
                in_domain(in_domain_map);

            //Mark facets that are inside the domain bounded by the polygon
            CGAL::mark_domain_in_triangulation(mCdt, in_domain);

            unsigned int count = 0;
            for (Face_handle f : mCdt.finite_face_handles())
            {
                if (get(in_domain, f))
                {
                    ++count;
                    auto p = f->vertex(0);
                    const Point pp = p->point();
                    p2t_cgal::Triangle tTriangle;
                    for (int i = 0; i < 3; ++i)
                    {
                        const auto& tPoint = f->vertex(i)->point();
                        tTriangle.mPoints[i].x = tPoint.x();
                        tTriangle.mPoints[i].y = tPoint.y();
                    }

                    mMesh.emplace_back(tTriangle);
                }
            }
        }
    };

    CDT::CDT(const std::vector<Point>& polyline)
        : mpContext(std::make_unique<CDTContext>())
    {
        mpContext->mPloygon = polyline;
    }

    CDT::~CDT()
    {
    }

    void CDT::AddHole(const std::vector<Point>& polyline)
    {
        mpContext->mHoles.emplace_back(polyline);
    }

    void CDT::AddPoint(const Point& point)
    {
    }

    void CDT::Triangulate()
    {
        mpContext->Triangulate();
    }

    const std::vector<Triangle>& CDT::GetTriangles()
    {
        return mpContext->mMesh;
    }
}

#ifdef POLY2TRI_BUILD_AS_EXECUTABLE
void main()
{
    std::vector<p2t_cgal::Point> tPolyline;
    tPolyline.emplace_back(p2t_cgal::Point{ 0, 0 });
    tPolyline.emplace_back(p2t_cgal::Point{ 2, 0 });
    tPolyline.emplace_back(p2t_cgal::Point{ 2, 2 });
    tPolyline.emplace_back(p2t_cgal::Point{ 1, 1.75 });
    tPolyline.emplace_back(p2t_cgal::Point{ 0, 2 });

    p2t_cgal::CDT tCdt(tPolyline);

    tCdt.Triangulate();

    tCdt.GetTriangles();
}
#endif // POLY2TRI_BUILD_AS_EXECUTABLE
