//
// Created by t-idkess on 18-Mar-18.
//

#ifndef INC_5_CGAL_DEFINES_H
#define INC_5_CGAL_DEFINES_H

#include <CGAL/Gmpq.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Arr_trapezoid_ric_point_location.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Fuzzy_iso_box.h>
//#include <CGAL/Boolean_set_operations_2.h>
//#include <CGAL/enum.h>
//#include <CGAL/bounding_box.h>

typedef typename double Number_type;
typedef typename CGAL::Cartesian<Number_type> Kernel;
typedef typename Kernel::FT FT;
typedef typename Kernel::Point_2 Point_2;
typedef typename Kernel::Segment_2 Segment_2;
typedef typename Kernel::Vector_2 Vector_2;
typedef typename Kernel::Line_2 Line_2;
typedef typename CGAL::Polygon_2<Kernel> Polygon_2;
typedef typename CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;
typedef typename CGAL::Polygon_set_2<Kernel> Polygon_set_2;
typedef typename Polygon_set_2::Arrangement_2 Arrangement_2;
typedef typename CGAL::Arr_trapezoid_ric_point_location<Polygon_set_2::Arrangement_2> Trap_pl;
typedef typename CGAL::Kd_tree<CGAL::Search_traits_2<Kernel>> KdTree;
typedef typename CGAL::Fuzzy_iso_box<CGAL::Search_traits_2<Kernel>> Fuzzy_iso_box;

#endif //INC_5_CGAL_DEFINES_H
