// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//static bool insideTriangle(int x, int y, const Vector3f* _v)//第三个数是_v指向Vector3f类型的指针，默认传入了三角形的三个顶点
//{
//    //.head(2)用来取出每个顶点的前两个值(x,y)
//    Eigen::Vector2f AB, BC, CA, AP, BP, CP, p;
//    float a, b, c;
//    p << x, y;
//    AB = _v[1].head(2) - _v[0].head(2);
//    AP = p - _v[0].head(2);
//    BC = _v[2].head(2) - _v[1].head(2);
//    BP = p - _v[1].head(2);
//    CA = _v[0].head(2) - _v[2].head(2);
//    CP = p - _v[2].head(2);
//    a = AB[0] * AP[1] - AB[1] * AP[0];
//    b = BC[0] * BP[1] - BC[1] * BP[0];
//    c = CA[0] * CP[1] - CA[1] * CP[0];
//    if (a > 0 && b > 0 && c > 0) return true;
//    else if (a < 0 && b < 0 && c < 0) return true;
//    return false;
//}

static bool insideTriangle(int x, int y, const Vector3f* v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f ba(v[0].x()-v[1].x(), v[0].y()-v[1].y(), 0);
    Vector3f cb(v[1].x()-v[2].x(), v[1].y()-v[2].y(), 0);
    Vector3f ac(v[2].x()-v[0].x(), v[2].y()-v[0].y(), 0);
//    std::vector<Eigen::Vector3f> vec;
    Vector3f vec[3];
    for(int i = 0; i < 3; i ++)  vec[i] << x - v[i].x(), y - v[i].y(), 0;
//    std::vector<Eigen::Vector3f> cross;
    Vector3f cross[3];
    cross[0] = vec[0].cross(ac);
    cross[1] = vec[1].cross(ba);
    cross[2] = vec[2].cross(cb);
   if(cross[0].dot(cross[1]) > 0 && cross[1].dot(cross[2]) > 0 && cross[2].dot(cross[0]) > 0) return true;
   else if(cross[0].dot(cross[1]) < 0 && cross[1].dot(cross[2]) < 0 && cross[2].dot(cross[0]) < 0) return true;
   else return false;        
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

////Screen space rasterization
//void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    //auto v = t.toVector4();
    //
    //// TODO : Find out the bounding box of current triangle.
    //// iterate through the pixel and find if the current pixel is inside the triangle
    //int maxx = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    //int minx = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    //
    //int miny = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    //int maxy = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));
    //for(int i = minx; i <= maxx; i ++)
	//for(int j = miny; j <= maxy; j ++)
	//{
	    //if(insideTriangle(i, j, t.v))
	    //{
	        //auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
            //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            //z_interpolated *= w_reciprocal;
            //if(-z_interpolated < depth_buf[get_index(i, j)])
            //{
                //depth_buf[get_index(i, j)] = -z_interpolated;
                //Vector3f point(i, j, 1);
                //set_pixel(point, t.getColor());
            //}
	    //}    
	//}
    //// If so, use the following code to get the interpolated z value.
    ////auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    ////float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    ////float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    ////z_interpolated *= w_reciprocal;
//
    //// TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
//}


//MSAA
//void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    //auto v = t.toVector4();
    //
    //// TODO : Find out the bounding box of current triangle.
    //// iterate through the pixel and find if the current pixel is inside the triangle
    //float alpha, beta, gamma, lmin=INT_MAX, rmax=INT_MIN, tmax=INT_MIN, bmin=INT_MAX, mindep = INT_MAX, count = 0;
    //// TODO : Find out the bounding box of current triangle.
    //for(auto &k:v){//找到bounding box的边界坐标
        //lmin = int(std::min(lmin,k.x()));
        //rmax = std::max(rmax,k.x());rmax = rmax == int(rmax) ? int(rmax)-1 : rmax;
        //tmax = std::max(tmax,k.y());tmax = tmax == int(tmax) ? int(tmax)-1 : tmax;
        //bmin = int(std::min(bmin,k.y()));
    //}
    //std::vector<float> a{0.25, 0.25, 0.75, 0.75, 0.25};
    //for(float i = lmin; i <= rmax; i ++)
    //{
        //for(float j = bmin; j <= tmax; j ++)
        //{
            //count = 0;
            //for(int k = 0; k < 4; k ++)
            //{
                //if(insideTriangle(i+a[k], j+a[k+1], t.v))
                //{
                    //auto[alpha, beta, gamma] = computeBarycentric2D(i+a[k], j+a[k+1], t.v);
                    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    //z_interpolated *= w_reciprocal;
                    //mindep = std::min(mindep, -z_interpolated);
                    //count += 0.25;
                //}    
           //}
//
            //if(count > 0 && mindep < depth_buf[get_index(i, j)])
            //{
                //depth_buf[get_index(i, j)] = mindep;
                //Vector3f point(i, j, 1);
                //set_pixel(point, t.getColor() * count);
            //}
        //}
    //}
    //// If so, use the following code to get the interpolated z value.
    ////auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    ////float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    ////float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    ////z_interpolated *= w_reciprocal;
//}

void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    Vector3f color; 
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    float alpha, beta, gamma, lmin=INT_MAX, rmax=INT_MIN, tmax=INT_MIN, bmin=INT_MAX, mindep = INT_MAX, count = 0, eid;
    // TODO : Find out the bounding box of current triangle.
    for(auto &k:v){//找到bounding box的边界坐标
        lmin = int(std::min(lmin,k.x()));
        rmax = std::max(rmax,k.x());rmax = rmax == int(rmax) ? int(rmax)-1 : rmax;
        tmax = std::max(tmax,k.y());tmax = tmax == int(tmax) ? int(tmax)-1 : tmax;
        bmin = int(std::min(bmin,k.y()));
    }
    std::vector<float> a{0.25, 0.25, 0.75, 0.75, 0.25};
    for(float i = lmin; i <= rmax; i ++)
    {
        for(float j = bmin; j <= tmax; j ++)
        {
            count = 0;mindep = INT_MAX;eid = get_index(i, j) * 4;
            for(int k = 0; k < 4; k ++)
            {
                if(insideTriangle(i+a[k], j+a[k+1], t.v))
                {
                    auto[alpha, beta, gamma] = computeBarycentric2D(i+a[k], j+a[k+1], t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if(-z_interpolated < depth_sample[eid + k])
                    {
                        depth_sample[eid + k] = -z_interpolated;
                        frame_sample[eid + k] = t.getColor()/4;
                    }
                    mindep = std::min(mindep, depth_sample[eid + k]);
                }    
           }
            color = frame_sample[eid] + frame_sample[eid + 1] + frame_sample[eid + 2] + frame_sample[eid + 3];
            set_pixel({i, j, 1}, color);
            depth_buf[get_index(i, j)] = std::min(depth_buf[get_index(i, j)], mindep);

        }
    }
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;
}



void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_sample.begin(), frame_sample.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_sample.begin(), depth_sample.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_sample.resize(w * h * 4);
    depth_sample.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on
