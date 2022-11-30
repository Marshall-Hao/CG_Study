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


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    // * 三角形三条边的向量
    // * 用eigen库定义
    Eigen::Vector2f p0_p1;
    Eigen::Vector2f p1_p2;
    Eigen::Vector2f p2_p0;
    p0_p1 << _v[1].x() - _v[0].x(), _v[1].y() - _v[0].y();
    p1_p2 << _v[2].x() - _v[1].x(), _v[2].y() - _v[1].y();
    p2_p0 << _v[0].x() - _v[2].x(), _v[0].y() - _v[2].y();

    // * 想找的那个点与三角形三个点形成的向量
    Eigen::Vector2f p0_q;
    Eigen::Vector2f p1_q;
    Eigen::Vector2f p2_q;

    p0_q << x - _v[0].x(), y - _v[0].y();
    p1_q << x - _v[1].x(), y - _v[1].y();
    p2_q << x - _v[2].x(), y - _v[2].y();

    // * 通过右手定则 获取 三条边向量与 确定顶点与点向量的叉积
    float p0_p1_p1_q = p0_p1.x() * p1_q.y() - p0_p1.y() * p1_q.x();
    float p1_p2_p2_q = p1_p2.x() * p2_q.y() - p1_p2.y() * p2_q.x();
    float p2_p0_p0_q = p2_p0.x() * p0_q.y() - p2_p0.y() * p0_q.x();

    // * 如果他们的结果都是大于0 或 小于 0 则代表 是在同一边
    return (p0_p1_p1_q > 0 && p1_p2_p2_q > 0 && p2_p0_p0_q > 0 ) || (p0_p1_p1_q < 0 && p1_p2_p2_q < 0 && p2_p0_p0_q < 0 );
   
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

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // * 三角形的三个向量放到一个 4*4的 其次matrix里了
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // * 获取装这个三角形的一个盒子，正正好好囊括
    float min_x = std::min(v[0][0], std::min(v[1][0],v[2][0])); // * 三点 x 最小值 ，以下以此类推
    float max_x = std::max(v[0][0], std::max(v[1][0],v[2][0]));   
    float min_y = std::min(v[0][1], std::min(v[1][1],v[2][1]));
    float max_y = std::max(v[0][1], std::max(v[1][1],v[2][1]));

    // * 做一下取值优化，小的向下取整，大的向上取整，让其确保能囊括
    min_x = static_cast<int>(std::floor(min_x));
    min_y = static_cast<int>(std::floor(min_y));
    max_x = static_cast<int>(std::ceil(max_x));
    max_y = static_cast<int>(std::ceil(max_y));

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;
    bool MSAA = true;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    if (MSAA) 
    {   
        std::vector<Eigen::Vector2f> pos
        {                               //对一个像素分割四份 当然你还可以分成4x4 8x8等等甚至你还可以为了某种特殊情况设计成不规则的图形来分割单元
            {0.25,0.25},                //左下
            {0.75,0.25},                //右下
            {0.25,0.75},                //左上
            {0.75,0.75}                 //右上
        };
        for (int i = min_x; i <= max_x; ++i)
        {
            for (int j = min_y; j <= max_y; ++j)
            {
                int count = 0;
                float minDepth = FLT_MAX;
                for (int MSAA_4 = 0; MSAA_4 < 4; ++MSAA_4)
                {
                    if (insideTriangle(static_cast<float>(i+pos[MSAA_4][0]), static_cast<float>(j+pos[MSAA_4][1]),t.v))
                    {
			    auto[alpha, beta, gamma] = computeBarycentric2D(static_cast<float>(i + pos[MSAA_4][0]), static_cast<float>(j + pos[MSAA_4][1]), t.v);
	                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
	                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
	                    z_interpolated *= w_reciprocal;

                        minDepth = std::min(minDepth, z_interpolated);
                        ++count;
                    }
                }
                if (count)
                {
                    if (depth_buf[get_index(i, j)] > minDepth)
                    {
                        depth_buf[get_index(i, j)] = minDepth;//更新深度

                        Eigen::Vector3f color = t.getColor() * (count / 4.0);//对颜色进行平均，使得边界更平滑，也是一种模糊的手段
                        Eigen::Vector3f point;
                        point << static_cast<float>(i), static_cast<float>(j), minDepth;
                        set_pixel(point, color);//设置颜色
                    }
                }
            }
        }
        // // * 2 x 2 supersampling
        // std::vector<Eigen::Vector2f> pos
        // {                               //* 对一个像素分割四份 当然你还可以分成4x4 8x8等等甚至你还可以为了某种特殊情况设计成不规则的图形来分割单元
        //     {0.25,0.25},                // * 左下 中心点
        //     {0.75,0.25},                //右下
        //     {0.25,0.75},                //左上
        //     {0.75,0.75}               //右上
        // };
        //  {
        // for (int i = min_x; i <=max_x; ++i) 
        // {   
        //     for (int j = min_y; j <= max_y; ++j) 
        // // * 遍历这个囊括此图形的盒子的每个点
        //     {   
        //         int count = 0;
        //         float minDepth = FLT_MAX; // * 无限大
        //         // * 遍历每个点的 四个卷积像素
        //         for (int MSAA_4 = 0; MSAA_4 < 4; ++MSAA_4) {
        //         // * 先看看这个像素点(点中心) 在不在这个三角形里面
        //         if (insideTriangle(static_cast<float>(i+pos[MSAA_4][0]), static_cast<float>(j+pos[MSAA_4][1]),t.v)) 
        //         {   
        //             // * 获取 每个点的 z深度
        //             auto[alpha, beta, gamma] = computeBarycentric2D(static_cast<float>(i+pos[MSAA_4][0]), static_cast<float>(j+pos[MSAA_4][1]), t.v);
        //             float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
        //             float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
        //             z_interpolated *= w_reciprocal;
        //             // * 如果现在找的这个点的z深度 是小于之前这个点的z深度的话，他不会被阻挡，就替代
        //             minDepth = std::min(minDepth, z_interpolated);
        //             ++count;
        //         }
        //         }
        //         // * 那一个像素块的四个卷积像素都弄完后，就来找他们的平均颜色了
        //         if (count) 
        //         {
        //             if (minDepth < depth_buf[get_index(i, j)]) 
        //             {   
        //                 // * 这个像素块的z 深度 替代
        //                 depth_buf[get_index(i, j)] = minDepth;
        //                 // * 进行这个三角形内部点的上色
        //                 Eigen::Vector3f color = t.getColor() * (count / 4.0);// * 对颜色进行平均，使得边界更平滑，也是一种模糊的手段
        //                 Eigen::Vector3f point;
        //                 // * 每个要上色像素块的坐标
        //                 point << static_cast<float>(i), static_cast<float>(j), minDepth;
        //                 set_pixel(point, color);
        //             }
        //         }
        //     }
        // }
    }

    else 
    {
        for (int i = min_x; i <=max_x; ++i) 
    {
        for (int j = min_y; j <= max_y; ++j) 
        // * 遍历这个囊括此图形的盒子的每个点
        {   
            // * 先看看这个像素点(点中心) 在不在这个三角形里面
            if (insideTriangle(static_cast<float>(i+0.5), static_cast<float>(j+0.5),t.v)) 
            {   
                // * 获取 每个点的 z深度
                auto[alpha, beta, gamma] = computeBarycentric2D(static_cast<float>(i+0.5), static_cast<float>(j+0.5), t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                // * 如果现在找的这个点的z深度 是小于之前这个点的z深度的话，他不会被阻挡，就替代
                if (z_interpolated < depth_buf[get_index(i, j)]) 
                {   
                    // * 替代
                    depth_buf[get_index(i, j)] = z_interpolated;
                    // * 进行这个三角形内部点的上色
                    set_pixel(Eigen::Vector3f(i,j,z_interpolated), t.getColor());
                }
            }
        }
    }
    }
    
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
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
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