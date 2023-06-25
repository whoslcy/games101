#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float rotation_radian = rotation_angle / 180 * MY_PI;
    float cosine_value = std::cos(rotation_radian);
    float sine_value = std::sin(rotation_radian);
    Eigen::Matrix2f rotation_matrix {
        {cosine_value, -sine_value},
        {sine_value, cosine_value},
    };
    model.topLeftCorner<2, 2>() = rotation_matrix;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f perspective_to_orthographic {
        {zNear, 0, 0, 0},
        {0, zNear, 0, 0},
        {0, 0, zNear + zFar, -zNear * zFar},
        {0, 0, 1, 0},
    };
    float yTop = 2 * std::abs(zNear) * std::tan(eye_fov / 2);
    float yBottom = -yTop;
    float xRight = aspect_ratio * yTop;
    float xLeft = -xRight;
    Eigen::Matrix4f orthographic_scale {
        {2 / (xRight - xLeft), 0, 0, 0},
        {0, 2 / (yTop - yBottom), 0, 0},
        {0, 0, 2 / (zNear - zFar), 0},
        {0, 0, 0, 1},
    };
    Eigen::Matrix4f orthographic_translate {
        {1, 0, 0, -(xRight + xLeft) / 2},
        {0, 1, 0, -(yTop + yBottom) / 2},
        {0, 0, 1, -(zNear + zFar) / 2},
        {0, 0, 0, 1},
    };
    return orthographic_scale * orthographic_translate * perspective_to_orthographic;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float rotation_angle){
    Eigen::Matrix4f rodrigues_rotation = Eigen::Matrix4f::Identity();
    float rotation_radian = rotation_angle / 180 * MY_PI;
    float cosine_value = std::cos(rotation_radian);
    float sine_value = std::sin(rotation_radian);
    float n_x = axis(0);
    float n_y = axis(1);
    float n_z = axis(2);
    Eigen::Matrix3f N {
        {0, -n_z, n_y},
        {n_z, 0, -n_x},
        {-n_y, n_x, 0},
    };
    rodrigues_rotation.topLeftCorner<3, 3>() = cosine_value * Eigen::Matrix3f::Identity() + (1 - cosine_value) * axis * axis.transpose() + sine_value * N;
    return rodrigues_rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        // zNear and zFar should be negative.
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
