/*
 * pnp_test:
 *
 * Example of OpenCV's solvePnP() function. Takes 3D points, projects them to
 * image coordinates, and then recovers the model-view transform using
 * solvePnP().
 *
 * Order of points matters: object points and image points must correspond.
 */

#include "util/math.hpp"
#include "pixart/camera_parameters.hpp"
#include <opencv2/opencv.hpp>

static cv::Mat transform_matrix(cv::Point3f euler, cv::Point3f xlat)
{
  using namespace util::math;

  // Construct the rotation matrices and concatenate them into one 4x4
  float ax = euler.x * Deg2Rad;
  float ay = euler.y * Deg2Rad;
  float az = euler.z * Deg2Rad;
  cv::Mat rot_x = (cv::Mat_<float>(4, 4) <<
    1, 0, 0, 0,
    0, cos(ax), -sin(ax), 0,
    0, sin(ax), cos(ax), 0,
    0, 0, 0, 1);
  cv::Mat rot_y = (cv::Mat_<float>(4, 4) <<
    cos(ay), 0, sin(ay), 0,
    0, 1, 0, 0,
    -sin(ay), 0, cos(ay), 0,
    0, 0, 0, 1);
  cv::Mat rot_z = (cv::Mat_<float>(4, 4) <<
    cos(az), -sin(az), 0, 0,
    sin(az), cos(az), 0, 0,   
    0, 0, 1, 0,
    0, 0, 0, 1);
  cv::Mat rotation = rot_x * rot_y * rot_z;

  cv::Mat rot3(rotation, cv::Range(0,3), cv::Range(0,3));
  cv::Mat rodrigues(3, 1, CV_32F);
  cv::Rodrigues(rot3, rodrigues);

  // Construct a 4x4 translation matrix
  cv::Mat translation = (cv::Mat_<float>(4, 4) << 
    1, 0, 0, xlat.x,
    0, 1, 0, xlat.y,
    0, 0, 1, xlat.z,
    0, 0, 0, 1);

  // Construct a 4x4 transformation matrix
  cv::Mat transform = translation * rotation;
  return transform;
}

static std::vector<cv::Point2f> generate_image_points(std::vector<cv::Point3f> object_points, cv::Mat intrinsic, cv::Mat modelview4x4)
{
  // Convert 4x4 modelview into 3x4 (we don't need the fourth row and the
  // camera intrinsic matrix is 3x3)
  cv::Mat modelview(modelview4x4, cv::Range(0,3), cv::Range(0,4));

  // Transform points to camera space, using transformation matrix, then image
  // space using intrinsic matrix
  std::vector<cv::Point2f> image_points;
  for (auto point: object_points)
  {
    cv::Vec4f point4 { point.x, point.y, point.z, 1 };
    cv::Mat camera_point = modelview * cv::Mat(point4);
    cv::Mat projected = camera_point / camera_point.at<float>(2, 0);
    cv::Mat image_point = intrinsic * projected;
    image_points.push_back({ image_point.at<float>(0, 0), image_point.at<float>(1, 0) });
  }
  
  // Print image points
  std::cout << "Image Points" << std::endl;
  std::cout << "------------" << std::endl;
  for (int i = 0; i < image_points.size(); i++)
  {
    std::cout << "  " << i << ": " << image_points[i].x << ", " << image_points[i].y << std::endl;
  }
  std::cout << std::endl;
  return image_points;
}

static void test_pnp()
{
  using namespace util::math;

  float f = pixart::camera_parameters::focal_length_x_pixels();
  float w = pixart::camera_parameters::pixels_x;
  float h = pixart::camera_parameters::pixels_y;
  float fov = 2 * atan((0.5f * w) / f) * Rad2Deg;
  std::cout << "PixArt Camera Intrinsics" << std::endl;
  std::cout << "------------------------" << std::endl;
  std::cout << "  focal length   = " << f << " pixels" << std::endl;
  std::cout << "  fov (computed) = " << fov << " degrees" << std::endl;
  std::cout << std::endl;
    
  // Camera intrinsic matrix
  float fx = pixart::camera_parameters::focal_length_x_pixels();
  float fy = pixart::camera_parameters::focal_length_y_pixels();
  float cx = 0;
  float cy = 0;
  float intrinsic[3][3] =
  {
    { fx, 0, cx },
    { 0, fy, cy },
    { 0, 0, 1 }
  };
  cv::Mat intrinsic_mat(3, 3, CV_32F, intrinsic);

  // Define object in world units, object-local space
  float object_width = 8e-2f;
  float object_height = 3e-2f;
  std::vector<cv::Point3f> object_points =
  {
    { -0.5f * object_width, 0.5f * object_height, 0 },  // top left corner (facing camera)
    { 0.5f * object_width, 0.5f * object_height, 0 },   // top right corner
    { 0.5f * object_width, -0.5f * object_height, 0 },  // bottom right corner
    { -0.5f * object_width, -0.5f * object_height, 0 }  // bottom left corner
  };
  
  // Model-view matrix (4x4): local rotation and translation from camera (which
  // is positioned at origin)
  //auto euler = cv::Point3f{20, 0, 40};
  //auto position = cv::Point3f{0, 0.1, 30e-2};
  auto euler = cv::Point3f{0,0,0};
  auto position = cv::Point3f{0, 0, 13e-2}; // at this distance, object should span full sensor frame horizontally
  cv::Mat modelview = transform_matrix(euler, position);

  // Print model-view transform information. Rotation is converted to Rodrigues
  // vector form the 3x3 rotation component of the model-view transform matrix.
  // This allows us to compare directly with what solvePnP() computes.
  cv::Mat rot3(modelview, cv::Range(0,3), cv::Range(0,3));  // 3x3 rotation
  cv::Mat rodrigues_mat(3, 1, CV_32F);
  cv::Rodrigues(rot3, rodrigues_mat);
  auto rodrigues = cv::Point3f(rodrigues_mat);  // prints on single line
  std::cout << "Model-View Transform" << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << "  position = " << position << std::endl;
  std::cout << "  rotation = " << rodrigues << std::endl;
  std::cout << std::endl;

  // Solve for model-view transform from image points and print results
  auto image_points = generate_image_points(object_points, intrinsic_mat, modelview);
  cv::Mat rotation_mat; // will be 3x1, single precision
  cv::Mat position_mat; // ""
  bool result = cv::solvePnP(object_points, image_points, intrinsic_mat, cv::Mat(), rotation_mat, position_mat, false, cv::SOLVEPNP_EPNP);
  if (!result)
  {
    std::cout << "Error: solvePnP() found no solution" << std::endl;
  }
  else
  {
    auto position = cv::Point3f(position_mat);
    auto rodrigues = cv::Point3f(rotation_mat);
    std::cout << "Solution" << std::endl;
    std::cout << "--------" << std::endl;
    std::cout << "  position = " << position << std::endl;
    std::cout << "  rotation = " << rodrigues << std::endl;
    std::cout << std::endl;
  }    
}

int main(int argc, char **argv)
{
  test_pnp();
  return 0;
}


