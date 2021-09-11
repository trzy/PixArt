#include "apps/object_visualizer/render.hpp"
#include <opencv2/opencv.hpp>
#include <GL/gl.h>
#include <GL/glu.h>

namespace render
{

  void set_camera(float fov_x, float aspect, vector3 position, euler3 rotation)
  {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float fov_y = fov_x * aspect;
    gluPerspective(fov_y, (GLfloat) aspect, 0.01f, 1e2f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(-rotation.z, 0, 0, 1);
    glRotatef(-rotation.y, 0, 1, 0);
    glRotatef(-rotation.x, 1, 0, 0);
    glTranslatef(-position.x, -position.y, -position.z);
  }

  namespace node
  {

    transform::transform(vector3 position, vector3 scale, euler3 rotation)
    {
      glPushMatrix();
      glTranslatef(position.x, position.y, position.z);
      glRotatef(rotation.z, 0, 0, 1);
      glRotatef(rotation.y, 0, 1, 0);
      glRotatef(rotation.x, 1, 0, 0);
      glScalef(scale.x, scale.y, scale.z);
    }

    transform::transform(const cv::Mat &rotation3x3, const cv::Mat &translation3x1)
    {
      assert(rotation3x3.type() == translation3x1.type());
      assert(rotation3x3.type() == CV_32F || rotation3x3.type() == CV_64F);

      glPushMatrix();

      if (rotation3x3.type() == CV_32F)
      {
        float matrix[] =
        {
          // Note that OpenGL matrix is stored in column-major format (transposed
          // relative to OpenCV matrix layout)
          rotation3x3.at<float>(0,0),    rotation3x3.at<float>(1,0),    rotation3x3.at<float>(2,0),    0,
          rotation3x3.at<float>(0,1),    rotation3x3.at<float>(1,1),    rotation3x3.at<float>(2,1),    0,
          rotation3x3.at<float>(0,2),    rotation3x3.at<float>(1,2),    rotation3x3.at<float>(2,2),    0,
          translation3x1.at<float>(0,0), translation3x1.at<float>(1,0), translation3x1.at<float>(2,0), 1
        };
        glMultMatrixf(matrix);
      }
      else
      {
        double matrix[] =
        {
          // Note that OpenGL matrix is stored in column-major format (transposed
          // relative to OpenCV matrix layout)
          rotation3x3.at<double>(0,0),    rotation3x3.at<double>(1,0),    rotation3x3.at<double>(2,0),    0,
          rotation3x3.at<double>(0,1),    rotation3x3.at<double>(1,1),    rotation3x3.at<double>(2,1),    0,
          rotation3x3.at<double>(0,2),    rotation3x3.at<double>(1,2),    rotation3x3.at<double>(2,2),    0,
          translation3x1.at<double>(0,0), translation3x1.at<double>(1,0), translation3x1.at<double>(2,0), 1
        };
        glMultMatrixd(matrix);
      }
    }

    transform::~transform()
    {
      glPopMatrix();
    }

    box::box(vector3 position, vector3 scale, euler3 rotation, color3 color)
    {
      glPushMatrix();
      glTranslatef(position.x, position.y, position.z);
      glRotatef(rotation.z, 0, 0, 1);
      glRotatef(rotation.y, 0, 1, 0);
      glRotatef(rotation.x, 1, 0, 0);
      glScalef(scale.x, scale.y, scale.z);
      glBegin(GL_QUADS);
      glColor3f(color.r, color.g, color.b);
      glVertex3f(-0.5f, 0.5f, 0.5f); glVertex3f(0.5f, 0.5f, 0.5f); glVertex3f(0.5f, -0.5f, 0.5f); glVertex3f(-0.5f, -0.5f, 0.5f);     // front
      glVertex3f(0.5f, 0.5f, -0.5f); glVertex3f(-0.5f, 0.5f, -0.5f); glVertex3f(-0.5f, -0.5f, -0.5f); glVertex3f(0.5f, -0.5f, -0.5f); // back
      glVertex3f(-0.5f, 0.5f, -0.5f); glVertex3f(0.5f, 0.5f, -0.5f); glVertex3f(0.5f, 0.5f, 0.5f); glVertex3f(-0.5f, 0.5f, 0.5f);     // top
      glVertex3f(-0.5f, -0.5f, 0.5f); glVertex3f(0.5f, -0.5f, 0.5f); glVertex3f(0.5f, -0.5f, -0.5f); glVertex3f(-0.5f, -0.5f, -0.5f); // bottom
      glVertex3f(-0.5f, 0.5f, -0.5f); glVertex3f(-0.5f, 0.5f, 0.5f); glVertex3f(-0.5f, -0.5f, 0.5f); glVertex3f(-0.5f, -0.5f, -0.5f); // left
      glVertex3f(0.5f, 0.5f, 0.5f); glVertex3f(0.5f, 0.5f, -0.5f); glVertex3f(0.5f, -0.5f, -0.5f); glVertex3f(0.5f, -0.5f, 0.5f);     // right
      glEnd();
    }

    box::~box()
    {
      glPopMatrix();
    }

  } // node

} // render