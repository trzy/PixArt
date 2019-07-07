#include "apps/object_visualizer/render.hpp"
#include <GL/gl.h>
#include <GL/glu.h>

namespace render
{

  void set_camera(float fov_x, float aspect, vector3 position, vector3 euler)
  {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float fov_y = fov_x * aspect;
    gluPerspective(fov_y, (GLfloat) aspect, 0.1f, 1e2f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(-euler.z, 0, 0, 1);
    glRotatef(-euler.y, 0, 1, 0);
    glRotatef(-euler.x, 1, 0, 0);
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