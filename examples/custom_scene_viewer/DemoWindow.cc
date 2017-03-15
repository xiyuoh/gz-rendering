/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "DemoWindow.hh"
#include <ctime>

#if __APPLE__
  #include <OpenGL/gl.h>
  #include <GLUT/glut.h>
#else
  #include <GL/glew.h>
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif

#include "ignition/rendering/rendering.hh"
#include "ManualSceneDemo.hh"

#if not (__APPLE__ || _WIN32)
  #include <GL/glx.h>
#endif

#define KEY_ESC 27
#define KEY_TAB  9

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////

ManualSceneDemoPtr g_demo;
ImagePtr g_image;

unsigned int imgw = 0;
unsigned int imgh = 0;

bool g_initContext = false;

#if not (__APPLE__ || _WIN32)
  GLXContext g_context;
  Display *g_display;
  GLXDrawable g_drawable;
  GLXContext g_glutContext;
  Display *g_glutDisplay;
  GLXDrawable g_glutDrawable;
#endif

double g_offset = 0.0;
double g_fps = 0.0;

const int g_fpsSize = 10;
double g_fpsQueue[g_fpsSize];
int g_fpsIndex = 0;
int g_fpsCount = 0;
clock_t g_prevTime;

//////////////////////////////////////////////////
void rendering::GlutRun(ManualSceneDemoPtr _demo)
{
#if not (__APPLE__ || _WIN32)
  g_context = glXGetCurrentContext();
  g_display = glXGetCurrentDisplay();
  g_drawable = glXGetCurrentDrawable();
#endif

  g_prevTime = clock();
  g_demo = _demo;
  GlutInitCamera(_demo->CurrentCamera());
  GlutInitContext();
  GlutPrintUsage();

#if not (__APPLE__ || _WIN32)
  g_glutDisplay = glXGetCurrentDisplay();
  g_glutDrawable = glXGetCurrentDrawable();
  g_glutContext = glXGetCurrentContext();
#endif

  glutMainLoop();
}

//////////////////////////////////////////////////
void rendering::GlutDisplay()
{
#if not (__APPLE__ || _WIN32)
  if (g_display)
  {
    glXMakeCurrent(g_display, g_drawable, g_context);
  }
#endif

  g_demo->Update();
  CameraPtr camera = g_demo->CurrentCamera();

  camera->Capture(*g_image);

#if not (__APPLE__ || _WIN32)
  glXMakeCurrent(g_glutDisplay, g_glutDrawable, g_glutContext);
#endif

  unsigned char *data = g_image->Data<unsigned char>();

  glClearColor(0.5, 0.5, 0.5, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPixelZoom(1, -1);
  glRasterPos2f(-1, 1);
  glDrawPixels(imgw, imgh, GL_RGB, GL_UNSIGNED_BYTE, data);

  GlutPrintEngine();
  GlutPrintFPS();

  glutSwapBuffers();
}

//////////////////////////////////////////////////
void rendering::GlutIdle()
{
  glutPostRedisplay();
}

//////////////////////////////////////////////////
void rendering::GlutKeyboard(unsigned char _key, int, int)
{
#if not (__APPLE__ || _WIN32)
  if (g_display)
  {
    glXMakeCurrent(g_display, g_drawable, g_context);
  }
#endif

  switch (_key)
  {
    case KEY_ESC:
    case 'q':
    case 'Q':
      exit(0);

    case KEY_TAB:
      g_demo->NextCamera();
      g_fpsIndex = 0;
      g_fpsCount = 0;

      // for (int i = 0; i < g_fpsSize; ++i) g_fpsQueue[i] = 0;

      break;

    case '-':
    case '_':
      g_demo->PrevScene();
      break;

    case '=':
    case '+':
      g_demo->NextScene();
      break;

    default:
      break;
  }

  if ('0' <= _key && _key <= '9')
  {
    int index = (_key - '1') % 10;
    g_demo->SelectScene(index);
  }

#if not (__APPLE__ || _WIN32)
  glXMakeCurrent(g_glutDisplay, g_glutDrawable, g_glutContext);
#endif
}

//////////////////////////////////////////////////
void rendering::GlutReshape(int, int)
{
}

//////////////////////////////////////////////////
void rendering::GlutInitCamera(CameraPtr _camera)
{
  imgw = _camera->ImageWidth();
  imgh = _camera->ImageHeight();
  Image image = _camera->CreateImage();
  g_image = std::make_shared<Image>(image);
  _camera->Capture(*g_image);
}

//////////////////////////////////////////////////
void rendering::GlutInitContext()
{
  int argc = 0;
  char **argv = 0;
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(imgw, imgh);
  glutCreateWindow("Ignition-Rendering");
  glutDisplayFunc(GlutDisplay);
  glutIdleFunc(GlutIdle);
  glutKeyboardFunc(GlutKeyboard);
  glutReshapeFunc(GlutReshape);

#if not (__APPLE__ || _WIN32)
  glewInit();
#endif
}

//////////////////////////////////////////////////
void rendering::GlutPrintUsage()
{
  std::cout << "===============================" << std::endl;
  std::cout << "  TAB : Switch render engines  " << std::endl;
  std::cout << "   -  : Previous scene         " << std::endl;
  std::cout << "   +  : Next scene             " << std::endl;
  std::cout << "  0-9 : Select scenes 0-9      " << std::endl;
  std::cout << "  ESC : Exit                   " << std::endl;
  std::cout << "===============================" << std::endl;
}

//////////////////////////////////////////////////
void rendering::GlutPrintEngine()
{
  int y = imgh - 20;
  std::string text = "Engine: " +
      g_demo->CurrentCamera()->Scene()->Engine()->Name();
  GlutPrintText(text, 10, y);
}

//////////////////////////////////////////////////
void rendering::GlutPrintFPS()
{
  double total = 0;
  GlutUpdateFPS();

  for (int i = 0; i < g_fpsCount; ++i)
  {
    total += g_fpsQueue[i];
  }

  int y = imgh - 40;
  double fps = total / g_fpsCount;
  std::string fpsText = "FPS: " + std::to_string(fps);
  GlutPrintText(fpsText, 10, y);
}

//////////////////////////////////////////////////
void rendering::GlutUpdateFPS()
{
  clock_t currTime = clock();
  double elapsedTime = double(currTime - g_prevTime) / CLOCKS_PER_SEC;
  g_fpsQueue[g_fpsIndex] = 1 / elapsedTime;
  g_fpsCount = (g_fpsCount >= g_fpsSize) ? g_fpsSize : g_fpsCount + 1;
  g_fpsIndex = (g_fpsIndex + 1) % g_fpsSize;
  g_prevTime = currTime;
}

//////////////////////////////////////////////////
void rendering::GlutPrintText(const std::string &_text, int x, int y)
{
  GlutPrintTextBack(_text, x, y);
  GlutPrintTextFore(_text, x, y);
}

void rendering::GlutPrintTextBack(const std::string &_text, int x, int y)
{
  glColor3f(0, 0, 0);

  for (int i = -2; i < 3; ++i)
  {
    for (int j = -2; j < 3; ++j)
    {
      GlutPrintTextImpl(_text, x + i, y + j);
    }
  }
}

void rendering::GlutPrintTextFore(const std::string &_text, int x, int y)
{
  glColor3f(1, 1, 1);
  GlutPrintTextImpl(_text, x, y);
}

void rendering::GlutPrintTextImpl(const std::string &_text, int x, int y)
{
  glWindowPos2i(x, y);
  const char *ctext = _text.c_str();
  while (*ctext) glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *ctext++);
}
