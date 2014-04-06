/**
 * From the OpenGL Programming wikibook: http://en.wikibooks.org/wiki/OpenGL_Programming
 * This file is in the public domain.
 * Contributors: Sylvain Beucler
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* Use glew.h instead of gl.h to get all the GL prototypes declared */
#include <GL/glew.h>
/* Using the GLUT library for the base windowing setup */
#include <GL/glut.h>
/* GLM */
// #define GLM_MESSAGES
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/freeglut_ext.h>
#include <GL/glxew.h>
#include <GL/glxext.h>

namespace {

int screen_width=800, screen_height=600;
GLuint vbo_cube_vertices, vbo_cube_colors;
GLuint ibo_cube_elements;
GLuint program;
GLint attribute_coord3d, attribute_v_color;
GLint uniform_mvp;

glm::mat4 view(1);
glm::mat4 anim(1);
int mouse_x, mouse_y;
int mouse_buttons;

const GLchar cube_v_glsl[] = R"(
  attribute vec3 coord3d;
  attribute vec3 v_color;
  uniform mat4 mvp;
  varying vec3 f_color;

  void main(void) {
    gl_Position = mvp * vec4(coord3d, 1.0);
    f_color = v_color;
  }
)";
const GLchar cube_f_glsl[] = R"(
  varying vec3 f_color;

  void main(void) {
    gl_FragColor = vec4(f_color.x, f_color.y, f_color.z, 1.0);
  }
)";


/**
 * Display compilation errors from the OpenGL shader compiler
 */
void print_log(GLuint object)
{
  GLint log_length = 0;
  if (glIsShader(object))
    glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
  else if (glIsProgram(object))
    glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
  else {
    fprintf(stderr, "printlog: Not a shader or a program\n");
    return;
  }

  char* log = (char*)malloc(log_length);

  if (glIsShader(object))
    glGetShaderInfoLog(object, log_length, NULL, log);
  else if (glIsProgram(object))
    glGetProgramInfoLog(object, log_length, NULL, log);

  fprintf(stderr, "%s", log);
  free(log);
}

/**
 * Compile the shader from file 'filename', with error handling
 */
GLuint create_shader(const char* source, GLenum type) {
  GLuint res = glCreateShader(type);
  const GLchar* sources[] = {
    // Define GLSL version
#ifdef GL_ES_VERSION_2_0
    "#version 100\n"
#else
    "#version 120\n"
#endif
    ,
    // GLES2 precision specifiers
#ifdef GL_ES_VERSION_2_0
    // Define default float precision for fragment shaders:
    (type == GL_FRAGMENT_SHADER) ?
    "#ifdef GL_FRAGMENT_PRECISION_HIGH\n"
    "precision highp float;           \n"
    "#else                            \n"
    "precision mediump float;         \n"
    "#endif                           \n"
    : ""
    // Note: OpenGL ES automatically defines this:
    // #define GL_ES
#else
    // Ignore GLES 2 precision specifiers:
    "#define lowp   \n"
    "#define mediump\n"
    "#define highp  \n"
#endif
    ,
    source };
  glShaderSource(res, 3, sources, NULL);

  glCompileShader(res);
  GLint compile_ok = GL_FALSE;
  glGetShaderiv(res, GL_COMPILE_STATUS, &compile_ok);
  if (compile_ok == GL_FALSE) {
    fprintf(stderr, "%s:", source);
    print_log(res);
    glDeleteShader(res);
    return 0;
  }

  return res;
}

int init_resources() {
  GLfloat cube_vertices[] = {
    // front
    -1.0, -1.0,  1.0,
     1.0, -1.0,  1.0,
     1.0,  1.0,  1.0,
    -1.0,  1.0,  1.0,
    // back
    -1.0, -1.0, -1.0,
     1.0, -1.0, -1.0,
     1.0,  1.0, -1.0,
    -1.0,  1.0, -1.0,
  };
  glGenBuffers(1, &vbo_cube_vertices);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_cube_vertices);
  glBufferData(GL_ARRAY_BUFFER, sizeof(cube_vertices), cube_vertices, GL_STATIC_DRAW);

  GLfloat cube_colors[] = {
    // front colors
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    1.0, 1.0, 1.0,
    // back colors
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    1.0, 1.0, 1.0,
  };
  glGenBuffers(1, &vbo_cube_colors);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_cube_colors);
  glBufferData(GL_ARRAY_BUFFER, sizeof(cube_colors), cube_colors, GL_STATIC_DRAW);

  GLushort cube_elements[] = {
    // front
    0, 1, 2,
    2, 3, 0,
    // top
    1, 5, 6,
    6, 2, 1,
    // back
    7, 6, 5,
    5, 4, 7,
    // bottom
    4, 0, 3,
    3, 7, 4,
    // left
    4, 5, 1,
    1, 0, 4,
    // right
    3, 2, 6,
    6, 7, 3,
  };
  glGenBuffers(1, &ibo_cube_elements);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_cube_elements);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cube_elements), cube_elements, GL_STATIC_DRAW);

  GLint link_ok = GL_FALSE;

  GLuint vs, fs;
  if ((vs = create_shader(cube_v_glsl, GL_VERTEX_SHADER))   == 0) return 0;
  if ((fs = create_shader(cube_f_glsl, GL_FRAGMENT_SHADER)) == 0) return 0;

  program = glCreateProgram();
  glAttachShader(program, vs);
  glAttachShader(program, fs);
  glLinkProgram(program);
  glGetProgramiv(program, GL_LINK_STATUS, &link_ok);
  if (!link_ok) {
    fprintf(stderr, "glLinkProgram:");
    print_log(program);
    return 0;
  }

  const char* attribute_name;
  attribute_name = "coord3d";
  attribute_coord3d = glGetAttribLocation(program, attribute_name);
  if (attribute_coord3d == -1) {
    fprintf(stderr, "Could not bind attribute %s\n", attribute_name);
    return 0;
  }
  attribute_name = "v_color";
  attribute_v_color = glGetAttribLocation(program, attribute_name);
  if (attribute_v_color == -1) {
    fprintf(stderr, "Could not bind attribute %s\n", attribute_name);
    return 0;
  }
  const char* uniform_name;
  uniform_name = "mvp";
  uniform_mvp = glGetUniformLocation(program, uniform_name);
  if (uniform_mvp == -1) {
    fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
    return 0;
  }

  view = glm::lookAt(glm::vec3(0.0, 2.0, 0.0), glm::vec3(0.0, 0.0, -4.0), glm::vec3(0.0, 1.0, 0.0));

  return 1;
}

void onIdle() {
  float angle = glutGet(GLUT_ELAPSED_TIME) / 1000.0 * 45;  // 45Â° per second
  glm::vec3 axis_y(0, 1, 0);
  glm::mat4 anim = glm::rotate(glm::mat4(1.0f), angle, axis_y);

  glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0, 0.0, -4.0));
  glm::mat4 view = glm::lookAt(glm::vec3(0.0, 2.0, 0.0), glm::vec3(0.0, 0.0, -4.0), glm::vec3(0.0, 1.0, 0.0));
  glm::mat4 projection = glm::perspective(45.0f, 1.0f*screen_width/screen_height, 0.1f, 10.0f);

  glm::mat4 mvp = projection * view * model * anim;

  glUseProgram(program);
  glUniformMatrix4fv(uniform_mvp, 1, GL_FALSE, glm::value_ptr(mvp));
  glutPostRedisplay();
}

void refresh() {
  glm::vec3 axis_y(0, 1, 0);

  glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(0.0, 0.0, -4.0));
  glm::mat4 projection = glm::perspective(45.0f, 1.0f*screen_width/screen_height, 0.1f, 20.0f);

  glm::mat4 mvp = projection * view * model * anim;
  glUseProgram(program);
  glUniformMatrix4fv(uniform_mvp, 1, GL_FALSE, glm::value_ptr(mvp));
  glutPostRedisplay();
}

void mouseMove(int x, int y) {
  float dx = x - mouse_x;
  float dy = y - mouse_y;
  mouse_x = x;
  mouse_y = y;

  if (mouse_buttons == 0) {
    anim = glm::rotate(anim, dx, glm::vec3(0.0, 1.0, 0.0));
    anim = glm::rotate(anim, dy, glm::vec3(1.0, 0.0, 0.0));
  } else if (mouse_buttons == 2) {
    view = glm::translate(view, glm::vec3(0.02 * dx, -0.02 * dy, 0));
  }
  refresh();
}

void mouseClick(int button, int state, int x, int y) {
  mouse_x = x;
  mouse_y = y;
  mouse_buttons = button;
  if (button < 3 || button > 4) 
    return;  // Not a mouse wheel event.

  if (state == GLUT_UP)
    return;  // It's the end of the event, not the start.

  float dz = 0.4;
  if (button == 4)
    dz = -dz;

  view = glm::translate(view, glm::vec3(0.0, 0.0, dz));
  refresh();
}

void onDisplay() {
  glClearColor(0.0, 0.0, 0.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

  glUseProgram(program);
  glEnableVertexAttribArray(attribute_coord3d);
  // Describe our vertices array to OpenGL (it can't guess its format automatically)
  glBindBuffer(GL_ARRAY_BUFFER, vbo_cube_vertices);
  glVertexAttribPointer(
    attribute_coord3d, // attribute
    3,                 // number of elements per vertex, here (x,y,z)
    GL_FLOAT,          // the type of each element
    GL_FALSE,          // take our values as-is
    0,                 // no extra data between each position
    0                  // offset of first element
  );

  glEnableVertexAttribArray(attribute_v_color);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_cube_colors);
  glVertexAttribPointer(
    attribute_v_color, // attribute
    3,                 // number of elements per vertex, here (R,G,B)
    GL_FLOAT,          // the type of each element
    GL_FALSE,          // take our values as-is
    0,                 // no extra data between each position
    0                  // offset of first element
  );

  /* Push each element in buffer_vertices to the vertex shader */
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_cube_elements);
  int size;  glGetBufferParameteriv(GL_ELEMENT_ARRAY_BUFFER, GL_BUFFER_SIZE, &size);
  glDrawElements(GL_TRIANGLES, size/sizeof(GLushort), GL_UNSIGNED_SHORT, 0);

  glDisableVertexAttribArray(attribute_coord3d);
  glDisableVertexAttribArray(attribute_v_color);
  glutSwapBuffers();
  //glFlush();
  //glFinish();
}

void onReshape(int width, int height) {
  screen_width = width;
  screen_height = height;
  glViewport(0, 0, screen_width, screen_height);
}

void free_resources() {
  glDeleteProgram(program);
  glDeleteBuffers(1, &vbo_cube_vertices);
  glDeleteBuffers(1, &vbo_cube_colors);
  glDeleteBuffers(1, &ibo_cube_elements);
}

}  // namespace

int gl_init(int argc, char* argv[]) {
  glutInit(&argc, argv);

  glutSetOption(GLUT_RENDERING_CONTEXT, GLUT_CREATE_NEW_CONTEXT);
  glutSetOption(GLUT_DIRECT_RENDERING, GLUT_FORCE_INDIRECT_CONTEXT);

  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);
  glutInitWindowSize(screen_width, screen_height);
  glutInitWindowPosition(1440, 0);
  int win = glutCreateWindow("My Rotating Cube");

  GLenum glew_status = glewInit();
  if (glew_status != GLEW_OK) {
    fprintf(stderr, "Error: %s\n", glewGetErrorString(glew_status));
    return 1;
  }

  if (!GLEW_VERSION_2_0) {
    fprintf(stderr, "Error: your graphic card does not support OpenGL 2.0\n");
    return 1;
  }

  Display *dpy = glXGetCurrentDisplay();
  GLXDrawable drawable = glXGetCurrentDrawable();
  const int interval = 2;

  if (drawable) {
    glXSwapIntervalEXT(dpy, drawable, interval);
  }

  unsigned int swap, maxSwap;

  if (drawable) {
      glXQueryDrawable(dpy, drawable, GLX_SWAP_INTERVAL_EXT, &swap);
      glXQueryDrawable(dpy, drawable, GLX_MAX_SWAP_INTERVAL_EXT,
                       &maxSwap);
      printf("The swap interval is %u and the max swap interval is "
             "%u\n", swap, maxSwap);
  }

  if (init_resources()) {
    glutDisplayFunc(onDisplay);
    glutReshapeFunc(onReshape);
    glutMotionFunc(mouseMove);
    glutMouseFunc(mouseClick);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_LESS);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glutMainLoop();
  }

  free_resources();
  return 0;
}
