//varying vec4 point;

isefoij
void main()
{
  gl_Position = ftransform();

  // Vertex in world space
  // point = gl_ModelViewMatrix * gl_Vertex;
  //gl_Position = vec4(1.0, 1.0, 1.0, 1.0);
//  gl_Position = gl_ModelViewMatrix * gl_Vertex;
}
