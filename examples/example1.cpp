#include <iostream>
#include <cstdlib>
#include <time.h>

#include "../Octree.hpp"  // Incluye la implementación del octree.
#include "utils.h"         // Incluye funciones de utilidad.

/* Ejemplo 1: Búsqueda de vecinos de radio con acceso predeterminado mediante variables públicas x,y,z.
 */

// Clase que representa un punto tridimensional con variables públicas x, y, z.
class Point3f
{
 public:
  Point3f(float x, float y, float z) : x(x), y(y), z(z)
  {
  }

  float x, y, z;
};

int main(int argc, char** argv)
{
  // Verifica si se proporcionó el nombre del archivo de la nube de puntos como argumento de línea de comandos.
  if (argc < 2)
  {
    std::cerr << "filename of point cloud missing." << std::endl;
    return -1;
  }

  // Lee el nombre del archivo de la nube de puntos desde los argumentos de línea de comandos.
  std::string filename = argv[1];

  // Vector para almacenar los puntos tridimensionales.
  std::vector<Point3f> points;

  // Lee los puntos desde el archivo utilizando la función de utilidad readPoints.
  readPoints<Point3f>(filename, points);

  // Imprime la cantidad de puntos leídos.
  std::cout << "Read " << points.size() << " points." << std::endl;

  // Verifica si la nube de puntos está vacía.
  if (points.size() == 0)
  {
    std::cerr << "Empty point cloud." << std::endl;
    return -1;
  }

  int64_t begin, end;

  // Inicializa el Octree con los puntos de la nube de puntos.
  unibn::Octree<Point3f> octree;
  unibn::OctreeParams params;
  octree.initialize(points);

  // Busca vecinos en un radio específico para un punto dado (primer punto en la nube).
  std::vector<uint32_t> results;
  const Point3f& q = points[0];
  octree.radiusNeighbors<unibn::L2Distance<Point3f>>(q, 0.2f, results);
  std::cout << results.size() << " radius neighbors (r = 0.2m) found for (" << q.x << ", " << q.y << "," << q.z << ")"
            << std::endl;
  for (uint32_t i = 0; i < results.size(); ++i)
  {
    const Point3f& p = points[results[i]];
    std::cout << "  " << results[i] << ": (" << p.x << ", " << p.y << ", " << p.z << ") => "
              << std::sqrt(unibn::L2Distance<Point3f>::compute(p, q)) << std::endl;
  }

  // Realiza consultas para cada punto en la nube de puntos.
  begin = clock();
  for (uint32_t i = 0; i < points.size(); ++i)
  {
    octree.radiusNeighbors<unibn::L2Distance<Point3f>>(points[i], 0.5f, results);
  }
  end = clock();
  double search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
  std::cout << "Searching for all radius neighbors (r = 0.5m) took " << search_time << " seconds." << std::endl;

  // Limpia el octree.
  octree.clear();

  return 0;
}

