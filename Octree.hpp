#ifndef UNIBN_OCTREE_H_
#define UNIBN_OCTREE_H_

#include <stdint.h>
#include <cassert>
#include <cmath>
#include <cstring>  // memset.
#include <limits>
#include <vector>

// necesario para obtener acceso de prueba a miembros protegidos/privados...
namespace
{
class OctreeTest;
}

namespace unibn
{
/*
  Algunos rasgos para acceder a coordenadas independientemente de la implementación
  específica de punto inspirada en boost.geometry, que necesita ser implementada por nuevos puntos.
*/

namespace traits {

  // Plantilla de estructura para los rasgos de acceso a coordenadas
  template <typename PointT, int D>
  struct access{};

  // Especialización para la dimensión 0 (x)
  template <class PointT>
  struct access<PointT, 0> {
    // Función estática que devuelve la coordenada x del punto
    static float get(const PointT& p)  {
      return p.x;
    }
  };

  // Especialización para la dimensión 1 (y)
  template <class PointT>
  struct access<PointT, 1> {
    // Función estática que devuelve la coordenada y del punto
    static float get(const PointT& p)  {
      return p.y;
    }
  };

  // Especialización para la dimensión 2 (z)
  template <class PointT>
  struct access<PointT, 2> {
    // Función estática que devuelve la coordenada z del punto
    static float get(const PointT& p)  {
      return p.z;
    }
  };
}

/** 
  Función de conveniencia para acceder a las coordenadas de un punto en un espacio multidimensional.
**/
template <int D, typename PointT>
inline float get(const PointT& p)
{
  // Utiliza la estructura de rasgos 'access' para obtener la coordenada correspondiente
  return traits::access<PointT, D>::get(p);
}

/*
  Algunas distancias genéricas: Manhattan, Euclídea (al cuadrado) y Distancia máxima.
  Una Distancia tiene que implementar los métodos
  1. compute de dos puntos p y q para calcular y devolver la distancia entre dos puntos, y
  2. norma de coordenadas x, y, z para calcular y devolver la norma de un punto p = (x, y, z)
  3. sqr y sqrt of value para calcular el radio correcto si se realiza una comparación utilizando normas al cuadrado (véase L2Distance)...
*/
template <typename PointT>
struct L1Distance
{
  // Método para calcular la distancia de Manhattan entre dos puntos p y q
  static inline float compute(const PointT& p, const PointT& q)  {
    float diff1 = get<0>(p) - get<0>(q);
    float diff2 = get<1>(p) - get<1>(q);
    float diff3 = get<2>(p) - get<2>(q);

    // La distancia de Manhattan es la suma de las diferencias absolutas en cada dimensión
    return std::abs(diff1) + std::abs(diff2) + std::abs(diff3);
  }

  // Método para calcular la norma de un punto en coordenadas x, y, z
  static inline float norm(float x, float y, float z)  {
    // La norma L1 es la suma de los valores absolutos de las coordenadas
    return std::abs(x) + std::abs(y) + std::abs(z);
  }

  // Método para calcular el cuadrado de un valor (se utiliza en comparaciones de distancias al cuadrado)
  static inline float sqr(float r)  {
    // En el caso de la distancia de Manhattan, el cuadrado es simplemente el valor sin cambios
    return r;
  }

  // Método para calcular la raíz cuadrada de un valor (también utilizado en comparaciones de distancias)
  static inline float sqrt(float r)  {
    // En el caso de la distancia de Manhattan, la raíz cuadrada es simplemente el valor sin cambios
    return r;
  }
};

template <typename PointT>
struct L2Distance
{
  // Método para calcular la distancia euclidiana al cuadrado entre dos puntos p y q
  static inline float compute(const PointT& p, const PointT& q)  {
    float diff1 = get<0>(p) - get<0>(q);
    float diff2 = get<1>(p) - get<1>(q);
    float diff3 = get<2>(p) - get<2>(q);

    // La distancia euclidiana al cuadrado es la suma de los cuadrados de las diferencias en cada dimensión
    return std::pow(diff1, 2) + std::pow(diff2, 2) + std::pow(diff3, 2);
  }

  // Método para calcular la norma euclidiana al cuadrado de un punto en coordenadas x, y, z
  static inline float norm(float x, float y, float z)  {
    // La norma euclidiana al cuadrado es la suma de los cuadrados de las coordenadas
    return std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2);
  }

  // Método para calcular el cuadrado de un valor (se utiliza en comparaciones de distancias)
  static inline float sqr(float r)  {
    // El cuadrado de un valor es el valor multiplicado por sí mismo
    return r * r;
  }

  // Método para calcular la raíz cuadrada de un valor (también utilizado en comparaciones de distancias)
  static inline float sqrt(float r)  {
    // Utiliza la función sqrt estándar para calcular la raíz cuadrada
    return std::sqrt(r);
  }
};

template <typename PointT>
struct MaxDistance
{
  // Método para calcular la distancia máxima entre dos puntos p y q
  static inline float compute(const PointT& p, const PointT& q)  {
    // Calcula las diferencias absolutas en cada dimensión
    float diff1 = std::abs(get<0>(p) - get<0>(q));
    float diff2 = std::abs(get<1>(p) - get<1>(q));
    float diff3 = std::abs(get<2>(p) - get<2>(q));

    // Encuentra el máximo de las diferencias absolutas
    float maximum = diff1;
    if (diff2 > maximum) maximum = diff2;
    if (diff3 > maximum) maximum = diff3;

    return maximum;
  }

  // Método para calcular la norma máxima de un punto en coordenadas x, y, z
  static inline float norm(float x, float y, float z)  {
    // Encuentra el máximo de las coordenadas
    float maximum = x;
    if (y > maximum) maximum = y;
    if (z > maximum) maximum = z;
    return maximum;
  }

  // Método para calcular el cuadrado de un valor (se utiliza en comparaciones de distancias)
  static inline float sqr(float r)  {
    // En el caso de la distancia máxima, el cuadrado es simplemente el valor sin cambios
    return r;
  }

  // Método para calcular la raíz cuadrada de un valor (también utilizado en comparaciones de distancias)
  static inline float sqrt(float r)  {
    // En el caso de la distancia máxima, la raíz cuadrada es simplemente el valor sin cambios
    return r;
  }
};

// Definición de la estructura OctreeParams
struct OctreeParams
{
 public:
  // Constructor con valores por defecto para los parámetros
  OctreeParams(uint32_t bucketSize = 32, bool copyPoints = false, float minExtent = 0.0f)
      : bucketSize(bucketSize), copyPoints(copyPoints), minExtent(minExtent)
  {}
  // Miembros de la estructura
  uint32_t bucketSize;  // Tamaño del cubo en el octree
  bool copyPoints;      // Indica si se deben copiar los puntos al octree
  float minExtent;      // Extensión mínima del octree
};
/*
  Implementación de Octree basada en índices que ofrece diferentes consultas e inserción/eliminación de puntos.
 
  El Octree basado en índices utiliza una relación sucesora y un startIndex en cada Octant para mejorar el rendimiento en tiempo de ejecución
  de las consultas de radio. El almacenamiento eficiente de los puntos mediante la revinculación de los elementos de la lista se basa en la idea
  de que los hijos de un Octante contienen subconjuntos disjuntos de puntos dentro del Octante y que podemos reorganizar los puntos de tal manera
  que obtengamos una lista continua de conexión única que podemos utilizar para almacenar en cada Octante el inicio de esta lista.

  Lo especial de la implementación es que permite buscar vecinos con p-normas arbitrarias, lo que la distingue de la mayoría de las otras
  implementaciones de Octree. Decidimos implementar el Octree usando una plantilla para puntos y contenedores. El contenedor debe tener un operador[],
  que permite acceder a los puntos, y una función miembro size(), que permite obtener el tamaño del contenedor. Para los puntos, utilizamos un rasgo
  de acceso para acceder a las coordenadas inspirado en boost.geometry.
  La implementación ya proporciona un trait de acceso general, que espera tener variables miembro públicas x,y,z.
 
  Si utilizas la implementación o ideas del paper correspondiente en tu trabajo académico, estaría bien que citaras el paper correspondiente:
  J. Behley, V. Steinhage, A.B. Cremers. Efficient Radius Neighbor Search in Three-dimensional Point Clouds (Búsqueda eficiente de vecinos de radio en nubes de puntos tridimensionales).
  En el futuro, podríamos añadir también otras consultas de vecinos e implementar la eliminación y adición de puntos.
 */


// Definición de la plantilla de clase Octree
template <typename PointT, typename ContainerT = std::vector<PointT>>
class Octree
{
 public:
  Octree();
  ~Octree();

  /* Inicializa el octree con todos los puntos */
  void initialize(const ContainerT& pts, const OctreeParams& params = OctreeParams());

  /* Inicializa el octree solo con los puntos que están dentro de los índices dados */
  void initialize(const ContainerT& pts, const std::vector<uint32_t>& indexes,
                  const OctreeParams& params = OctreeParams());

  /* Elimina todos los datos dentro del octree */
  void clear();

  /* Consulta de vecinos dentro de un radio dado donde el radio determina el radio máximo de los índices de puntos en resultIndices */
  template <typename Distance>
  void radiusNeighbors(const PointT& query, float radius, std::vector<uint32_t>& resultIndices) const;

  /* Consulta de vecinos dentro de un radio dado con cálculo explícito (al cuadrado) de la distancia */
  template <typename Distance>
  void radiusNeighbors(const PointT& query, float radius, std::vector<uint32_t>& resultIndices,
                       std::vector<float>& distances) const;

  /* Consulta del vecino más cercano. Con minDistance >= 0, se prohíben explícitamente las auto coincidencias.
   * @return índice del vecino más cercano n con Distance::compute(query, n) > minDistance y, de lo contrario, -1.
   */
  template <typename Distance>
  int32_t findNeighbor(const PointT& query, float minDistance = -1) const;

 protected:
  // Clase Octant (octante) interna
  class Octant
  {
   public:
    Octant();
    ~Octant();

    bool isLeaf; // Determina si es nodo hoja

    // Caja delimitadora del octante necesaria para pruebas de superposición y contención...
    float x, y, z;  // centro
    float extent;   // mitad de la longitud del lado

    uint32_t start, end;  // inicio y fin en succ_
    uint32_t size;        // número de puntos

    Octant* child[8];  // hijos del octante
  };

  // No se puede copiar ni asignar...
  Octree(Octree&);
  Octree& operator=(const Octree& oct);

  /**
   * Creación de un octante utilizando los elementos a partir de startIdx.
   *
   * El método reordena el índice de modo que todos los puntos estén vinculados correctamente a sucesores que pertenecen
   * al mismo octante.
   *
   * \param x,y,z           coordenadas del centro del octante
   * \param extent          extensión del octante
   * \param startIdx        primer índice de puntos dentro del octante
   * \param endIdx          último índice de puntos dentro del octante
   * \param size            número de puntos en el octante
   *
   * \return octante con nodos hijos.
   */
  Octant* createOctant(float x, float y, float z, float extent, uint32_t startIdx, uint32_t endIdx, uint32_t size);

  /** @return true, si la búsqueda ha terminado, de lo contrario, false. **/
  template <typename Distance>
  bool findNeighbor(const Octant* octant, const PointT& query, float minDistance, float& maxDistance,
                    int32_t& resultIndex) const;

  template <typename Distance>
  void radiusNeighbors(const Octant* octant, const PointT& query, float radius, float sqrRadius,
                       std::vector<uint32_t>& resultIndices) const;

  template <typename Distance>
  void radiusNeighbors(const Octant* octant, const PointT& query, float radius, float sqrRadius,
                       std::vector<uint32_t>& resultIndices, std::vector<float>& distances) const;

  /* prueba si la esfera de búsqueda S(q,r) se superpone con el octante
   *
   * @param query   punto de consulta
   * @param radius  radio "al cuadrado"
   * @param o       puntero al octante
   *
   * @return true, si la esfera de búsqueda se superpone con el octante, false en caso contrario.
   */
  template <typename Distance>
  static bool overlaps(const PointT& query, float radius, float sqRadius, const Octant* o);

  /* prueba si la esfera de búsqueda S(q,r) contiene el octante
   *
   * @param query    punto de consulta
   * @param sqRadius radio "al cuadrado"
   * @param octant   puntero al octante
   *
   * @return true, si la esfera de búsqueda se superpone con el octante, false en caso contrario.
   */
  template <typename Distance>
  static bool contains(const PointT& query, float sqRadius, const Octant* octant);

  /* prueba si la esfera de búsqueda S(q,r) está completamente dentro del octante.
   *
   * @param query   punto de consulta
   * @param radius  radio r
   * @param octant  puntero al octante.
   *
   * @return true, si la esfera de búsqueda está completamente dentro del octante, false en caso contrario.
   */
  template <typename Distance>
  static bool inside(const PointT& query, float radius, const Octant* octant);

  OctreeParams params_;  // Parámetros del octree
  Octant* root_;         // Puntero al nodo raíz del octree
  const ContainerT* data_;  // Puntero a los datos del contenedor

  std::vector<uint32_t> successors_;  // Lista conectada de índices de puntos sucesivos...

  friend class ::OctreeTest;  // Clase de prueba amiga
};

//Funciones del Octree

// Constructor de la clase interna Octant
template <typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::Octant::Octant()
    : isLeaf(true), x(0.0f), y(0.0f), z(0.0f), extent(0.0f), start(0), end(0), size(0)
{
  // Inicializa todos los punteros a hijos como nulos
  memset(&child, 0, 8 * sizeof(Octant*));
}

// Destructor de la clase interna Octant
template <typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::Octant::~Octant() {
  // Libera la memoria de los hijos
  for (uint32_t i = 0; i < 8; ++i) {
    delete child[i];
  }
}

// Constructor de la clase Octree
template <typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::Octree()
    : root_(0), data_(0) 
{}

// Destructor de la clase Octree
template <typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::~Octree(){
  // Destructor: Libera la memoria de la raíz
  delete root_;
  // Si se debe copiar los puntos, libera la memoria de los datos del contenedor
  if (params_.copyPoints) {
    delete data_;
  }
}


// Función para inicializar el octree con un conjunto de puntos
template <typename PointT, typename ContainerT>
void Octree<PointT, ContainerT>::initialize(const ContainerT& pts, const OctreeParams& params){
  // Limpia cualquier configuración existente del octree
  clear();
  // Almacena los parámetros proporcionados
  params_ = params;
  // Copia los puntos o almacena un puntero a los puntos, según el parámetro params_.copyPoints
  if (params_.copyPoints)
    data_ = new ContainerT(pts);  // Se copian los puntos si se especifica en params_
  else
    data_ = &pts;  // Se almacena un puntero a los puntos originales

  const uint32_t N = pts.size();
  successors_ = std::vector<uint32_t>(N);

  // Determina la caja delimitadora alineada con los ejes (axis-aligned bounding box).
  float min[3], max[3];
  min[0] = get<0>(pts[0]);
  min[1] = get<1>(pts[0]);
  min[2] = get<2>(pts[0]);
  max[0] = min[0];
  max[1] = min[1];
  max[2] = min[2];

  // Inicializa la lista de sucesores y determina la caja delimitadora alineada con los ejes
  for (uint32_t i = 0; i < N; ++i)
  {
    successors_[i] = i + 1;  // Inicialmente, cada elemento apunta simplemente al siguiente elemento.

    const PointT& p = pts[i];

    if (get<0>(p) < min[0]) min[0] = get<0>(p);
    if (get<1>(p) < min[1]) min[1] = get<1>(p);
    if (get<2>(p) < min[2]) min[2] = get<2>(p);
    if (get<0>(p) > max[0]) max[0] = get<0>(p);
    if (get<1>(p) > max[1]) max[1] = get<1>(p);
    if (get<2>(p) > max[2]) max[2] = get<2>(p);
  }

  // Calcula el centro y la mitad de la longitud del lado de la caja delimitadora
  float ctr[3] = {min[0], min[1], min[2]};
  float maxextent = 0.5f * (max[0] - min[0]);
  ctr[0] += maxextent;

  // Ajusta el centro y la mitad de la longitud del lado para asegurar una caja delimitadora cúbica
  for (uint32_t i = 1; i < 3; ++i)
  {
    float extent = 0.5f * (max[i] - min[i]);
    ctr[i] += extent;
    if (extent > maxextent) maxextent = extent;
  }

  // Crea el octante raíz
  root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, 0, N - 1, N);
}


// Función para inicializar el octree con un conjunto de puntos y una lista de índices
template <typename PointT, typename ContainerT>
void Octree<PointT, ContainerT>::initialize(const ContainerT& pts, const std::vector<uint32_t>& indexes,
                                            const OctreeParams& params)
{
  // Limpia cualquier configuración existente del octree
  clear();

  // Almacena los parámetros proporcionados
  params_ = params;

  // Copia los puntos o almacena un puntero a los puntos, según el parámetro params_.copyPoints
  if (params_.copyPoints)
    data_ = new ContainerT(pts);  // Se copian los puntos si se especifica en params_
  else
    data_ = &pts;  // Se almacena un puntero a los puntos originales

  const uint32_t N = pts.size();
  successors_ = std::vector<uint32_t>(N);

  // Si la lista de índices está vacía, no hay nada más que hacer
  if (indexes.size() == 0) return;

  // Determina la caja delimitadora alineada con los ejes de los puntos especificados por los índices
  uint32_t lastIdx = indexes[0];
  float min[3], max[3];
  min[0] = get<0>(pts[lastIdx]);
  min[1] = get<1>(pts[lastIdx]);
  min[2] = get<2>(pts[lastIdx]);
  max[0] = min[0];
  max[1] = min[1];
  max[2] = min[2];

  // Inicializa la lista de sucesores y determina la caja delimitadora alineada con los ejes
  for (uint32_t i = 1; i < indexes.size(); ++i)
  {
    uint32_t idx = indexes[i];
    // Inicialmente, cada elemento apunta simplemente al siguiente elemento.
    successors_[lastIdx] = idx;

    const PointT& p = pts[idx];

    if (get<0>(p) < min[0]) min[0] = get<0>(p);
    if (get<1>(p) < min[1]) min[1] = get<1>(p);
    if (get<2>(p) < min[2]) min[2] = get<2>(p);
    if (get<0>(p) > max[0]) max[0] = get<0>(p);
    if (get<1>(p) > max[1]) max[1] = get<1>(p);
    if (get<2>(p) > max[2]) max[2] = get<2>(p);

    lastIdx = idx;
  }

  // Calcula el centro y la mitad de la longitud del lado de la caja delimitadora
  float ctr[3] = {min[0], min[1], min[2]};
  float maxextent = 0.5f * (max[0] - min[0]);
  ctr[0] += maxextent;

  // Ajusta el centro y la mitad de la longitud del lado para asegurar una caja delimitadora cúbica
  for (uint32_t i = 1; i < 3; ++i)
  {
    float extent = 0.5f * (max[i] - min[i]);
    ctr[i] += extent;
    if (extent > maxextent) maxextent = extent;
  }

  // Crea el octante raíz utilizando los parámetros calculados y los índices proporcionados
  root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, indexes[0], lastIdx, indexes.size());
}

// Función para limpiar y liberar los recursos del octree
template <typename PointT, typename ContainerT>
void Octree<PointT, ContainerT>::clear() {
  // Elimina la raíz y libera la memoria recursivamente
  delete root_;
  // Si se deben copiar los puntos, libera la memoria de los datos del contenedor
  if (params_.copyPoints) {
    delete data_;
  }
  // Reinicia los punteros y la lista de sucesores
  root_ = 0;
  data_ = 0;
  successors_.clear();
}

// Función para crear un nuevo octante en el octree
template <typename PointT, typename ContainerT>
typename Octree<PointT, ContainerT>::Octant* Octree<PointT, ContainerT>::createOctant(float x, float y, float z,
                                                                                      float extent, uint32_t startIdx,
                                                                                      uint32_t endIdx, uint32_t size)
{
  // Crea un nuevo octante
  Octant* octant = new Octant;

  // Inicializa las propiedades del octante
  octant->isLeaf = true;
  octant->x = x;
  octant->y = y;
  octant->z = z;
  octant->extent = extent;
  octant->start = startIdx;
  octant->end = endIdx;
  octant->size = size;

  // Factor utilizado para calcular las coordenadas de los hijos
  static const float factor[] = {-0.5f, 0.5f};

  // Subdivide el conjunto de puntos y vuelve a vincular los puntos según los códigos de Morton
  if (size > params_.bucketSize && extent > 2 * params_.minExtent)
  {
    octant->isLeaf = false;

    const ContainerT& points = *data_;
    std::vector<uint32_t> childStarts(8, 0);
    std::vector<uint32_t> childEnds(8, 0);
    std::vector<uint32_t> childSizes(8, 0);

    // Re-vincula subconjuntos de puntos no superpuestos...
    uint32_t idx = startIdx;

    for (uint32_t i = 0; i < size; ++i)
    {
      const PointT& p = points[idx];

      // Determina el código de Morton para cada punto...
      uint32_t mortonCode = 0;
      if (get<0>(p) > x) mortonCode |= 1;
      if (get<1>(p) > y) mortonCode |= 2;
      if (get<2>(p) > z) mortonCode |= 4;

      // Establece los inicios de los hijos y actualiza los sucesores...
      if (childSizes[mortonCode] == 0)
        childStarts[mortonCode] = idx;
      else
        successors_[childEnds[mortonCode]] = idx;
      childSizes[mortonCode] += 1;

      childEnds[mortonCode] = idx;
      idx = successors_[idx];
    }

    // Ahora, podemos crear los nodos hijos...
    float childExtent = 0.5f * extent;
    bool firsttime = true;
    uint32_t lastChildIdx = 0;

    for (uint32_t i = 0; i < 8; ++i)
    {
      if (childSizes[i] == 0) continue;

      float childX = x + factor[(i & 1) > 0] * extent;
      float childY = y + factor[(i & 2) > 0] * extent;
      float childZ = z + factor[(i & 4) > 0] * extent;

      // Crea el nodo hijo recursivamente
      octant->child[i] = createOctant(childX, childY, childZ, childExtent, childStarts[i], childEnds[i], childSizes[i]);

      if (firsttime)
        octant->start = octant->child[i]->start;
      else
        successors_[octant->child[lastChildIdx]->end] =
            octant->child[i]->start;  // Asegura que también los finales de los hijos se vinculen al inicio del siguiente hijo.

      lastChildIdx = i;
      octant->end = octant->child[i]->end;
      firsttime = false;
    }
  }

  return octant;
}


// Función para buscar vecinos dentro de un radio alrededor de un punto de consulta en el octree
template <typename PointT, typename ContainerT>
template <typename Distance>
void Octree<PointT, ContainerT>::radiusNeighbors(const Octant* octant, const PointT& query, float radius,
                                                 float sqrRadius, std::vector<uint32_t>& resultIndices) const
{
  const ContainerT& points = *data_;

  // Si la esfera de búsqueda S(q, r) contiene el octante, simplemente agregamos los índices de los puntos.
  if (contains<Distance>(query, sqrRadius, octant))
  {
    uint32_t idx = octant->start;
    for (uint32_t i = 0; i < octant->size; ++i)
    {
      resultIndices.push_back(idx);
      idx = successors_[idx];
    }

    return;  // Podamos temprano.
  }

  if (octant->isLeaf)
  {
    // Si el octante es una hoja, revisamos cada punto dentro del octante.
    uint32_t idx = octant->start;
    for (uint32_t i = 0; i < octant->size; ++i)
    {
      const PointT& p = points[idx];
      float dist = Distance::compute(query, p);

      // Si la distancia es menor que el cuadrado del radio, agregamos el índice del punto.
      if (dist < sqrRadius)
        resultIndices.push_back(idx);

      idx = successors_[idx];
    }

    return;
  }

  // Verificamos si los nodos hijos están en el rango.
  for (uint32_t c = 0; c < 8; ++c)
  {
    if (octant->child[c] == 0) continue;

    // Si no hay superposición entre la esfera de búsqueda y el octante hijo, continuamos con el siguiente hijo.
    if (!overlaps<Distance>(query, radius, sqrRadius, octant->child[c]))
      continue;

    // Llamada recursiva para buscar vecinos en el octante hijo.
    radiusNeighbors<Distance>(octant->child[c], query, radius, sqrRadius, resultIndices);
  }
}

// Función para buscar vecinos dentro de un radio alrededor de un punto de consulta en el octree
template <typename PointT, typename ContainerT>
template <typename Distance>
void Octree<PointT, ContainerT>::radiusNeighbors(const Octant* octant, const PointT& query, float radius,
                                                 float sqrRadius, std::vector<uint32_t>& resultIndices,
                                                 std::vector<float>& distances) const
{
  const ContainerT& points = *data_;

  // Si la esfera de búsqueda S(q, r) contiene el octante, simplemente agregamos los índices de los puntos y calculamos las distancias al cuadrado.
  if (contains<Distance>(query, sqrRadius, octant))
  {
    uint32_t idx = octant->start;
    for (uint32_t i = 0; i < octant->size; ++i)
    {
      resultIndices.push_back(idx);
      distances.push_back(Distance::compute(query, points[idx]));
      idx = successors_[idx];
    }

    return;  // Podamos temprano.
  }

  if (octant->isLeaf)
  {
    // Si el octante es una hoja, revisamos cada punto dentro del octante.
    uint32_t idx = octant->start;
    for (uint32_t i = 0; i < octant->size; ++i)
    {
      const PointT& p = points[idx];
      float dist = Distance::compute(query, p);

      // Si la distancia es menor que el cuadrado del radio, agregamos el índice del punto y la distancia al vector de resultados.
      if (dist < sqrRadius)
      {
        resultIndices.push_back(idx);
        distances.push_back(dist);
      }
      idx = successors_[idx];
    }

    return;
  }

  // Verificamos si los nodos hijos están en el rango.
  for (uint32_t c = 0; c < 8; ++c)
  {
    if (octant->child[c] == 0) continue;

    // Si no hay superposición entre la esfera de búsqueda y el octante hijo, continuamos con el siguiente hijo.
    if (!overlaps<Distance>(query, radius, sqrRadius, octant->child[c])) continue;

    // Llamada recursiva para buscar vecinos en el octante hijo.
    radiusNeighbors<Distance>(octant->child[c], query, radius, sqrRadius, resultIndices, distances);
  }
}

// Función para buscar vecinos dentro de un radio alrededor de un punto de consulta en el octree
template <typename PointT, typename ContainerT>
template <typename Distance>
void Octree<PointT, ContainerT>::radiusNeighbors(const PointT& query, float radius,
                                                 std::vector<uint32_t>& resultIndices) const
{
  resultIndices.clear();  // Limpiamos el vector de resultados.
  if (root_ == 0) return;  // Si el octree está vacío, terminamos la función.

  float sqrRadius = Distance::sqr(radius);  // Calculamos el cuadrado del radio.

  // Llamamos a la función recursiva para buscar vecinos en el octree.
  radiusNeighbors<Distance>(root_, query, radius, sqrRadius, resultIndices);
}

// Función para buscar vecinos dentro de un radio alrededor de un punto de consulta en el octree
template <typename PointT, typename ContainerT>
template <typename Distance>
void Octree<PointT, ContainerT>::radiusNeighbors(const PointT& query, float radius,
                                                 std::vector<uint32_t>& resultIndices,
                                                 std::vector<float>& distances) const
{
  resultIndices.clear();   // Limpiamos el vector de resultados de índices.
  distances.clear();        // Limpiamos el vector de resultados de distancias.
  if (root_ == 0) return;   // Si el octree está vacío, terminamos la función.

  float sqrRadius = Distance::sqr(radius);  // Calculamos el cuadrado del radio.

  // Llamada a la función recursiva para buscar vecinos en el octree.
  radiusNeighbors<Distance>(root_, query, radius, sqrRadius, resultIndices, distances);
}

// Función para verificar si una esfera se superpone con un octante en el octree
template <typename PointT, typename ContainerT>
template <typename Distance>
bool Octree<PointT, ContainerT>::overlaps(const PointT& query, float radius, float sqRadius, const Octant* o)
{
  // Exploitamos la simetría para reducir la prueba a verificar si está dentro de la suma de Minkowski alrededor del cuadrante positivo.
  float x = get<0>(query) - o->x;
  float y = get<1>(query) - o->y;
  float z = get<2>(query) - o->z;

  x = std::abs(x);
  y = std::abs(y);
  z = std::abs(z);

  float maxdist = radius + o->extent;

  // Completamente afuera, ya que q' está fuera del área relevante.
  if (x > maxdist || y > maxdist || z > maxdist) return false;

  int32_t num_less_extent = (x < o->extent) + (y < o->extent) + (z < o->extent);

  // Verificando diferentes casos:

  // a. Dentro de la región superficial del octante.
  if (num_less_extent > 1) return true;

  // b. Verificando la región de la esquina y la región del borde.
  x = std::max(x - o->extent, 0.0f);
  y = std::max(y - o->extent, 0.0f);
  z = std::max(z - o->extent, 0.0f);

  return (Distance::norm(x, y, z) < sqRadius);
}

// Función para verificar si un octante contiene completamente una esfera en el octree
template <typename PointT, typename ContainerT>
template <typename Distance>
bool Octree<PointT, ContainerT>::contains(const PointT& query, float sqRadius, const Octant* o)
{
  // Explotamos la simetría para reducir la prueba a verificar
  // si la esquina más lejana está dentro de la bola de búsqueda.
  float x = get<0>(query) - o->x;
  float y = get<1>(query) - o->y;
  float z = get<2>(query) - o->z;

  x = std::abs(x);
  y = std::abs(y);
  z = std::abs(z);
  // Recordatorio: (x, y, z) - (-e, -e, -e) = (x, y, z) + (e, e, e)
  x += o->extent;
  y += o->extent;
  z += o->extent;

  // Verificamos si la norma euclidiana al cuadrado de la esquina más lejana está dentro del cuadrado del radio.
  return (Distance::norm(x, y, z) < sqRadius);
}


// Función para encontrar el vecino más cercano dentro del octree
template <typename PointT, typename ContainerT>
template <typename Distance>
int32_t Octree<PointT, ContainerT>::findNeighbor(const PointT& query, float minDistance) const
{
  float maxDistance = std::numeric_limits<float>::infinity();
  int32_t resultIndex = -1;

  // Si el octree está vacío, retornamos -1 indicando que no hay vecino.
  if (root_ == 0)
    return resultIndex;

  // Llamada a la función recursiva para encontrar el vecino más cercano.
  findNeighbor<Distance>(root_, query, minDistance, maxDistance, resultIndex);

  // Devolvemos el índice del vecino más cercano encontrado.
  return resultIndex;
}

// Función privada para encontrar el vecino más cercano de manera recursiva
template <typename PointT, typename ContainerT>
template <typename Distance>
bool Octree<PointT, ContainerT>::findNeighbor(const Octant* octant, const PointT& query, float minDistance,
                                              float& maxDistance, int32_t& resultIndex) const
{
  const ContainerT& points = *data_;

  // 1. Descendemos al nodo hoja y comprobamos en los puntos de la hoja.
  if (octant->isLeaf)
  {
    uint32_t idx = octant->start;
    float sqrMaxDistance = Distance::sqr(maxDistance);
    float sqrMinDistance = (minDistance < 0) ? minDistance : Distance::sqr(minDistance);

    for (uint32_t i = 0; i < octant->size; ++i)
    {
      const PointT& p = points[idx];
      float dist = Distance::compute(query, p);
      if (dist > sqrMinDistance && dist < sqrMaxDistance)
      {
        resultIndex = idx;
        sqrMaxDistance = dist;
      }
      idx = successors_[idx];
    }

    // Actualizamos la distancia máxima con el vecino más cercano encontrado y verificamos si está dentro del octante.
    maxDistance = Distance::sqrt(sqrMaxDistance);
    return inside<Distance>(query, maxDistance, octant);
  }

  // Determinamos el código Morton para cada punto.
  uint32_t mortonCode = 0;
  if (get<0>(query) > octant->x) mortonCode |= 1;
  if (get<1>(query) > octant->y) mortonCode |= 2;
  if (get<2>(query) > octant->z) mortonCode |= 4;

  // 2. Si el vecino está en un octante hijo, realizamos una llamada recursiva.
  if (octant->child[mortonCode] != 0)
  {
    if (findNeighbor<Distance>(octant->child[mortonCode], query, minDistance, maxDistance, resultIndex))
      return true;
  }

  // 3. Si el mejor punto actual está completamente dentro, simplemente retornamos.
  float sqrMaxDistance = Distance::sqr(maxDistance);

  // 4. Comprobamos octantes adyacentes para ver si se superponen y los comprobamos si es necesario.
  for (uint32_t c = 0; c < 8; ++c)
  {
    if (c == mortonCode) continue;
    if (octant->child[c] == 0) continue;
    if (!overlaps<Distance>(query, maxDistance, sqrMaxDistance, octant->child[c])) continue;
    if (findNeighbor<Distance>(octant->child[c], query, minDistance, maxDistance, resultIndex))
      return true;  // Podamos temprano.
  }

  // Todos los hijos han sido comprobados... comprobamos si el punto está dentro del octante actual.
  return inside<Distance>(query, maxDistance, octant);
}

// Función privada para verificar si una esfera está completamente dentro de un octante
template <typename PointT, typename ContainerT>
template <typename Distance>
bool Octree<PointT, ContainerT>::inside(const PointT& query, float radius, const Octant* octant)
{
  // Explotamos la simetría para reducir la prueba y verificar
  // si la esquina más lejana está dentro de la esfera de búsqueda.
  float x = get<0>(query) - octant->x;
  float y = get<1>(query) - octant->y;
  float z = get<2>(query) - octant->z;

  // Añadimos el radio a las coordenadas absolutas de la diferencia.
  x = std::abs(x) + radius;
  y = std::abs(y) + radius;
  z = std::abs(z) + radius;

  // Comprobamos si las coordenadas ajustadas están dentro de la mitad del tamaño del octante.
  if (x > octant->extent) return false;
  if (y > octant->extent) return false;
  if (z > octant->extent) return false;

  // Si todas las coordenadas ajustadas están dentro, la esfera está completamente dentro del octante.
  return true;
}

}

#endif /* OCTREE_HPP_ */
