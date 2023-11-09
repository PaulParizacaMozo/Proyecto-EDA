
# Proyecto - Estructura de Datos Avanzados

Replica del proyecto: Efficient Radius Neighbor Search in Three-dimensional Point Clouds

Link proyecto original: [Pagina original](https://github.com/jbehley/octree)


## Construccion

El octree en sí mismo no tiene dependencias.
Sin embargo, para compilar los ejemplos, necesitas [CMake](http://www.cmake.org/) y la [biblioteca Boost C++](http://www.boost.org/).
Para construir los ejemplos, primero debes construir el proyecto:

```bash
mkdir build
cd build
cmake ..
make
```

Para ejecutar los ejemplos, necesitas algunos datos de nube de puntos:

```bash
wget http://jbehley.github.io/data/wachtberg_folds.zip
unzip wachtberg_folds.zip -d data
```

Ahora puedes ejecutar los ejemplos:

```bash
./example1 data/scan_001_points.dat
```

que realiza algunas consultas y demuestra la flexibilidad de nuestra implementación de Octree para manejar diferentes implementaciones de puntos.

Los diferentes ejemplos muestran algunos casos de uso del octree. `example1` demuestra el uso general con tipos de datos de puntos que proporcionan acceso público a las coordenadas x, y, z. `example2` muestra cómo utilizar un tipo de punto diferente, que tiene coordenadas no públicas. `example3` muestra cómo utilizar el método de plantilla dentro de un descriptor también templado.

También proporcionamos un caso de prueba utilizando el [Google Test Framework (GTest)](https://code.google.com/p/googletest/), que se compila automáticamente si el paquete se encuentra ya sea por CMake o en el directorio fuente correspondiente, por ejemplo, /usr/src/gtest/.
Puedes invocar la suite de pruebas con:

```bash
./octree-test
```

## Analisis:

Las siguientes clases estan presentes en el proyecto:

![Texto alternativo](img/ClasesOctree.png)


## Codigo

```cpp
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
        struct access {};

        // Especialización para la dimensión 0 (x)
        template <class PointT>
        struct access<PointT, 0> {
            // Función estática que devuelve la coordenada x del punto
            static float get(const PointT& p) {
                return p.x;
            }
        };

        // Especialización para la dimensión 1 (y)
        template <class PointT>
        struct access<PointT, 1> {
            // Función estática que devuelve la coordenada y del punto
            static float get(const PointT& p) {
                return p.y;
            }
        };

        // Especialización para la dimensión 2 (z)
        template <class PointT>
        struct access<PointT, 2> {
            // Función estática que devuelve la coordenada z del punto
            static float get(const PointT& p) {
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
}
```

Este código es parte de una implementación del octree (árbol octal) y esta enfocado en proporcionar acceso a las coordenadas de puntos en un espacio tridimensional (o más dimensional) de manera genérica.

A continuación, una breve explicación de las partes clave del código:

1. **Namespace `unibn`:**
   - Contiene la implementación del octree y los rasgos para acceder a coordenadas.

2. **Namespace `traits`:**
   - Define un conjunto de rasgos para acceder a coordenadas de puntos en un espacio multidimensional. Estos rasgos se especializan para las dimensiones x, y, y z.

3. **Clase de rasgos (`access`):**
   - Plantilla de estructura que proporciona una interfaz para acceder a coordenadas en una dimensión específica.
   - Se especializa para las dimensiones x, y y z, cada una con una función estática `get` que devuelve la coordenada correspondiente.

4. **Función de conveniencia (`get`):**
   - Ofrece una interfaz sencilla para acceder a las coordenadas de un punto en un espacio multidimensional.
   - Utiliza la estructura de rasgos `access` para obtener la coordenada correspondiente a la dimensión especificada.

Este código modulariza el acceso a las coordenadas de un punto en un espacio multidimensional. La implementación del octree probablemente utilizará estas funciones para acceder a las coordenadas de los puntos almacenados en el árbol. Además, la encapsulación en namespaces y clases permitirá una extensión y prueba más sencilla del código.


Algunas distancias genéricas: Manhattan, Euclídea (al cuadrado) y Distancia máxima.
Una Distancia tiene que implementar los métodos
1. compute de dos puntos p y q para calcular y devolver la distancia entre dos puntos, y
2. norma de coordenadas x, y, z para calcular y devolver la norma de un punto p = (x, y, z)
3. sqr y sqrt of value para calcular el radio correcto si se realiza una comparación utilizando normas al cuadrado

### 1. L1Distance


Método para calcular la distancia de Manhattan entre dos puntos p y q:

```cpp
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
```

### 2. L2Distance

Método para calcular la distancia euclidiana al cuadrado entre dos puntos p y q

```cpp
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
    return r * r;
  }

  // Método para calcular la raíz cuadrada de un valor (también utilizado en comparaciones de distancias)
  static inline float sqrt(float r)  {
    return std::sqrt(r);
  }
};
```

### 3. MaxDistance

Método para calcular la distancia máxima entre dos puntos p y q:

```cpp
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
```

Cada estructura tiene métodos para calcular la distancia entre dos puntos, la norma de un punto y operaciones relacionadas.

### **Declaración de la estructura `OctreeParams`**

```cpp
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
```

**Análisis:**

1. **Estructura `OctreeParams`:**
   - Esta estructura contiene tres miembros de datos: `bucketSize`, `copyPoints`, y `minExtent`.
   - Se declara como `struct` y todos los miembros son públicos (`public:`), lo que significa que pueden ser accedidos directamente desde fuera de la estructura.

2. **Constructor:**
   - La estructura tiene un constructor que toma tres parámetros con valores predeterminados: `bucketSize`, `copyPoints`, y `minExtent`.
   - El constructor inicializa los miembros de la estructura con los valores proporcionados o con los valores predeterminados si no se proporciona ningún valor.

3. **Miembros de la Estructura:**
   - `uint32_t bucketSize`: Representa el tamaño del cubo en el octree. Es un número entero sin signo de 32 bits.
   - `bool copyPoints`: Indica si se deben copiar los puntos al octree. Es un booleano que por defecto se establece en `false`.
   - `float minExtent`: Representa la extensión mínima del octree. Es un número de punto flotante de 32 bits.

Definición de clase llamada `Octree`, una estructura de datos utilizada para particionar el espacio tridimensional.

### **Declaración de la Plantilla de Clase `Octree`**

```cpp
template <typename PointT, typename ContainerT = std::vector<PointT>>
class Octree
{
 public:
  Octree();
  ~Octree();

  void initialize(const ContainerT& pts, const OctreeParams& params = OctreeParams());
  void initialize(const ContainerT& pts, const std::vector<uint32_t>& indexes,
                  const OctreeParams& params = OctreeParams());
  void clear();

  template <typename Distance>
  void radiusNeighbors(const PointT& query, float radius, std::vector<uint32_t>& resultIndices) const;

  template <typename Distance>
  void radiusNeighbors(const PointT& query, float radius, std::vector<uint32_t>& resultIndices,
                       std::vector<float>& distances) const;

  template <typename Distance>
  int32_t findNeighbor(const PointT& query, float minDistance = -1) const;

 protected:
  class Octant
  {
   public:
    Octant();
    ~Octant();

    bool isLeaf;
    float x, y, z;
    float extent;
    uint32_t start, end;
    uint32_t size;
    Octant* child[8];
  };

  Octree(Octree&);
  Octree& operator=(const Octree& oct);

  Octant* createOctant(float x, float y, float z, float extent, uint32_t startIdx, uint32_t endIdx, uint32_t size);

  template <typename Distance>
  bool findNeighbor(const Octant* octant, const PointT& query, float minDistance, float& maxDistance,
                    int32_t& resultIndex) const;

  template <typename Distance>
  void radiusNeighbors(const Octant* octant, const PointT& query, float radius, float sqrRadius,
                       std::vector<uint32_t>& resultIndices) const;

  template <typename Distance>
  void radiusNeighbors(const Octant* octant, const PointT& query, float radius, float sqrRadius,
                       std::vector<uint32_t>& resultIndices, std::vector<float>& distances) const;

  template <typename Distance>
  static bool overlaps(const PointT& query, float radius, float sqRadius, const Octant* o);

  template <typename Distance>
  static bool contains(const PointT& query, float sqRadius, const Octant* octant);

  template <typename Distance>
  static bool inside(const PointT& query, float radius, const Octant* octant);

  OctreeParams params_;
  Octant* root_;
  const ContainerT* data_;
  std::vector<uint32_t> successors_;

  friend class ::OctreeTest;
};
```

#### **Análisis:**

1. **Plantilla de Clase `Octree`:**
   - La clase `Octree` es una plantilla que toma dos tipos de parámetros, `PointT` y `ContainerT`, con este último teniendo un valor predeterminado de `std::vector<PointT>`.
   - La clase incluye funciones para inicializar el octree, limpiar los datos, buscar vecinos dentro de un radio y algunas funciones de utilidad protegidas.

2. **Métodos Públicos:**
   - `initialize`: Inicializa el octree con puntos proporcionados y parámetros opcionales.
   - `clear`: Elimina todos los datos dentro del octree.
   - `radiusNeighbors`: Encuentra vecinos dentro de un radio dado utilizando una plantilla de distancia.
   - `findNeighbor`: Encuentra el vecino más cercano con opciones adicionales.

3. **Métodos Protegidos:**
   - `createOctant`: Crea un octante utilizando puntos en un rango específico.
   - `findNeighbor`: Función auxiliar para buscar vecinos.
   - `radiusNeighbors`: Función auxiliar para encontrar vecinos dentro de un radio en un octante específico.

4. **Clase Interna `Octant`:**
   - Representa un octante dentro del octree con información como coordenadas, tamaño, índices de inicio y fin, entre otros.

5. **Miembros de Datos:**
   - `params_`: Almacena los parámetros del octree.
   - `root_`: Puntero al nodo raíz del octree.
   - `data_`: Puntero a los datos del contenedor.
   - `successors_`: Lista conectada de índices de puntos sucesivos.

6. **Amistad con Clase de Prueba:**
   - La clase `Octree` es amiga de una clase de prueba llamada `OctreeTest`.

Esta plantilla de clase representa un octree con funcionalidades para inicializar, buscar vecinos y realizar operaciones relacionadas en un espacio tridimensional. La implementación se basa en una estructura de octantes y utiliza plantillas de distancias para proporcionar flexibilidad en el cálculo de vecinos.

