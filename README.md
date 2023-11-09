
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


