
# Proyecto - Estructura de Datos Avanzados

Replica del proyecto: Efficient Radius Neighbor Search in Three-dimensional Point Clouds

Link poryecto original: [Pagina original](https://github.com/jbehley/octree)


## Contruccion

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
