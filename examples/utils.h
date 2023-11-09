#ifndef EXAMPLES_UTILS_H_
#define EXAMPLES_UTILS_H_

#include <fstream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

// Declaración de la función de utilidad para leer puntos desde un archivo.
template <typename PointT, typename ContainerT>
void readPoints(const std::string& filename, ContainerT& points)
{
  // Abre el archivo para lectura.
  std::ifstream in(filename.c_str());

  // Almacena cada línea del archivo.
  std::string line;

  // Configura el separador de tokens como un espacio en blanco.
  boost::char_separator<char> sep(" ");

  // Lee el archivo en formato "freiburg".
  while (!in.eof())
  {
    std::getline(in, line);
    in.peek();

    // Utiliza boost::tokenizer para dividir la línea en tokens basados en el separador.
    boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

    // Verifica que la línea tenga el formato esperado con 6 tokens.
    if (tokens.size() != 6) continue;

    // Extrae las coordenadas x, y, z de los tokens y las convierte a tipo float.
    float x = boost::lexical_cast<float>(tokens[3]);
    float y = boost::lexical_cast<float>(tokens[4]);
    float z = boost::lexical_cast<float>(tokens[5]);

    // Almacena el punto en el contenedor proporcionado.
    points.push_back(PointT(x, y, z));
  }

  // Cierra el archivo después de la lectura.
  in.close();
}

#endif /* EXAMPLES_UTILS_H_ */

