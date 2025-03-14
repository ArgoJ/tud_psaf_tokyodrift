#ifndef PSAF_UCBRIDGE__DATA_LOGGER_HPP_
#define PSAF_UCBRIDGE__DATA_LOGGER_HPP_

#include <string>
#include <vector>
#include <fstream>

class DataLogger
{
public:
  explicit DataLogger(std::string filename);
  ~DataLogger();
  void log(const std::string & data);

private:
  std::ofstream filehandle;
  std::vector<std::string> buffer;
};

#endif  // PSAF_UCBRIDGE__DATA_LOGGER_HPP_
