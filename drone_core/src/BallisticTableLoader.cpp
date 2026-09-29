#include "service/BallisticTableLoader.hpp"

#include <fstream>
#include <stdexcept>

BallisticTableDTO load_ballistic_table(const char* source)
{
  if (source == nullptr || *source == '\0') {
    throw std::runtime_error("load_ballistic_table: no table source given");
  }

  std::ifstream f(source);
  if (!f.is_open()) {
    throw std::runtime_error("load_ballistic_table: cannot open table source");
  }

  BallisticTableDTO table;

  int nZ, nV, nM, nD, nL;
  f >> nZ >> nV >> nM >> nD >> nL;

  table.z.resize(nZ);
  for (auto& v : table.z)
    f >> v;
  table.v.resize(nV);
  for (auto& v : table.v)
    f >> v;
  table.m.resize(nM);
  for (auto& v : table.m)
    f >> v;
  table.d.resize(nD);
  for (auto& v : table.d)
    f >> v;
  table.l.resize(nL);
  for (auto& v : table.l)
    f >> v;

  if (!f) {
    throw std::runtime_error("load_ballistic_table: malformed table source");
  }

  size_t total = (size_t)nZ * nV * nM * nD * nL;
  table.data.resize(total);

  for (size_t i = 0; i < total; i++) {
    f >> table.data[i].ammo_time_to_fall >> table.data[i].ammo_distance_to_fall;
  }

  if (!f) {
    throw std::runtime_error("load_ballistic_table: malformed table source");
  }

  return table;
}
