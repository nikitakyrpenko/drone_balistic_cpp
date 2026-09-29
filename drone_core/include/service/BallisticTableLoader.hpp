#pragma once

#include "dto/BallisticTableDTO.hpp"

// Throws std::runtime_error if source is empty/unreadable/malformed.
BallisticTableDTO load_ballistic_table(const char* source);
