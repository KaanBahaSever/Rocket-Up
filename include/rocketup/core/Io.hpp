#pragma once

#include <string>
#include <vector>

#include "rocketup/core/Types.hpp"

namespace rocketup::io {

/// Read a whole file into a string (throws RocketUpError on failure).
std::string readTextFile(const std::string& path);

/// Write a string to a file, creating parent directories (throws on failure).
void writeTextFile(const std::string& path, const std::string& content);

/// Parse a JSON file (comments allowed).
json readJsonFile(const std::string& path);

/// Write pretty-printed JSON.
void writeJsonFile(const std::string& path, const json& j, int indent = 2);

/// Resolve `path` relative to `baseDirectory` when it is not absolute.
std::string resolvePath(const std::string& path, const std::string& baseDirectory);

/// Directory part of a path ("" if none).
std::string directoryOf(const std::string& path);

/// Lower-case file extension including the dot (".eng"). Handles double extensions
/// like ".rumotor.json" by returning only the last one.
std::string extensionOf(const std::string& path);

std::string toLower(std::string s);
std::string trim(const std::string& s);
std::vector<std::string> split(const std::string& s, char delimiter);
/// Split on runs of whitespace.
std::vector<std::string> splitWhitespace(const std::string& s);

/// A numeric CSV table. Lines starting with '#' or ';' are comments, the first
/// non-numeric row (if any) becomes the header, quoted fields are unquoted.
struct CsvTable {
    std::vector<std::string> header;
    std::vector<std::vector<double>> rows;
    std::vector<std::string> comments;      ///< comment lines and non-numeric "key, value" lines
    size_t columnCount() const { return rows.empty() ? header.size() : rows.front().size(); }
    std::vector<double> column(size_t index) const;
    /// Find a column whose (lower-cased) header contains `needle`; -1 if absent.
    int findColumn(const std::string& needle) const;
};

CsvTable parseCsv(const std::string& text);
CsvTable readCsvFile(const std::string& path);

/// Parse a double, accepting a decimal comma ("3,14"); returns false if not a number.
bool tryParseDouble(const std::string& text, double& out);

}  // namespace rocketup::io
