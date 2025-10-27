#include <string>
#include <sstream>
#include <vector>
#include <cctype>
using namespace std;

int verify_material(const std::string& tag, const std::string& inputs) {
    // Only these tags are valid
    if (tag != "matte" && tag != "metal" && tag != "refractive") {
        return -1;
    }

    // Split input string by spaces
    // Split input string by spaces
    istringstream iss(inputs);
    vector<string> tokens;
    string token;
    // Try reading once before the loop
    iss >> token;

    // Keep looping while reading succeeds
    while (iss.good()) {

        // Store the successfully read token
        tokens.push_back(token);

        // Attempt to read the next one for the next iteration
        iss >> token;
    }

    while (iss >> token) tokens.push_back(token);

    // Validate that all tokens are numeric
    for (const auto& v : tokens) {
        char* endptr = nullptr;
        std::strtod(v.c_str(), &endptr);
        if (endptr == v.c_str() || *endptr != '\0') {
            return -1; // not a valid number
        }
    }

    // Check number of numeric values per material type
    if (tag == "matte") {
        if (tokens.size() != 3) return -1;
    }
    else if (tag == "metal") {
        if (tokens.size() != 4) return -1;
    }
    else if (tag == "refractive") {
        if (tokens.size() != 1) return -1;
    }

    return 0; // all good
}
