#pragma once

// Minimal dependency-free test framework.

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace rutest {

struct Case {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

struct Failure {
    std::string message;
};

inline int& checks() {
    static int n = 0;
    return n;
}

inline void fail(const char* file, int line, const std::string& msg) {
    std::ostringstream ss;
    ss << file << ":" << line << ": " << msg;
    throw Failure{ss.str()};
}

inline int runAll(const std::string& filter = "") {
    int failed = 0, ran = 0;
    for (const auto& c : registry()) {
        if (!filter.empty() && std::string(c.name).find(filter) == std::string::npos) continue;
        ++ran;
        try {
            c.fn();
            std::cout << "[ OK ] " << c.name << "\n";
        } catch (const Failure& f) {
            ++failed;
            std::cout << "[FAIL] " << c.name << "\n       " << f.message << "\n";
        } catch (const std::exception& e) {
            ++failed;
            std::cout << "[FAIL] " << c.name << "\n       exception: " << e.what() << "\n";
        }
    }
    std::cout << "\n" << ran - failed << "/" << ran << " tests passed (" << checks() << " checks)\n";
    return failed == 0 ? 0 : 1;
}

}  // namespace rutest

#define RU_CONCAT2(a, b) a##b
#define RU_CONCAT(a, b) RU_CONCAT2(a, b)
#define TEST_CASE(name)                                                                         \
    static void RU_CONCAT(ru_test_fn_, __LINE__)();                                             \
    static ::rutest::Registrar RU_CONCAT(ru_test_reg_, __LINE__)(name, &RU_CONCAT(ru_test_fn_, __LINE__)); \
    static void RU_CONCAT(ru_test_fn_, __LINE__)()

#define CHECK(cond)                                                                       \
    do {                                                                                  \
        ++::rutest::checks();                                                             \
        if (!(cond)) ::rutest::fail(__FILE__, __LINE__, std::string("CHECK failed: ") + #cond); \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                          \
    do {                                                                                               \
        ++::rutest::checks();                                                                          \
        const double ru_a = (a), ru_b = (b);                                                           \
        if (!(std::abs(ru_a - ru_b) <= (tol))) {                                                       \
            std::ostringstream ru_ss;                                                                  \
            ru_ss << "CHECK_NEAR failed: " #a " = " << ru_a << ", " #b " = " << ru_b << ", tol " << (tol); \
            ::rutest::fail(__FILE__, __LINE__, ru_ss.str());                                           \
        }                                                                                              \
    } while (0)

#define CHECK_REL(a, b, rel) CHECK_NEAR(a, b, std::abs(b) * (rel))

#define CHECK_THROWS(expr)                                                     \
    do {                                                                       \
        ++::rutest::checks();                                                  \
        bool ru_thrown = false;                                                \
        try {                                                                  \
            (void)(expr);                                                      \
        } catch (...) {                                                        \
            ru_thrown = true;                                                  \
        }                                                                      \
        if (!ru_thrown) ::rutest::fail(__FILE__, __LINE__, "expected exception: " #expr); \
    } while (0)
