#include "t6.hpp"

#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: t6_lut_probe LUT_DIRECTORY < theta,s,phiL,phiR.csv\n";
        return 2;
    }
    try {
        aviator::t6::Lookup lookup(argv[1]);
        std::cout << std::setprecision(17);
        double theta, s, left, right;
        char c1, c2, c3;
        while (std::cin >> theta >> c1 >> s >> c2 >> left >> c3 >> right) {
            if (c1 != ',' || c2 != ',' || c3 != ',') return 2;
            const auto r = lookup.query({theta, s}, {left, right});
            for (double q : r.q) std::cout << q << ',';
            std::cout << r.clearance << ',' << r.margin_minus[0] << ','
                      << r.margin_minus[1] << ',' << r.margin_plus[0] << ','
                      << r.margin_plus[1] << ',' << r.joint_margin << ','
                      << int(r.branch) << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
