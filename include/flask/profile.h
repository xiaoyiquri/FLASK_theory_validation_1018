#pragma once
#include <chrono>
namespace flask {
class ProfileScope {
    bool enabled_;
    double &total_;
    std::chrono::steady_clock::time_point begin_;
public:
    ProfileScope(bool enabled,double &total):enabled_(enabled),total_(total) {
        if(enabled_) begin_=std::chrono::steady_clock::now();
    }
    ~ProfileScope() { if(enabled_) total_+=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin_).count(); }
};
}
