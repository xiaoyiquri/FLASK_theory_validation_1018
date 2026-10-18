#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace theory {
inline void require(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}
inline void relative(double a, double b, double tolerance, const std::string &message) {
    require(std::isfinite(a) && std::isfinite(b) &&
                std::abs(a-b) <= tolerance * std::max(std::abs(b), 1e-300), message);
}
inline void dimensions(int n, int r) {
    require(n > 0 && r > 0, "n and r must be positive");
    // Dense validation has cubic time and quadratic storage. Reject an
    // oversized request instead of exhausting the user's machine.
    require(n <= 256 && r <= 7 && n*r <= 256,
            "dense numerical audit budget: n <= 256, r <= 7, rn <= 256");
}
inline std::pair<int,int> arguments(int argc, char **argv) {
    require(argc == 3, "usage: executable [n r]");
    auto parse = [](const char *s) {
        std::size_t used = 0;
        const int value = std::stoi(s, &used);
        require(s[used] == '\0', "integer dimensions required");
        return value;
    };
    const int n = parse(argv[1]), r = parse(argv[2]);
    dimensions(n, r);
    return {n, r};
}
template<class F> int run(F f) {
    try { f(); std::cout << "PASS\n"; return 0; }
    catch(const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
inline std::pair<Eigen::MatrixXd, Eigen::MatrixXd> system(int n, int r) {
    dimensions(n,r);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n*r,n*r);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n*r,n);
    for(int i=0;i<r-1;++i) A.block(i*n,(i+1)*n,n,n).setIdentity();
    B.bottomRows(n).setIdentity();
    return {A,B};
}
inline double factorial(int p) { return std::tgamma(p+1.0); }
inline Eigen::MatrixXd gramian(int n, int r, double T, const Eigen::MatrixXd &Q) {
    dimensions(n,r);
    require(std::isfinite(T) && T>0, "positive finite time required");
    require(Q.rows()==n && Q.cols()==n && Q.allFinite(), "invalid inverse weight");
    require(Q.isApprox(Q.transpose(),1e-12), "inverse weight must be symmetric");
    require(Eigen::LLT<Eigen::MatrixXd>(Q).info()==Eigen::Success,
            "inverse weight must be positive definite");
    Eigen::MatrixXd G(n*r,n*r);
    for(int i=0;i<r;++i) for(int j=0;j<r;++j) {
        const int p=r-1-i, q=r-1-j;
        G.block(i*n,j*n,n,n)=Q*std::pow(T,p+q+1)/(factorial(p)*factorial(q)*(p+q+1));
    }
    return G;
}
inline double logdet(const Eigen::MatrixXd &M) {
    Eigen::LLT<Eigen::MatrixXd> llt(M);
    require(llt.info()==Eigen::Success, "Gramian lost positive definiteness at this precision");
    const Eigen::VectorXd d=llt.matrixL().toDenseMatrix().diagonal();
    require((d.array()>0).all(), "positive Cholesky diagonal required");
    return 2*d.array().log().sum();
}
// Exact integration of a degree 2r-2 polynomial using r+1 Gauss nodes.
// This independently checks the closed-form block formula.
inline Eigen::MatrixXd quadratureGramian(int n,int r,double T,const Eigen::MatrixXd &Q) {
    const int count=r+1;
    Eigen::MatrixXd J=Eigen::MatrixXd::Zero(count,count);
    for(int k=1;k<count;++k) J(k-1,k)=J(k,k-1)=k/std::sqrt(4.0*k*k-1);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(J);
    require(eig.info()==Eigen::Success,"Gauss nodes eigensolve failed");
    Eigen::MatrixXd G=Eigen::MatrixXd::Zero(n*r,n*r);
    for(int k=0;k<count;++k) {
        const double t=T*(eig.eigenvalues()(k)+1)/2;
        const double weight=T*std::pow(eig.eigenvectors()(0,k),2);
        Eigen::MatrixXd E(n*r,n);
        for(int i=0;i<r;++i) E.middleRows(i*n,n)=
            Eigen::MatrixXd::Identity(n,n)*std::pow(t,r-1-i)/factorial(r-1-i);
        G.noalias()+=weight*E*Q*E.transpose();
    }
    return G;
}
} // namespace theory
