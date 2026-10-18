#include "theory_test_support.h"
#include "pff/pff_integrator.h"
#include <iomanip>
using namespace theory;

Eigen::MatrixXd inverseWeight(int n) {
    // Fixed SPD weight, with cross-channel terms; not just R=I.
    Eigen::VectorXd v=Eigen::VectorXd::LinSpaced(n,0.1,0.3);
    return Eigen::MatrixXd::Identity(n,n)+v*v.transpose();
}
void verify(int n,int r) {
    const auto Q=inverseWeight(n);
    const auto H=gramian(1,r,1,Eigen::MatrixXd::Identity(1,1));
    const double logC=n*logdet(H)+r*logdet(Q);
    const double exponent=double(r)*r*n;
    double previousLog=0,previousT=0;
    double slope=0;
    for(double T:{1e-1,3e-2,1e-2,3e-3,1e-3,1e-4}) {
        const auto G=gramian(n,r,T,Q);
        Eigen::VectorXd scale(n*r);
        for(int i=0;i<r;++i) scale.segment(i*n,n).setConstant(std::pow(T,r-1-i+0.5));
        const Eigen::MatrixXd normalized=scale.cwiseInverse().asDiagonal()*G*
                                         scale.cwiseInverse().asDiagonal();
        const double logG=logdet(normalized)+2*scale.array().log().sum();
        // High-order Hilbert matrices limit floating-point accuracy, even
        // after time normalization. The symbolic identity remains exact.
        const double tol=3e-6;
        require(std::abs(logG-exponent*std::log(T)-logC)<tol,
                "det(G)/T^(r*r*n) must be constant and positive");
        require(logG>=logC+exponent*std::log(T)-tol &&
                logG<=logC+exponent*std::log(T)+tol,"Theta bounds in log space");
        const Eigen::MatrixXd numeric=quadratureGramian(n,r,T,Q);
        // Compare every block to avoid masking errors in small entries by
        // the norm of the bottom block.
        for(int i=0;i<r;++i) for(int j=0;j<r;++j) {
            const auto a=G.block(i*n,j*n,n,n), b=numeric.block(i*n,j*n,n,n);
            require((a-b).norm()<=2e-11*a.norm(),"independent Gramian integral mismatch");
        }
        if(r<=2 && n<=4) relative(G.determinant(),std::exp(logC)*std::pow(T,exponent),
                                  1e-8,"raw determinant sanity check");
        if(previousT) { slope=(logG-previousLog)/std::log(T/previousT);
            require(std::abs(slope-exponent)<tol*2,"small-time log/log slope r^2*n"); }
        previousLog=logG; previousT=T;
    }
    std::cout << "n="<<n<<" r="<<r<<" exponent="<<exponent
              <<" slope="<<slope<<" log(C)="<<logC<<'\n';
}
template<int n,int r> void production() {
    const typename rrtpff::IntegratorChain<n,r>::Weight Q=inverseWeight(n);
    for(double T:{0.1,0.01,0.0001}) {
        const auto actual=rrtpff::IntegratorChain<n,r>::gramian(T,Q);
        const auto expected=quadratureGramian(n,r,T,Q);
        for(int i=0;i<r;++i) for(int j=0;j<r;++j)
            require((actual.template block<n,n>(i*n,j*n)-expected.block(i*n,j*n,n,n)).norm()
                    < 2e-11*expected.block(i*n,j*n,n,n).norm(),"existing Gramian kernel integral");
    }
}
int main(int argc,char **argv) {
    return run([&] {
        std::cout << std::setprecision(12);
        if(argc!=1) { const auto [n,r]=arguments(argc,argv); verify(n,r); return; }
        for(int n=1;n<=8;++n) for(int r=1;r<=6;++r) verify(n,r);
        production<1,1>(); production<1,2>(); production<2,2>(); production<3,2>();
        production<4,2>(); production<2,3>(); production<2,4>(); production<4,4>(); production<1,5>();
        bool caught=false;
        try { gramian(2,2,0.1,Eigen::MatrixXd::Zero(2,2)); }
        catch(const std::exception &) { caught=true; }
        require(caught,"singular R inverse must be rejected");
    });
}
