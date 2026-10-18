#include "theory_test_support.h"
#include "pff/pff_integrator.h"

using namespace theory;
void verify(const Eigen::MatrixXd &A,const Eigen::MatrixXd &B,int n,int r) {
    require(A.rows()==n*r && A.cols()==n*r && B.rows()==n*r && B.cols()==n,
            "derivative-major system dimensions");
    // Include A^(rn)B as printed in Eq.36; all columns from A^r B vanish.
    Eigen::MatrixXd C=Eigen::MatrixXd::Zero(n*r,(n*r+1)*n), power=B;
    for(int k=0;k<=n*r;++k) {
        C.middleCols(k*n,n)=power;
        Eigen::FullPivLU<Eigen::MatrixXd> lu(C.leftCols((k+1)*n));
        require(lu.rank()==n*std::min(k+1,r), "rank growth must be n per derivative block");
        if(k>=r) require(power.isZero(0),"nilpotent tail in Eq.36");
        power=(A*power).eval();
    }
    require(Eigen::FullPivLU<Eigen::MatrixXd>(C).rank()==n*r,"Eq.36 full rank rn");
    for(int j=0;j<n;++j) {
        Eigen::MatrixXd chain(n*r,r);
        Eigen::VectorXd b=B.col(j);
        for(int k=0;k<r;++k) { chain.col(k)=b; b=(A*b).eval(); }
        require(Eigen::FullPivLU<Eigen::MatrixXd>(chain).rank()==r,"Eq.37 each input index r");
        require(b.isZero(0),"each input chain terminates at r");
    }
}
template<int n,int r> void production() {
    const auto actual=rrtpff::IntegratorChain<n,r>::system();
    const auto expected=system(n,r);
    require(actual.A.isApprox(expected.first,0) && actual.B.isApprox(expected.second,0),
            "existing integrator implementation disagrees with Eq.16");
    verify(actual.A,actual.B,n,r);
}
int main(int argc,char **argv) {
    return run([&] {
        if(argc!=1) { const auto [n,r]=arguments(argc,argv); const auto [A,B]=system(n,r);
            verify(A,B,n,r); std::cout << "runtime n="<<n<<" r="<<r<<" rank="<<n*r<<'\n'; return; }
        for(int n=1;n<=8;++n) for(int r=1;r<=6;++r) {
            const auto [A,B]=system(n,r); verify(A,B,n,r);
        }
        production<1,1>(); production<1,2>(); production<2,2>(); production<3,2>();
        production<4,2>(); production<2,3>(); production<2,4>(); production<4,4>();
        production<1,5>();
        auto [A,B]=system(2,3); B.col(1).setZero();
        bool caught=false;
        try { verify(A,B,2,3); } catch(const std::exception &) { caught=true; }
        require(caught,"negative control: a missing input must fail the rank test");
        std::cout << "48 runtime pairs + 9 existing template kernels; all indices equal r\n";
    });
}
