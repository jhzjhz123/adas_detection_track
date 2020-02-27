#include<Eigen/Dense>
#include<iostream>
#include<list>
#include<vector>

using namespace Eigen;
using namespace std;

#define XYZMIN(x, y) (x)<(y)?(x):(y)
#define XYZMAX(x, y) (x)>(y)?(x):(y)

class Hungarian
{
    static constexpr int NORMAL = 0;
    static constexpr int STAR   = 1;
    static constexpr int PRIME  = 2;

private:
    Eigen::Matrix<float,-1,-1,Eigen::RowMajor> mask_matrix;
    Eigen::Matrix<float,-1,-1,Eigen::RowMajor> matrix;
    Eigen::Matrix<float,-1,-1,Eigen::RowMajor> input_matrix;
    Eigen::Matrix<float,-1,-1,Eigen::RowMajor> output_matrix;

    bool *row_mask, *col_mask;
    size_t cols,rows;
    size_t size;
    int min_size;
    size_t saverow = 0, savecol = 0;

    bool find_uncovered_in_matrix(const double item, size_t &row, size_t &col) const ;
    bool pair_in_list(const std::pair<size_t,size_t> &needle, const std::list<std::pair<size_t,size_t> > &haystack);
    void minimize_along_direction(Eigen::Matrix<float,-1,-1,Eigen::RowMajor> &matrix, const bool over_columns);

    void initiate_resize_matrix();
    void initiate_line_mask();

    int step1();
    int step2();
    int step3();
    int step4();
    int step5();

public:
    Hungarian();
    ~Hungarian();
    Eigen::Matrix<float, -1, 2, Eigen::RowMajor> solve(Eigen::Matrix<float,-1,-1,Eigen::RowMajor> m);
};
